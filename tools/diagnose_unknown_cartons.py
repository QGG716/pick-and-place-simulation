"""Saved RGB-D/mask contact surfaces for mixed cartons of unknown dimensions.

No inference, devices, inferred full cartons, collision certificates or permits.
"""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from PIL import Image
from unloading_sim.workbench.ui import ROOT
from unloading_sim.workbench.detection import metric_points
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.geometry import contact_candidates,transform
from unloading_sim.workbench.single_carton.size_policy import size_policy


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('record',type=Path)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/workbench/single_carton.example.json',
        help='Explicit UNKNOWN_PER_INSTANCE configuration for historical surface-only diagnosis')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    config_path=args.config.resolve();config=load(config_path)
    if size_policy(config.get('geometry'))!='UNKNOWN_PER_INSTANCE':raise ValueError('UNKNOWN_PER_INSTANCE_POLICY_REQUIRED')
    record=args.record.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    names=['observation.json','observation.npz','segmentation.npz','geometry.json']
    hashes={n:hashlib.sha256((record/n).read_bytes()).hexdigest() for n in names}
    obs=json.loads((record/'observation.json').read_text());arrays=np.load(record/'observation.npz',allow_pickle=False)
    saved=np.load(record/'segmentation.npz',allow_pickle=False);old=json.loads((record/'geometry.json').read_text())
    frame=SimpleNamespace(rgb=arrays['rgb'],depth=arrays['depth'],intrinsics=obs['intrinsics'],
        depth_scale_m=obs['depth_scale_m'],observation_id=obs['observation_id'])
    masks=[saved[f'mask_{i}'] for i in range(len(old['detections']))]
    boxes,rejected=recover_boxes(frame,obs['T_world_camera'],old['detections'],masks,config['geometry'])
    assert not boxes,'UNKNOWN_DIMENSIONS_MUST_NOT_CREATE_COMPLETE_CARTONS'
    items=[r for r in rejected if (r.get('evidence') or {}).get('observed_face')]
    if not items:raise ValueError('NO_TRUSTWORTHY_OBSERVED_SURFACE:'+str([r['reason'] for r in rejected]))
    items.sort(key=lambda r:-r['evidence']['observed_face']['center_m'][2])
    tool_path=ROOT/'configs/local/eco65_desktop/tool.json';tool=json.loads(tool_path.read_text())
    points=metric_points(frame);world=transform(obs['T_world_camera'],points.reshape(-1,3)).reshape(points.shape)
    image=frame.rgb.copy();clouds=[]
    for rank,item in enumerate(items):
        mask=masks[item['detection_index']];e=item['evidence'];face=e['observed_face']
        color=np.array([60,230,220] if rank==0 else [245,180,60]);image[mask]=(image[mask]*.6+color*.4).astype(np.uint8)
        try:
            candidates=contact_candidates(dict(faces=[face]),tool,config['topology'],config.get('contact_search'))
            e['contact_candidate_check']=dict(count=len(candidates),
                anticipated_seal_counts=sorted({len(c['active_ids']) for c in candidates}),
                all_observed_coverage=all(c['seal_check']['observed_surface_covered'] for c in candidates),
                scope='OBSERVED_CONTACT_GEOMETRY_ONLY_NOT_A_PLAN')
        except ValueError as error:e['contact_candidate_check']=dict(count=0,reason=str(error))
        valid=mask&np.isfinite(points).all(-1)&(points[...,2]>.1)&(points[...,2]<3.)
        z=np.where(valid,points[...,2],0).astype('float32');kernel=np.ones((3,3),'uint8')
        valid &= (cv2.dilate(z,kernel)-cv2.erode(z,kernel))<.012
        valid &= abs((world-face['center_m'])@np.asarray(face['normal']))<=.002
        cloud=world[valid];clouds.append(cloud[::max(1,int(np.ceil(len(cloud)/8000)))].tolist())
    report=dict(scope='UNKNOWN_MIXED_CARTON_SURFACE_DIAGNOSIS',source_record=str(record.relative_to(ROOT)),
        source_hashes=hashes,geometry_config=config['geometry'],prior_applied=False,shared_prior_applicable=False,
        current_carton_overlap_m=None,old_overlap_applicability='INVALID_FOR_CURRENT_MIXED_CARTONS',
        geometry_status='GEOMETRY_UNRESOLVED',complete_carton_count=0,
        observed_surface_count=len(items),instances=items,rejected_without_surface=[r for r in rejected if r not in items],
        full_path_status='NOT_PLANNED_UNBOUNDED_INSTANCE_EXTENTS',hardware_execution='NOT_RUN',
        permits_issued=0,inference_rerun=False,gpu_used=False,source_unchanged=True,
        config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),tool_sha256=hashlib.sha256(tool_path.read_bytes()).hexdigest())
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    buf=io.BytesIO();Image.fromarray(image).save(buf,format='PNG');(out/'same-frame-segmentation.png').write_bytes(buf.getvalue())
    data=dict(report=report,clouds=clouds,image='data:image/png;base64,'+base64.b64encode(buf.getvalue()).decode())
    vendor=ROOT/'tools/search_replay/vendor';js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    template=(ROOT/'tools/search_replay/unknown_surfaces.html').read_text()
    (out/'index.html').write_text(template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':')).replace('</','<\\/')))
    for n,h in hashes.items():assert hashlib.sha256((record/n).read_bytes()).hexdigest()==h,'SOURCE_CHANGED'
    print(json.dumps(dict(output=str(out),surfaces=len(items),full_cartons=0,
        candidates=[r['evidence']['contact_candidate_check'] for r in items]),ensure_ascii=False))


if __name__=='__main__':main()
