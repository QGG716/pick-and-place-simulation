"""Explain a saved cuboid-envelope conflict. No inference, devices or permits.

Reuses original RGB-D and masks. A separation witness answers only whether the
old encoded face/prior constraints imply penetration; it is not a recovered
unique pose, a collision-qualified grasp, or a replacement execution envelope.
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
from scipy.optimize import linprog
from unloading_sim.workbench.detection import metric_points
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.geometry import corners,transform,obb_overlap_depth
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.ui import ROOT


def separated_witness(boxes):
    """LP feasibility for fixed stored yaw centres, with variable sizes/centres.

    Retain the original +/- size tolerance, centre intervals, 3 mm observed
    patch noise and the measured lower support height. Do not claim to use
    constraints (e.g. side-plane intercept or stack relations) the old recovery
    did not encode. A witness disproves an all-hypotheses-penetrate claim only.
    """
    support=boxes[1]['quality'].get('observed_support_plane')
    if not support:return None
    level=support['top_world_m'];best=None;count=0
    for ia,a in enumerate(boxes[0]['uncertainty']['hypotheses']):
        for ib,b in enumerate(boxes[1]['uncertainty']['hypotheses']):
            A=[];B=[];bounds=[];rows=[];frames=[]
            for k,(box,h) in enumerate(zip(boxes,(a,b))):
                ref=np.asarray(box['uncertainty']['reference_pose']);angle=h['yaw_center_rad']
                rot=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
                R=ref[:3,:3]@rot;frames.append((ref,R,rot))
                uv=np.asarray(box['faces'][0]['support_points_uv_m'])@rot[:2,:2]
                lo,hi=np.asarray(h['center_interval_uv_m']);dims=np.asarray(h['dimensions_m']);tol=box['uncertainty']['dimension_tolerance_m']
                bounds.extend(list(zip(lo,hi))+[(dims[j]-tol if j<2 else dims[j],dims[j]+tol) for j in range(3)])
                for j in range(2):
                    row=np.zeros(10);row[5*k+j]=-1;row[5*k+2+j]=-.5;A.append(row);B.append(-uv[:,j].max()+.003)
                    row=np.zeros(10);row[5*k+j]=1;row[5*k+2+j]=-.5;A.append(row);B.append(uv[:,j].min()+.003)
                bottom=np.zeros(10);top=np.zeros(10);bottom[5*k:5*k+2]=top[5*k:5*k+2]=R[2,:2]
                bottom[5*k+2:5*k+5]=-.5*abs(R[2]);top[5*k+2:5*k+5]=.5*abs(R[2])
                bottom[5*k+4]-=.5*ref[2,2];top[5*k+4]-=.5*ref[2,2];rows.append((bottom,top,ref[2,3]))
            gap=rows[0][0]-rows[1][1];offset=rows[0][2]-rows[1][2];A.append(-gap);B.append(offset)
            sol=linprog(gap,A_ub=A,b_ub=B,A_eq=[rows[1][0]],b_eq=[level-rows[1][2]],bounds=bounds,method='highs')
            if not sol.success:continue
            states=[]
            for k,(ref,R,rot) in enumerate(frames):
                center=sol.x[k*5:k*5+2];dims=sol.x[k*5+2:k*5+5];T=ref.copy();T[:3,:3]=R
                T[:3,3]=ref[:3,3]+R[:,:2]@center-ref[:3,2]*dims[2]/2;cr=corners(T,dims)
                local=transform(np.linalg.inv(boxes[k]['pose']),cr)
                excess=float(np.max(abs(local)-np.asarray(boxes[k]['size_m'])/2))
                uv=np.asarray(boxes[k]['faces'][0]['support_points_uv_m'])@rot[:2,:2]
                visible_excess=float(np.max(abs(uv-center)-dims[:2]/2))
                assert excess<1e-9 and visible_excess<=.003+1e-9,'INVALID_WITNESS'
                states.append(dict(hypothesis=[ia,ib][k],center_uv_m=center.tolist(),size_m=dims.tolist(),pose=T.tolist(),
                    z_m=[float(cr[:,2].min()),float(cr[:,2].max())],observed_face_excess_m=visible_excess,old_envelope_excess_m=excess))
            depth=obb_overlap_depth(states[0]['pose'],np.asarray(states[0]['size_m'])/2,states[1]['pose'],np.asarray(states[1]['size_m'])/2)
            assert depth<1e-9 and abs(states[1]['z_m'][0]-level)<1e-9,'PENETRATING_WITNESS'
            count+=1
            item=dict(boxes=states,obb_overlap_m=depth,lower_support_top_m=level,scope='EXISTENCE_ONLY_NOT_UNIQUE_NOT_EXECUTABLE')
            if best is None or max(s['observed_face_excess_m'] for s in states)<max(s['observed_face_excess_m'] for s in best['boxes']):best=item
    return None if best is None else dict(best,feasible_stored_yaw_pairs=count)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('record',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    record=args.record.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    names=['observation.json','observation.npz','segmentation.npz','geometry.json','planning-failures.json']
    hashes={n:hashlib.sha256((record/n).read_bytes()).hexdigest() for n in names}
    obs=json.loads((record/'observation.json').read_text());arrays=np.load(record/'observation.npz',allow_pickle=False)
    saved=np.load(record/'segmentation.npz',allow_pickle=False);old=json.loads((record/'geometry.json').read_text())
    frame=SimpleNamespace(rgb=arrays['rgb'],depth=arrays['depth'],intrinsics=obs['intrinsics'],depth_scale_m=obs['depth_scale_m'],observation_id=obs['observation_id'])
    masks=[saved[f'mask_{i}'] for i in range(len(saved.files))]
    config_path=ROOT/'configs/workbench/single_carton.example.json';config=load(config_path)
    current,rejected=recover_boxes(frame,obs['T_world_camera'],old['detections'],masks,config['geometry'])
    if len(old['boxes'])!=2 or len(current)!=2:raise ValueError('TWO_RECOVERED_BOXES_REQUIRED:'+str(rejected))
    old['boxes'].sort(key=lambda b:-b['faces'][0]['center_m'][2])
    current_by_id={b['id']:b for b in current};current=[current_by_id[b['id']] for b in old['boxes']]
    witness=separated_witness(old['boxes']);points=metric_points(frame);W=np.asarray(obs['T_world_camera'])
    image=frame.rgb.copy();clouds=[];stats=[]
    for i,(before,after) in enumerate(zip(old['boxes'],current)):
        mask=masks[before['detection_index']];color=np.array([60,230,220] if i==0 else [245,180,60])
        image[mask]=(image[mask]*.6+color*.4).astype(np.uint8)
        f=before['faces'][0];pw=transform(W,points.reshape(-1,3)).reshape(points.shape)
        valid=mask&np.isfinite(points).all(-1)&(points[...,2]>.1)&(points[...,2]<3.)
        z=np.where(valid,points[...,2],0).astype('float32')
        valid &= (cv2.dilate(z,np.ones((3,3),'uint8'))-cv2.erode(z,np.ones((3,3),'uint8')))<.012
        on_plane=valid&(abs((pw-f['center_m'])@np.asarray(f['normal']))<=.002)
        cloud=pw[on_plane];clouds.append(cloud[::max(1,len(cloud)//7000)].tolist())
        def bounds(b):return [float(np.asarray(b['corners_m'])[:,2].min()),float(np.asarray(b['corners_m'])[:,2].max())]
        stats.append(dict(id=before['id'],observed_plane_z_m=[float(cloud[:,2].min()),float(cloud[:,2].max())],
            old_envelope_z_m=bounds(before),new_envelope_z_m=bounds(after),
            old_hypotheses=len(before['uncertainty']['hypotheses']),new_hypotheses=len(after['uncertainty']['hypotheses']),
            old_edges=before['quality']['credible_visible_edges'],new_edges=after['quality']['credible_visible_edges']))
    overlap=lambda bs:obb_overlap_depth(bs[0]['pose'],np.asarray(bs[0]['size_m'])/2,bs[1]['pose'],np.asarray(bs[1]['size_m'])/2)
    report=dict(scope='GEOMETRY_DIAGNOSIS_ONLY',source_record=str(record.relative_to(ROOT)),source_hashes=hashes,
        recovery_config=str(config_path.relative_to(ROOT)),recovery_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        recovery_geometry_config=config['geometry'],
        original_overlap_m=overlap(old['boxes']),updated_overlap_m=overlap(current),boxes=stats,witness=witness,
        geometry_status='UNRESOLVED_NOT_PHYSICAL_PENETRATION_PROOF',hardware_execution='NOT_RUN',permits_issued=0,
        inference_rerun=False,stack_relation_joint_recovery='NOT_IMPLEMENTED',source_unchanged=True)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (out/'geometry.json').write_text(json.dumps(dict(boxes=current,rejected=rejected),ensure_ascii=False))
    Image.fromarray(image).save(out/'same-frame-segmentation.png');buf=io.BytesIO();Image.fromarray(image).save(buf,format='PNG')
    data=dict(report=report,original=old['boxes'],updated=current,clouds=clouds,image='data:image/png;base64,'+base64.b64encode(buf.getvalue()).decode())
    vendor=ROOT/'tools/search_replay/vendor';js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    template=(ROOT/'tools/search_replay/envelope_diagnosis.html').read_text()
    (out/'index.html').write_text(template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':')).replace('</','<\\/')))
    for n,h in hashes.items():assert hashlib.sha256((record/n).read_bytes()).hexdigest()==h,'SOURCE_CHANGED'
    print(json.dumps(dict(output=str(out),old_overlap_mm=report['original_overlap_m']*1000,new_overlap_mm=report['updated_overlap_m']*1000,
                         witness=witness is not None,hardware_execution='NOT_RUN')))


if __name__=='__main__':main()
