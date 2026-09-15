"""Frozen single-box gate, stage diagnostics and actual final-JSON pictures."""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.metric_faces import extract_observation_labels, fit_metric_faces, MetricFitConfig
from unloading_perception.metric_support import check_metric_plane
from unloading_perception.final_geometry import project, coherent_cuboid, validate_final_record, SIGNS
from run_isaac_rgbd_geometry import _worker_artifacts, sha256


def load_extractor(vision):
    import subprocess
    from unloading_perception.upstream_v4 import UPSTREAM_V4_SHA
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=vision,text=True).strip()!=UPSTREAM_V4_SHA:
        raise ValueError('UPSTREAM_SHA_MISMATCH')
    spec=importlib.util.spec_from_file_location('pinned_recover',Path(vision)/'pipeline/geometry/recover_box_cuboids_3d.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.fit_planes


def evaluate(record, truth, camera):
    T=np.asarray(camera['T_W_C']); G=np.asarray(truth['T_W_object']); dims=np.asarray(truth['full_dimensions_m'])
    rows=[]
    for face in record['camera_facing_faces']:
        p=np.asarray(face.get('corners_3d_m',np.asarray(record.get('corners_3d',[]))[face['corner_indices']]))
        world=p@T[:3,:3].T+T[:3,3]; local=(world-G[:3,3])@G[:3,:3]
        normal=np.asarray(face.get('plane_normal',np.cross(p[1]-p[0],p[3]-p[0]))); normal/=np.linalg.norm(normal)
        nl=normal@T[:3,:3].T@G[:3,:3]; axis=int(np.argmax(np.abs(nl)))
        sign=1 if np.mean(local[:,axis])>0 else -1
        overflow=np.maximum(np.abs(local)-dims/2,0); overflow[:,axis]=0
        rows.append({'label':face.get('support_label'),'plane_distance_m':float(np.mean(np.abs(local[:,axis]-sign*dims[axis]/2))),
                     'normal_error_degrees':float(np.degrees(np.arccos(np.clip(abs(nl[axis]),-1,1)))),
                     'boundary_outside_m':float(np.linalg.norm(overflow,axis=1).max()),
                     'gt_axis':axis,'gt_sign':sign,'depth':face.get('final_support')})
    result={'faces':rows,'complete_accepted':bool(record.get('accepted'))}
    if record.get('accepted') and len(record.get('corners_3d',[]))==8:
        center,axes,pdims,_=coherent_cuboid(record['corners_3d'])
        pa=axes@T[:3,:3].T@G[:3,:3]; mapping=np.argmax(np.abs(pa),axis=1)
        result['center_error_m']=float(np.linalg.norm(center@T[:3,:3].T+T[:3,3]-G[:3,3]))
        result['dimension_error_m']=np.abs(pdims-dims[mapping]).tolist()
        result['orientation_error_degrees']=float(np.degrees(np.arccos(np.clip(np.min(np.max(np.abs(pa),axis=1)),-1,1))))
        gt=G[:3,3]+(SIGNS*dims/2)@G[:3,:3].T
        predicted=np.asarray(record['corners_3d'])@T[:3,:3].T+T[:3,3]
        result['corner_set_max_error_m']=float(np.linalg.norm(predicted[:,None]-gt[None,:],axis=2).min(axis=1).max())
    return result


def stage_audit(raw, worker, fallback, depth, mask, K, labels, seeds):
    stages=[]; previous=None
    for name,record in [('C_INITIAL_PLANES',raw),('D_UNANCHORED', {**raw,'corners_3d':raw.get('unanchored_corners_3d',[]),'camera_facing_faces':raw.get('unanchored_camera_facing_faces',[])}),
                        ('F_PNP_SCALE',worker),('G_FALLBACK',fallback)]:
        item={'stage':name,'planes':[],'corners_3d':record.get('corners_3d'),
              'scale_changes':record.get('multiplane_refinement_scales',record.get('shape_certificate_scales')),
              'reprojection_rmse':record.get('multiplane_pnp_rmse_px'),
              'cross_entity':'EVALUATED_SEPARATELY_WITH_GT_ONLY'}
        if name=='C_INITIAL_PLANES':
            planes=[(np.asarray(eq[:3]),eq[3]) for eq in raw.get('plane_equations',[])]
        else:
            planes=[]
            for face in record.get('camera_facing_faces',[]):
                p=np.asarray(record['corners_3d'])[face['corner_indices']]
                n=np.cross(p[1]-p[0],p[3]-p[0]); n/=np.linalg.norm(n)
                planes.append((n,-float(n@p.mean(axis=0))))
        for n,d in planes:
            label=int(np.argmax([abs(n@np.asarray(s['normal'])) for s in seeds]))
            audit=check_metric_plane(n,d,depth,mask,labels==label,K)
            item['planes'].append({'normal':n.tolist(),'offset_m':d,'support_label':label,'check':audit})
        try:
            center,axes,dims,_=coherent_cuboid(record['corners_3d'])
            item.update(center=center.tolist(),axes=axes.tolist(),dimensions=dims.tolist())
            if previous is not None:
                item['center_change_m']=float(np.linalg.norm(center-previous[0]))
                item['dimension_change_m']=(dims-previous[2]).tolist()
                item['rotation_change_degrees']=float(np.degrees(np.arccos(np.clip((np.trace(axes@previous[1].T)-1)/2,-1,1))))
            previous=(center,axes,dims)
        except ValueError as exc:
            item['cuboid_consistency']=str(exc)
        stages.append(item)
    return stages


def picture(directory, source, mask, depth, K, old, new, truth, camera, labels):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    rgb=np.asarray(Image.open(source)); ys,xs=np.nonzero(mask)
    extent=(max(0,xs.min()-25),min(mask.shape[1],xs.max()+26),min(mask.shape[0],ys.max()+26),max(0,ys.min()-25))
    fig,axs=plt.subplots(2,3,figsize=(15,10),layout='constrained')
    T=np.asarray(camera['T_W_C']); G=np.asarray(truth['T_W_object']); dims=np.asarray(truth['full_dimensions_m'])
    gt=((G[:3,3]+(SIGNS*dims/2)@G[:3,:3].T)-T[:3,3])@T[:3,:3]
    for ax,title in zip(axs.flat,['RGB','SAM + frozen depth face labels','Raw optical-Z depth','Legacy V4 (rejected)','New metric: accepted patches','Independent GT (evaluation only)']):
        ax.imshow(rgb); ax.set(xlim=extent[:2],ylim=extent[2:],title=title); ax.set_aspect('equal')
    axs[0,1].imshow(np.ma.masked_where(~mask,labels),alpha=.65,cmap='tab10')
    axs[0,2].imshow(np.ma.masked_where(~mask,depth),alpha=.9,cmap='viridis')
    for ax,record,color in [(axs[1,0],old,'red'),(axs[1,1],new,'lime')]:
        for f in record.get('camera_facing_faces',[]):
            points=np.asarray(f['corners_3d_m']) if 'corners_3d_m' in f else np.asarray(record['corners_3d'])[f['corner_indices']]
            polygon=project(points,K); ax.plot(*np.vstack((polygon,polygon[0])).T,color=color,lw=1.5)
        if record.get('accepted') and len(record.get('corners_3d',[]))==8:
            corners=project(record['corners_3d'],K)
            for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
                ax.plot(*corners[[a,b]].T,color='cyan',ls='--',lw=.8)
    corners=project(gt,K)
    for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
        axs[1,2].plot(*corners[[a,b]].T,color='cyan',lw=1.)
    fig.suptitle('GEOMETRY_ALGORITHM_ONLY | frozen calibration RGB-D | '+camera['module_id'])
    fig.savefig(directory/'single_box_comparison.png',dpi=170); plt.close(fig)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--capture',type=Path,required=True); p.add_argument('--legacy',type=Path,required=True)
    p.add_argument('--vision',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    extractor=load_extractor(args.vision); scene=args.capture/'RGBD_CALIBRATION_BOX'
    summary={}
    for folder in (scene,scene/'modules/module_1_lower'):
        metadata=json.loads((folder/'capture_metadata.json').read_text())
        manifest=json.loads((args.capture/'scene_bundle/RGBD_CALIBRATION_BOX.manifest.json').read_text())
        camera=next(c for c in manifest['cameras'] if c['frame_id']==metadata['rgb_frame_id'])
        K=np.asarray(camera['K']).reshape(3,3)
        directory=args.output/camera['module_id']; directory.mkdir()
        artifacts=_worker_artifacts(folder)
        a=np.load(artifacts['cargo_masks.npz']['path']); mask=a['masks'][0].astype(bool)
        depth=np.load(folder/'metric_depth_m.npy'); raw=json.loads((folder/'rgbd_cuboids_baseline_raw.json').read_text())['instances'][0]
        legacy=args.legacy/'RGBD_CALIBRATION_BOX'/camera['module_id']
        worker=json.loads((legacy/'worker.json').read_text())['instances'][0]
        fallback=json.loads((legacy/'registered_fallback.json').read_text())['instances'][0]
        gt=json.loads((folder/'gt_annotations.json').read_text()); truth=next(g for g in gt['objects'] if g['mask_pixel_count']>0)
        oracle=np.load(folder/'gt_instance_masks.npz')[truth['mask_key']]
        matrix={}
        for name,m in [('ORACLE_MASK_DIAGNOSTIC',oracle),('SAM',mask)]:
            labels,seeds,audit=extract_observation_labels(depth,m,K,extractor)
            output=fit_metric_faces(depth,m,K,labels,seeds,mask_id=int(a['mask_ids'][0]),
                                    config=MetricFitConfig(positive_infinity_is_no_hit=True))
            output['observation_segmentation']=audit
            (directory/(name+'.json')).write_text(json.dumps(output,indent=2))
            np.savez_compressed(directory/(name+'_support.npz'),labels=labels)
            matrix[name]=evaluate(output,truth,camera)
            if name=='SAM':
                (directory/'stages.json').write_text(json.dumps(stage_audit(raw,worker,fallback,depth,m,K,labels,seeds),indent=2))
                picture(directory,folder/'sensor_rgb.png',m,depth,K,worker,output,truth,camera,labels)
        matrix['legacy_final_faces']=len(json.loads((legacy/'validated_geometry.json').read_text())['instances'][0]['camera_facing_faces'])
        matrix['inputs']={str(f):sha256(f) for f in [folder/'sensor_rgb.png',folder/'metric_depth_m.npy',folder/'capture_metadata.json',Path(artifacts['cargo_masks.npz']['path'])]}
        summary[camera['module_id']]=matrix
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps({camera['module_id']:matrix}),flush=True)


if __name__=='__main__': main()
