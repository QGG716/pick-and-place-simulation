"""Finite oracle/SAM/legacy/metric comparison on two isolated Isaac scenes."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from diagnose_metric_calibration import load_extractor, evaluate
from unloading_perception.metric_faces import extract_observation_labels,fit_metric_faces,MetricFitConfig
from run_isaac_rgbd_geometry import _worker_artifacts,_run_secondary_module,IsaacSceneManifest,sha256


def oracle_proposals(folder,camera):
    binding=json.loads((folder/'capture_metadata.json').read_text())
    annotations=json.loads((folder/'gt_annotations.json').read_text())
    payload={'schema_version':'isaac_oracle_proposals_v1','coordinate_space':'source_image',
             'source_size':camera['resolution'],'source':'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL',
             'raw_image_automatic':False,'simulation_epoch':binding['sensor_epoch'],
             'frame_sequence':binding['frame_sequence'],'rgb_sha256':sha256(folder/'sensor_rgb.png'),
             'instances':[{'id':i,'instance_id':i,'simulation_object_id':o['simulation_object_id'],
                            'oracle_proposal_source_id':o['simulation_object_id'],'label':'box','bbox':o['bbox_xyxy'],
                            'visible':True,'occluded':o['occluded'],'proposal_source':'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL'}
                           for i,o in enumerate((g for g in annotations['objects'] if g['visible']),1)]}
    (folder/'oracle_proposals.json').write_text(json.dumps(payload,indent=2))
    return binding,annotations,payload


def infer(runtime,folder,binding,camera,request_id):
    source,proposal=folder/'sensor_rgb.png',folder/'oracle_proposals.json'
    request={'schema_version':'1.1.0','op':'infer','request_id':request_id,'worker_epoch':'metric-small-scenes',
             'proposal_reference':{'uri':proposal.resolve().as_uri(),'sha256':sha256(proposal)},
             'frame':{'source':'isaac-sim-6.0.1','stream':'perception_validation','epoch':binding['sensor_epoch'],
                       'sequence':binding['frame_sequence'],'capture_time':binding['capture_center_time'],
                       'receive_time':binding['capture_center_time'],'clock_domain':binding['clock_domain'],
                       'frame_id':binding['rgb_frame_id'],'width':camera['resolution'][0],'height':camera['resolution'][1],
                       'encoding':'rgb8','rgb_uri':source.resolve().as_uri(),'rgb_sha256':sha256(source)}}
    response=runtime.infer(request)
    if response['status']!='COMPLETE': raise RuntimeError(str(response))
    (folder/'mode_b1_worker_response.json').write_text(json.dumps(response))
    assert runtime.moge_model is None and not any(k=='moge' or k.startswith('moge.') for k in sys.modules)


def draw(folder,output,old,new,masks,truth,camera):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    from unloading_perception.final_geometry import project,SIGNS
    rgb=np.asarray(Image.open(folder/'sensor_rgb.png')); union=np.any(masks,axis=0)
    yy,xx=np.nonzero(union); bounds=(max(0,xx.min()-30),min(rgb.shape[1],xx.max()+31),min(rgb.shape[0],yy.max()+31),max(0,yy.min()-30))
    fig,axs=plt.subplots(1,4,figsize=(18,7),layout='constrained')
    for ax,title in zip(axs,['SAM instances','Legacy final accepted faces','Depth metric final faces','GT only for evaluation']):
        ax.imshow(rgb); ax.set(xlim=bounds[:2],ylim=bounds[2:],title=title)
    for i,mask in enumerate(masks): axs[0].contour(mask,levels=[.5],colors=[f'C{i}'],linewidths=1)
    for ax,payload in [(axs[1],old),(axs[2],new)]:
        for i,r in enumerate(payload['instances']):
            for face in r['camera_facing_faces']:
                points=face['corners_3d_m'] if 'corners_3d_m' in face else np.asarray(r['corners_3d'])[face['corner_indices']]
                q=project(points,camera['K']); ax.plot(*np.vstack([q,q[0]]).T,color=f'C{i}',lw=1.6)
                ax.text(*q.mean(axis=0),f"SAM {r['mask_id']}",fontsize=8,color=f'C{i}')
    T=np.asarray(camera['T_W_C'])
    for g in truth:
        G=np.asarray(g['T_W_object']); p=(SIGNS*np.asarray(g['full_dimensions_m'])/2)@G[:3,:3].T+G[:3,3]
        q=project((p-T[:3,3])@T[:3,:3],camera['K'])
        for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]: axs[3].plot(*q[[a,b]].T,color='cyan',lw=.8)
    fig.suptitle('GEOMETRY_ALGORITHM_ONLY | '+output.parent.name+' | '+camera['module_id'])
    fig.savefig(output/'two_box_comparison.png',dpi=150); plt.close(fig)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--capture',type=Path,required=True); p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--vision',type=Path,required=True); p.add_argument('--models',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reuse-sam',action='store_true')
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    from vision_resident_worker import ResidentRuntime,parser as worker_parser
    from unloading_perception.lineage import load_instance_lineage
    from unloading_perception.rgbd import CaptureMetadata
    from metric_v4_runner import run_metric_v4
    import yaml
    extractor=load_extractor(args.vision); config=MetricFitConfig(positive_infinity_is_no_hit=True)
    models=json.loads(args.models.read_text()); runtime=None
    if not args.reuse_sam:
        worker=worker_parser().parse_args(['--upstream-root',str(args.vision),'--output-root',str(args.capture/'sam-runs'),'--input-root',str(args.capture),'--sam-model',models['sam']['snapshot_path']])
        runtime=ResidentRuntime(worker)
    summary=[]
    for entry in json.loads((args.bundle/'index.json').read_text())['scenes']:
        manifest=IsaacSceneManifest.from_dict(json.loads((args.bundle/entry['path']).read_text()))
        for camera in manifest.cameras:
            folder=args.capture/entry['scene']/'modules'/camera['module_id']
            output=args.output/entry['scene']/camera['module_id']; output.mkdir(parents=True)
            binding,annotations,proposals=oracle_proposals(folder,camera)
            truth={g['simulation_object_id']:g for g in annotations['objects'] if g['visible']}
            depth=np.load(folder/'metric_depth_m.npy'); oracle=np.load(folder/'gt_instance_masks.npz')
            row={'scene':entry['scene'],'module':camera['module_id'],'gt_visible_denominator':len(truth),'ORACLE_MASK_DIAGNOSTIC':[],'SAM_PAIRED':[]}
            # Oracle geometry first, before real SAM is allowed to distract diagnosis.
            for i,g in enumerate(truth.values(),1):
                mask=oracle[g['mask_key']]; labels,seeds,audit=extract_observation_labels(depth,mask,camera['K'],extractor,config)
                record=fit_metric_faces(depth,mask,camera['K'],labels,seeds,mask_id=i,config=config)
                metrics=evaluate(record,g,camera); row['ORACLE_MASK_DIAGNOSTIC'].append({'entity':g['simulation_object_id'],**metrics})
                (output/f'oracle_{i}.json').write_text(json.dumps(record,indent=2))
                if not metrics['faces'] or any(f['plane_distance_m']>.005 or f['normal_error_degrees']>1 or f['boundary_outside_m']>.01 for f in metrics['faces']):
                    (output/'oracle_gate_failure.json').write_text(json.dumps(row,indent=2)); raise RuntimeError('ORACLE_GEOMETRY_GATE_FAILED')
            if runtime and not (folder/'mode_b1_worker_response.json').exists():
                infer(runtime,folder,binding,camera,entry['scene']+'-'+camera['module_id'])
            artifacts=_worker_artifacts(folder); masks=Path(artifacts['cargo_masks.npz']['path'])
            result=_run_secondary_module(scene=entry['scene'],module_dir=folder,manifest=manifest,artifacts=artifacts,
                    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text()),vision_root=args.vision,
                    upstream_python=Path(sys.executable),timeout=1200)
            new=json.loads((folder/'rgbd_cuboids.json').read_text())
            old=run_metric_v4(raw=json.loads((folder/'rgbd_cuboids_baseline_raw.json').read_text()),source=folder/'sensor_rgb.png',masks=masks,
                    pointmap=folder/'registered_metric_pointmap.npz',depth=depth,K=camera['K'],metadata=CaptureMetadata.from_dict(binding),
                    output=output/'legacy',vision_root=args.vision,python=sys.executable)
            lineage=load_instance_lineage(masks,masks.parent/'cargo_instances.json',proposals,new,sensor_epoch=binding['sensor_epoch'],
                    module_id=camera['module_id'],capture_id=binding['capture_id'],source_path=folder/'sensor_rgb.png',proposal_path=folder/'oracle_proposals.json')
            old_by={r['mask_id']:r for r in old['instances']}
            for r in new['instances']:
                item=lineage[r['mask_id']]; entities=item.audit['evaluation_entity_ids']
                paired={'mask_id':r['mask_id'],'entities':entities,'source_lineage':item.audit,
                        'new_face_count':len(r['camera_facing_faces']),'old_face_count':len(old_by[r['mask_id']]['camera_facing_faces']),
                        'new_complete_accepted':r['accepted']}
                if len(entities)==1:
                    paired['new']=evaluate(r,truth[entities[0]],camera)
                    paired['old']=evaluate(old_by[r['mask_id']],truth[entities[0]],camera)
                else: paired['identity_status']='AMBIGUOUS_MERGED_ENTITIES'
                row['SAM_PAIRED'].append(paired)
            (output/'metric_final.json').write_text(json.dumps(new,indent=2))
            draw(folder,output,old,new,np.load(masks)['masks'],list(truth.values()),camera)
            summary.append(row); (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
            print(json.dumps({'scene':entry['scene'],'module':camera['module_id'],'oracle_faces':[len(r['faces']) for r in row['ORACLE_MASK_DIAGNOSTIC']],
                              'sam':[(r['mask_id'],r['old_face_count'],r['new_face_count'],r['new_complete_accepted']) for r in row['SAM_PAIRED']]}),flush=True)


if __name__=='__main__': main()
