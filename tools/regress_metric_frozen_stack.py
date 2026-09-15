"""One unchanged frozen full-stack comparison after the small-scene gates."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from run_isaac_rgbd_geometry import _worker_artifacts,_observation,CaptureMetadata,IsaacSceneManifest,dumps
from metric_depth_runner import run_metric_depth
from diagnose_metric_calibration import evaluate
from unloading_perception.fusion import ModuleFaceBatch,fuse_module_face_batches
from unloading_perception.algorithm_handoff import fused_algorithm_observation


def main():
    p=argparse.ArgumentParser(); p.add_argument('--capture',type=Path,required=True); p.add_argument('--legacy',type=Path,required=True)
    p.add_argument('--vision',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gate-report',type=Path,required=True)
    args=p.parse_args()
    if not json.loads(args.gate_report.read_text()).get('allow_single_frozen_stack_regression'):
        raise ValueError('SMALL_SCENE_GEOMETRY_GATES_NOT_PASSED')
    args.output.mkdir(parents=True,exist_ok=False)
    scene='FULL_STACK_NOMINAL'; root=args.capture/scene
    manifest=IsaacSceneManifest.from_dict(json.loads((args.capture/'scene_bundle'/f'{scene}.manifest.json').read_text()))
    observations=[]; batches=[]; rows=[]
    for folder in (root,root/'modules/module_1_lower'):
        metadata=CaptureMetadata.from_dict(json.loads((folder/'capture_metadata.json').read_text()))
        camera=next(c for c in manifest.cameras if c['frame_id']==metadata.rgb_frame_id)
        output=args.output/scene/camera['module_id']; output.mkdir(parents=True)
        artifacts=_worker_artifacts(folder); masks=Path(artifacts['cargo_masks.npz']['path'])
        proposals=json.loads((folder/'oracle_proposals.json').read_text()); proposals['_scene_dir']=str(folder)
        raw=json.loads((folder/'rgbd_cuboids_baseline_raw.json').read_text())
        started=perf_counter()
        final=run_metric_depth(raw=raw,source=folder/'sensor_rgb.png',masks=masks,pointmap=folder/'registered_metric_pointmap.npz',
                    depth=np.load(folder/'metric_depth_m.npy'),K=camera['K'],metadata=metadata,output=output,vision_root=args.vision)
        obs,_,_,faces=_observation(scene,manifest,final,masks,proposals,perf_counter()-started)
        (output/'observation.json').write_text(dumps(obs)); observations.append(obs)
        batches.append(ModuleFaceBatch(camera['module_id'],metadata.sensor_epoch,metadata.frame_sequence,metadata.capture_center_time,tuple(f for f in faces if f.faces)))
        old=json.loads((args.legacy/scene/camera['module_id']/'validated_geometry.json').read_text())
        old_by={r['mask_id']:r for r in old['instances']}; cargo_by={c.raw_result['instance_lineage']['mask_id']:c for c in obs.cargo}
        truth={o['simulation_object_id']:o for o in json.loads((folder/'gt_annotations.json').read_text())['objects']}
        for r in final['instances']:
            c=cargo_by[r['mask_id']]; audit=c.raw_result['instance_lineage']; entities=audit['evaluation_entity_ids']
            row={'module':camera['module_id'],'mask_id':r['mask_id'],'source_instance_id':c.source_instance_id,
                 'entities':entities,'lineage':audit,'old_face_count':len(old_by[r['mask_id']]['camera_facing_faces']),
                 'new_face_count':len(r['camera_facing_faces']),'new_complete_accepted':r['accepted'],
                 'new_final_checks':r['final_face_validation']}
            if len(entities)==1:
                row['old']=evaluate(old_by[r['mask_id']],truth[entities[0]],camera)
                row['new']=evaluate(r,truth[entities[0]],camera)
                a={(f['gt_axis'],f['gt_sign']):f for f in row['old']['faces']}
                b={(f['gt_axis'],f['gt_sign']):f for f in row['new']['faces']}
                row['paired_common_faces']=[{'gt_face':key,'old':a[key],'new':b[key]} for key in sorted(a.keys() & b.keys())]
            else: row['identity_status']='AMBIGUOUS_MERGED_ENTITIES'
            rows.append(row)
        print(json.dumps({'module':camera['module_id'],'instances':len(final['instances']),'new_faces':sum(len(r['camera_facing_faces']) for r in final['instances'])}),flush=True)
    fusion=fuse_module_face_batches(batches,expected_modules=[c['module_id'] for c in manifest.cameras])
    observation=fused_algorithm_observation(observations,fusion)
    scene_out=args.output/scene
    (scene_out/'fusion.json').write_text(json.dumps(asdict(fusion),indent=2))
    (scene_out/'fused_algorithm_observation.json').write_text(dumps(observation))
    summary={'scope':'UNCHANGED_FULL_STACK_FROZEN_INPUT_REGRESSION','mechanical_occlusion':'DEFERRED_MECHANICAL_OCCLUSION',
             'sam_instance_denominator':len(rows),'old_faces':sum(r['old_face_count'] for r in rows),'new_faces':sum(r['new_face_count'] for r in rows),
             'new_complete_accepted':sum(r['new_complete_accepted'] for r in rows),'paired_instances':rows,
             'mechanical_layout_modified':False,'fusion_thresholds_modified':False,'fusion_container_count':len(fusion.objects)}
    (args.output/'paired_stack_report.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='paired_instances'}),flush=True)


if __name__=='__main__': main()
