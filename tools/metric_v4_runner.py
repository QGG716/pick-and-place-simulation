"""Thin pinned V4 invocation with distinct metric/fallback inputs and gates."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np

from unloading_perception.upstream_v4 import (
    UPSTREAM_V4_SHA, build_upstream_v4_command, prepare_metric_multiplane_input,
    prepare_registered_depth_baseline, finalize_registered_depth_result,
)
from unloading_perception.final_geometry import validate_final_record


def run_metric_v4(*, raw, source, masks, pointmap, depth, K, metadata, output, vision_root, python, timeout=1200):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=vision_root,text=True).strip()
    if actual!=UPSTREAM_V4_SHA: raise ValueError('UPSTREAM_SHA_MISMATCH')
    with np.load(pointmap,allow_pickle=False) as archive:
        # MetricPointMap.write_npz adds source/capture metadata separately from
        # the upstream's normalized intrinsics. Do not infer metric from labels.
        if not {'points','intrinsics'}.issubset(archive.files): raise ValueError('INVALID_POINTMAP')
        provenance=json.loads(str(archive['metadata_json']))
        if provenance.get('source') not in {'ISAAC_IDEAL_REGISTERED_DEPTH','REGISTERED_RGBD'}:
            raise ValueError('UNVERIFIED_METRIC_SOURCE')
        if provenance['capture_id']!=metadata.capture_id or provenance['camera_frame']!=metadata.rgb_frame_id or provenance['calibration_identity']!=metadata.calibration_identity or not np.allclose(archive['K'],np.asarray(K).reshape(3,3),atol=1e-9,rtol=0):
            raise ValueError('POINTMAP_CAPTURE_CALIBRATION_MISMATCH')
    primary=prepare_metric_multiplane_input(raw,verified_metric_source=True)
    fallback=prepare_registered_depth_baseline(raw,verified_metric_source=True)
    with np.load(masks,allow_pickle=False) as archive:
        by_mask={int(i):m.astype(bool) for i,m in zip(archive['mask_ids'],archive['masks'])}
    audits={}
    for r in primary['instances']:
        mask=by_mask[r['mask_id']]; candidates=0; passed=0
        for face in r.get('camera_facing_faces',[]):
            if face.get('evidence')!='monocular_depth_plane': continue
            candidates+=1; pixels=np.zeros(mask.shape,np.uint8)
            cv2.fillPoly(pixels,[np.asarray(face['corners_2d']).round().astype('int32')],1)
            intersection=int((pixels.astype(bool)&mask).sum())
            passed+=int(intersection/max(1,int(pixels.sum()))>=.72 and intersection/max(1,int(mask.sum()))>=.04)
        audits[r['mask_id']]={'raw_depth_plane_count':r.get('depth_plane_candidate_count',len(r.get('plane_equations',[]))),'visible_candidates':candidates,'face_checks_passed':passed,'joint_entered':bool(r.get('accepted') and passed>=2),'joint_succeeded':False,'joint_accepted':False}
    for name,payload in [('metric_multiplane_input.json',primary),('registered_fallback.json',fallback)]:
        (output/name).write_text(json.dumps(payload))
    command=build_upstream_v4_command(python,vision_root,source=source,masks=masks,pointmap=pointmap,baseline=output/'metric_multiplane_input.json',fallback=output/'registered_fallback.json',json_output=output/'worker.json',image_output=output/'worker.png')
    result=subprocess.run(command,cwd=vision_root,capture_output=True,text=True,timeout=timeout)
    (output/'command.json').write_text(json.dumps({'argv':command,'upstream_sha':actual,'returncode':result.returncode},indent=2))
    (output/'worker.log').write_text(result.stdout+'\n'+result.stderr)
    result.check_returncode()
    worker=finalize_registered_depth_result(json.loads((output/'worker.json').read_text()))
    fallback=finalize_registered_depth_result(fallback)
    fallback_by_id={r['mask_id']:r for r in fallback['instances']}
    records=[]
    for r in worker['instances']:
        audit=audits[r['mask_id']]; mask=by_mask[r['mask_id']]
        success=r.get('multiplane_refinement_status')=='joint_sam_silhouette+depth_faces+fixed_intrinsics_pnp'
        audit['joint_succeeded']=success
        validated=validate_final_record(r,depth,mask,K)
        if success and (len(validated['camera_facing_faces'])!=len(r['camera_facing_faces']) or validated['complete_cuboid_consistency']['status']!='PASS'):
            audit['rejected_joint_final_checks']=validated['final_face_validation']
            validated=validate_final_record(fallback_by_id[r['mask_id']],depth,mask,K)
            audit['fallback_reason']='JOINT_RESULT_FAILED_INDEPENDENT_METRIC_GATE'
        elif success: audit['joint_accepted']=True; audit['fallback_reason']=None
        else: audit['fallback_reason']='UPSTREAM_JOINT_FIT_NOT_ACCEPTED' if audit['joint_entered'] else 'FEWER_THAN_TWO_RELIABLE_DEPTH_FACES'
        audit['final_published_face_count']=len(validated['camera_facing_faces'])
        validated['v4_path_audit']=audit
        records.append(validated)
    final={**worker,'instances':records,'v4_path_audit':audits}
    (output/'validated_geometry.json').write_text(json.dumps(final,indent=2))
    return final


def main():
    import argparse
    from dataclasses import asdict
    from run_isaac_rgbd_geometry import _worker_artifacts,_observation,CaptureMetadata,IsaacSceneManifest,dumps
    from unloading_perception.fusion import ModuleFaceBatch,fuse_module_face_batches
    parser=argparse.ArgumentParser()
    parser.add_argument('--capture-directory',type=Path,required=True)
    parser.add_argument('--output-directory',type=Path,required=True)
    parser.add_argument('--vision-root',type=Path,required=True)
    parser.add_argument('--python',type=Path,required=True)
    args=parser.parse_args(); root=args.capture_directory
    args.output_directory.mkdir(parents=True,exist_ok=False)
    index=json.loads((root/'scene_bundle/index.json').read_text())
    summaries={}
    for scene in ('RGBD_CALIBRATION_BOX','PARTIAL_OCCLUSION','FULL_STACK_NOMINAL'):
        entry=next(v for v in index['scenes'] if v['scene']==scene)
        manifest=IsaacSceneManifest.from_dict(json.loads((root/'scene_bundle'/entry['path']).read_text()))
        batches=[]; observations=[]
        for directory in (root/scene,root/scene/'modules/module_1_lower'):
            artifacts=_worker_artifacts(directory)
            metadata=CaptureMetadata.from_dict(json.loads((directory/'capture_metadata.json').read_text()))
            camera=next(c for c in manifest.cameras if c['frame_id']==metadata.rgb_frame_id)
            output=args.output_directory/scene/camera['module_id']
            masks=Path(artifacts['cargo_masks.npz']['path'])
            proposals=json.loads((directory/'oracle_proposals.json').read_text()); proposals['_scene_dir']=str(directory)
            # Identity gate precedes V4 computation.
            from unloading_perception.lineage import load_instance_lineage
            raw=json.loads((directory/'rgbd_cuboids_baseline_raw.json').read_text())
            load_instance_lineage(masks,Path(artifacts['cargo_instances.json']['path']),proposals,raw,sensor_epoch=metadata.sensor_epoch,module_id=camera['module_id'],capture_id=metadata.capture_id,source_path=directory/'sensor_rgb.png',proposal_path=directory/'oracle_proposals.json')
            final=run_metric_v4(raw=raw,source=directory/'sensor_rgb.png',masks=masks,pointmap=directory/'registered_metric_pointmap.npz',depth=np.load(directory/'metric_depth_m.npy'),K=camera['K'],metadata=metadata,output=output,vision_root=args.vision_root,python=args.python)
            old=json.loads((directory/'mode_b_rgbd_observation.json').read_text())['payload']
            observation,_,_,faces=_observation(scene,manifest,final,masks,proposals,old['processed_time']-old['capture_time'])
            (output/'observation.json').write_text(dumps(observation)); observations.append(observation)
            batches.append(ModuleFaceBatch(camera['module_id'],metadata.sensor_epoch,metadata.frame_sequence,metadata.capture_center_time,tuple(f for f in faces if f.faces)))
            summaries[f'{scene}/{camera["module_id"]}']=final['v4_path_audit']
        fusion=fuse_module_face_batches(batches,expected_modules=[c['module_id'] for c in manifest.cameras])
        from unloading_perception.algorithm_handoff import fused_algorithm_observation
        (args.output_directory/scene/'fused_algorithm_observation.json').write_text(dumps(fused_algorithm_observation(observations,fusion)))
        (args.output_directory/scene/'fusion.json').write_text(json.dumps(asdict(fusion),indent=2))
        # Post-hoc GT lineage evaluation never enters production association.
        entities={c.source_instance_id:c.raw_result['instance_lineage']['evaluation_entity_ids'] for obs in observations for c in obs.cargo}
        correspondence=[]
        for obj in fusion.objects:
            sets=[set(entities[identity]) for _,identity in obj.source_members]
            union=set.union(*sets)
            correspondence.append({'fusion_id':obj.fusion_id,'source_members':obj.source_members,'gt_entities':sorted(union),'association_status':obj.association_status,'classification':'AMBIGUOUS_SOURCE_MERGE' if any(len(s)>1 for s in sets) else 'WRONG_MERGE' if len(union)>1 else 'CORRECT_ASSOCIATION' if len(sets)>1 else 'SINGLE_OBSERVATION'})
        (args.output_directory/scene/'association_evaluation.json').write_text(json.dumps(correspondence,indent=2))
    (args.output_directory/'v4_path_audit.json').write_text(json.dumps(summaries,indent=2))
    print(json.dumps({k:{'instances':len(v),'entered':sum(r['joint_entered'] for r in v.values()),'succeeded':sum(r['joint_succeeded'] for r in v.values()),'accepted':sum(r['joint_accepted'] for r in v.values()),'faces':sum(r['final_published_face_count'] for r in v.values())} for k,v in summaries.items()},indent=2))


if __name__=='__main__':
    import sys
    root=Path(__file__).resolve().parents[1]
    sys.path[:0]=[str(root/'src'),str(root/'packages/unloading_contracts/src')]
    main()
