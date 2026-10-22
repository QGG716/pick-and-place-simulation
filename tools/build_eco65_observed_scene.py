"""Bind a fresh validated RGB-D estimate to an unchanged physical layout."""
import argparse
from copy import deepcopy
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.camera_installation import installation_fingerprint
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.perception import verify_record, require_tcp_calibration
from unloading_sim.moveit2_backend import digest


def build(a):
    source=verify_snapshot(load_json(a.snapshot)); obs=verify_record(load_json(a.observations))
    cal=verify_record(load_json(a.calibration)); detection=verify_record(load_json(a.detection))
    require_tcp_calibration(cal)
    scene=deepcopy(source['scene'])
    if (cal['status']!='PASS' or detection['status']!='PASS' or
        obs['snapshot_fingerprint']!=source['fingerprint'] or cal['snapshot_fingerprint']!=source['fingerprint'] or
        cal['installation_fingerprint']!=installation_fingerprint(scene['permanent_camera']) or
        cal['tcp_observations']['identifiers']['camera']!=digest(scene['permanent_camera']) or
        cal['observation_fingerprint']!=obs['fingerprint'] or detection['observation_fingerprint']!=obs['fingerprint'] or
        detection['calibration_fingerprint']!=cal['fingerprint']):
        raise ValueError('CURRENT_LAYOUT_CALIBRATION_VISION_BINDING_FAILED')
    out=a.output.resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True,exist_ok=False)
    columns=sorted({b['column'] for b in scene['boxes']},key=lambda c:np.mean([b['pose'][1] for b in scene['boxes'] if b['column']==c]))
    if len(columns)!=len(detection['detections']):raise ValueError('OBSERVED_COLUMN_INVENTORY_MISMATCH')
    if columns[detection['selected_column_rank']] != scene['box']['column']:
        raise ValueError('OBSERVED_TARGET_SELECTION_MISMATCH')
    assignments=[]
    for column,d in zip(columns,detection['detections']):
        group=[b for b in scene['boxes'] if b['column']==column]; top=max(b['layer'] for b in group)
        for b in group:
            if b['size_m']!=d['size_m']:raise ValueError('OBSERVED_SIZE_PRIOR_MISMATCH')
            b['pose']=(np.asarray(d['pose_D_m'])-[0,0,(top-b['layer'])*d['size_m'][2]]).tolist()
            b['evidence']='RGBD_TOP_FACE_ESTIMATE' if b['layer']==top else 'OCCLUDED_STACK_SIZE_ALIGNMENT_PRIOR_COMPLETION'
            assignments.append(dict(id=b['id'],pose_D_m=b['pose'],evidence=b['evidence']))
    scene['box']=deepcopy(next(b for b in scene['boxes'] if b['id']==scene['target_box_id']))
    image_root=Path(a.observations).resolve().parent
    evidence_images=[]
    for row in [*cal['tcp_observations']['photos'],obs['carton_image']]:
        path=image_root/row['image']
        if file_hash(path)!=row['sha256']:raise ValueError('PERCEPTION_BOUND_IMAGE_CHANGED')
        evidence_images.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=row['sha256'],
            label=row.get('sample_id','cartons_RGBD'),q_rad=row['q_rad']))
    binding=dict(schema='eco65_fresh_perception_plan_binding_v1',source='SYNTHETIC_RENDERED_RGBD',
        source_layout_snapshot_fingerprint=source['fingerprint'],installation_fingerprint=cal['installation_fingerprint'],
        observations=obs,calibration=cal,detection=detection,assignments=assignments,
        evidence_images=evidence_images,
        initial_q_rad=obs['carton_image']['q_rad'],
        physical_layout_changed=False,scope='ESTIMATED_PLANNING_WORLD_WITH_EXPLICIT_OCCLUSION_PRIOR',
        physical_execution='NOT_RUN',execution_ready=False)
    binding['fingerprint']=digest(binding);scene['perception_binding']=binding
    scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'});verify_scene(scene)
    model=build_native_model(scene,out/'model',native_asset_root=a.native_asset_root)
    assets=dict(source['asset_sha256']);assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    for name in (a.observations,a.calibration,a.detection):
        path=Path(name).resolve();assets[path.relative_to(ROOT).as_posix()]=file_hash(path)
    snapshot=dict(schema=source['schema'],scene=scene,tool=model['tool'],native_model=model,
        asset_sha256=assets,config_sha256=source['config_sha256']);snapshot['fingerprint']=digest(snapshot)
    verify_snapshot(snapshot);save_json(out/'snapshot.json',snapshot)
    save_json(out/'home_candidate.json',make_candidate(binding['initial_q_rad'],model['tool'],scene,
        dict(source='ACTUAL_FRESH_RGBD_CAPTURE_Q',perception_binding_fingerprint=binding['fingerprint'])))
    save_json(out/'perception_binding.json',binding)
    print('OBSERVED_SCENE_BUILT',snapshot['fingerprint'],scene['box']['pose'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot','observations','calibration','detection'):p.add_argument('--'+key,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--native-asset-root',default='/work-eco65-20261015')
    build(p.parse_args())
