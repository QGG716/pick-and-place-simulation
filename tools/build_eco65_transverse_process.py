"""Incrementally bind the 100 mm transverse process to the current snapshot."""
import argparse
from copy import deepcopy
from pathlib import Path
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import make_candidate, verify_candidate
from unloading_sim.eco65.coupled_scene import build_coupled_scene, CONFIG
from unloading_sim.eco65.transverse_process import POLICY, process_options, corners
from unloading_sim.moveit2_backend import digest


def build(args):
    source=verify_snapshot(load_json(args.snapshot))
    home=verify_candidate(load_json(args.home_candidate),source['tool'],source['scene'])
    output=args.output.resolve();output.relative_to(ROOT/'outputs');output.mkdir(parents=True,exist_ok=False)
    scene=build_coupled_scene(source['scene'])
    scene['transverse_process_policy']=deepcopy(POLICY)
    scene['native_task']['allowed_faces']=['top','front','left','right']
    # The former C02 diagnostic target may need the middle channel opened
    # first. Dispatch from geometry before cups/IK, retaining all identities.
    dispatch=[]
    boxes=scene['boxes'];remaining={b['id'] for b in boxes}
    for box in boxes:
        if any(name in remaining for name in box.get('supports',[])):continue
        options=process_options(box,boxes,boxes,scene['coupled_conveyor_spec'],scene['coupled_state']['receiving'])
        dispatch.append(dict(target_id=box['id'],**options))
    legal=[row for row in dispatch if row['options']]
    legal.sort(key=lambda row:(-row['frozen']['bottom_world_m'],
        row['target_id']!=scene['target_box_id'],row['options'][0]['requires_open_channel'],
        abs(next(b['pose'][1] for b in boxes if b['id']==row['target_id']))))
    if legal:
        target=deepcopy(next(b for b in boxes if b['id']==legal[0]['target_id']))
        scene['box']=target;scene['target_box_id']=target['id']
        scene['native_task'].update(target_id=target['id'],target_face='front',
                                    original_support_names=target['supported_by'])
        # This is explicit geometry dispatch, not a new user-picked carton.
        scene['native_task']['target_selection']=dict(schema='eco65_explicit_exposed_target_v1',
            target_id=target['id'],inventory_scope='UNCHANGED_TWELVE_CARTONS',source='TRANSVERSE_PROCESS_DISPATCH')
    scene['initial_process_dispatch']=dispatch
    scene['process_migration']=dict(source_snapshot_fingerprint=source['fingerprint'],
        inherited_height_m=source['scene']['coupled_state']['height_m'],
        camera_robot_cartons_unchanged=True,calibration_reused=True,
        geometry_change='TRANSVERSE_FIXED_BASE_PIN_26_TO_22_MM_WITH_MATCHING_FIXED_FEET; LOWER_LONGITUDINAL_ENDPOINT_REMAINS_180_MM')
    hashes=dict(source['config_sha256']);hashes[CONFIG.relative_to(ROOT).as_posix()]=file_hash(CONFIG)
    scene['config_sha256']=hashes
    scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'})
    verify_scene(scene)
    model=build_native_model(scene,output/'model',native_asset_root=args.native_asset_root)
    assets=dict(source['asset_sha256'])
    assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    snapshot=dict(schema=source['schema'],scene=scene,tool=model['tool'],native_model=model,
                  asset_sha256=assets,config_sha256=hashes)
    snapshot['fingerprint']=digest(snapshot);verify_snapshot(snapshot)
    candidate=make_candidate(home['q_rad'],model['tool'],scene,
        dict(source='INHERITED_ACTUAL_CAPTURE_Q_UNCHANGED_REQUIRES_CURRENT_NATIVE_CHECK'))
    save_json(output/'snapshot.json',snapshot);save_json(output/'home_candidate.json',candidate)
    save_json(output/'build_report.json',dict(status='BUILT_REQUIRES_NATIVE_RUN',
        snapshot_fingerprint=snapshot['fingerprint'],process_policy=POLICY,
        **scene['process_migration'],planning_only=True,physical_execution='NOT_RUN',execution_ready=False))
    print(snapshot['fingerprint'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',required=True)
    parser.add_argument('--home-candidate',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--native-asset-root',default='/work-eco65-20261015')
    build(parser.parse_args())
