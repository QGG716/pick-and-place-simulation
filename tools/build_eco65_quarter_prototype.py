"""Build a parameterized lab scene; retain verified real equipment assets."""
import argparse
from copy import deepcopy
from pathlib import Path
import yaml
from unloading_sim.eco65.model import ROOT,load_json,save_json
from unloading_sim.eco65.native_io import verify_snapshot,file_hash
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.quarter_prototype import build_scene
from unloading_sim.eco65.transverse_process import process_options
from unloading_sim.moveit2_backend import digest


def build(a):
    source=verify_snapshot(load_json(a.source));config=yaml.safe_load(a.config.read_text(encoding='utf-8'))
    if a.base is not None:config['installation']['position_m']=a.base
    if a.yaw is not None:config['installation']['rpy_rad'][2]=a.yaw
    if a.columns is not None:config['cartons']['columns']=a.columns
    if a.layers is not None:config['cartons']['layers']=a.layers
    if a.inner_width is not None:config['trailer']['inner_width_m']=a.inner_width
    if a.inner_height is not None:config['trailer']['inner_height_m']=a.inner_height
    if a.target_id is not None:config['initial_target_id']=a.target_id
    out=a.output.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    scene=build_scene(source['scene'],config)
    scene['initial_process_dispatch']=[dict(target_id=b['id'],**process_options(b,scene['boxes'],scene['boxes'],
        scene['coupled_conveyor_spec'],scene['coupled_state']['receiving'])) for b in scene['boxes'] if not b['supports']]
    hashes=dict(source['config_sha256']);hashes[a.config.resolve().relative_to(ROOT).as_posix()]=file_hash(a.config)
    scene['config_sha256']=hashes
    scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'})
    verify_scene(scene)
    model=build_native_model(scene,out/'model',native_asset_root=a.native_asset_root)
    assets=dict(source['asset_sha256']);assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    snapshot=dict(schema=source['schema'],scene=scene,tool=model['tool'],native_model=model,asset_sha256=assets,config_sha256=hashes)
    snapshot['fingerprint']=digest(snapshot);verify_snapshot(snapshot)
    save_json(out/'snapshot.json',snapshot);save_json(out/'native_model_prepared.json',model)
    home=make_candidate(a.initial_q,model['tool'],scene,dict(source='NEW_LAYOUT_INITIAL_PROPOSAL_REQUIRES_NATIVE_COLLISION_CHECK'))
    save_json(out/'home_candidate.json',home)
    save_json(out/'build_report.json',dict(status='BUILT_NOT_NATIVE_VALIDATED',snapshot_fingerprint=snapshot['fingerprint'],
        cartons=len(scene['boxes']),physical_cups=len(model['tool']['seals']),prototype=scene['prototype'],
        calibration='INVALIDATED_REQUIRES_NEW_TCP_OBSERVATIONS',planning_only=True,physical_execution='NOT_RUN',execution_ready=False))
    print(snapshot['fingerprint'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/workcells/eco65_lab_prototype_v2.yaml')
    p.add_argument('--columns',type=int);p.add_argument('--layers',type=int)
    p.add_argument('--inner-width',type=float);p.add_argument('--inner-height',type=float)
    p.add_argument('--target-id',help='Explicit representative target in the complete configured inventory')
    p.add_argument('--base',nargs=3,type=float);p.add_argument('--yaw',type=float)
    p.add_argument('--initial-q',nargs=6,type=float,default=[-1.5707963267948966,0.,0.,0.,0.,0.])
    p.add_argument('--native-asset-root',default='/work-eco65-20261015')
    build(p.parse_args())
