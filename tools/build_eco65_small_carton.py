"""Rebuild the authorized small layout from verified native geometry, without STEP conversion."""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import numpy as np
import yaml
from unloading_sim.eco65.model import ROOT, load_json, save_json, transform
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.moveit2_backend import digest


def build(source_path,home_path,output,config_path,base_index=0,native_asset_root='/work-eco65-20261015',collision_cache=None):
    from eco65_official_conveyor import build as conveyor_build
    from build_eco65_native_12carton import prepare_conveyors
    started=perf_counter();old=verify_snapshot(load_json(source_path))
    out=Path(output).resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    config_path=Path(config_path).resolve();c=yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if (c['schema']!='eco65_small_carton_config_v1' or c['box_size_m']!=[.15,.1,.1] or
        [c[k] for k in ('depth_rows','width_columns','height_layers')]!=[1,3,4] or
        c['physical_contour_count']!=12 or c['minimum_enabled_cups']!=1 or c['enable_hardware'] is not False):
        raise ValueError('SMALL_LAYOUT_CONTRACT_INVALID')
    scene=deepcopy(old['scene']);layout=deepcopy(scene['layout_config'])
    extra_assets={}
    if collision_cache:
        cache_data=load_json(collision_cache)
        for name,rows in cache_data['robot_collision_components'].items():
            for row in rows:
                for path,expected in [(row['source'],row['source_sha256']),(row['mesh'],row['mesh_sha256'])]:
                    if file_hash(ROOT/path)!=expected:raise ValueError('ROBOT_COLLISION_CACHE_CHANGED:'+path)
                    extra_assets[path]=expected
            scene['robot_collision_components'][name]=rows
    layout.update(box_size_m=c['box_size_m'],column_gap_m=c['column_gap_m'],layer_gap_m=c['layer_gap_m'],
                  belt_top_D_m=c['belt_top_D_m'],carton_conveyor_gap_m=c['carton_conveyor_gap_m'])
    layout['conveyor']['usable_width_m']=c['usable_belt_width_m']
    lock=load_json(ROOT/layout['conveyor']['asset_lock'])
    suffix=lock['a06']['path']
    paths=[p for p,h in old['asset_sha256'].items() if p.endswith(suffix) and h==lock['a06']['sha256']]
    if len(paths)!=1:raise ValueError('VERIFIED_CONVEYOR_CACHE_AMBIGUOUS')
    cache=ROOT/paths[0][:-len(suffix)]
    (out/'conveyors').mkdir();conveyors=conveyor_build(cache,lock,layout,out/'conveyors')
    size=np.array(c['box_size_m']);floor=c['belt_top_D_m'];front=max(x['bounds_D_m'][1][0] for x in conveyors['items'])+c['carton_conveyor_gap_m']
    for item in conveyors['items']:
        item['usd_path']=Path(item['usd_path']).relative_to(ROOT).as_posix()
        lo,hi=np.array(item['belt_bounds_D_m']);axis=1 if item['id'].endswith('transverse') else 0
        lo[axis]+=layout['conveyor']['end_exclusion_m'];hi[axis]-=layout['conveyor']['end_exclusion_m']
        item['receiving_surface_bounds_D_m']=[lo.tolist(),hi.tolist()]
        item['box_center_receiving_bounds_D_xy_m']=[(lo[:2]+size[:2]/2).tolist(),(hi[:2]-size[:2]/2).tolist()]
    scene['conveyors']=conveyors;scene['mesh_obstacles']=conveyors['items'];scene['layout_config']=layout
    for b in scene['boxes']:
        b['size_m']=size.tolist();b['pose']=[front+size[0]/2,(b['column']-1)*(size[1]+c['column_gap_m']),floor+(b['layer']+.5)*size[2]]
        b['evidence']='SMALL_CARTON_LAYOUT_REQUIRES_NEW_NATIVE_ACCEPTANCE'
    scene['box']=deepcopy(next(b for b in scene['boxes'] if b['id']==c['target_id']))
    scene['base_position_m']=c['base_candidates_D_m'][base_index]
    obsolete={'cargo_pedestal','trailer_left','trailer_right','trailer_rear','trailer_roof','junction_transition_plate'}
    scene['obstacles']=[o for o in scene['obstacles'] if o['id'] not in obsolete]
    for o in scene['obstacles']:
        if o['id']=='robot_mount_plate':o['position_m']=[*scene['base_position_m'][:2],scene['base_position_m'][2]/2];o['size_m'][2]=scene['base_position_m'][2]
    def box(name,size,pos,role):scene['obstacles'].append(dict(id=name,size_m=list(size),position_m=list(pos),role=role,owner=name,rgba=[.45,.53,.58,1]))
    width=3*size[1]+2*c['column_gap_m'];rear=front+size[0]+layout['rear_clearance_m'];opening=front+layout['panel_front_offset_from_carton_m']
    t=layout['panel_thickness_m'];height=floor+scene['effective_cargo_height_m'];inner=scene['inner_width_m']
    box('cargo_pedestal',[size[0],width,floor],[front+size[0]/2,0,floor/2],'support')
    for name,sign in [('left',1),('right',-1)]:box('trailer_'+name,[rear-opening,t,height],[(rear+opening)/2,sign*(inner+t)/2,height/2],'wall')
    box('trailer_rear',[t,inner+2*t,height],[rear+t/2,0,height/2],'wall')
    if layout.get('roof_enabled',True):box('trailer_roof',[rear-opening+t,inner+2*t,t],[(rear+opening+t)/2,0,height+t/2],'roof')
    transverse,longitudinal=[np.array(i['belt_bounds_D_m']) for i in conveyors['items']]
    x0,x1=transverse[:,0];y0,y1=longitudinal[1,1],transverse[0,1]
    if y1<=y0:raise ValueError('SMALL_CONVEYOR_JUNCTION_OVERLAP')
    box('junction_transition_plate',[x1-x0,y1-y0,.003],[(x0+x1)/2,(y0+y1)/2,floor-.0015],'project_static_transition')
    scene.update(box_front_x_D_m=front,opening_x_D_m=opening,rear_inner_x_D_m=rear,
                 layout_id=c['schema'],small_carton_configuration=c,source_native_snapshot_fingerprint=old['fingerprint'])
    scene.pop('top_geometry',None) # historical all-twelve hull proof is inapplicable
    objects,records,surfaces=prepare_conveyors(scene)
    scene['native_world_objects']=objects;scene['receiving_surfaces']=surfaces
    receiver=deepcopy(surfaces['conveyor_longitudinal']);lo,hi=np.array(receiver['bounds_D_m'])
    receiver['box_pose_D']=transform([*((lo[:2]+hi[:2])/2),hi[2]+size[2]/2]).tolist()
    task=scene['native_task'];task.update(target_id=c['target_id'],target_face='top',allowed_faces=c['allowed_faces'],
        cup_selection_mode=c['cup_selection_mode'],receiving=receiver)
    for k in ('preapproach_m','preplace_m','withdrawal_m','minimum_lift_m'):task[k]=c[k]
    scene['source_equipment_config']['conveyors']=dict(surface_z_m=floor,longitudinal=dict(receive_center_xy_m=receiver['box_pose_D'][0:2]))
    scene['source_equipment_config']['conveyors']['longitudinal']['receive_center_xy_m']=np.array(receiver['box_pose_D'])[:2,3].tolist()
    scene['source_config']=deepcopy(scene['source_equipment_config']);scene['support_surface_ids']=['cargo_pedestal',*receiver['support_ids']]
    scene['components_by_id']={o['id']:dict(role=o['role'],owner=o['owner']) for o in scene['obstacles']}
    if len(scene['components_by_id'])!=len(scene['obstacles']):raise ValueError('SMALL_LAYOUT_DUPLICATE_OBJECT')
    config_hashes={**old['config_sha256'],config_path.relative_to(ROOT).as_posix():file_hash(config_path)}
    if collision_cache:config_hashes[Path(collision_cache).resolve().relative_to(ROOT).as_posix()]=file_hash(collision_cache)
    scene['config_sha256']=config_hashes;scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'});verify_scene(scene)
    model=build_native_model(scene,out/'model',native_asset_root=native_asset_root)
    assets=dict(old['asset_sha256'],**extra_assets);assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    assets.update({i['usd_path']:i['usd_sha256'] for i in conveyors['items']})
    snapshot=dict(schema=old['schema'],scene=scene,tool=model['tool'],native_model=model,asset_sha256=assets,config_sha256=config_hashes)
    snapshot['fingerprint']=digest(snapshot);verify_snapshot(snapshot)
    save_json(out/'snapshot.json',snapshot);save_json(out/'native_model_prepared.json',model)
    h=load_json(home_path);save_json(out/'home_candidate.json',make_candidate(h['q_rad'],model['tool'],scene,
        dict(source='NEW_LAYOUT_PROPOSAL_REQUIRES_NATIVE_CHECK',source_home_sha256=file_hash(home_path))))
    save_json(out/'scene_geometry.json',dict(conveyors=records,cartons=scene['boxes'],base_position_m=scene['base_position_m'],source_snapshot_fingerprint=old['fingerprint']))
    report=dict(status='BUILT_NOT_RUNTIME_VALIDATED',snapshot_fingerprint=snapshot['fingerprint'],base_index=base_index,
                box_size_m=size.tolist(),stack_size_m=[size[0],width,4*size[2]],base_position_m=scene['base_position_m'],
                front_x_m=front,receiving=receiver,cup_selection_mode=c['cup_selection_mode'],no_STEP_conversion=True,wall_s=perf_counter()-started)
    save_json(out/'build_report.json',report);return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','home','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/workcells/eco65_small_carton_v1.yaml')
    p.add_argument('--base-index',type=int,default=0);p.add_argument('--native-asset-root',default='/work-eco65-20261015')
    p.add_argument('--collision-cache',type=Path)
    a=p.parse_args();print(build(a.source,a.home,a.output,a.config,a.base_index,a.native_asset_root,a.collision_cache))
