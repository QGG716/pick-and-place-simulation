"""Build a separate full-face static geometry snapshot. Never a native plan."""
import argparse
import copy
import math
from pathlib import Path
import numpy as np
import trimesh
import yaml
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_model import POLICY
from unloading_sim.moveit2_backend import digest
from prepare_eco65_static_pose import prepare
from eco65_official_conveyor import build as build_conveyors


def verify_static(snapshot):
    check=dict(snapshot);fp=check.pop('fingerprint')
    if snapshot['schema']!='eco65_full_face_static_snapshot_v1' or digest(check)!=fp:
        raise ValueError('STATIC_SNAPSHOT_IDENTITY_INVALID')
    for name,sha in {**snapshot['asset_sha256'],**snapshot['config_sha256']}.items():
        p=(ROOT/name).resolve();p.relative_to(ROOT)
        if file_hash(p)!=sha:raise ValueError('STATIC_ASSET_CHANGED:'+name)
    s=snapshot['scene'];c=s['layout_config'];boxes=s['boxes']
    if len(boxes)!=c['depth_rows']*c['width_columns']*c['height_layers'] or len({b['id'] for b in boxes})!=len(boxes):
        raise ValueError('STATIC_BOX_IDENTITIES_INVALID')
    if snapshot['status']!='FULL_FACE_TARGET_NOT_MOTION_VALIDATED':raise ValueError('STATIC_SCOPE_INVALID')
    return snapshot


def geometry(scene,tool,config):
    # Contact TCP Z points into the carton; +height is -TCP Z.
    T=np.linalg.inv(np.array(tool['T_tool_cad_tcp']))
    vertices=np.concatenate([np.asarray(trimesh.load(ROOT/p['visual'],process=False).vertices) for p in tool['visual_parts']])
    v=vertices@T[:3,:3].T+T[:3,3]
    flange=np.linalg.inv(np.array(tool['T_flange_tcp']))[:3,3]
    height=max(float(-v[:,2].min()),float(-flange[2]))
    required=height+config['preapproach_m']+POLICY['required_pair_clearance_m']
    rings=np.concatenate([s['ring_cad_m'] for s in tool['seals']]);rings=rings@T[:3,:3].T+T[:3,3]
    from scipy.spatial import ConvexHull
    hull=rings[ConvexHull(rings[:,:2]).vertices,:2];edges=np.roll(hull,-1,axis=0)-hull
    normals=np.column_stack([-edges[:,1],edges[:,0]]);normals/=np.linalg.norm(normals,axis=1)[:,None]
    minimum_width=float(np.min(np.ptp(hull@normals.T,axis=0)))
    available=scene['roof_inner_z_m']-max(b['pose'][2]+b['size_m'][2]/2 for b in scene['boxes'])
    return dict(status='BLOCKED' if available<required else 'PASS_HEIGHT_NECESSARY_ONLY',
        contact_tool_height_m=height,tool_bounds_TCP_m=[v.min(0).tolist(),v.max(0).tolist()],flange_origin_TCP_m=flange.tolist(),
        preapproach_m=config['preapproach_m'],required_pair_clearance_m=POLICY['required_pair_clearance_m'],required_headroom_m=required,
        available_headroom_m=max(0.,available),height_only_layer_bound=math.floor((scene['effective_cargo_height_m']-required)/config['box_size_m'][2]),
        height_bound_scope='NECESSARY_HEIGHT_ONLY_NOT_IK_OR_PATH_PROOF',seal_count=len(tool['seals']),
        seal_footprint_TCP_xy_m=np.ptp(rings[:,:2],axis=0).tolist(),minimum_seal_footprint_width_any_roll_m=minimum_width,
        all_12_side_face_fit='BLOCKED_MINIMUM_WIDTH_EXCEEDS_100MM' if minimum_width>.100 else 'NOT_EVALUATED',
        seal_edge_margin_m=POLICY['seal_edge_margin_m'],side_faces_m=[[.210,.100],[.250,.100]],
        coverage_risks=dict(bottom='Overlying carton occupies top approach; low belt/pedestal and full robot chain NOT_EVALUATED',
            middle='Overlying layers occupy top approach; lateral reach and full chain NOT_EVALUATED',
            top='Zero headroom: real tool/flange and preapproach cannot fit below retained roof'),
        diagnostic='Hypothetical top contact envelope only; robot remains at inherited HOME; no IK was run')


def build(args):
    out=args.output.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    old=verify_snapshot(load_json(args.snapshot));c=yaml.safe_load(args.config.read_text(encoding='utf8'))
    prepare(args.snapshot,args.candidate,args.failure,out/'inherited_accepted_pose.json')
    conveyor_dir=out/'conveyors';conveyor_dir.mkdir()
    lock=load_json(ROOT/c['conveyor']['asset_lock'])
    conveyors=build_conveyors(args.conveyor_cache,lock,c,conveyor_dir)
    for item in conveyors['items']:item['usd_path']=Path(item['usd_path']).relative_to(ROOT).as_posix()
    front=max(item['bounds_D_m'][1][0] for item in conveyors['items'])+c['carton_conveyor_gap_m']
    sx,sy,sz=c['box_size_m'];floor=c['belt_top_D_m'];inner_width=c['reference_trailer_width_m']*c['reference_scale']
    height=c['reference_trailer_height_m']*c['reference_scale'];roof=floor+height;rear=front+sx+c['rear_clearance_m']
    boxes=[]
    for row in range(c['depth_rows']):
        for layer in range(c['height_layers']):
            for col in range(c['width_columns']):
                id=f'carton_r{row:02}_l{layer:02}_c{col:02}'
                boxes.append(dict(id=id,size_m=[sx,sy,sz],pose=[front+(row+.5)*sx,(col-(c['width_columns']-1)/2)*(sy+c['column_gap_m']),floor+sz/2+layer*(sz+c['layer_gap_m'])],
                    row=row,layer=layer,column=col,supported_by=['cargo_pedestal' if layer==0 else f'carton_r{row:02}_l{layer-1:02}_c{col:02}'],
                    supports=[] if layer==c['height_layers']-1 else [f'carton_r{row:02}_l{layer+1:02}_c{col:02}'],evidence='STATIC_GEOMETRIC_TARGET_NOT_MOTION_VALIDATED'))
    obs=[copy.deepcopy(o) for o in old['scene']['obstacles'] if not (o['id'].startswith(('conveyor_','carton_')) or o['id'] in ['trailer_left','trailer_right','trailer_rear','trailer_roof'])]
    def cube(id,size,pos,role):obs.append(dict(id=id,size_m=size,position_m=pos,role=role,rgba=[.45,.53,.58,1]))
    t=c['panel_thickness_m'];opening=front+c['panel_front_offset_from_carton_m'];length=rear-opening
    # Pedestal footprint exactly supports every bottom carton, never bridging the gap.
    wallwidth=c['width_columns']*sy+(c['width_columns']-1)*c['column_gap_m']
    cube('cargo_pedestal',[sx,wallwidth,floor],[front+sx/2,0,floor/2],'support')
    for name,sign in [('left',1),('right',-1)]:cube('trailer_'+name,[length,t,roof],[(rear+opening)/2,sign*(inner_width+t)/2,roof/2],'wall')
    cube('trailer_rear',[t,inner_width+2*t,roof],[rear+t/2,0,roof/2],'wall')
    cube('trailer_roof',[length+t,inner_width+2*t,t],[(rear+opening+t)/2,0,roof+t/2],'roof')
    scene=dict(schema='eco65_full_face_static_scene_v1',layout_id=c['schema'],layout_config=c,
        status='FULL_FACE_TARGET_NOT_MOTION_VALIDATED',axis_convention=old['scene']['axis_convention'],
        base_position_m=old['scene']['base_position_m'],T_W_D=old['scene']['T_W_D'],
        environment=old['scene']['environment'],environment_status=old['scene']['environment_status'],
        boxes=boxes,obstacles=obs,mesh_obstacles=conveyors['items'],conveyors=conveyors,
        effective_floor_z_D_m=floor,effective_cargo_height_m=height,inner_width_m=inner_width,roof_inner_z_m=roof,
        box_front_x_D_m=front,opening_x_D_m=opening,rear_inner_x_D_m=rear,source_snapshot_fingerprint=old['fingerprint'],
        transport='STATIC_ONLY',dynamics='NOT_EVALUATED',enable_hardware=False)
    for item in conveyors['items']:
        b=np.array(item['belt_bounds_D_m']);a=1 if item['id'].endswith('transverse') else 0
        lo,hi=b[0].copy(),b[1].copy();lo[a]+=c['conveyor']['end_exclusion_m'];hi[a]-=c['conveyor']['end_exclusion_m']
        item['receiving_surface_bounds_D_m']=[lo.tolist(),hi.tolist()]
        item['box_center_receiving_bounds_D_xy_m']=[(lo[:2]+np.array([sx,sy])/2).tolist(),(hi[:2]-np.array([sx,sy])/2).tolist()]
    scene['top_geometry']=geometry(scene,old['tool'],c)
    scene['scene_fingerprint']=digest(scene)
    assets=dict(old['asset_sha256'])
    for row in lock['dependencies']:
        p=(args.conveyor_cache/row['path']).resolve();assets[p.relative_to(ROOT).as_posix()]=row['sha256']
    for item in conveyors['items']:assets[item['usd_path']]=item['usd_sha256']
    config_hashes={**old['config_sha256'],args.config.resolve().relative_to(ROOT).as_posix():file_hash(args.config),c['conveyor']['asset_lock']:file_hash(ROOT/c['conveyor']['asset_lock'])}
    snapshot=dict(schema='eco65_full_face_static_snapshot_v1',status=scene['status'],scene=scene,tool=old['tool'],
        # Retained only for robot/tool convex identity. No new worker init is fabricated.
        native_model=old['native_model'],asset_sha256=assets,config_sha256=config_hashes,
        source_native_snapshot_fingerprint=old['fingerprint'],native_model_scope='SOURCE_ROBOT_TOOL_GEOMETRY_ONLY_NOT_NEW_SCENE_INIT')
    snapshot['fingerprint']=digest(snapshot);verify_static(snapshot);save_json(out/'snapshot.json',snapshot)
    pose=load_json(out/'inherited_accepted_pose.json');pose['source_snapshot_fingerprint']=snapshot['fingerprint']
    pose.update(source='Inherited actual accepted old four-carton HOME; unchanged q, not revalidated in new layout',
                new_scene_status='FULL_FACE_TARGET_NOT_MOTION_VALIDATED',source_native_snapshot_fingerprint=old['fingerprint'])
    save_json(out/'static_pose.json',pose);save_json(out/'geometry_report.json',scene['top_geometry'])
    print('STATIC_FULL_FACE_BUILT',len(boxes),scene['top_geometry'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['snapshot','candidate','failure','conveyor-cache','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/workcells/eco65_full_face_layout_v1.yaml')
    build(p.parse_args())
