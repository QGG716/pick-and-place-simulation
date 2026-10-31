"""Quarter-scale scene with unscaled ECO65, tool and camera assemblies.

All geometry is in D, the single ground datum. This is new project geometry,
not a scale operation on the robot or on the previous elevated carton scene.
"""
from copy import deepcopy
from itertools import product
import numpy as np
from .model import transform
from .coupled_scene import _hull, _clip_convex, coupled_conveyor_state, with_coupled_state
from ..moveit2_backend import digest

SCHEMA = 'eco65_quarter_prototype_scene_v1'


def inventory(scene):
    if scene.get('schema') == SCHEMA:
        c = scene['prototype']['configuration']['cartons']
        return {f'carton_r{r:02d}_l{l:02d}_c{k:02d}'
                for r in range(c['rows']) for l in range(c['layers']) for k in range(c['columns'])}
    return {f'carton_r00_l{l:02d}_c{k:02d}' for l in range(4) for k in range(3)}


def carton_grid(cfg):
    if (cfg['rows']!=1 or any(type(cfg[k]) is not int or cfg[k]<1 for k in ('columns','layers')) or cfg['columns']*cfg['layers']>256 or cfg['column_gap_m']<0):
        raise ValueError('PROTOTYPE_POPULATION_INVALID')
    size=np.asarray(cfg['size_m']);boxes=[]
    if cfg['bottom_z_m']!=0. or cfg['layer_gap_m']!=0.:
        raise ValueError('PROTOTYPE_STACK_MUST_START_ON_COMMON_GROUND_WITHOUT_LAYER_GAPS')
    for row,layer,col in product(range(cfg['rows']),range(cfg['layers']),range(cfg['columns'])):
        name=f'carton_r{row:02d}_l{layer:02d}_c{col:02d}'
        p=[cfg['front_x_m']+(row+.5)*size[0],
           (col-(cfg['columns']-1)/2)*(size[1]+cfg['column_gap_m']),
           cfg['bottom_z_m']+layer*(size[2]+cfg['layer_gap_m'])+size[2]/2]
        boxes.append(dict(id=name,size_m=size.tolist(),pose=p,row=row,layer=layer,column=col,
            supported_by=[f'carton_r{row:02d}_l{layer-1:02d}_c{col:02d}' if layer else 'trailer_floor'],
            supports=[f'carton_r{row:02d}_l{layer+1:02d}_c{col:02d}'] if layer+1<cfg['layers'] else [],
            evidence='NOMINAL_CONFIGURED_STACK_NOT_A_VISION_OBSERVATION'))
    return boxes


def mechanism_spec(config, source_fingerprint, contact_modes):
    c = deepcopy(config['receiver']); c.update(contact_modes)
    c['process_policy_schema']='eco65_quarter_transverse_templates_v3'
    c['carton_height_m']=config['cartons']['size_m'][2]
    bounds = np.asarray(c['transverse_bounds_xy_m'], float)
    width, trans_len = bounds[1]-bounds[0]
    length, long_width = c['longitudinal_length_m'], c['longitudinal_width_m']
    rise = c['high_height_m']-c['longitudinal_lower_height_m']
    slope = rise/np.sqrt(length**2-rise**2)
    depth = c['belt_depth_m']; templates=[]
    for owner, x, y in [('conveyor_transverse',width,trans_len),('conveyor_longitudinal',length,long_width)]:
        mesh=_hull(list(product((-x/2,x/2),(-y/2,y/2),(-depth,0.))))
        if owner=='conveyor_transverse':mesh=_clip_convex(mesh,[1.,0.,slope],-width/2)
        templates.append(dict(id=owner+'_prototype_belt',owner=owner,role='conveyor_belt',
                              convex_mesh=mesh,rgba=[.17,.20,.21,1]))
    spec=dict(schema='eco65_coupled_spec_v1',geometry_revision=c['geometry_revision'],
        belt_geometry_revision='QUARTER_REFERENCE_NEW_INDEPENDENT_WIDTH_BELTS_V1',
        support_geometry_revision='QUARTER_100_525_FIXED_LENGTH_SCISSORS_V1',
        source_scene_fingerprint=source_fingerprint,config=c,
        low_height_m=c['low_height_m'],high_height_m=c['high_height_m'],
        longitudinal_lower_height_m=c['longitudinal_lower_height_m'],
        longitudinal_length_m=length,longitudinal_width_m=long_width,
        transverse_width_m=float(width),transverse_length_m=float(trans_len),
        transverse_center_xy_m=bounds.mean(0).tolist(),
        longitudinal_center_y_m=c['longitudinal_center_y_m'],transverse_xmin_m=float(bounds[0,0]),
        seam_relief_slope=float(slope),belt_depth_m=depth,belt_templates=templates,
        replaced_object_ids=[],replaced_obstacle_ids=[],replaced_visual_ids=[],
        engineering_geometry=dict(belts='NEW_PARAMETRIC_BELTS_AT_QUARTER_REFERENCE_DIMENSIONS',
            supports='NEW_FIXED_LENGTH_THREE_AND_FOUR_STAGE_SCISSORS; NARROW_LOWER_BEARINGS',
            frame='NEW_24MM_FRAME_WITH_FIXED_SEAM_UNDERSIDE_RELIEF',
            qualification='PROJECT_GEOMETRY_NOT_MANUFACTURING_OR_LOAD_RATING'))
    spec['fingerprint']=digest(spec)
    for h in (c['low_height_m'],c['high_height_m']):coupled_conveyor_state(spec,h)
    return spec


def build_scene(source, config):
    if (config['schema']!='eco65_quarter_prototype_config_v1' or config['scene_scale']!=.25
            or config['equipment_scale']!=1. or config['enable_hardware'] is not False):
        raise ValueError('QUARTER_PROTOTYPE_CONFIG_INVALID')
    scene=deepcopy(source)
    for key in ('perception_binding','base_translation','small_carton_configuration','process_migration',
                'initial_process_dispatch','source_static_snapshot_fingerprint','source_static_scene_fingerprint',
                'coupled_state','coupled_conveyor_spec','layout_installation'):
        scene.pop(key,None)
    scene.update(schema=SCHEMA,layout_id=config['layout_id'],layout_config=deepcopy(config),
        base_position_m=list(config['installation']['position_m']),
        base_rpy_rad=list(config['installation']['rpy_rad']),
        T_W_D=transform(config['external_translation_m']).tolist(),
        effective_floor_z_D_m=0.,
        effective_cargo_height_m=config['cartons']['layers']*config['cartons']['size_m'][2],
        inner_width_m=config['trailer']['inner_width_m'],roof_inner_z_m=config['trailer']['inner_height_m'],
        box_front_x_D_m=config['cartons']['front_x_m'],opening_x_D_m=config['trailer']['entrance_x_m'],
        rear_inner_x_D_m=config['trailer']['closed_end_x_m'],
        carton_floor_id='trailer_floor',mesh_obstacles=[],native_world_objects=[],obstacles=[])
    cfg=config['cartons'];boxes=carton_grid(cfg)
    if config['layout_id']=='eco65_lab_prototype_fixed_100mm_cartons_v2' and cfg['size_m']!=[.15,.1,.1]:
        raise ValueError('LAB_CARTON_SIZE_MUST_REMAIN_150_100_100_MM')
    scene['boxes']=boxes
    def part(name,center,dimensions,role,color):
        scene['obstacles'].append(dict(id=name,position_m=list(map(float,center)),
            size_m=list(map(float,dimensions)),owner=name,role=role,rgba=color))
    t=config['trailer'];x0,x1=t['entrance_x_m'],t['closed_end_x_m'];w=t['inner_width_m'];h=t['inner_height_m'];th=t['panel_thickness_m']
    if not np.isfinite([x0,x1,w,h,th]).all() or min(x1-x0,w,h,th)<=0:raise ValueError('PROTOTYPE_ENCLOSURE_INVALID')
    from .transverse_process import corners
    stack=np.vstack([corners(b) for b in boxes])
    if (stack[:,0].min()<x0 or stack[:,0].max()>x1 or np.max(np.abs(stack[:,1]))>w/2
            or stack[:,2].min()<-1e-10 or stack[:,2].max()>h):raise ValueError('CONFIGURED_STACK_OUTSIDE_ENCLOSURE')
    search=config.get('layout_search')
    if search and not (min(search['trailer_inner_widths_m'])<=w<=max(search['trailer_inner_widths_m'])
                      and min(search['trailer_inner_heights_m'])<=h<=max(search['trailer_inner_heights_m'])):
        raise ValueError('ENCLOSURE_OUTSIDE_BOUNDED_LAYOUT_SEARCH')
    for name,p,s in [('floor',[(x0+x1)/2,0,-th/2],[x1-x0,w+2*th,th]),
                     ('roof',[(x0+x1)/2,0,h+th/2],[x1-x0,w+2*th,th]),
                     ('left_wall',[(x0+x1)/2,w/2+th/2,h/2],[x1-x0,th,h]),
                     ('right_wall',[(x0+x1)/2,-w/2-th/2,h/2],[x1-x0,th,h]),
                     ('closed_end',[x1+th/2,0,h/2],[th,w+2*th,h])]:
        part('trailer_'+name,p,s,'prototype_enclosure',[.46,.57,.64,1.])
    ch=config['chassis'];part('prototype_chassis',ch['center_m'],ch['size_m'],'prototype_chassis',[.25,.28,.31,1.])
    base=np.asarray(scene['base_position_m']);mount_size=np.array([.140,.140,base[2]-.150])
    limits=np.asarray(config['installation']['search_xyz_bounds_m'])
    chlo=np.array(ch['center_m'])-np.array(ch['size_m'])/2;chhi=np.array(ch['center_m'])+np.array(ch['size_m'])/2
    if (np.any(base<limits[0]) or np.any(base>limits[1]) or mount_size[2]<=0
            or np.any(base[:2]-mount_size[:2]/2<chlo[:2]) or np.any(base[:2]+mount_size[:2]/2>chhi[:2])):
        raise ValueError('PROTOTYPE_INSTALLATION_NOT_FULLY_SUPPORTED_OR_OUT_OF_BOUNDS')
    part('robot_mount_plate',[*base[:2],.150+mount_size[2]/2],mount_size,'mount',[.38,.4,.42,1.])
    # The entry has no front wall; support below Z=0 does not raise the cartons.
    part('external_support_floor',[-1.1,0.,-th/2],[.6,w,th],'external_support',[.45,.45,.45,1.])
    scene['prototype']=dict(schema='eco65_quarter_reference_adaptation_v1',configuration=deepcopy(config),
        source_scene_fingerprint=source['scene_fingerprint'],
        coordinate_origin='INITIAL_STACK_FRONT_CENTRE_PROJECTED_TO_COMMON_GROUND',
        equipment_geometry='OFFICIAL_AND_PRIVATE_ASSETS_UNSCALED',
        engineering_exceptions=[dict(item='receiver_minimum',nominal_scaled_m=.050,actual_m=.100),
            dict(item='robot_and_tool',scene_scale=.25,equipment_scale=1.),
            dict(item='cartons',actual_size_m=cfg['size_m'],columns=cfg['columns'],layers=cfg['layers'],
                 basis='FIXED_USER_CARTON_SIZE_AND_FINITE_FULL_CYCLE_LAYOUT_SEARCH'),
            dict(item='robot_installation',nominal_m=config['installation']['nominal_position_m'],
                 actual_m=scene['base_position_m'],actual_rpy_rad=scene['base_rpy_rad']),
            dict(item='supports',design='NEW_RIGID_SCISSORS_FRAMES_RAILS_AND_BASE_PEDESTAL')],
        installation_mode=config['installation']['mode'],
        base_reposition_status='RESERVED_TRAVEL_NOT_EXECUTED_OR_VALIDATED')
    from .transverse_process import PROTOTYPE_POLICY
    scene['transverse_process_policy']=deepcopy(PROTOTYPE_POLICY)
    contact={key:source[key] for key in ('extraction_carton_contact_mode','cup_neighbor_contact_mode')}
    scene['coupled_conveyor_spec']=mechanism_spec(config,source['scene_fingerprint'],contact)
    target=next(b for b in boxes if b['layer']==cfg['layers']-1 and b['column']==cfg['columns']//2)
    if config.get('initial_target_id'):
        target=next(b for b in boxes if b['id']==config['initial_target_id'])
        if target['supports']:raise ValueError('INITIAL_FULL_STACK_TARGET_SUPPORTS_UPPER_CARTONS')
    scene['box']=deepcopy(target);scene['target_box_id']=target['id']
    scene['native_task'].update(target_id=target['id'],target_face='front',stack_carton_ids=sorted(b['id'] for b in boxes),
        inventory_count=len(boxes),inventory_schema=SCHEMA,original_support_names=target['supported_by'],
        allowed_faces=['top','front','left','right'],target_selection=dict(schema='eco65_explicit_exposed_target_v1',
            target_id=target['id'],inventory_scope='FROZEN_CONFIGURED_CARTONS',source='TRANSVERSE_PROCESS_DISPATCH'))
    scene=with_coupled_state(scene,config['receiver']['initial_height_m'])
    scene['conveyors']=dict(schema='eco65_parametric_conveyors_v1',items=[
        dict(id='conveyor_transverse',effective_bounds_xy_m=config['receiver']['transverse_bounds_xy_m']),
        dict(id='conveyor_longitudinal',effective_length_m=config['receiver']['longitudinal_length_m'],
             effective_width_m=config['receiver']['longitudinal_width_m'])])
    scene['environment']=dict(schema='eco65_parametric_enclosure_v1',T_W_D=scene['T_W_D'],
        complete_traversal=True,objects=deepcopy(scene['obstacles']),asset_sha256={},
        source='REFERENCE_SCENE_QUARTER_DIMENSIONS_NOT_MEASURED_TRUCK')
    scene['environment_asset_sha256']={};scene['environment_status']='ALL_BOUNDS_INCLUDED_CONSERVATIVELY'
    camera=scene['permanent_camera']
    camera['layout_remount']['prototype']=deepcopy(scene['prototype'])
    from .camera_installation import installation_fingerprint
    camera['calibration'].update(status='NOMINAL_FROM_CHANGED_INSTALLATION_NOT_CALIBRATED',
        prior_calibration_valid=False,installation_fingerprint=installation_fingerprint(camera))
    scene['camera_extrinsics']=deepcopy(camera['calibration'])
    scene['components_by_id']={o['id']:dict(role=o['role'],owner=o['owner']) for o in scene['obstacles']}
    scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'})
    return scene
