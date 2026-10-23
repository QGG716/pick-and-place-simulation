"""Three explicitly requested l01 routes, bound to frozen carton identities."""
from copy import deepcopy
import numpy as np
from .lifted_reception import StartHeight, directional_contact, receiving_region

SCOPE = 'XY_TOP_L01_SELECTED_SIDE_TOP_HMAX'
SIDE_TEMPLATE = 'SELECTED_SIDE_POS_Y_HMAX'
TOP_TEMPLATE = 'SELECTED_TOP_HMAX'
ORDER = ('carton_l01_c01', 'carton_l01_c00', 'carton_l01_c04')
TOP_TARGET = 'carton_l01_c04'
C01_CHANNEL_SCOPE = 'XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX'
C01_FRONT_SCOPE = 'XY_TOP_L01_C01_FRONT_FINITE_CHANNEL_HMAX'
FRONT_TEMPLATE = 'SELECTED_FRONT_POS_Y_HMAX'
C01 = 'carton_l01_c01'


def selected_routes_permission(frozen, *, finite_c01_channel=False, front_c01=False):
    if front_c01 and not finite_c01_channel:
        raise ValueError('C01_FRONT_REQUIRES_FINITE_NONJUNCTION_CHANNEL')
    evidence=frozen.evidence();routes={}
    if evidence['original_carton_count']!=40:
        raise ValueError('SELECTED_ROUTES_ORIGINAL_POPULATION_CHANGED')
    order=(C01,) if finite_c01_channel else ORDER
    for name in order:
        h=evidence['heights'][name]
        if abs(h['z_b_m']-.3)>1e-9 or abs(h['h_m']-.3)>1e-9 or abs(h['Hmax_world_m']-.25)>1e-9:
            raise ValueError('SELECTED_ROUTES_ORIGINAL_LAYER_CHANGED')
        top=name==TOP_TARGET
        routes[name]=dict(start_box_identity=h['start_box_identity'],height_m=h['Hmax_world_m'],
            template=TOP_TEMPLATE if top else FRONT_TEMPLATE if front_c01 else SIDE_TEMPLATE,
            geometric_face_id='z+' if top else 'x-' if front_c01 else 'y+',
            segment_directions_world=[[0.,0.,1.] if top else [0.,1.,0.],[-1.,0.,0.]],
            original_role=evidence['roles'][name]['role'])
        if finite_c01_channel:routes[name]['finite_side_channel']=True
    return dict(original_cartons_sha256=evidence['original_cartons_sha256'],routes=routes,
        dispatch_order=list(order),height_source='FROZEN_INITIAL_HMAX',stationary_receiver=True,
        authorization='USER_SELECTED_THREE_CARTONS_ONLY',planning_only=True)


def selected_route_option(target,belt,longitudinal,frozen,current_height,permission):
    finite=permission.get('routes',{}).get(C01,{}).get('finite_side_channel',False)
    front=permission.get('routes',{}).get(C01,{}).get('template')==FRONT_TEMPLATE
    if permission!=selected_routes_permission(frozen,finite_c01_channel=finite,front_c01=front):
        raise ValueError('SELECTED_ROUTES_PERMISSION_CHANGED')
    route=permission['routes'].get(target.name)
    if route is None:raise ValueError('TARGET_OUTSIDE_SELECTED_ROUTES')
    h=StartHeight.capture(target)
    if h.box_identity!=route['start_box_identity'] or current_height!=route['height_m']:
        raise ValueError('SELECTED_ROUTE_TARGET_OR_FIXED_HMAX_CHANGED')
    if route['template']==TOP_TEMPLATE:
        from .low_top_receiver import _fixed_top_option
        option=_fixed_top_option(target,belt,longitudinal,frozen,current_height,h,'USER_SELECTED_TOP_HMAX')
    else:
        option=dict(face=directional_contact(target,[-1.,0.,0.] if front else [0.,1.,0.]),height_m=current_height,
            pre_approach_height_m=current_height,synchronization='ALREADY_AT_RECEIVE',
            stationary_receiver_during_target=True,requires_open_channel=True,
            region=receiving_region(target,belt),endpoint='REGION_NOT_FIXED_POINT',
            acceptance='NOT_PLANNED',spatial_orientation='UPSTREAM_FACE_POSE_HELD',fixed_wrist_joint_values=False)
    option.update(template=route['template'],target_id=target.name,original_role=route['original_role'],
        segment_directions_world=deepcopy(route['segment_directions_world']),
        height_source='FROZEN_INITIAL_HMAX',selected_routes_permission=deepcopy(permission))
    if finite:option['finite_side_channel']=True
    return option


def validate_selected_route(target,option,permission,root_height):
    if permission is None or option.get('selected_routes_permission')!=permission:
        raise ValueError('SELECTED_ROUTES_PERMISSION_MISSING')
    route=permission['routes'].get(target.target_id)
    h=StartHeight.capture(target.box)
    if (route is None or h.box_identity!=route['start_box_identity']
            or target.geometric_face_id!=route['geometric_face_id'] or target.world_y_offset_m!=0
            or target.eccentric_authority is not None or option['template']!=route['template']
            or option['segment_directions_world']!=route['segment_directions_world']
            or root_height!=route['height_m'] or option['height_m']!=h.hmax
            or option.get('pre_approach_height_m')!=root_height
            or option.get('stationary_receiver_during_target') is not True):
        raise ValueError('SELECTED_ROUTE_GEOMETRY_OR_PROCESS_CHANGED')
    if option.get('finite_side_channel',False)!=route.get('finite_side_channel',False):
        raise ValueError('SELECTED_ROUTE_CHANNEL_PERMISSION_CHANGED')


def finite_side_exposure(connector,q):
    """Physical owned tool depth plus standoff and gap; no robot reachability claim."""
    from .moveit2_backend import native_tool_collision_boxes
    tools,ownership=native_tool_collision_boxes(connector,q)
    local=np.linalg.inv(connector.physical_contact_pose(q))
    depths={b.name:-float(b.transformed(local).corners()[:,2].min()) for b in tools}
    depth=max(depths.values());standoff=connector.budget.pregrasp_standoff_m
    gap=connector.collision_policy.free_space_clearance_m
    return dict(depth_m=depth+standoff+gap,tool_depth_m=depth,standoff_m=standoff,clearance_m=gap,
        tool_bodies=len(tools),ownership=ownership,body_depths_m=depths,
        scope='FACE_SCREEN_ONLY_NATIVE_FULL_COLLISION_STILL_REQUIRED')


def nonjunction_side_turn(payload,world):
    """Full carton projection beyond the +Y edge of the original inlet corridor."""
    from .lifted_reception import bounds,translated
    from .junction_reception import reception_classification
    by_name={b.name:b for b in world};belt=by_name['conveyor_transverse']
    release_projection=translated(payload,[belt.center[0]-payload.center[0],0,0])
    classification=reception_classification(release_projection,belt,by_name['conveyor_longitudinal'])
    edge=max(p[1] for p in classification['influence_polygon_world_xy_m'])
    minimum=edge+float(payload.center[1]-bounds(payload)[0][1])
    return dict(minimum_center_y_world_m=minimum,actual_center_y_world_m=float(payload.center[1]),
        projected_release_reception=classification,passed=not classification['junction'],
        scope='FULL_PAYLOAD_PROJECTION_AT_TRANSVERSE_RECEIVER_X')
