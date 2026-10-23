"""Three explicitly requested l01 routes, bound to frozen carton identities."""
from copy import deepcopy
from .lifted_reception import StartHeight, directional_contact, receiving_region

SCOPE = 'XY_TOP_L01_SELECTED_SIDE_TOP_HMAX'
SIDE_TEMPLATE = 'SELECTED_SIDE_POS_Y_HMAX'
TOP_TEMPLATE = 'SELECTED_TOP_HMAX'
ORDER = ('carton_l01_c01', 'carton_l01_c00', 'carton_l01_c04')
TOP_TARGET = 'carton_l01_c04'


def selected_routes_permission(frozen):
    evidence=frozen.evidence();routes={}
    if evidence['original_carton_count']!=40:
        raise ValueError('SELECTED_ROUTES_ORIGINAL_POPULATION_CHANGED')
    for name in ORDER:
        h=evidence['heights'][name]
        if abs(h['z_b_m']-.3)>1e-9 or abs(h['h_m']-.3)>1e-9 or abs(h['Hmax_world_m']-.25)>1e-9:
            raise ValueError('SELECTED_ROUTES_ORIGINAL_LAYER_CHANGED')
        top=name==TOP_TARGET
        routes[name]=dict(start_box_identity=h['start_box_identity'],height_m=h['Hmax_world_m'],
            template=TOP_TEMPLATE if top else SIDE_TEMPLATE,geometric_face_id='z+' if top else 'y+',
            segment_directions_world=[[0.,0.,1.] if top else [0.,1.,0.],[-1.,0.,0.]],
            original_role=evidence['roles'][name]['role'])
    return dict(original_cartons_sha256=evidence['original_cartons_sha256'],routes=routes,
        dispatch_order=list(ORDER),height_source='FROZEN_INITIAL_HMAX',stationary_receiver=True,
        authorization='USER_SELECTED_THREE_CARTONS_ONLY',planning_only=True)


def selected_route_option(target,belt,longitudinal,frozen,current_height,permission):
    if permission!=selected_routes_permission(frozen):
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
        option=dict(face=directional_contact(target,[0.,1.,0.]),height_m=current_height,
            pre_approach_height_m=current_height,synchronization='ALREADY_AT_RECEIVE',
            stationary_receiver_during_target=True,requires_open_channel=True,
            region=receiving_region(target,belt),endpoint='REGION_NOT_FIXED_POINT',
            acceptance='NOT_PLANNED',spatial_orientation='UPSTREAM_FACE_POSE_HELD',fixed_wrist_joint_values=False)
    option.update(template=route['template'],target_id=target.name,original_role=route['original_role'],
        segment_directions_world=deepcopy(route['segment_directions_world']),
        height_source='FROZEN_INITIAL_HMAX',selected_routes_permission=deepcopy(permission))
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
