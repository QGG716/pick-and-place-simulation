"""Explicit frozen-l01 Hmax permission; the three requested face trials only."""
from copy import deepcopy
import json
import numpy as np

from .lifted_reception import StartHeight
from .moveit2_backend import box_message, digest
from .coupled_batch import message_box

SCOPE = 'XY_TOP_L01_ALL_HMAX_THREE_TARGETS'
TOP_TEMPLATE = 'L01_HMAX_TOP'
ORDER = ('carton_l01_c01', 'carton_l01_c00', 'carton_l01_c04')
FACES = {ORDER[0]: ['x-'], ORDER[1]: ['x-', 'y+'], ORDER[2]: ['x-', 'z+']}
SCHEMA = 'frozen_l01_hmax_v1'
C04_SIDE_SCOPE = 'XY_TOP_L01_C04_SIDE_NEG_Y_HMAX'
C04_FACE_SCOPE = 'XY_TOP_L01_C04_FACE_EXTENSION200_HMAX'
C04_FACE_IDS = {'top':'z+', 'side':'y-', 'front':'x-'}


def c04_face_permission(frozen, face):
    if face not in C04_FACE_IDS:
        raise ValueError('C04_FACE_NOT_REQUESTED')
    permission=c04_side_permission(frozen)
    permission.update(scope=C04_FACE_SCOPE, requested_face=face,
        geometric_face_id=C04_FACE_IDS[face], tool_connector_extension_m=.2,
        comparison_mode='INDEPENDENT_SAME_ACCEPTED_34_BOX_ROOT',
        segment_directions_world=([[0.,0.,1.],[-1.,0.,0.]] if face=='top' else
                                  [[0.,-1.,0.],[-1.,0.,0.]] if face=='side' else [[-1.,0.,0.]]))
    return permission


def c04_side_permission(frozen):
    height = layer_permission(frozen)
    evidence = frozen.evidence();name = ORDER[2]
    if evidence['roles'][name]['role']!='LEFT_END':
        raise ValueError('C04_SIDE_ORIGINAL_ROLE_CHANGED')
    return dict(scope=C04_SIDE_SCOPE, target_id=name, dispatch_order=[name],
        original_cartons_sha256=height['original_cartons_sha256'],
        start_box_identity=evidence['heights'][name]['start_box_identity'],
        original_role='LEFT_END', geometric_face_id='y-',
        segment_directions_world=[[0.,-1.,0.],[-1.,0.,0.]], planning_only=True)


def layer_permission(frozen):
    original = json.loads(frozen.original_cartons_json)
    heights = {b['id']: StartHeight.capture(message_box(b)) for b in original}
    anchor = heights[ORDER[0]]
    allowed = sorted(n for n, h in heights.items()
                     if abs(h.z_b-anchor.z_b)<1e-9 and abs(h.h-anchor.h)<1e-9)
    if (len(original)!=40 or len(heights)!=40 or len(allowed)!=5
            or abs(anchor.z_b-.3)>1e-9 or abs(anchor.h-.3)>1e-9
            or abs(anchor.hmax-.25)>1e-9 or not set(ORDER)<=set(allowed)
            or digest(original)!=frozen.evidence()['original_cartons_sha256']):
        raise ValueError('L01_HMAX_FROZEN_LAYER_CHANGED')
    return dict(schema=SCHEMA, original_cartons=original,
        original_cartons_sha256=digest(original), anchor_id=ORDER[0],
        allowed_target_ids=allowed, dispatch_order=list(ORDER), faces=deepcopy(FACES),
        height_source='FROZEN_INITIAL_HMAX', stationary_receiver=True, planning_only=True)


def validate_height_permission(original, permission, height):
    # Re-derive the scope from the entire frozen original population. Neither a
    # name prefix nor the geometry of the remaining cartons can grant access.
    from .carton_population import FrozenCartonPopulation
    if permission is None:
        raise ValueError('L01_HMAX_PERMISSION_MISSING')
    frozen = FrozenCartonPopulation.capture(tuple(message_box(b) for b in permission['original_cartons']))
    if permission != layer_permission(frozen):
        raise ValueError('L01_HMAX_PERMISSION_CHANGED')
    originals = {b['id']: b for b in permission['original_cartons']}
    if (original.name not in permission['allowed_target_ids']
            or box_message(original)!=originals[original.name]
            or not np.isfinite(height) or abs(height-StartHeight.capture(original).hmax)>1e-8):
        raise ValueError('L01_HMAX_TARGET_OR_HEIGHT_CHANGED')


def options(target, belt, longitudinal, frozen, current_height, permission, *, c04_side=None, c04_face=None):
    if permission!=layer_permission(frozen):
        raise ValueError('L01_HMAX_PERMISSION_CHANGED')
    validate_height_permission(target, permission, current_height)
    if target.name not in FACES:
        raise ValueError('L01_HMAX_TARGET_OUTSIDE_REQUESTED_TRIALS')
    if c04_side is not None and (c04_side!=c04_side_permission(frozen) or target.name!=c04_side['target_id']):
        raise ValueError('C04_SIDE_PERMISSION_CHANGED')
    if c04_face is not None and (c04_side is not None or c04_face!=c04_face_permission(frozen,c04_face.get('requested_face'))
                                or target.name!=c04_face['target_id']):
        raise ValueError('C04_FACE_PERMISSION_CHANGED')
    from .top_layer_lifted import process_options
    from .low_top_receiver import _fixed_top_option
    standard = process_options(target, belt, frozen, (target,), longitudinal=longitudinal)
    result = []
    for face in ([c04_face['geometric_face_id']] if c04_face is not None else ['y-'] if c04_side is not None else FACES[target.name]):
        if face=='z+':
            option = _fixed_top_option(target, belt, longitudinal, frozen, current_height,
                                       StartHeight.capture(target), SCHEMA)
            option['template'] = TOP_TEMPLATE
        else:
            option = deepcopy(next(o for o in standard if o['face']['geometric_face_id']==face))
            option['template'] = 'L01_HMAX_'+('FRONT_DIRECT' if len(option['segment_directions_world'])==1
                                            else 'FRONT_LATERAL' if face=='x-' else 'SIDE')
        option.update(height_m=current_height, pre_approach_height_m=current_height,
            synchronization='ALREADY_AT_RECEIVE', stationary_receiver_during_target=True,
            height_source='FROZEN_INITIAL_HMAX', third_sync_disabled=True,
            l01_hmax_permission=deepcopy(permission))
        if c04_side is not None:option['c04_side_permission']=deepcopy(c04_side)
        if c04_face is not None:option['c04_face_permission']=deepcopy(c04_face)
        if target.name==ORDER[0] and option['segment_directions_world']!=[[-1.,0.,0.]]:
            raise ValueError('L01_C01_DIRECT_X_GEOMETRY_CHANGED')
        result.append(option)
    return result


def validate_option(target, option, permission, root_height, *, c04_side=None, c04_face=None):
    validate_height_permission(target.box, permission, root_height)
    face = target.geometric_face_id
    if c04_side is not None:
        from .carton_population import FrozenCartonPopulation
        frozen=FrozenCartonPopulation.capture(tuple(message_box(b) for b in permission['original_cartons']))
        if (c04_side!=c04_side_permission(frozen) or option.get('c04_side_permission')!=c04_side
                or target.target_id!=c04_side['target_id']):
            raise ValueError('C04_SIDE_PERMISSION_CHANGED')
    elif option.get('c04_side_permission') is not None:
        raise ValueError('C04_SIDE_PERMISSION_MISSING')
    if c04_face is not None:
        from .carton_population import FrozenCartonPopulation
        frozen=FrozenCartonPopulation.capture(tuple(message_box(b) for b in permission['original_cartons']))
        if (c04_side is not None or c04_face!=c04_face_permission(frozen,c04_face.get('requested_face'))
                or option.get('c04_face_permission')!=c04_face or target.target_id!=c04_face['target_id']):
            raise ValueError('C04_FACE_PERMISSION_CHANGED')
    elif option.get('c04_face_permission') is not None:
        raise ValueError('C04_FACE_PERMISSION_MISSING')
    if (option.get('l01_hmax_permission')!=permission or target.target_id not in FACES
            or face not in ([c04_face['geometric_face_id']] if c04_face is not None else ['y-'] if c04_side is not None else FACES[target.target_id]) or option['face']['geometric_face_id']!=face
            or target.world_y_offset_m!=0 or target.eccentric_authority is not None
            or option['height_m']!=root_height or option.get('pre_approach_height_m')!=root_height
            or option.get('stationary_receiver_during_target') is not True
            or option.get('synchronization')!='ALREADY_AT_RECEIVE'):
        raise ValueError('L01_HMAX_ROUTE_CHANGED')
    legs=option['segment_directions_world']
    expected=(c04_face['segment_directions_world'] if c04_face is not None else
              [[0.,-1.,0.],[-1.,0.,0.]] if c04_side is not None else
              [[0.,0.,1.],[-1.,0.,0.]] if face=='z+' else
              [[0.,1.,0.],[-1.,0.,0.]] if face=='y+' else [[-1.,0.,0.]])
    if legs!=expected:
        raise ValueError('L01_HMAX_ROUTE_CHANGED')
