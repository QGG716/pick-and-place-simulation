"""User-requested l05_c03 top/side trials, each at the same frozen root."""
from copy import deepcopy
import numpy as np
from .lifted_reception import StartHeight, directional_contact, receiving_region

TARGET='carton_l05_c03'
SCOPE='L05_C03_INDEPENDENT_FACE_TRIAL'
FACES={'top':'z+','side_positive_y':'y+','side_negative_y':'y-'}


def permission(frozen,face,initial_height=1.45,receive_mode='Hmax'):
    if face not in FACES:raise ValueError('L05_REQUESTED_FACE_UNKNOWN')
    e=frozen.evidence();h=e['heights'][TARGET]
    if e['original_carton_count']!=40 or abs(h['z_b_m']-1.5)>1e-9 or abs(h['h_m']-.3)>1e-9:
        raise ValueError('L05_FACE_FROZEN_POPULATION_CHANGED')
    if initial_height not in (.25,1.45) or (face=='top' and initial_height!=1.45):
        raise ValueError('L05_FACE_INITIAL_HEIGHT_OUTSIDE_SAVED_ROOTS')
    if receive_mode not in ('Hmax','Hmin') or (receive_mode=='Hmin' and (face!='side_negative_y' or initial_height!=.25)):
        raise ValueError('L05_HMIN_REQUESTED_NEGATIVE_Y_TERMINAL_ONLY')
    result=dict(scope=SCOPE,target_id=TARGET,requested_face=face,geometric_face_id=FACES[face],
        start_box_identity=h['start_box_identity'],original_cartons_sha256=e['original_cartons_sha256'],
        height_m=h['Hmax_world_m'],initial_height_m=initial_height,
        one_centred_pose_per_face=True,all_joint_rate_limits_retained=True)
    if receive_mode=='Hmin':
        result.update(receive_mode='Hmin',height_m=h['Hmin_world_m'],
            synchronization='PREGRASP_OR_CONTACT',approach_before_receiver_raise=True)
    return result


def option_for(target,belt,longitudinal,frozen,root_height,grant):
    if grant!=permission(frozen,grant['requested_face'],root_height,grant.get('receive_mode','Hmax')) or target.name!=TARGET:
        raise ValueError('L05_FACE_PERMISSION_CHANGED')
    h=StartHeight.capture(target)
    if h.box_identity!=grant['start_box_identity'] or root_height!=grant['initial_height_m']:
        raise ValueError('L05_FACE_ROOT_OR_TARGET_CHANGED')
    layer=None
    if grant['requested_face']=='top':
        from .low_top_receiver import selected_layer_top_permission,layer_top_option
        layer=selected_layer_top_permission(frozen,TARGET)
        option=layer_top_option(target,belt,longitudinal,frozen,root_height,layer)
    else:
        sign=1. if grant['geometric_face_id']=='y+' else -1.
        option=dict(template='L05_SELECTED_SIDE_HMAX',face=directional_contact(target,[0.,sign,0.]),
            height_m=h.hmax,pre_approach_height_m=h.hmax,synchronization='ALREADY_AT_RECEIVE',
            stationary_receiver_during_target=True,requires_open_channel=True,
            segment_directions_world=[[0.,sign,0.],[-1.,0.,0.]],region=receiving_region(target,belt),
            endpoint='REGION_NOT_FIXED_POINT',acceptance='NOT_PLANNED',
            spatial_orientation='UPSTREAM_FACE_POSE_HELD',fixed_wrist_joint_values=False,
            target_id=target.name,original_role=frozen.evidence()['roles'][target.name]['role'],
            height_source='FROZEN_INITIAL_HMAX',third_sync_disabled=True)
        if grant.get('receive_mode')=='Hmin':
            option.update(template='L05_SELECTED_SIDE_HMIN',height_m=h.hmin,pre_approach_height_m=root_height,
                synchronization='PREGRASP_OR_CONTACT',stationary_receiver_during_target=False,
                height_source='FROZEN_INITIAL_HMIN')
    option['l05_face_permission']=deepcopy(grant)
    return option,layer


def validate_option(target,option,grant,root_height):
    h=StartHeight.capture(target.box)
    face=grant.get('geometric_face_id') if grant else None
    sign=1. if face=='y+' else -1.
    top=face=='z+'
    hmin=grant is not None and grant.get('receive_mode')=='Hmin'
    receive=h.hmin if hmin else h.hmax
    temporary=root_height if hmin else receive
    expected_legs=[[0.,0.,1.],[-1.,0.,0.]] if top else [[0.,sign,0.],[-1.,0.,0.]]
    expected_pose=directional_contact(target.box,[0.,0.,1.] if top else [0.,sign,0.],
        **({'in_plane_x':[1.,0.,0.]} if top else {}))
    if (grant is None or grant.get('scope')!=SCOPE or grant.get('target_id')!=TARGET
            or target.target_id!=TARGET or h.box_identity!=grant.get('start_box_identity')
            or FACES.get(grant.get('requested_face'))!=face or target.geometric_face_id!=face
            or option.get('l05_face_permission')!=grant or option['face']!=expected_pose
            or option['template']!=('TOP_FIXED_CURRENT_RECEIVER' if top else 'L05_SELECTED_SIDE_HMIN' if hmin else 'L05_SELECTED_SIDE_HMAX')
            or option['segment_directions_world']!=expected_legs
            or option['height_m']!=receive or option['pre_approach_height_m']!=temporary
            or root_height!=grant['initial_height_m'] or grant['height_m']!=receive
            or option['synchronization']!=('PREGRASP_OR_CONTACT' if hmin else 'ALREADY_AT_RECEIVE')
            or option['stationary_receiver_during_target']!=(not hmin)
            or (hmin and (face!='y-' or root_height!=.25 or not grant.get('approach_before_receiver_raise')))
            or target.world_y_offset_m!=0 or target.eccentric_authority is not None
            or not np.allclose(target.contact_pose,option['face']['contact_pose'],atol=1e-12,rtol=0)):
        raise ValueError('L05_FACE_ACTUAL_CANDIDATE_CHANGED')


def validate_binding(option,root,binding):
    if 'l05_face_permission' not in option:return
    grant=option['l05_face_permission'];pre=binding.get('pre_approach_transition')
    hmin=grant.get('receive_mode')=='Hmin'
    if (root['height_m']!=grant['initial_height_m'] or not pre or pre['target_id']!=TARGET
            or pre['samples'][0]['height_m']!=root['height_m']
            or pre['samples'][-1]['height_m']!=(root['height_m'] if hmin else grant['height_m'])
            or pre['samples'][-1]['world']!=binding['samples'][0]['world']
            or binding['samples'][-1]['height_m']!=grant['height_m']
            or (not hmin and binding['samples'][0]['world']!=binding['samples'][-1]['world'])
            or (hmin and pre['samples'][0]['world']!=pre['samples'][-1]['world'])
            or binding['maximum_height_step_m']>.010000001 or binding['continuous_swept_proof']
            or pre['maximum_height_step_m']>.010000001 or pre['continuous_swept_proof']):
        raise ValueError('L05_FACE_CHECKED_RECEIVER_TRANSITION_CHANGED')
