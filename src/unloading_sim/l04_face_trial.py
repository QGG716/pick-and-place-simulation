"""Explicit l04_c03 top/side trials after the user's l05 removal assumption."""
from copy import deepcopy
import numpy as np
from .lifted_reception import StartHeight,directional_contact,receiving_region

TARGET='carton_l04_c03'
SCOPE='L04_C03_INDEPENDENT_FACE_TRIAL_AFTER_ASSUMED_L05_REMOVAL'
FACES={'top':'z+','side_positive_y':'y+','side_negative_y':'y-'}


def permission(frozen,face,initial_height=.25):
    e=frozen.evidence();h=e['heights'][TARGET]
    if (face not in FACES or initial_height!=.25 or e['original_carton_count']!=40
            or abs(h['z_b_m']-1.2)>1e-9 or abs(h['h_m']-.3)>1e-9):
        raise ValueError('L04_FACE_FROZEN_ROOT_CHANGED')
    return dict(scope=SCOPE,target_id=TARGET,requested_face=face,geometric_face_id=FACES[face],
        start_box_identity=h['start_box_identity'],original_cartons_sha256=e['original_cartons_sha256'],
        initial_height_m=.25,height_m=h['Hmax_world_m'],receive_mode='Hmax',
        synchronization='PREGRASP' if face=='top' else 'PREGRASP_OR_CONTACT',
        assumed_removed_target_ids=['carton_l05_c03'],one_centred_pose_per_face=True,
        all_joint_rate_limits_retained=True,approach_before_receiver_raise=True)


def option_for(target,belt,longitudinal,frozen,root_height,grant):
    if grant!=permission(frozen,grant['requested_face'],root_height) or target.name!=TARGET:
        raise ValueError('L04_FACE_PERMISSION_CHANGED')
    h=StartHeight.capture(target);layer=None
    if h.box_identity!=grant['start_box_identity']:raise ValueError('L04_FACE_TARGET_CHANGED')
    if grant['requested_face']=='top':
        from .low_top_receiver import selected_layer_top_permission,layer_top_option
        layer=selected_layer_top_permission(frozen,TARGET)
        option=layer_top_option(target,belt,longitudinal,frozen,h.hmax,layer)
        option.update(receiver_height_source='CHECKED_PREGRASP_HEIGHT_FIXED_FROM_CONTACT_ONWARD')
    else:
        sign=1. if grant['geometric_face_id']=='y+' else -1.
        option=dict(template='L04_SELECTED_SIDE_HMAX',face=directional_contact(target,[0.,sign,0.]),
            height_m=h.hmax,requires_open_channel=True,segment_directions_world=[[0.,sign,0.],[-1.,0.,0.]],
            region=receiving_region(target,belt),endpoint='REGION_NOT_FIXED_POINT',acceptance='NOT_PLANNED',
            spatial_orientation='UPSTREAM_FACE_POSE_HELD',fixed_wrist_joint_values=False,
            target_id=TARGET,original_role=frozen.evidence()['roles'][TARGET]['role'],
            height_source='FROZEN_INITIAL_HMAX',third_sync_disabled=True)
    option.update(pre_approach_height_m=root_height,synchronization=grant['synchronization'],
        stationary_receiver_during_target=False,l04_face_permission=deepcopy(grant))
    return option,layer


def validate_option(target,option,grant,root_height):
    if grant is None:raise ValueError('L04_FACE_PERMISSION_MISSING')
    h=StartHeight.capture(target.box);face=grant['geometric_face_id'];top=face=='z+'
    sign=1. if face=='y+' else -1.
    pose=directional_contact(target.box,[0.,0.,1.] if top else [0.,sign,0.],**({'in_plane_x':[1.,0.,0.]} if top else {}))
    if (grant['scope']!=SCOPE or grant['target_id']!=TARGET or target.target_id!=TARGET
            or h.box_identity!=grant['start_box_identity'] or FACES.get(grant['requested_face'])!=face
            or target.geometric_face_id!=face or option.get('l04_face_permission')!=grant
            or option['face']!=pose or not np.allclose(target.contact_pose,pose['contact_pose'],atol=1e-12,rtol=0)
            or option['height_m']!=h.hmax or grant['height_m']!=h.hmax
            or root_height!=.25 or grant['initial_height_m']!=root_height or option['pre_approach_height_m']!=root_height
            or option['synchronization']!=('PREGRASP' if top else 'PREGRASP_OR_CONTACT')
            or option['stationary_receiver_during_target']
            or option['template']!=('TOP_FIXED_CURRENT_RECEIVER' if top else 'L04_SELECTED_SIDE_HMAX')
            or option['segment_directions_world']!=([[0.,0.,1.],[-1.,0.,0.]] if top else [[0.,sign,0.],[-1.,0.,0.]])
            or target.world_y_offset_m!=0 or target.eccentric_authority is not None
            or grant['assumed_removed_target_ids']!=['carton_l05_c03']
            or (top and abs(option['release_bottom_world_m']-1.25)>1e-9)):
        raise ValueError('L04_FACE_ACTUAL_CANDIDATE_CHANGED')


def validate_binding(option,root,binding):
    if 'l04_face_permission' not in option:return
    pre=binding.get('pre_approach_transition');grant=option['l04_face_permission']
    if (root.get('assumed_removed')!=['carton_l05_c03'] or root['height_m']!=.25
            or not pre or pre['target_id']!=TARGET or binding['target_id']!=TARGET
            or pre['samples'][0]['height_m']!=.25 or pre['samples'][-1]['height_m']!=.25
            or pre['samples'][0]['world']!=pre['samples'][-1]['world']
            or pre['samples'][-1]['world']!=binding['samples'][0]['world']
            or binding['samples'][-1]['height_m']!=grant['height_m']
            or pre['maximum_height_step_m']>.010000001 or pre['continuous_swept_proof']
            or binding['maximum_height_step_m']>.010000001 or binding['continuous_swept_proof']):
        raise ValueError('L04_FACE_RECEIVER_BINDING_CHANGED')
