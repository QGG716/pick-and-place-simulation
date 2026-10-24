"""The explicitly requested bottom c03 world-Z quarter turns, at face centre."""
from dataclasses import replace
from copy import deepcopy
import numpy as np
from .lifted_reception import StartHeight, directional_contact

SCOPE='XY_TOP_CURRENT_BOTTOM_ROTATED_C03_C04'
ORDER=('carton_l00_c03','carton_l01_c04','carton_l00_c04')
RETAINED=('carton_l00_c00',)


def rotation_permission(frozen):
    evidence=frozen.evidence();name=ORDER[0]
    height=evidence['heights'][name]
    if abs(height['z_b_m'])>1e-9 or abs(height['h_m']-.3)>1e-9:
        raise ValueError('ROTATED_TOP_FROZEN_LAYER_CHANGED')
    return dict(target_id=name,start_box_identity=height['start_box_identity'],
        original_cartons_sha256=evidence['original_cartons_sha256'],
        world_z_degrees=[90,-90],face_id='z+',contact_offset_m=[0.,0.,0.])


def rotated_option(option, target, permission, degrees):
    if (target.name!=permission['target_id'] or degrees not in permission['world_z_degrees']
            or degrees not in (90,-90) or option['template']!='TOP_FIXED_CURRENT_RECEIVER'
            or StartHeight.capture(target).box_identity!=permission['start_box_identity']):
        raise ValueError('ROTATED_TOP_PERMISSION_CHANGED')
    result=deepcopy(option)
    result.update(world_z_rotation_degrees=degrees,rotation_permission=deepcopy(permission),
        face=directional_contact(target,[0.,0.,1.],in_plane_x=[0.,float(degrees/90),0.]))
    return result


def rotated_target(target, option, permission):
    if option.get('rotation_permission')!=permission or permission is None:
        raise ValueError('ROTATED_TOP_AUTHORITY_MISSING')
    expected=rotated_option(option,target.box,permission,option['world_z_rotation_degrees'])
    if option['face']!=expected['face'] or target.geometric_face_id!='z+':
        raise ValueError('ROTATED_TOP_FRAME_CHANGED')
    contact=np.asarray(option['face']['contact_pose'],dtype=float)
    approach=contact.copy();approach[:3,3]+=target.standoff_m*target.outward_normal
    return replace(target,contact_pose=contact,approach_pose=approach,
        in_plane_x=contact[:3,0].copy(),source='SIM_GROUND_TRUTH_USER_WORLD_Z_QUARTER_TURN').validate()


def validate_rotation(target,option,permission):
    expected=rotated_target(target,option,permission)
    if (not np.allclose(target.contact_pose,expected.contact_pose,atol=1e-12,rtol=0)
            or not np.allclose(target.approach_pose,expected.approach_pose,atol=1e-12,rtol=0)):
        raise ValueError('ROTATED_TOP_ACTUAL_TARGET_CHANGED')
