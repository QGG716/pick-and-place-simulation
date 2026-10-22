"""Translate the fixed robot installation without rebuilding the workcell."""
from copy import deepcopy
import numpy as np
from .camera_installation import installation_fingerprint
from ..moveit2_backend import digest


def translate_installation(scene, delta_m, target_id):
    delta = np.asarray(delta_m, float)
    if delta.shape != (3,) or not np.isfinite(delta).all():
        raise ValueError('BASE_TRANSLATION_INVALID')
    result = deepcopy(scene)
    mounts = [o for o in result['obstacles'] if o['id'] == 'robot_mount_plate']
    if len(mounts) != 1 or mounts[0]['owner'] != 'robot_mount_plate':
        raise ValueError('BASE_MOUNT_IDENTITY_INVALID')
    target = next(b for b in result['boxes'] if b['id'] == target_id)
    result['base_position_m'] = (np.asarray(scene['base_position_m']) + delta).tolist()
    mounts[0]['position_m'] = (np.asarray(mounts[0]['position_m']) + delta).tolist()
    # All mast/camera/tool solids are already local to robot links. Translating
    # their coordinates here would apply the remount twice.
    result['base_translation'] = dict(schema='eco65_base_translation_v1',
        source_scene_fingerprint=scene['scene_fingerprint'],delta_D_m=delta.tolist(),
        old_base_position_m=scene['base_position_m'],new_base_position_m=result['base_position_m'],
        translated_world_object_ids=['robot_mount_plate'],
        link_local_attachments_unchanged=True,cartons_and_conveyors_unchanged=True)
    camera = result['permanent_camera']
    camera['layout_remount']['base_translation'] = deepcopy(result['base_translation'])
    camera['calibration'].update(status='NOMINAL_FROM_CHANGED_INSTALLATION_NOT_CALIBRATED',
        prior_calibration_valid=False,installation_fingerprint=installation_fingerprint(camera))
    result['camera_extrinsics'] = deepcopy(camera['calibration'])
    result.pop('perception_binding', None)
    result['box'] = deepcopy(target)
    result['target_box_id'] = target_id
    result['native_task'].update(target_id=target_id,
        original_support_names=list(target['supported_by']),
        target_selection=dict(schema='eco65_explicit_exposed_target_v1',target_id=target_id,
            inventory_scope='UNCHANGED_TWELVE_CARTONS',source='USER_REQUEST'))
    result['scene_fingerprint'] = digest({k:v for k,v in result.items() if k!='scene_fingerprint'})
    return result
