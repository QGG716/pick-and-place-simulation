from copy import deepcopy
import numpy as np
import pytest
from test_eco65_native_12scene import synthetic_scene
from unloading_sim.eco65.base_translation import translate_installation
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.camera_installation import installation_fingerprint


def fixture_scene():
    _, scene = synthetic_scene()
    scene['base_position_m'] = [-.34, 0., .24]
    scene['obstacles'][0]['position_m'] = [-.34, 0., .12]
    scene['permanent_camera'] = dict(axis_rotation={},layout_remount={},
        calibration=dict(T_parent_child=np.eye(4).tolist()))
    return scene


def test_translate_exact_installation_preserves_world_and_local_attachments():
    before=fixture_scene(); original=deepcopy(before)
    after=translate_installation(before,[-.1,0,0],'carton_r00_l03_c02')
    verify_scene(after)
    assert before==original
    assert after['base_position_m']==pytest.approx([-.44,0,.24])
    assert after['obstacles'][0]['position_m']==pytest.approx([-.44,0,.12])
    assert after['obstacles'][1:]==before['obstacles'][1:]
    for key in ('boxes','native_world_objects','moving_attachments','mast','conveyors'):
        assert after[key]==before[key]
    assert len(after['boxes'])==12
    assert after['target_box_id']=='carton_r00_l03_c02'
    assert installation_fingerprint(after['permanent_camera'])!=installation_fingerprint(before['permanent_camera'])
    assert after['camera_extrinsics']['prior_calibration_valid'] is False


def test_explicit_target_cannot_select_covered_box():
    result=translate_installation(fixture_scene(),[-.1,0,0],'carton_r00_l02_c02')
    with pytest.raises(ValueError,match='EXPLICIT_TARGET_NOT_EXPOSED'):
        verify_scene(result)


def test_translation_does_not_guess_mount_ownership():
    scene=fixture_scene();scene['obstacles'][0]['owner']='other'
    with pytest.raises(ValueError,match='MOUNT_IDENTITY'):
        translate_installation(scene,[-.1,0,0],'carton_r00_l03_c02')
