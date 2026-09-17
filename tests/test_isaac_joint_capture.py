"""CPU contract regression using explicitly synthetic capture records."""
import copy

import pytest

from unloading_perception import isaac_validation as contract
from ros2_ws.src.unloading_ros_bridge.test.isaac_joint_fixture import capture_fixture


def validate(manifest, binding):
    return contract.validate_capture_robot_state(binding.get('robot_state'), manifest,
        contract.IsaacCaptureBinding.from_dict(binding))


def test_capture_velocity_and_readback_position_are_preserved(tmp_path):
    manifest, _, binding = capture_fixture(tmp_path)
    state = validate(manifest, binding)
    assert tuple(state['velocity_rad_s']) == (.25, -.5)
    assert tuple(state['position_rad']) == (.125, -.225)
    assert tuple(state['position_rad']) != tuple(manifest.robot['q_rad'])
    assert state['sample_time'] == binding['simulation_time']
    assert state['synthetic_fixture'] is True


@pytest.mark.parametrize('patch', [
    {'velocity_rad_s': None}, {'velocity_rad_s': []}, {'velocity_rad_s': [0.]},
    {'position_rad': [0.]}, {'joint_names': ['fixture_a']},
    {'joint_names': ['fixture_a', 'fixture_a']}, {'joint_names': ['fixture_b', 'fixture_a']},
    {'position_rad': [float('nan'), 0.]}, {'velocity_rad_s': [0., float('inf')]},
    {'sample_time': float('nan')}, {'sample_time': 9.}, {'sample_time': 0.},
    {'frame_sequence': 8}, {'simulation_epoch': 'other'}, {'clock_domain': 'wall'},
    {'robot_asset_hash': 'f'*64}, {'robot_model_identity': 'other'},
    {'manifest_fingerprint': 'f'*64}, {'source': 'FILE_REPLAY'}, {'schema_version': 'unknown'},
])
def test_invalid_capture_state_is_rejected(tmp_path, patch):
    manifest, _, binding = capture_fixture(tmp_path)
    binding['robot_state'].update(patch)
    with pytest.raises((ValueError, TypeError)):
        validate(manifest, binding)


def test_history_and_static_flag_never_supply_missing_velocity(tmp_path):
    manifest, _, binding = capture_fixture(tmp_path)
    binding.pop('robot_state')
    with pytest.raises(ValueError, match='missing'):
        validate(manifest, binding)
    binding['robot_state'] = {'static': True}
    with pytest.raises(ValueError):
        validate(manifest, binding)


def test_static_readback_requires_capture_owned_hold_evidence(tmp_path):
    manifest, _, binding = capture_fixture(tmp_path, velocity=(0., 0.))
    state = binding['robot_state']
    state['source'] = 'KINEMATIC_HOLD_READBACK'
    with pytest.raises(ValueError, match='hold evidence'):
        validate(manifest, binding)
    state['kinematic_hold'] = {'render_without_physics_step': True,
        'position_command_rad': list(state['position_rad']), 'velocity_command_rad_s': [0., 0.]}
    assert validate(manifest, binding)['velocity_rad_s'] == [0., 0.]
    for patch in ({'render_without_physics_step': False}, {'position_command_rad': [9., 0.]},
                  {'velocity_command_rad_s': [1., 0.]}):
        invalid = copy.deepcopy(binding)
        invalid['robot_state']['kinematic_hold'].update(patch)
        with pytest.raises(ValueError): validate(manifest, invalid)
    state.pop('velocity_rad_s')
    with pytest.raises(ValueError): validate(manifest, binding)


def test_capture_writer_reorders_readback_and_hold_commands_together(tmp_path):
    manifest, _, binding = capture_fixture(tmp_path)
    record = contract.capture_robot_state_record(manifest, ['fixture_b', 'fixture_a'],
        [-.225, .125], [0., 0.], kinematic_hold={'render_without_physics_step': True,
        'position_command_rad': [-.225, .125], 'velocity_command_rad_s': [0., 0.]})
    binding['robot_state'] = record
    assert validate(manifest, binding)['position_rad'] == [.125, -.225]
    assert record['kinematic_hold']['position_command_rad'] == [.125, -.225]
    assert record['source'] == 'KINEMATIC_HOLD_READBACK'
    assert record['synthetic_fixture'] is False
    with pytest.raises(ValueError):
        contract.capture_robot_state_record(manifest, ['fixture_b', 'fixture_a'],
            [-.225, .125], [float('nan'), 0.], kinematic_hold=record['kinematic_hold'])
