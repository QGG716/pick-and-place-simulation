"""Model-adapter contracts. Synthetic fixtures are NOT evidence of S50 accuracy."""
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from unloading_sim.robot import URDFRobot
from unloading_sim.robot_model_binding import ModelNotReady, RobotModelBinding
from unloading_sim.robot_native_binding import native_model_from_binding, configured_wrist_tool_pairs

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def fixture_config(tmp_path):
    """A small analytical six-axis fixture, deliberately not a vendor robot."""
    xml = ET.Element('robot', name='analytical_fixture_not_s50')
    ET.SubElement(xml, 'link', name='mount_base')
    parent = 'mount_base'
    for index, axis in enumerate(['0 0 1', '0 1 0', '0 1 0', '1 0 0', '0 1 0', '1 0 0'], 1):
        child = 'segment_'+str(index)
        ET.SubElement(xml, 'link', name=child)
        joint = ET.SubElement(xml, 'joint', name='axis_'+str(index), type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz='0.1 0 0', rpy='0 0 0')
        ET.SubElement(joint, 'axis', xyz=axis)
        # Intentionally no effort or inertial values: FK/collision are usable.
        ET.SubElement(joint, 'limit', lower='-2', upper='2', velocity='1.5')
        parent = child
    ET.SubElement(xml, 'link', name='physical_tool_face')
    joint = ET.SubElement(xml, 'joint', name='fixed_tool_face', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link='physical_tool_face')
    (tmp_path/'fixture.urdf').write_text(ET.tostring(xml, encoding='unicode'))
    (tmp_path/'fixture.srdf').write_text('<robot name="analytical_fixture_not_s50"><group name="manipulator"><chain base_link="mount_base" tip_link="physical_tool_face"/></group></robot>')
    data = dict(model_id='analytical_fixture_not_s50', kinematics=dict(urdf_path='fixture.urdf',
        srdf_path='fixture.srdf', active_joint_names=['axis_'+str(i) for i in range(1, 7)],
        base_link='mount_base', flange_link='physical_tool_face', tip_link='physical_tool_face',
        mounting_from_base=np.eye(4).tolist()), collision=dict(owned_tool_exempt_robot_links=[]))
    path = tmp_path/'fixture.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def change_config(path, update):
    data = yaml.safe_load(path.read_text())
    update(data)
    path.write_text(yaml.safe_dump(data))


def test_missing_model_has_machine_readable_requirements(tmp_path):
    path = tmp_path/'unready.yaml'
    path.write_text('model: unavailable_test_model\nkinematics: {}\nlink_inertials: null\njoint_effort_limits_nm: null\n')
    with pytest.raises(ModelNotReady) as error:
        RobotModelBinding.from_config(path)
    assert 'kinematics.urdf_path' in error.value.missing
    assert 'kinematics.mounting_from_base' in error.value.errors
    assert not any('inertial' in x or 'effort' in x for x in error.value.errors)


def test_factory_mount_and_literal_axis_direction(fixture_config):
    mount = np.eye(4); mount[:3, 3] = [-1.85, .15, .9]
    adapter = np.eye(4); adapter[:3, 3] = [.01, .02, .03]
    change_config(fixture_config, lambda data: data['kinematics'].update(mounting_from_base=adapter.tolist()))
    robot = URDFRobot.from_config(fixture_config, mounting_world_transform=mount)
    np.testing.assert_allclose(robot.fk(np.zeros(6))[:3, 3], [-1.24, .17, .93], atol=1e-12)
    q = np.zeros(6); q[0] = np.pi/2
    np.testing.assert_allclose(robot.fk(q)[:3, 3], [-1.74, .67, .93], atol=1e-12)
    q[0] = -np.pi/2
    np.testing.assert_allclose(robot.fk(q)[:3, 3], [-1.74, -.33, .93], atol=1e-12)
    assert robot.self_collision_exclusions == set()
    assert robot.within_limits(np.full(6, 1.9))
    assert not robot.within_limits(np.full(6, 2.1))
    with pytest.raises(ValueError, match='CAPSULE_PROXY_UNAVAILABLE'):
        robot.link_capsules(np.zeros(6))


def test_installed_tool_requires_explicit_transform(fixture_config):
    with pytest.raises(ValueError, match='INSTALLED_TOOL_REQUIRES_EXPLICIT'):
        URDFRobot.from_config(fixture_config, tool_collision_size=[.1, .2, .3])
    tool = np.eye(4); tool[2, 3] = .23
    robot = URDFRobot.from_config(fixture_config, tip_from_tcp=tool, tool_collision_size=[.1, .2, .3])
    np.testing.assert_allclose(robot.fk(np.zeros(6))[:3, 3], [.6, 0, .23], atol=1e-12)


def test_wrong_joint_order_rejected(fixture_config):
    change_config(fixture_config, lambda data: data['kinematics']['active_joint_names'].reverse())
    with pytest.raises(ModelNotReady, match='ACTIVE_JOINT_ORDER'):
        RobotModelBinding.from_config(fixture_config)


@pytest.mark.parametrize('change, expected', [
    (lambda xml: xml.find("joint[@name='axis_2']/limit").set('velocity', '0'), 'URDF_VELOCITY'),
    (lambda xml: xml.find("joint[@name='axis_3']/axis").set('xyz', '0 0 0'), 'URDF_AXIS'),
    (lambda xml: xml.find("joint[@name='axis_3']/axis").set('xyz', '0 0 1 1'), 'URDF_AXIS'),
])
def test_invalid_joint_definition_rejected(fixture_config, change, expected):
    urdf = fixture_config.with_name('fixture.urdf'); xml = ET.parse(urdf).getroot()
    change(xml); urdf.write_text(ET.tostring(xml, encoding='unicode'))
    with pytest.raises(ModelNotReady, match=expected):
        RobotModelBinding.from_config(fixture_config)


def test_limit_source_mismatch_rejected(fixture_config):
    change_config(fixture_config, lambda data: data.update(joint_velocity_limits_rad_s=[9.]*6))
    with pytest.raises(ModelNotReady, match='CONFIG_URDF_VELOCITY'):
        RobotModelBinding.from_config(fixture_config)


def test_official_fanuc_loader_regression():
    config = ROOT/'configs/robots/fanuc_m710id_70.yaml'
    binding = RobotModelBinding.from_config(config)
    configured = URDFRobot.from_config(config)
    legacy = URDFRobot.fanuc_m710id_70(binding.urdf_path, base_position=[0., 0., 0.], tool_length=0.)
    for q in binding.fk_probe_configurations():
        assert configured.within_limits(q)
        np.testing.assert_array_equal(configured.fk(q), legacy.fk(q))
        for name, frame in legacy.named_link_frames(q).items():
            np.testing.assert_array_equal(configured.named_link_frames(q)[name], frame)
    assert legacy.self_collision_exclusions  # Historical factory semantics retained.


def test_native_uses_actual_model_and_zero_length_flange_alias(fixture_config, tmp_path):
    binding = RobotModelBinding.from_config(fixture_config)
    mount = np.eye(4); mount[:3, 3] = [-1.85, .15, .9]
    xml, semantic, roles = native_model_from_binding(binding, mount, project_root=tmp_path)
    alias = xml.find("joint[@name='configured_physical_flange_alias']")
    assert alias.find('parent').get('link') == 'physical_tool_face'
    assert alias.find('origin').get('xyz') == '0.0 0.0 0.0'
    assert alias.find('origin').get('rpy') == '0.0 -0.0 0.0'
    assert xml.find("link[@name='flange']/collision") is None
    assert not roles['physical_adapter_added']
    assert semantic.find('group/chain').get('base_link') == 'mount_base'
    exported = tmp_path/'native.urdf'; exported.write_text(ET.tostring(xml, encoding='unicode'))
    native_definition = URDFRobot.from_urdf(exported, binding.active_joint_names, 'world', 'flange', tool_length=0.)
    reference = URDFRobot.from_config(fixture_config, mounting_world_transform=mount)
    for q in binding.fk_probe_configurations():
        np.testing.assert_allclose(native_definition.fk(q), reference.fk(q), atol=1e-12)


def test_native_alias_conflict_fails_closed(fixture_config, tmp_path):
    path = fixture_config.with_name('fixture.urdf'); xml = ET.parse(path).getroot()
    ET.SubElement(xml, 'link', name='flange')
    alias = ET.SubElement(xml, 'joint', name='existing_other_flange', type='fixed')
    ET.SubElement(alias, 'parent', link='mount_base'); ET.SubElement(alias, 'child', link='flange')
    path.write_text(ET.tostring(xml, encoding='unicode'))
    binding = RobotModelBinding.from_config(fixture_config)
    with pytest.raises(ModelNotReady, match='FLANGE_ALIAS_NAME_CONFLICT'):
        native_model_from_binding(binding, np.eye(4), project_root=tmp_path)


def test_owned_wrist_pairs_require_explicit_source_and_policy(fixture_config):
    policy = SimpleNamespace(wrist_tool_pairs=lambda tools: {('J5_link', name) for name in tools})
    binding = RobotModelBinding.from_config(fixture_config)
    with pytest.raises(ModelNotReady, match='OWNERSHIP_MISMATCH'):
        configured_wrist_tool_pairs(binding, policy, ['tool'])
    policy.wrist_tool_pairs = lambda _: set()
    assert configured_wrist_tool_pairs(binding, policy, ['tool']) == set()
    change_config(fixture_config, lambda data: data['collision'].update(owned_tool_exempt_robot_links=['segment_6']))
    with pytest.raises(ModelNotReady, match='owned_tool_exemptions_source'):
        configured_wrist_tool_pairs(RobotModelBinding.from_config(fixture_config), policy, ['tool'])


def test_model_request_explicit_binding_no_fanuc_assets(fixture_config, tmp_path, monkeypatch):
    from unloading_sim import moveit2_backend, independent_cups
    robot = URDFRobot.from_config(fixture_config)
    policy = SimpleNamespace(wrist_tool_pairs=lambda _: set(), to_mapping=lambda: {})
    connector = SimpleNamespace(robot=robot, collision_policy=policy,
        robot_state_validator=SimpleNamespace(_compliant_boxes=lambda _: []), validator_identity='fixture_only',
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4))
    scene = SimpleNamespace(policy=SimpleNamespace(project_root=tmp_path, policy_fingerprint='test', data={'suction': {}}),
        snapshot={'scene_fingerprint': 'fixture_scene', 'schema': 'fixture_scene'})
    monkeypatch.setattr(moveit2_backend, 'native_tool_collision_boxes', lambda *_: ([], {}))
    monkeypatch.setattr(independent_cups, 'm710_cup_array_from_mapping', lambda _: SimpleNamespace(cups=[]))
    request, _, _ = moveit2_backend.model_request(connector, scene, model_binding=fixture_config)
    assert request['joint_names'] == ['axis_'+str(i) for i in range(1, 7)]
    assert request['identity']['robot_model_binding']['model_id'] == 'analytical_fixture_not_s50'
    assert request['identity']['scene_fingerprint'] == 'fixture_scene'
    assert request['identity']['dynamics_qualified'] is False
    assert request['identity']['native_urdf_parser_compatibility']['effort_placeholder_joint_names'] == request['joint_names']
    assert 'J1_link' not in request['parameters']['robot_description']
    assert 'physical_tool_face' in request['parameters']['robot_description']
    with pytest.raises(ModelNotReady, match='config:FILE_MISSING'):
        moveit2_backend.model_request(connector, scene, model_binding=tmp_path/'missing.yaml')


def test_model_policy_is_scoped_and_legacy_identity_preserved(fixture_config):
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    legacy = SimulationCollisionPolicy(wrist_tool_exempt_links=('J5_link', 'J6_link'))
    assert not any('robot_binding' in key for key in legacy.to_mapping())
    assert SimulationCollisionPolicy.from_mapping(legacy.to_mapping()).fingerprint == legacy.fingerprint
    with pytest.raises(ValueError, match='explicitly authorized'):
        SimulationCollisionPolicy(wrist_tool_exempt_links=('segment_6',))
    change_config(fixture_config, lambda data: data['collision'].update(
        owned_tool_exempt_robot_links=['segment_6'], owned_tool_exemptions_source='analytical fixture review; not S50'))
    binding = RobotModelBinding.from_config(fixture_config)
    configured = SimulationCollisionPolicy.for_robot_binding(binding, legacy)
    assert configured.wrist_tool_pairs(['owned_tool']) == {('segment_6', 'owned_tool')}
    assert configured.robot_binding_model_id == binding.model_id
    assert configured.robot_binding_urdf_sha256 == binding.identity()['urdf_sha256']
    assert SimulationCollisionPolicy.from_mapping(configured.to_mapping()).fingerprint == configured.fingerprint
    assert configured.required_pair_clearance_m == legacy.required_pair_clearance_m
    assert configured.compliant_cup_neighbor_contact_mode == legacy.compliant_cup_neighbor_contact_mode
    change_config(fixture_config, lambda data: data['collision'].update(owned_tool_exempt_robot_links=['segment_2']))
    with pytest.raises(ModelNotReady, match='PROXIMAL_ARM'):
        SimulationCollisionPolicy.for_robot_binding(RobotModelBinding.from_config(fixture_config), legacy)


def test_native_missing_effort_only_changes_transport_copy(fixture_config, tmp_path):
    binding = RobotModelBinding.from_config(fixture_config)
    before = binding.urdf_path.read_bytes()
    xml, _, metadata = native_model_from_binding(binding, np.eye(4), project_root=tmp_path)
    compatibility = metadata['urdf_parser_compatibility']
    assert compatibility['effort_placeholder_joint_names'] == list(binding.active_joint_names)
    assert compatibility['physical_effort_unknown_joint_names'] == list(binding.active_joint_names)
    assert compatibility['placeholder_effort_nm'] == 0.
    assert compatibility['dynamics_qualified'] is False
    assert compatibility['source_urdf_modified'] is False
    for name in binding.active_joint_names:
        assert xml.find(f"joint[@name='{name}']/limit").get('effort') == '0'
    assert binding.urdf_path.read_bytes() == before
    assert all(element.get('effort') is None for element in ET.fromstring(before).iter('limit'))


def test_native_known_fanuc_effort_values_preserved():
    binding = RobotModelBinding.from_config(ROOT/'configs/robots/fanuc_m710id_70.yaml')
    before = binding.urdf_path.read_bytes()
    original = ET.fromstring(before)
    xml, _, metadata = native_model_from_binding(binding, np.eye(4), project_root=ROOT)
    assert metadata['urdf_parser_compatibility']['effort_placeholder_joint_names'] == []
    for name in binding.active_joint_names:
        assert xml.find(f"joint[@name='{name}']/limit").attrib == original.find(f"joint[@name='{name}']/limit").attrib
    assert binding.urdf_path.read_bytes() == before


@pytest.mark.parametrize('invalid', ['nan', 'inf', '-1', ''])
def test_native_invalid_known_effort_not_replaced(fixture_config, tmp_path, invalid):
    path = fixture_config.with_name('fixture.urdf'); xml = ET.parse(path).getroot()
    xml.find("joint[@name='axis_1']/limit").set('effort', invalid)
    path.write_text(ET.tostring(xml, encoding='unicode'))
    before = path.read_bytes()
    with pytest.raises(ModelNotReady, match='NATIVE_EFFORT_VALUE_INVALID'):
        native_model_from_binding(RobotModelBinding.from_config(fixture_config), np.eye(4), project_root=tmp_path)
    assert path.read_bytes() == before


def test_binding_identity_covers_visual_and_collision_mesh_bytes(fixture_config, tmp_path):
    path = fixture_config.with_name('fixture.urdf'); xml = ET.parse(path).getroot()
    for kind in ('visual', 'collision'):
        shape = ET.SubElement(xml.find("link[@name='segment_2']"), kind)
        ET.SubElement(ET.SubElement(shape, 'geometry'), 'mesh', filename=kind+'.stl')
        (tmp_path/(kind+'.stl')).write_text('solid fixture\nendsolid fixture\n')
    path.write_text(ET.tostring(xml, encoding='unicode'))
    binding = RobotModelBinding.from_config(fixture_config)
    old_identity = binding.identity()
    assert set(old_identity['mesh_sha256']) == {'visual.stl', 'collision.stl'}
    (tmp_path/'visual.stl').write_text('solid changed_fixture\nendsolid changed_fixture\n')
    assert binding.identity() == old_identity
    assert RobotModelBinding.from_config(fixture_config).identity() != old_identity
    with pytest.raises(ModelNotReady, match='MODEL_SOURCE_CHANGED_SINCE_BINDING'):
        native_model_from_binding(binding, np.eye(4), project_root=tmp_path)


def test_native_policy_cannot_use_another_robot_scope(fixture_config):
    from dataclasses import replace
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    change_config(fixture_config, lambda data: data['collision'].update(
        owned_tool_exempt_robot_links=['segment_6'], owned_tool_exemptions_source='fixture review'))
    binding = RobotModelBinding.from_config(fixture_config)
    policy = SimulationCollisionPolicy.for_robot_binding(binding, SimulationCollisionPolicy())
    assert configured_wrist_tool_pairs(binding, policy, ['tool']) == {('segment_6', 'tool')}
    with pytest.raises(ModelNotReady, match='ROBOT_BINDING_MISMATCH'):
        configured_wrist_tool_pairs(binding, replace(policy, robot_binding_model_id='different_robot'), ['tool'])


def test_readiness_metadata_does_not_override_validated_fields(fixture_config):
    change_config(fixture_config, lambda data: data.update(readiness={'kinematics_collision_ready': False}))
    assert RobotModelBinding.from_config(fixture_config).model_id == 'analytical_fixture_not_s50'
    change_config(fixture_config, lambda data: data['kinematics'].update(flange_link=None))
    change_config(fixture_config, lambda data: data.update(readiness={'kinematics_collision_ready': True}))
    with pytest.raises(ModelNotReady, match='kinematics.flange_link'):
        RobotModelBinding.from_config(fixture_config)


@pytest.mark.parametrize('mutation, reason', [
    (lambda root: root.find("joint[@name='axis_2']/child").set('link', 'undeclared'), 'INVALID_TREE_REFERENCES'),
    (lambda root: ET.SubElement(root.find("joint[@name='axis_2']"), 'mimic', joint='axis_1'), 'FORBIDS_MIMIC'),
    (lambda root: root.find("joint[@name='axis_2']/origin").set('xyz', 'nan 0 0'), 'ORIGIN_INVALID'),
])
def test_unmodeled_joint_semantics_fail_closed(fixture_config, mutation, reason):
    path = fixture_config.with_name('fixture.urdf'); root = ET.parse(path).getroot()
    mutation(root); path.write_text(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ModelNotReady, match=reason):
        RobotModelBinding.from_config(fixture_config)
