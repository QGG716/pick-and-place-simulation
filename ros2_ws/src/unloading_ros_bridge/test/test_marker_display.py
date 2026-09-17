"""Synthetic geometry, real ROS messages and production publish_markers path."""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from unloading_interfaces.msg import CargoObservation, PlanningWorldSnapshot
from visualization_msgs.msg import Marker
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode
from unloading_ros_bridge.marker_display import MarkerScene
from unloading_ros_bridge.marker_display import BLOCKED, CONFLICT, HISTORY
from unloading_interfaces.msg import UnknownRegion


def cube(identity, x=1.):
    cargo = CargoObservation(object_id=identity, source_instance_id=identity,
        has_pose=True, has_full_dimensions=True, pose_frame_id='world',
        full_dimensions_m=[.4, .5, .6], candidate_eligible=False,
        association_status='SYNTHETIC_UNTRACKED')
    cargo.pose.position.x = x
    cargo.pose.orientation.z = .6
    cargo.pose.orientation.w = .8
    return cargo


def surface(module='m0', capture='cap', face='face:0'):
    return dict(schema_version='observed_surface_v1', module_id=module, capture_id=capture, source_instance_id='instance',
        face_id=face, frame_id='world', capture_time=100., sensor_epoch='epoch',
        corners_3d_m=[[1., 0., 1.], [1.4, .1, 1.], [1.4, .5, 1.], [1., .4, 1.]],
        T_W_C_at_capture=[[1., 0., 0., 99.], [0., 1., 0., 99.],
                          [0., 0., 1., 99.], [0., 0., 0., 1.]])


def patch(*surfaces):
    return CargoObservation(source_instance_id='fused-instance',
        association_status='UNRESOLVED', observed_surfaces_json=json.dumps(surfaces or [surface()]))


def snapshot(*cargo, epoch='epoch', reasons=()):
    value = PlanningWorldSnapshot(source_epoch=epoch, obstacles=list(cargo),
        planning_admissible=False, blocking_reasons=list(reasons))
    value.source_capture_time.sec = 100
    return value


@pytest.fixture
def display():
    # Only transport is captured here. No alternate marker generation logic.
    output = []
    owner = SimpleNamespace(marker_publisher=SimpleNamespace(publish=output.append),
        marker_scene=MarkerScene(), get_logger=lambda: SimpleNamespace(warning=lambda text: None))
    def publish(value):
        before = deepcopy(value)
        WorldBridgeNode.publish_markers(owner, value, 'world')
        assert value == before
        return output[-1].markers
    return publish


def geometry(markers):
    return [m for m in markers if m.action == Marker.ADD and m.type != Marker.TEXT_VIEW_FACING]


def test_surface_without_complete_volume_is_visible(display):
    markers = geometry(display(snapshot(patch())))
    assert len(markers) == 1
    assert markers[0].type == Marker.LINE_STRIP
    assert [[p.x, p.y, p.z] for p in markers[0].points] == surface()['corners_3d_m'] + [surface()['corners_3d_m'][0]]


def test_disappeared_cube_is_explicitly_deleted(display):
    first = geometry(display(snapshot(cube('A'), cube('B', 2.))))
    removed = next(m for m in first if m.pose.position.x == 1.)
    second = display(snapshot(cube('B', 2.)))
    assert (removed.ns, removed.id) in {(m.ns, m.id) for m in second if m.action == Marker.DELETE}


def test_reordering_does_not_swap_identity(display):
    first = geometry(display(snapshot(cube('A'), cube('B', 2.))))
    second = geometry(display(snapshot(cube('B', 2.), cube('A'))))
    assert {(m.ns, m.id): m.pose.position.x for m in first} == {(m.ns, m.id): m.pose.position.x for m in second}


def labels(markers):
    return '\n'.join(m.text for m in markers if m.action == Marker.ADD)


def rgba(marker):
    return (marker.color.r, marker.color.g, marker.color.b, marker.color.a)


def test_exact_pose_scale_and_valid_patch_markers(display):
    cargo = cube('A')
    result = geometry(display(snapshot(cargo)))
    assert result[0].pose == cargo.pose
    assert [result[0].scale.x, result[0].scale.y, result[0].scale.z] == list(cargo.full_dimensions_m)
    assert 'candidate_eligible=False' in labels(display(snapshot(cargo)))
    marker, = geometry(display(snapshot(patch())))
    assert marker.pose.orientation.w == 1.
    assert marker.pose.position.x == marker.pose.position.y == marker.pose.position.z == 0.
    assert marker.scale.x > 0 and 0 < marker.color.a <= 1
    assert marker.header.frame_id == 'world' and marker.header.stamp.sec == 100
    assert marker.lifetime.sec == marker.lifetime.nanosec == 0


def test_representatives_full_source_and_raw_conflict_survive(display):
    first, second = surface(), surface('m1')
    cargo = patch(first, second)
    both = geometry(display(snapshot(cargo)))
    assert len({(m.ns, m.id) for m in both}) == 2
    key = lambda s: {k: s[k] for k in ('module_id', 'capture_id', 'source_instance_id', 'face_id')}
    cargo.raw_result_json = json.dumps({'fusion_diagnostics': {'face_reduction': {
        'representatives': [key(second)], 'conflicts': [{'first': key(first), 'second': key(second),
        'reason': 'OVERLAPPING_PLANE_POSITION_CONFLICT'}]}}})
    messages = display(snapshot(cargo))
    selected, = geometry(messages)
    assert selected.ns == both[1].ns
    assert rgba(selected) == pytest.approx(CONFLICT)
    assert 'CONFLICT' in labels(messages) and 'm0' in labels(messages) and 'm1' in labels(messages)
    assert 'representatives=1; raw source patches=2' in labels(messages)
    assert json.loads(cargo.observed_surfaces_json) == [first, second]


@pytest.mark.parametrize('mode', ['missing', 'ambiguous'])
def test_bad_representative_mapping_never_falls_back(display, mode):
    first = surface()
    cargo = patch(first, first) if mode == 'ambiguous' else patch(first)
    reference = {k: first[k] for k in ('module_id', 'capture_id', 'source_instance_id', 'face_id')}
    if mode == 'missing': reference['module_id'] = 'not-m0'
    cargo.raw_result_json = json.dumps({'fusion_diagnostics': {'face_reduction': {'representatives': [reference]}}})
    messages = display(snapshot(cargo))
    assert not geometry(messages)
    assert 'missing/ambiguous representative' in labels(messages)


@pytest.mark.parametrize('change', ['json', 'nan', 'frame', 'short', 'shape', 'overflow', 'nested-json'])
def test_bad_record_does_not_crash_or_hide_other_object(display, change):
    s = surface()
    if change == 'nan': s['corners_3d_m'][0][0] = float('nan')
    if change == 'frame': s['frame_id'] = 'unknown-camera'
    if change == 'short': s['corners_3d_m'] = []
    if change == 'shape': s['corners_3d_m'][0] = None
    if change == 'overflow': s['corners_3d_m'][0][0] = 10**1000
    cargo = patch(s)
    if change == 'json': cargo.observed_surfaces_json = '{bad'
    if change == 'nested-json': cargo.observed_surfaces_json = '[' * 2000 + '0' + ']' * 2000
    messages = display(snapshot(cargo, cube('valid')))
    assert len(geometry(messages)) == 1 and geometry(messages)[0].type == Marker.CUBE
    assert 'DISPLAY DIAGNOSTICS' in labels(messages)


def test_unknown_bbox_is_only_nonspatial_ui_and_time_status_is_preserved(display):
    value = snapshot(patch(), reasons=['OBSERVATION_STALE', 'UNKNOWN_VOLUME'])
    value.unknown_regions = [UnknownRegion(region_id='unknown', frame_id='camera',
        reason='NO_DEPTH', has_bbox=True, bbox_xyxy=[100., 100., 200., 200.])]
    messages = display(value)
    assert len(geometry(messages)) == 1
    assert rgba(geometry(messages)[0]) == pytest.approx(HISTORY)
    assert 'HISTORY / TIME INVALID' in labels(messages) and 'NOT PLANNING ADMISSIBLE' in labels(messages)
    assert 'Unknown regions (no 3D extent): 1' in labels(messages)
    assert 'fixed anchor, NOT a spatial region' in labels(messages)
    value.blocking_reasons = ['UNKNOWN_VOLUME']
    messages = display(value)
    assert rgba(geometry(messages)[0]) == pytest.approx(BLOCKED)
    assert 'HISTORY / TIME INVALID' not in labels(messages)
    assert 'NO_DEPTH' in labels(messages) and 'UNKNOWN_VOLUME' in labels(messages)


def test_lifecycle_deletes_all_attachments_and_keeps_full_current_scene(display):
    visible = {}
    def apply(messages):
        for m in messages:
            if m.action == Marker.DELETE: visible.pop((m.ns, m.id), None)
            else: visible[m.ns, m.id] = m
    apply(display(snapshot(cube('A'))))
    cube_keys = {key for key, m in visible.items() if m.type != Marker.TEXT_VIEW_FACING or 'COMPLETE' in m.text}
    cargo = patch()
    cargo.object_id = 'A'
    messages = display(snapshot(cargo))
    apply(messages)
    assert not cube_keys.intersection(visible)
    patch_keys = {(m.ns, m.id) for m in geometry(messages)}
    apply(display(snapshot(cargo, epoch='new-epoch')))
    assert not patch_keys.intersection(visible)
    messages = display(snapshot())
    apply(messages)
    assert len(visible) == 1 and not geometry(list(visible.values()))
    # Reconnection after missing multiple updates: tombstones still clear A.
    assert cube_keys.issubset({(m.ns, m.id) for m in messages if m.action == Marker.DELETE})
    assert any(m.action == Marker.ADD and 'UI LEGEND' in m.text for m in messages)
    assert all(m.action != Marker.DELETEALL for m in messages)


def test_ambiguous_object_identity_is_not_first_wins(display):
    messages = display(snapshot(cube('same'), cube('same', 2.)))
    assert not geometry(messages)
    assert 'ambiguous object identity' in labels(messages)
