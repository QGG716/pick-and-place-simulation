"""Real USD hierarchy regressions; generated geometry is not ECO65 asset evidence.

The fixture mirrors the exporter's parent/child structure in an in-memory USD
stage. Its small numerical joint chain exercises display transforms only; no
native planning, collision qualification, physics, or private assets are used.
"""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip('pxr.Usd', reason='Real USD runtime is required for hierarchy checks')
from pxr import Gf, Usd, UsdGeom

from unloading_sim.eco65.native_replay import apply_display_state, state_at


PREFIX = '/World/Workstation'
Q_A = np.array([.35, -.40, .55, -.25, .30, -.45])
Q_B = np.array([-.65, .30, -.50, .45, -.35, .70])
TARGET = 'carton_r00_l03_c01'


def _matrix(xyz=(0., 0., 0.), angles=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    result[:3, 3] = xyz
    return result


class NumericalJointChain:
    """Test-only FK provider, independent of the display writer and private CAD."""

    def named_link_frames(self, q):
        current = _matrix([-.36, .10, .18])
        frames = {'base_link': current.copy()}
        axes = ['z', 'y', 'y', 'x', 'y', 'x']
        offsets = [[0., 0., .12], [.03, 0., .08], [.22, 0., 0.],
                   [.18, 0., 0.], [.06, 0., 0.], [.05, 0., 0.]]
        for index, (angle, axis, offset) in enumerate(zip(q, axes, offsets), 1):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler(axis, angle).as_matrix()
            current = current @ _matrix(offset) @ rotation
            frames[f'link_{index}'] = current.copy()
        return frames


def _write_transform(prim, value):
    UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(value).T.tolist()))


def _xform(stage, path, value):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    _write_transform(prim, value)
    return prim


def _mesh(stage, path, value):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(0., 0., 0.), (.01, 0., 0.), (0., .02, 0.), (0., 0., .03)])
    mesh.CreateFaceVertexCountsAttr([3, 3, 3, 3])
    mesh.CreateFaceVertexIndicesAttr([0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3])
    _write_transform(mesh.GetPrim(), value)
    return mesh.GetPrim()


def _world(stage, path):
    return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def _local(stage, path):
    return np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetLocalTransformation()).T


@pytest.fixture
def usd_scene():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    root = _xform(stage, '/World', np.eye(4))
    stage.SetDefaultPrim(root)
    world_from_workstation = _matrix([.32, -.10, .80])
    _xform(stage, PREFIX, world_from_workstation)
    for group in ['Robot', 'Tool', 'Adapter', 'Cartons', 'Frames']:
        _xform(stage, PREFIX + '/' + group, np.eye(4))

    robot = NumericalJointChain()
    initial_frames = robot.named_link_frames([.12, -.16, .20, -.11, .13, -.17])
    for name, frame in initial_frames.items():
        _xform(stage, PREFIX + '/Robot/' + name, frame)
        for role in ['Visual', 'Collision']:
            _mesh(stage, PREFIX + '/Robot/' + name + '/' + role, np.eye(4))

    mast_local = _matrix([.085, .140, .035], [0., 0., -.4])
    camera_local = _matrix([.11, -.004, .46], [np.pi / 2, 0., 0.])
    _xform(stage, PREFIX + '/Robot/link_1/Mast', mast_local)
    _xform(stage, PREFIX + '/Robot/link_1/Mast/CameraFrame', camera_local)
    for role in ['visual_mast_part_019', 'collision_mast_part_019']:
        _mesh(stage, PREFIX + '/Robot/link_1/Mast/' + role, np.eye(4))
    _xform(stage, PREFIX + '/Robot/link_1/MountingSeat', np.eye(4))
    for role in ['Visual', 'Collision']:
        _mesh(stage, PREFIX + '/Robot/link_1/MountingSeat/' + role, np.eye(4))

    tool = dict(T_flange_tool_cad=_matrix([.018, -.009, .036], [.12, -.08, .20]).tolist(),
                T_flange_tcp=_matrix([.025, .010, .125], [.15, -.10, -.25]).tolist(),
                visual_parts=[dict(id='part_000')], collisions=[dict(id='rigid_plate')],
                adapter=[dict(id='flange_plate')])
    flange = initial_frames['link_6']
    cad = flange @ tool['T_flange_tool_cad']
    _mesh(stage, PREFIX + '/Tool/visual_part_000', cad)
    _mesh(stage, PREFIX + '/Tool/collision_rigid_plate', cad)
    _mesh(stage, PREFIX + '/Adapter/flange_plate', flange)
    _mesh(stage, PREFIX + '/Adapter/flange_plate_collision', flange)
    for name in [*initial_frames, 'flange', 'tool_cad', 'tcp']:
        _xform(stage, PREFIX + '/Frames/' + name, np.eye(4))

    boxes = []
    for layer in range(4):
        for column in range(3):
            ident = f'carton_r00_l{layer:02d}_c{column:02d}'
            box = dict(id=ident, pose=[.315, (column - 1) * .22, .23 + layer * .10],
                       size_m=[.25, .21, .10])
            boxes.append(box)
            prim = _xform(stage, PREFIX + '/Cartons/' + ident, _matrix(box['pose']))
            prim.SetCustomDataByKey('stable_id', ident)
            collision = UsdGeom.Cube.Define(stage, str(prim.GetPath()) + '/Collision')
            collision.CreateSizeAttr(1.)
            collision.AddTranslateOp().Set(Gf.Vec3d(0.))
            collision.AddScaleOp().Set(Gf.Vec3d(*box['size_m']))
    snapshot = dict(tool=tool, scene=dict(T_W_D=world_from_workstation.tolist(),
                    boxes=boxes, target_box_id=TARGET,
                    box=deepcopy(next(box for box in boxes if box['id'] == TARGET))))
    return stage, snapshot, robot, mast_local, camera_local


def test_real_usd_updates_link_parents_and_j1_accessories_once(usd_scene):
    stage, snapshot, robot, mast_local, camera_local = usd_scene
    W = np.array(snapshot['scene']['T_W_D'])
    original_cartons = {box['id']: _matrix(box['pose']) for box in snapshot['scene']['boxes']}
    cameras = []
    for q in [Q_A, Q_B]:
        frames = robot.named_link_frames(q)
        result = apply_display_state(stage, snapshot, dict(frames=frames, cartons=original_cartons))
        assert result['carton_count'] == 12
        assert result['max_matrix_error'] < 1e-12
        assert result['tcp_matrix_error'] < 1e-12
        np.testing.assert_allclose(_world(stage, PREFIX), W, atol=1e-12, rtol=0)
        for name, frame in frames.items():
            parent = PREFIX + '/Robot/' + name
            np.testing.assert_allclose(_world(stage, parent), W @ frame, atol=1e-12, rtol=0)
            for role in ['Visual', 'Collision']:
                np.testing.assert_allclose(_local(stage, parent + '/' + role), np.eye(4), atol=1e-12, rtol=0)
                np.testing.assert_allclose(_world(stage, parent + '/' + role), W @ frame, atol=1e-12, rtol=0)
        mast = PREFIX + '/Robot/link_1/Mast'
        np.testing.assert_allclose(_local(stage, mast), mast_local, atol=1e-12, rtol=0)
        np.testing.assert_allclose(_local(stage, mast + '/CameraFrame'), camera_local, atol=1e-12, rtol=0)
        np.testing.assert_allclose(_world(stage, mast), W @ frames['link_1'] @ mast_local, atol=1e-12, rtol=0)
        camera = _world(stage, mast + '/CameraFrame')
        np.testing.assert_allclose(camera, W @ frames['link_1'] @ mast_local @ camera_local, atol=1e-12, rtol=0)
        cameras.append(camera)
        for role in ['Visual', 'Collision']:
            np.testing.assert_allclose(_world(stage, PREFIX + '/Robot/link_1/MountingSeat/' + role),
                                       W @ frames['link_1'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(_world(stage, PREFIX + '/Tool/visual_part_000'),
                                   W @ frames['link_6'] @ snapshot['tool']['T_flange_tool_cad'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(_world(stage, PREFIX + '/Frames/tcp'),
                                   W @ frames['link_6'] @ snapshot['tool']['T_flange_tcp'], atol=1e-12, rtol=0)
    assert np.linalg.norm(cameras[1][:3, 3] - cameras[0][:3, 3]) > .05


@pytest.mark.parametrize('role', ['Visual', 'Collision'])
def test_real_usd_rejects_nonidentity_link_mesh_children(usd_scene, role):
    stage, snapshot, robot, _, _ = usd_scene
    path = PREFIX + '/Robot/link_3/' + role
    _write_transform(stage.GetPrimAtPath(path), _matrix([.001, 0., 0.]))
    state = dict(frames=robot.named_link_frames(Q_A),
                 cartons={box['id']: _matrix(box['pose']) for box in snapshot['scene']['boxes']})
    with pytest.raises(ValueError, match='REPLAY_LINK_MESH_LOCAL_TRANSFORM_CHANGED'):
        apply_display_state(stage, snapshot, state)


def test_real_usd_preserves_carton_ids_and_attachment_release_poses(usd_scene):
    stage, snapshot, robot, _, _ = usd_scene
    W = np.asarray(snapshot['scene']['T_W_D'])
    initial_target = _matrix(snapshot['scene']['box']['pose'])
    attachment = np.linalg.inv(robot.named_link_frames(Q_A)['link_6']) @ initial_target
    release = robot.named_link_frames(Q_B)['link_6'] @ attachment
    phases = ['pregrasp', 'contact', 'extraction', 'transit', 'place', 'withdrawal', 'residence']
    endpoints = [(Q_A, Q_A), (Q_A, Q_A), (Q_A, Q_B), (Q_B, Q_B),
                 (Q_B, Q_B), (Q_B, Q_A), (Q_A, Q_A)]
    # State interpolation fixture only; never fed to native acceptance or saved as a plan.
    plan = dict(stages=[dict(stage=phase, times=[0., 1.], q=[a.tolist(), b.tolist()])
                        for phase, (a, b) in zip(phases, endpoints)],
                events=[dict(flange_from_target=attachment.tolist()), dict(pose=release.tolist())])
    expected_ids = {box['id'] for box in snapshot['scene']['boxes']}
    carton_prims = {str(prim.GetPath()) for prim in stage.GetPrimAtPath(PREFIX + '/Cartons').GetChildren()}
    for time_s in [.5, 2.25, 2.75, 4.5, 5.5, 6.5]:
        state = state_at(snapshot, plan, robot, time_s)
        apply_display_state(stage, snapshot, state)
        assert set(state['cartons']) == expected_ids
        assert len(expected_ids) == 12
        assert {str(prim.GetPath()) for prim in stage.GetPrimAtPath(PREFIX + '/Cartons').GetChildren()} == carton_prims
        for box in snapshot['scene']['boxes']:
            path = PREFIX + '/Cartons/' + box['id']
            assert stage.GetPrimAtPath(path).GetCustomDataByKey('stable_id') == box['id']
            if box['id'] != TARGET:
                expected = _matrix(box['pose'])
            elif state['attached']:
                expected = state['frames']['link_6'] @ attachment
            elif state['phase'] in {'withdrawal', 'residence'}:
                expected = release
            else:
                expected = initial_target
            np.testing.assert_allclose(_world(stage, path), W @ expected, atol=1e-12, rtol=0)
    assert np.linalg.norm(release[:3, 3] - initial_target[:3, 3]) > .05


def test_compound_j1_collision_children_receive_fk_once(usd_scene):
    stage, snapshot, robot, _, _ = usd_scene
    stage.RemovePrim(PREFIX+'/Robot/link_1/Collision')
    rows = [dict(id='robot_link_1__convex_000'), dict(id='robot_link_1__convex_001')]
    snapshot['scene']['robot_collision_components'] = {'link_1': rows}
    for row in rows:
        _mesh(stage, PREFIX+'/Robot/link_1/Collision_'+row['id'], np.eye(4))
    for q in [Q_A, Q_B]:
        state = dict(frames=robot.named_link_frames(q),
            cartons={box['id']: _matrix(box['pose']) for box in snapshot['scene']['boxes']})
        result = apply_display_state(stage, snapshot, state)
        assert result['max_matrix_error'] < 1e-12
        for row in rows:
            np.testing.assert_allclose(_world(stage, PREFIX+'/Robot/link_1/Collision_'+row['id']),
                np.asarray(snapshot['scene']['T_W_D'])@state['frames']['link_1'], atol=1e-12, rtol=0)
