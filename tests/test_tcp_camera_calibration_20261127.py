"""TCP-based recalibration of the link_1 camera (user request 2026-11-27).
Synthetic images rendered from the board definition, fake FK; no device."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import tcp_calibration as tc

ROOT = Path(__file__).resolve().parents[1]
K = np.array([[606.8, 0, 330.6], [0, 606.56, 248.7], [0, 0, 1.]])
FIELD = json.loads((ROOT/'configs/workbench/reference/active_extrinsics.json').read_text())
Y_TRUE = np.asarray(FIELD['matrix'])@tc.rigid(tc.from_rotvec(np.radians([1.5, -2., .8])), [.006, -.004, .008])
X_TRUE = tc.rigid(tc.from_rotvec(np.radians([3, 178, -2])), [.05, -.12, .03])


def board_poses(board, n, rng, far=0.):
    lo, hi = board.extent(); c = (lo+hi)/2
    front = np.diag([1., -1., -1.]) if board.front_z > 0 else np.diag([-1., -1., 1.])
    for _ in range(n):
        R = tc.from_rotvec(rng.normal(0, np.radians(18), 3))@front
        p = np.array([rng.uniform(-.08, .08), rng.uniform(-.06, .06), rng.uniform(.75, 1.05)+far])
        yield tc.rigid(R, p-R@c)


def hand_eye_samples(board, n=12, seed=0, far=0., noise=1.):
    rng = np.random.default_rng(seed); samples, A = [], []
    for i, B in enumerate(board_poses(board, n, rng, far)):
        img = board.render(B, K, (640, 480))
        img = np.clip(img+rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
        A.append(Y_TRUE@B@tc.inv(X_TRUE))
        samples.append(dict(name=f'{i+1:03d}', q=np.array([i, 0, 0, 0, 0, 0.]), K=K, dist=np.zeros(5),
                            found=tc.detect(img, board), rgb=img, B=B))
    return samples, A


@pytest.mark.parametrize('which', ['grid', 'field'])
def test_board_on_tool_recovers_the_camera(which):
    board = tc.Board.grid(4, 5, .06, .015) if which == 'grid' else tc.Board.from_extrinsics_settings(FIELD['settings'])
    samples, A = hand_eye_samples(board, far=0. if which == 'grid' else .35)
    assert all(len(s['found']) == len(board.corners) for s in samples)
    Y, report = tc.solve(samples, board, 'BOARD_ON_TOOL', link1_tcp=lambda q: A[int(q[0])])
    d = tc.difference(Y_TRUE, Y)
    assert d['translation_m'] < .002 and d['rotation_deg'] < .1
    assert np.allclose(np.asarray(report['X_tcp_board'])[:3, 3], X_TRUE[:3, 3], atol=.003)
    assert report['passed'], report['checks']


def test_field_board_is_seen_from_its_minus_z_side_and_the_corner_order_is_decided():
    field = tc.Board.from_extrinsics_settings(FIELD['settings'])
    assert field.front_z == -1 and tc.Board.grid(2, 2, .05, .01).front_z == 1
    samples, A = hand_eye_samples(field, n=3, far=.35)
    order, info = tc.choose_corner_order(samples, field, K, np.zeros(5))
    assert order == (0, False) and info['second_median_px'] > 10*info['best_median_px']


def test_translation_only_poses_are_not_accepted():
    board = tc.Board.grid(4, 5, .06, .015); rng = np.random.default_rng(4)
    lo, hi = board.extent(); c = (lo+hi)/2; R = np.diag([1., -1., -1.])@tc.from_rotvec(np.radians([0, 0, 1]))
    samples, A = [], []
    for i in range(9):
        B = tc.rigid(R, np.array([rng.uniform(-.1, .1), rng.uniform(-.05, .05), rng.uniform(.8, 1.)])-R@c)
        A.append(Y_TRUE@B@tc.inv(X_TRUE))
        samples.append(dict(name=str(i), q=np.array([i, 0, 0, 0, 0, 0.]), K=K, dist=np.zeros(5),
                            found=tc.detect(board.render(B, K, (640, 480)), board)))
    with pytest.raises(ValueError, match='NOT_DIVERSE'):
        tc.solve(samples, board, 'BOARD_ON_TOOL', link1_tcp=lambda q: A[int(q[0])])


def link1(q):
    c, s = np.cos(q[0]), np.sin(q[0])
    return tc.rigid(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), [0, 0, .2])


def test_tip_touch_registers_the_board_and_recovers_the_camera():
    board = tc.Board.from_extrinsics_settings(FIELD['settings']); rng = np.random.default_rng(3)
    lo, hi = board.extent(); c = (lo+hi)/2
    R = tc.from_rotvec(np.radians([8, -6, 3]))@np.diag([-1., -1., 1.])
    board_base = link1([1.4])@Y_TRUE@tc.rigid(R, [0, 0, 1.2]-R@c)
    tip = np.array([0, 0, .07]); pts = {}; touches = []
    for k, name in enumerate(['tag20_c2', 'tag24_c3', 'tag0_c1', 'tag4_c0', 'tag12_c2', 'tag17_c0', 'tag8_c1']):
        pts[10+k] = (board_base@np.r_[board.corner(name), 1])[:3]+rng.normal(0, .0003, 3)
        touches.append((name, np.array([10+k, 0, 0, 0, 0, 0.])))
    samples = []
    for i, j1 in enumerate([1.34, 1.37, 1.40, 1.43, 1.46]):
        q = np.array([j1, 0, 0, 0, 0, 0.]); B = tc.inv(link1(q)@Y_TRUE)@board_base
        samples.append(dict(name=str(i), q=q, K=K, dist=np.zeros(5), found=tc.detect(board.render(B, K, (640, 480)), board)))
    Y, report = tc.solve(samples, board, 'TIP_TOUCH', base_link1=link1,
                         base_link6=lambda q: tc.rigid(np.eye(3), pts[int(q[0])]-tip), touches=touches, tip_link6_m=tip)
    d = tc.difference(Y_TRUE, Y)
    assert d['translation_m'] < .004 and d['rotation_deg'] < .2
    assert report['touch_registration']['max_m'] < .002 and report['checks']['touch_rms_m']['ok']
    with pytest.raises(ValueError, match='TOUCHES'):
        tc.solve(samples, board, 'TIP_TOUCH', base_link1=link1, base_link6=lambda q: np.eye(4), touches=touches[:3], tip_link6_m=tip)


def test_session_round_trip_writes_loadable_extrinsics_and_a_field_config(tmp_path):
    from unloading_sim.workbench.coordinates import Extrinsics
    from unloading_sim.workbench.ui import URDF_SHA
    board = tc.Board.grid(4, 5, .06, .015)
    samples, A = hand_eye_samples(board, n=10, seed=2)
    tool = tc.rigid(tc.from_rotvec([0, 0, .3]), [0, 0, .14])
    session = tc.Session.create(tmp_path, 'BOARD_ON_TOOL', board, tool, URDF_SHA, basis='SYNTHETIC')
    intr = dict(width=640, height=480, fx=K[0, 0], fy=K[1, 1], ppx=K[0, 2], ppy=K[1, 2], coeffs=[0.]*5)
    depth = np.zeros((480, 480*4//3), np.uint16)
    for s in samples:
        item = session.add_photo(s['rgb'], depth, .001, intr, s['q'], dict(rule='SYNTHETIC'))
        assert len(item['tags']) == 20
    frames = lambda q: dict(link_1=np.eye(4), link_6=A[int(q[0])]@tc.inv(tool))
    reference = dict(matrix=FIELD['matrix'], sha256='x', path='active')
    path, report = tc.solve_session(tc.Session.load(session.folder), frames, reference)
    assert path.name == 'extrinsics.json' and report['passed']
    loaded = Extrinsics.load(path)
    assert loaded.parent_frame == 'link_1' and loaded.status == tc.STATUS_PASS and loaded.session_status == 'PRESENT_NOT_REVALIDATED'
    assert tc.difference(Y_TRUE, loaded.matrix)['translation_m'] < .002
    assert abs(report['against_reference']['rotation_deg']-tc.difference(np.asarray(FIELD['matrix']), Y_TRUE)['rotation_deg']) < .1
    assert len(list((session.folder/'overlays').glob('*.png'))) == 10
    cfg = tmp_path/'field.json'; cfg.write_text(json.dumps(dict(schema='x', camera_extrinsics='configs/old.json')))
    tc.field_config_with(cfg, path, tmp_path/'field.calib.json')
    new = json.loads((tmp_path/'field.calib.json').read_text())
    assert new['camera_extrinsics'] == str(path.resolve()) and 'configs/old.json' in new['camera_extrinsics_basis']
    rejected = json.loads(path.read_text()); rejected['status'] = tc.STATUS_FAIL; path.write_text(json.dumps(rejected))
    with pytest.raises(ValueError, match='NOT_USABLE'):
        tc.field_config_with(cfg, path, tmp_path/'bad.json')


def test_board_definitions_and_printable_scale():
    g = tc.Board.grid(3, 5, .06, .015)
    assert sorted(g.corners) == list(range(15)) and np.allclose(g.corner('tag6_c3'), [.075, .075, 0])
    img = g.printable(300)
    assert abs(img.shape[1]-round((5*.06+4*.015+.04)*300/.0254)) <= 1
    for bad in ((0, 3, .06, .01), (3, 3, .5, .01), (1, 2, .05, .01)):
        with pytest.raises(ValueError):
            tc.Board.grid(*bad)
    assert tc.Board.from_dict(g.to_dict()).corners.keys() == g.corners.keys()


class FakeRoot:
    def __init__(self): self.calls = []
    def after(self, ms, fn): self.calls.append(fn)


def stamp(ns):
    return SimpleNamespace(received_monotonic_ns=ns)


def test_panel_saves_only_stationary_samples(tmp_path, monkeypatch):
    """The stationarity rule of the operator panel, without a display."""
    from unloading_sim.workbench.single_carton import calibration_panel as cp
    import time as _time
    panel = cp.CalibrationPanel.__new__(cp.CalibrationPanel)
    panel.root = FakeRoot(); panel.busy = False; messages = []
    panel.show = lambda text, append=False: messages.append(text)
    now = _time.monotonic_ns()
    q0 = (0., .1, .2, .3, .4, .5)
    panel.backend = SimpleNamespace(joint=SimpleNamespace(sample_id='a', q_rad=q0, stamp=stamp(now), mapping_id='m'),
                                    frame=SimpleNamespace(observation_id='f0', stamp=stamp(now)))
    saved = []
    panel._stationary(lambda frame, q, ev: saved.append((frame.observation_id, ev['moved_deg'])))
    panel.backend.frame = SimpleNamespace(observation_id='f1', stamp=stamp(now+10**12))
    panel.backend.joint = SimpleNamespace(sample_id='b', q_rad=tuple(np.array(q0)+np.radians(.05)), stamp=stamp(now+2*10**12), mapping_id='m')
    panel.root.calls.pop()()
    assert not saved and '仍在运动' in messages[-1]
    panel._stationary(lambda frame, q, ev: saved.append((frame.observation_id, ev['moved_deg'])))
    panel.backend.frame = SimpleNamespace(observation_id='f2', stamp=stamp(now+3*10**12))
    panel.backend.joint = SimpleNamespace(sample_id='c', q_rad=panel.backend.joint.q_rad, stamp=stamp(now+4*10**12), mapping_id='m')
    panel.root.calls.pop()()
    assert saved == [('f2', 0.)]
    panel._stationary(lambda *a: saved.append(a))                       # no new frame after the click
    panel.backend.joint = SimpleNamespace(sample_id='d', q_rad=panel.backend.joint.q_rad, stamp=stamp(now+5*10**12), mapping_id='m')
    panel.root.calls.pop()()
    assert len(saved) == 1 and '新相机帧' in messages[-1]
