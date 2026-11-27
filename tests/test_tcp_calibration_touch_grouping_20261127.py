"""Touch grouping while recording (field 2026-11-27 session
20261127-203257-13455b: three corners, each touched in two or three
orientations, were all stored under the preselected tag0_c3 and the solve
failed). Field joints with the pinned URDF FK, otherwise synthetic; no device."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import tcp_calibration as tc

ROOT = Path(__file__).resolve().parents[1]
FIELD = json.loads((ROOT/'configs/workbench/reference/active_extrinsics.json').read_text())
BOARD = tc.Board.from_extrinsics_settings(FIELD['settings'])
# Field session 20261127-203257-13455b, touches 1-3 / 4-5 / 6-7 at three corners.
FIELD_Q = np.array([
    [-0.424568794, -0.401984233, -0.982969435, -1.495816982, -2.146754980, -4.847460011],
    [-0.524296907, -0.492776261, -0.977977793, -1.096503103, -2.535806324, -4.274712764],
    [-0.210364535, -0.374425484, -0.991975334, -1.578894655, -1.500232665, -4.736108005],
    [-0.279531933, -0.542622864, -0.978344312, -1.638096223, -2.315109440, -4.760263362],
    [0.134721965, -0.544141301, -0.983754833, -1.603486344, -1.033077838, -4.883378887],
    [0.011763519, -0.515605168, -1.486200218, -1.107341597, -1.873506232, -4.931026376],
    [0.009023352, -0.662073198, -1.723879155, 1.845842764, 1.933912078, -5.671115792]])


def test_field_touches_at_three_corners_are_not_stored_under_one_name():
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import CoordinateModel
    model = CoordinateModel(URDF, URDF_SHA, np.eye(4), None)
    stored, verdicts = [], []
    for q in FIELD_Q:
        T = model.frames(q)['link_6']
        v = tc.classify_touch(BOARD, stored, 1, T, 'tag0_c3'); verdicts.append(v)
        if v['accept']:
            stored.append((v['label'], 1, T))
    assert [v['accept'] for v in verdicts] == [True]*3+[False]*4
    assert all('新的点' in v['message'] and '已经记录在另一个位置' in v['message'] for v in verdicts[3:])
    est = tc.tip_estimate(stored)
    assert est['usable'] and est['max_residual_m'] < .004 and est['spread_deg'] >= 10
    assert np.allclose(est['tip'], [0, 0, .147], atol=.006)          # ~4 mm in front of the TCP (z 142.75 mm)
    # the three groups of the field session fit one tip within 3.3 mm
    groups = [[model.frames(FIELD_Q[i])['link_6'] for i in g] for g in ([0, 1, 2], [3, 4], [5, 6])]
    tip = tc.pivot_tip(groups)
    for Ts in groups:
        P = np.array([T[:3, :3]@tip+T[:3, 3] for T in Ts])
        assert np.linalg.norm(P-P.mean(0), axis=1).max() < .0035


def synthetic(seed=5):
    rng = np.random.default_rng(seed)
    lo, hi = BOARD.extent(); c = (lo+hi)/2
    R = tc.from_rotvec(np.radians([6, -4, 2]))@np.diag([-1., -1., 1.])
    board_base = tc.rigid(R, [-.65, 0, .3]-R@c)
    tip = np.array([.012, -.02, .21]); down = np.diag([1., -1., -1.])

    def pose(name, tilt):
        P = (board_base@np.r_[BOARD.corner(name), 1])[:3]+rng.normal(0, .0004, 3)
        Rt = tc.from_rotvec(np.radians(tilt))@down
        return tc.rigid(Rt, P-Rt@tip)
    return pose


def test_pivot_then_new_points_need_their_own_corner_and_repeats_are_grouped():
    pose = synthetic(); stored = []

    def rec(name, tilt, selected):
        v = tc.classify_touch(BOARD, stored, 1, pose(name, tilt), selected)
        if v['accept']:
            stored.append((v['label'], 1, pose(name, tilt)))
        return v
    assert not rec('tag12_c2', [0, 0, 0], '')['accept']                               # nothing selected yet
    for tilt in ([0, 0, 0], [20, 0, 0], [-20, 0, 0], [0, 20, 0]):
        assert rec('tag12_c2', tilt, 'tag12_c2')['kind'] == 'PIVOT'
    assert tc.tip_estimate(stored)['usable']
    v = rec('tag20_c2', [5, 0, 0], '')
    assert not v['accept'] and '请在“触点角点”里选' in v['message']
    v = rec('tag20_c2', [5, 0, 0], 'tag12_c2')
    assert not v['accept'] and '已经记录在另一个位置' in v['message']
    v = rec('tag20_c2', [5, 0, 0], 'tag24_c3')                                          # wrong corner: distance mismatch
    assert not v['accept'] and '实测相距' in v['message'] and '角点选错了' in v['message']
    assert rec('tag20_c2', [5, 0, 0], 'tag20_c2')['kind'] == 'NEW_POINT'
    v = rec('tag20_c2', [-12, 8, 0], 'tag0_c1')                                         # same point, other selection
    assert v['accept'] and v['kind'] == 'SAME_POINT' and v['label'] == 'tag20_c2'
    assert rec('tag24_c3', [0, -6, 0], 'tag24_c3')['label'] == 'tag24_c3'
    v = rec('tag4_c0', [3, 3, 0], '')                                                   # recognised from the board
    assert v['accept'] and v['label'] == 'tag4_c0' and '自动识别' in v['message']
    v = rec('tag0_c1', [0, 4, 0], 'tag8_c1')
    assert v['accept'] and v['label'] == 'tag0_c1' and '所选 tag8_c1 不对' in v['message']
    assert len({n for n, _, _ in stored}) == 5


def test_before_the_tip_is_known_a_different_point_under_the_same_name_is_refused():
    pose = synthetic(); stored = []
    for tilt in ([0, 0, 0], [3, 0, 0], [0, 3, 0]):                                     # too little tilt: no tip yet
        v = tc.classify_touch(BOARD, stored, 1, pose('tag12_c2', tilt), 'tag12_c2')
        assert v['accept']; stored.append(('tag12_c2', 1, pose('tag12_c2', tilt)))
    assert not tc.tip_estimate(stored)['usable']
    v = tc.classify_touch(BOARD, stored, 1, pose('tag6_c0', [0, 0, 0]), 'tag12_c2')
    assert not v['accept'] and '不是同一个点' in v['message']
    assert tc.classify_touch(BOARD, stored, 1, pose('tag12_c2', [-3, 0, 0]), 'tag12_c2')['accept']


def test_readiness_and_error_text_say_what_is_missing(tmp_path):
    pose = synthetic(); poses = {}
    s = tc.Session.create(tmp_path, 'TIP_TOUCH', BOARD, None, 'x')
    for k, (name, tilt) in enumerate([('tag12_c2', [0, 0, 0]), ('tag12_c2', [20, 0, 0]), ('tag12_c2', [0, 20, 0])]):
        poses[k] = pose(name, tilt); s.add_touch(name, [k, 0, 0, 0, 0, 0], {})
    s.data['photos'].append(dict(name='001', tags=[0, 1, 2, 3, 4], placement=1, active=True))
    lines = tc.readiness(s, lambda q: poses[int(q[0])])
    text = '\n'.join(lines)
    assert '照片：可用 1 张' in text and '触点 3 次' in text and '不同角点 1 个' in text and '尖端' not in text
    assert '每张至少要识别到 4 个标签' in tc.explain(ValueError('CALIBRATION_TOO_FEW_USABLE_SAMPLES'))
    assert tc.explain(ValueError('OTHER')) == 'OTHER'


class Var:
    def __init__(self, v=''):
        self.v = v

    def get(self):
        return self.v

    def set(self, v):
        self.v = v


def test_panel_clears_the_corner_once_the_tip_is_solved_and_stores_the_grouped_label(tmp_path):
    from unloading_sim.workbench.single_carton import calibration_panel as cp
    pose = synthetic(); poses = {}
    panel = cp.CalibrationPanel.__new__(cp.CalibrationPanel)
    panel.busy = False; messages = []
    panel.show = lambda text, append=False: messages.append(text)
    panel.session = tc.Session.create(tmp_path, 'TIP_TOUCH', BOARD, None, 'x')
    panel.model = SimpleNamespace(frames=lambda q: dict(link_6=poses[int(q[0])]))
    panel.tree = SimpleNamespace(insert=lambda *a, **k: None)
    panel.corner = Var('tag12_c2'); nxt = {}
    panel._stationary = lambda then: then(None, np.array([nxt['k'], 0, 0, 0, 0, 0.]), {})

    def touch(name, tilt):
        nxt['k'] = len(poses); poses[nxt['k']] = pose(name, tilt); panel.capture_touch()
    for tilt in ([0, 0, 0], [20, 0, 0], [0, 20, 0]):
        touch('tag12_c2', tilt)
    assert panel.corner.get() == '' and '尖端：已求出' in messages[-1]
    touch('tag20_c2', [0, 0, 0])                                                        # new point, nothing selected
    assert '触点未保存' in messages[-1] and len(panel.session.data['touches']) == 3
    panel.corner.set('tag20_c2'); touch('tag20_c2', [0, 0, 0])
    touch('tag20_c2', [10, 0, 0])                                                       # same point, selection cleared
    labels = [t['corner'] for t in panel.session.data['touches']]
    assert labels == ['tag12_c2']*3+['tag20_c2']*2
    assert panel.session.data['touches'][-1]['evidence']['classification']['kind'] == 'SAME_POINT'
