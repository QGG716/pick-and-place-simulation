"""User review of the face-unloading simulation, 2026-11-28: standing cup array with
one sideways placement toward the robot, no reach-over posture on any segment
(also not through a pose-goal approach), straight front-face detach only, and
the wrist-cluster option. Literal geometry, fake planner stubs; no feasibility
or timing claim."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.workbench.single_carton import posture
from unloading_sim.workbench.single_carton import optimization_defaults as D
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.single_carton.geometry import contact_candidates, contact_search_settings, transform

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT/'configs/workbench/single_carton.example.json'
BASIS = 'USER_DECISION_2026_11_28_TEST'
# Simulated carton front face: 130 mm wide (base Y) x 110 mm high, centre of column C5.
FACE_C5 = (-0.5076, 0.1355, 0.476)
GAP = 0.0095


def tool():
    return json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())


def face(centre, half=(0.065, 0.055)):
    uu, vv = np.meshgrid(np.arange(-half[0]+.001, half[0], .002), np.arange(-half[1]+.001, half[1], .002))
    return dict(observed=True, center_m=list(centre), normal=[1., 0., 0.], u=[0., 1., 0.], v=[0., 0., 1.],
                size_m=[2*half[0], 2*half[1]], support_radius_m=.002, support_points_uv_m=np.c_[uu.ravel(), vv.ravel()].tolist())


def search(mode='FACE_CENTER_LONG_SIDE_ALONG_BASE_Z', side='+Z', shift='LATERAL_TOWARD_ROBOT_MAX'):
    single = dict(mode=mode, long_side_toward=side, basis=BASIS, vertical_placement='LOWER_TWO_ROWS_ON_FACE',
                  vertical_placement_basis=BASIS, in_face_shift=shift, in_face_shift_basis=BASIS)
    return dict(reference_face_roll_deg=90, j6_rotation_deg=180, lift_offsets_m=[0.0], max_candidates=1, precontact_m=.06,
                basis=BASIS, single_contact=single)


def topology():
    return load(EXAMPLE, code_defaults=False)['topology']


def full_rings(cand, which):
    T = np.asarray(cand['tcp_pose'])
    return {k: transform(T, np.asarray(v)) for k, v in cand['target_lip_rings_tcp'].items() if k in which}


def on_face(points, centre, half, margin=.003):
    d = np.asarray(points)-np.asarray(centre)
    return bool(np.all(abs(d[:, 1]) <= half[0]-margin+1e-9) and np.all(abs(d[:, 2]) <= half[1]-margin+1e-9))


def test_standing_array_one_contact_lower_two_rows_toward_the_robot():
    """"把吸具竖起来吸一箱（即吸具的长边平行于z轴）" + "吸取点太靠右，可以适当往左挪一点"."""
    box = dict(id='c5', faces=[face(FACE_C5)])
    grid, = contact_candidates(box, tool(), topology(), search(shift='LATERAL_TOWARD_ROBOT'))
    far, = contact_candidates(box, tool(), topology(), search())
    for cand in (grid, far):
        info = cand['contact_search']['single_contact']
        assert np.allclose(info['long_side_world'], [0., 0., 1.]) and info['long_side_from_base_z_deg'] == pytest.approx(0., abs=1e-9)
        assert len(cand['active_ids']) == 4                           # two columns x the two lowest rows
        assert info['up_shift_in_face_m'] == pytest.approx(0.045)      # lower two of four 45 mm rows centred on the face
        T = np.asarray(cand['tcp_pose']); assert np.allclose(T[:3, 2], [-1., 0., 0.])
        assert T[2, 3] == pytest.approx(FACE_C5[2]+0.045)
        rings = np.concatenate(list(full_rings(cand, cand['target_lip_rings_tcp']).values()))
        assert np.ptp(rings[:, 2]) > 0.17 and np.ptp(rings[:, 1]) < 0.132   # long side vertical, 131 mm across
    # Symmetric two-column placement 22.5 mm toward the J1 axis (base -Y here) ...
    assert grid['contact_search']['single_contact']['lateral_shift']['chosen_shift_m'] == pytest.approx(-0.0225)
    # ... and the 1 mm search keeps four seals up to 41 mm toward it (C5: link_3 cleared the side panel only there).
    shift = far['contact_search']['single_contact']['lateral_shift']['chosen_shift_m']
    assert shift == pytest.approx(-0.041)
    assert np.linalg.norm(np.asarray(far['tcp_pose'])[:2, 3]) < np.linalg.norm(np.asarray(grid['tcp_pose'])[:2, 3])


def test_standing_array_never_seals_the_neighbour_but_the_lying_grid_does():
    """"吸具没有对吸盘进行分区 ... 要么一次吸两箱": a passive cup whose whole ring lies on the
    next carton would hold it too (common valve). Neighbours 9.5 mm apart on both sides."""
    half = (0.065, 0.055)
    neighbours = [(FACE_C5[0], FACE_C5[1]+s*(2*half[0]+GAP), FACE_C5[2]) for s in (-1, 1)]
    box = dict(id='c5', faces=[face(FACE_C5)])
    for shift in ('LATERAL_TOWARD_ROBOT', 'LATERAL_TOWARD_ROBOT_MAX'):
        cand, = contact_candidates(box, tool(), topology(), search(shift=shift))
        passive = full_rings(cand, cand['expected_passively_closed_ids'])
        assert not [k for k, r in passive.items() for n in neighbours if on_face(r, n, half)]
    lying = contact_candidates(box, tool(), topology(), search('FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', '+Y', 'CUP_GRID'))
    assert len(lying) > 20                                             # the 2026-11-26 cup-grid shifts
    two_box = [c for c in lying for r in full_rings(c, c['expected_passively_closed_ids']).values()
               for n in neighbours if on_face(r, n, half)]
    assert two_box                                                     # some lying placements hold the neighbour as well


def test_single_contact_settings_accept_the_standing_array():
    ok = search()
    assert contact_search_settings(ok)['single_contact']['mode'] == 'FACE_CENTER_LONG_SIDE_ALONG_BASE_Z'
    assert contact_search_settings(search(shift='LATERAL_TOWARD_ROBOT'))
    assert contact_search_settings(search('FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', '-Y', 'LATERAL_TOWARD_ROBOT_MAX'))
    for bad in (dict(long_side_toward='+Y'), dict(mode='FACE_CENTER_LONG_SIDE_ALONG_BASE_Y'), dict(in_face_shift='LATERAL'),
                dict(in_face_shift_basis=' ')):
        with pytest.raises(ValueError):
            contact_search_settings(dict(ok, single_contact=dict(ok['single_contact'], **bad)))


def test_code_defaults_stand_the_array_up_and_detach_front_faces_straight():
    c = load(EXAMPLE)
    single = c['contact_search']['single_contact']
    assert (single['mode'], single['long_side_toward'], single['vertical_placement'], single['in_face_shift']) == (
        'FACE_CENTER_LONG_SIDE_ALONG_BASE_Z', '+Z', 'LOWER_TWO_ROWS_ON_FACE', 'LATERAL_TOWARD_ROBOT_MAX')
    assert c['detach_profile_policy']['mode'] == 'FRONT_FACE_STRAIGHT_OUTWARD_ONLY'
    assert '正吸的脱离方案只能是走直线' in c['detach_profile_policy']['basis']
    assert issues(c) == []
    native = json.loads(EXAMPLE.read_text()); native['planning_module'] = 'NATIVE'; D.apply(native)
    assert 'detach_profile_policy' not in native
    # An explicit lying array in a file still wins.
    raw = json.loads(EXAMPLE.read_text())
    raw['contact_search'] = dict(search('FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', '+Y', 'CUP_GRID'))
    D.apply(raw)
    assert raw['contact_search']['single_contact']['mode'] == 'FACE_CENTER_LONG_SIDE_ALONG_BASE_Y'


def test_detach_policy_validation_and_filter():
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    c = load(EXAMPLE)
    for bad in (dict(mode='STRAIGHT', basis=BASIS), dict(mode='FRONT_FACE_STRAIGHT_OUTWARD_ONLY'),
                dict(mode='FRONT_FACE_STRAIGHT_OUTWARD_ONLY', basis=BASIS, max_normal_from_horizontal_deg=90)):
        assert [e for e in issues(dict(c, detach_profile_policy=bad)) if 'detach_profile_policy' in e]
    assert [e for e in issues(dict(c, planning_module='NATIVE')) if 'detach_profile_policy' in e]

    def goals(normal, policy):
        n = np.asarray(normal, float); R = np.eye(3)
        if abs(n[2]) > .5: R = np.array([[1., 0, 0], [0, 1, 0], [0, 0, 1]])
        else: R = np.column_stack(([0., 1, 0], [0., 0, 1], n))
        pose = np.eye(4); pose[:3, :3] = R; pose[:3, 3] = [-.6, 0., .4]
        size = [.13, .11, .18] if abs(n[2]) < .5 else [.18, .13, .11]
        box = dict(pose=pose.tolist(), size_m=size, corners_m=(np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*np.abs(R@np.array(size))/2+[-.6, 0, .4]).tolist())
        actual = np.eye(4); actual[:3, 3] = [-.5, 0., .45]
        stub = SimpleNamespace(site=dict(detach_profile_policy=policy) if policy else {}, detach_goals=SinglePlanner.detach_goals)
        return [name for name, _ in SinglePlanner.allowed_detach_goals(stub, actual, box, dict(face=dict(normal=list(normal))))]

    straight = dict(mode='FRONT_FACE_STRAIGHT_OUTWARD_ONLY', basis=BASIS, max_normal_from_horizontal_deg=45.)
    assert goals([1., 0., 0.], None)[0] == 'OUTWARD' and len(goals([1., 0., 0.], None)) > 1
    assert goals([1., 0., 0.], straight) == ['OUTWARD']
    assert goals([.8, 0., .6], straight) == ['OUTWARD']                      # 37 deg above horizontal: still a front grip
    assert len(goals([0., 0., 1.], straight)) > 1                            # top grip keeps its profiles
    assert goals([1., 0., 0.], dict(mode='ALL_PROFILES')) == goals([1., 0., 0.], None)


def bound():
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    c = load(EXAMPLE)
    inst = Installation.load(c['execution']['installation'])
    return c, posture.bind(c, CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool))


# Literal configurations of the 2026-11-28 layer-12 simulation (degrees).
C5_BACKSTROKE = np.radians([166.9, 100.4, -82.1, -18.3, -76.9, 0.1])      # accepted C5 approach end ("仰泳")
C3_FRONT = np.radians([14.9, 17.0, -101.7, 84.6, 75.1, 0.1])              # C3 approach end, wrist link straight up
RELEASE = np.radians([71.8, 29.2, -111.3, -7.8, 90.0, 5.1])               # tool down at the belt


def test_reach_over_path_check_matches_the_facing_metric_and_catches_the_c5_plan():
    c, policy = bound()
    rng = np.random.default_rng(7)
    Q = rng.uniform(-3.0, 3.0, (400, 6))
    batch = posture._batch_transforms(policy, Q)
    for q, k in zip(Q[:40], range(40)):
        m = posture.metrics(policy, q, q)
        chain = policy['chain']; link1 = batch[chain[0]['child']][k]
        wrist = batch[chain[4]['child']][k][:2, 3]-link1[:2, 3]
        if np.linalg.norm(wrist) < posture.FACING_MIN_RADIUS_M: continue
        shoulder = (link1@np.asarray(chain[1]['origin']))[:2, 3]-link1[:2, 3]
        angle = np.degrees(np.arccos(np.clip(shoulder@wrist/np.linalg.norm(shoulder)/np.linalg.norm(wrist), -1, 1)))
        assert angle == pytest.approx(m['arm_facing_deg'], abs=1e-6)
    assert posture.reach_over_violation(policy, [C3_FRONT, RELEASE]) is None
    path = np.linspace(RELEASE, C5_BACKSTROKE, 50)
    bad = posture.reach_over_violation(policy, path)
    assert bad['reason'] == 'ARM_FACING_REACH_OVER_STATE' and bad['max_arm_facing_deg'] > 160 and 0 < bad['index'] < 50
    assert posture.reach_over_violation(dict(policy, arm_facing='FRONT_FIRST'), path) is None


def test_wrist_cluster_setting_gate_and_goal_check():
    c, policy = bound()
    base = dict(c['posture_preference'])
    assert posture.settings(dict(posture_preference=dict(base, wrist_cluster='UP')))
    with pytest.raises(ValueError):
        posture.settings(dict(posture_preference=dict(base, wrist_cluster='SIDE')))
    assert [e for e in issues(dict(c, planning_module='NATIVE', posture_preference=dict(base, wrist_cluster='DOWN'))) if 'wrist_cluster' in e]
    m = posture.metrics(policy, C3_FRONT, C3_FRONT)
    assert m['wrist_cluster'] == 'UP' and m['wrist_link_z'] == pytest.approx(1., abs=1e-3)
    assert posture.metrics(policy, RELEASE, RELEASE)['wrist_cluster'] == 'NEUTRAL'   # tool down: wrist link horizontal
    up, down = dict(policy, wrist_cluster='UP'), dict(policy, wrist_cluster='DOWN')
    assert posture.wrist_gate(up, [list(C3_FRONT), list(RELEASE)], C3_FRONT) == ([list(C3_FRONT), list(RELEASE)], [])
    kept, removed = posture.wrist_gate(down, [list(C3_FRONT), list(RELEASE)], C3_FRONT)
    assert kept == [list(RELEASE)] and removed[0]['wrist_cluster'] == 'UP'
    assert posture.goal_violation(policy, C3_FRONT, C3_FRONT) is None
    assert posture.goal_violation(down, C3_FRONT, C3_FRONT)['gate'] == 'WRIST_CLUSTER_GATE'
    assert posture.goal_violation(policy, C5_BACKSTROKE, RELEASE)['gate'] == 'UPPER_ARM_GATE'
    loose = {k: v for k, v in policy.items() if k != 'upper_arm_max_from_vertical_deg'}
    assert posture.goal_violation(loose, C5_BACKSTROKE, RELEASE)['gate'] == 'ARM_FACING_GATE'


def test_pose_goal_approach_goes_through_the_branch_gates():
    """C5 2026-11-28: the only screened front branch hit the side panel, the
    screen was NOT_CONCLUSIVE and the pose-goal approach chose the reach-over
    branch; it is now rejected like a screened branch."""
    from unloading_sim.workbench.single_carton.planner import SinglePlanner, SegmentRejected
    c, policy = bound()
    calls = []

    def plan(context, goal, stage, *a, **k):
        calls.append(stage)
        return dict(prepared=dict(points=[dict(q=list(RELEASE)), dict(q=list(C5_BACKSTROKE))]))
    stub = SimpleNamespace(posture_policy=policy, context=lambda *a, **k: {})
    candidate = dict(precontact_pose=np.eye(4).tolist())
    with pytest.raises(SegmentRejected) as e:
        SinglePlanner._plan_candidate_chain(stub, RELEASE, [], {}, candidate, None, dict(screen=dict(conclusion='NOT_CONCLUSIVE')),
                                            lambda *_: None, None, [None], None, False, None, None, plan, None, 1.0, np.eye(4), [], None)
    assert e.value.stage == 'approach' and 'POSE_GOAL_POSTURE_GATE_REJECTED' in str(e.value) and calls == ['approach']


def test_loaded_return_retry_retries_the_same_profile_once():
    from unloading_sim.workbench.single_carton.planner import SinglePlanner, SegmentRejected
    c = load(EXAMPLE)
    assert c['loaded_return_retry']['mode'] == 'VIA_THEN_OMPL_AFTER_PILZ_SENT_PATH_REJECTED'
    assert c['release_branch']['mode'] == 'NEAREST_GRASP_BRANCH' and issues(c) == []
    for key, bad in (('loaded_return_retry', dict(mode='OMPL')), ('release_branch', dict(mode='NEAREST_GRASP_BRANCH'))):
        assert [e for e in issues(dict(c, **{key: bad})) if key in e]
        assert [e for e in issues(dict(c, planning_module='NATIVE')) if key in e]
    stub = SimpleNamespace(site=dict(loaded_return_retry=c['loaded_return_retry']))
    pilz = dict(rejected=[], profile='OUTWARD', returning=dict(planner=dict(pipeline_id='pilz_industrial_motion_planner')))
    error = SegmentRejected('return', "return:SENT_PATH_REJECTED:{'pair': ['box', 'site_layout_layout_part_072']}")
    retry = SinglePlanner.replan_after(stub, pilz, error)
    assert retry['return_retry'] == 'OUTWARD' and retry['rejected'] == []          # same profile, retried return
    ompl = dict(retry, returning=dict(planner=dict(pipeline_id='ompl')))
    assert [x['profile'] for x in SinglePlanner.replan_after(stub, ompl, error)['rejected']] == ['OUTWARD']
    off = SimpleNamespace(site={})
    assert [x['profile'] for x in SinglePlanner.replan_after(off, pilz, error)['rejected']] == ['OUTWARD']
    planning = SegmentRejected('return', 'return:NATIVE_REJECTED:NATIVE_PLANNING_FAILED')
    assert [x['profile'] for x in SinglePlanner.replan_after(stub, pilz, planning)['rejected']] == ['OUTWARD']
