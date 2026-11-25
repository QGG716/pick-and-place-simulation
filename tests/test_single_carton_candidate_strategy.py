"""Directed checks of the 2026-11-25 candidate strategy switches (user decisions:
near-duplicate candidates, IK branch slots, approach path in the screen,
cross-cycle inheritance, approximate posture preference). Fake native worker;
no feasibility or timing claim."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

from test_single_carton_planner import case  # noqa: F401
from unloading_sim.workbench.single_carton import candidate_strategy as cs
from unloading_sim.workbench.single_carton import posture
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.ui import ROOT

BASIS = 'USER_DECISION_2026_11_25_TEST'


def example():
    return load(ROOT/'configs/workbench/single_carton.example.json')


def test_switches_are_validated_and_native_fails_closed():
    c = example()
    ok = dict(deepcopy(c), candidate_family_order=dict(mode='INTERLEAVE_LIFT_SIBLINGS', basis=BASIS),
              screen_approach_path=dict(mode='APPROACH_JOINT_LINE', basis=BASIS),
              cross_cycle_inheritance=dict(mode='IK_SEEDS_AND_FAMILY_FIRST', basis=BASIS))
    keys = ('candidate_family_order', 'screen_approach_path', 'cross_cycle_inheritance')
    assert not any(k in e for e in issues(ok) for k in keys)
    for key, bad in (('candidate_family_order', dict(mode='DROP', basis=BASIS)),
                     ('screen_approach_path', dict(mode='APPROACH_JOINT_LINE')),
                     ('cross_cycle_inheritance', dict(mode='PATHS', basis=BASIS)),
                     ('cross_cycle_inheritance', dict(mode='IK_SEEDS', basis=BASIS, max_entries=0)),
                     ('cross_cycle_inheritance', dict(mode='IK_SEEDS', basis=BASIS, family_tolerance_rad=3.)),
                     ('cross_cycle_inheritance', dict(mode='IK_SEEDS', basis=BASIS, store='../x.json'))):
        assert any(key in e for e in issues(dict(ok, **{key: bad}))), (key, bad)
    native = issues(dict(ok, planning_module='NATIVE'))
    assert all(any(k in e and 'PYTHON' in e for e in native) for k in keys)
    # Defaults written explicitly are the unchanged behaviour.
    assert cs.family_policy(dict(candidate_family_order=dict(mode='KEEP_ORDER', basis=BASIS))) is None
    assert cs.screen_policy(dict(screen_approach_path=dict(mode='ENDPOINTS_ONLY', basis=BASIS))) is None
    assert cs.family_policy({}) is None and cs.screen_policy({}) is None and cs.inheritance_policy({}) is None


def chain_policy(**extra):
    """Synthetic 6R chain: at q=0 upper arm +Z, forearm +X, wrist (link_4->link_5) +X."""
    def joint(name, child, xyz, axis):
        T = np.eye(4); T[:3, 3] = xyz
        return dict(name=name, child=child, origin=T.tolist(), axis=axis, type='revolute')
    chain = [joint('joint_1', 'link_1', [0, 0, 0], [0, 0, 1]), joint('joint_2', 'link_2', [0, 0, .3], [0, 1, 0]),
             joint('joint_3', 'link_3', [0, 0, .4], [0, 1, 0]), joint('joint_4', 'link_4', [.4, 0, 0], [1, 0, 0]),
             joint('joint_5', 'link_5', [.1, 0, 0], [0, 1, 0]), joint('joint_6', 'link_6', [0, 0, -.1], [0, 0, 1])]
    policy = dict(mode='SOFT_LINK_DIRECTIONS', upper_up_weight=4., forearm_horizontal_weight=2., joint_travel_weight=.05,
                  wrist_down_weight=2., basis=BASIS, chain=chain, base=np.eye(4).tolist(),
                  metric_links=['link_2', 'link_3', 'link_4'], wrist_links=['link_4', 'link_5'])
    policy.update(extra)
    return policy


def test_posture_tolerance_bands_literal_values():
    zero = [0.]*6
    plain = posture.metrics(chain_policy(), zero, zero)
    assert plain['upper_from_up_deg'] == pytest.approx(0., abs=1e-9)
    assert plain['forearm_from_horizontal_deg'] == pytest.approx(0., abs=1e-9)
    assert plain['wrist_from_down_deg'] == pytest.approx(90., abs=1e-9)
    assert plain['score'] == pytest.approx(2.)                           # 2*(1-cos 90 deg)
    banded = posture.metrics(chain_policy(tolerance_deg=dict(wrist_down=60.)), zero, zero)
    assert banded['score'] == pytest.approx(2*(1-math.cos(math.radians(30))))   # only 30 deg excess
    tilt = [0., math.radians(15), 0., 0., 0., 0.]
    a = posture.metrics(chain_policy(), tilt, zero)
    b = posture.metrics(chain_policy(tolerance_deg=dict(upper_up=20., forearm_horizontal=20., wrist_down=90.)), tilt, zero)
    assert a['upper_from_up_deg'] == pytest.approx(15.)
    # Inside every band only the joint travel (0.05 * 15 deg) remains.
    assert b['score'] == pytest.approx(.05*math.radians(15))
    # Zero tolerance reproduces the original score.
    c = posture.metrics(chain_policy(tolerance_deg=dict(upper_up=0., forearm_horizontal=0., wrist_down=0.)), tilt, zero)
    assert c['score'] == pytest.approx(a['score'], abs=1e-12)


def test_posture_tolerance_settings_are_validated():
    base = dict(mode='SOFT_LINK_DIRECTIONS', upper_up_weight=4., forearm_horizontal_weight=2., joint_travel_weight=.05, basis=BASIS)
    assert posture.settings(dict(posture_preference=dict(base, tolerance_deg=dict(upper_up=20., forearm_horizontal=20.))))
    for bad in (dict(upper_up=70.), dict(wrist_down=10.), dict(), dict(upper_up='20')):   # wrist_down needs its weight
        with pytest.raises(ValueError):
            posture.settings(dict(posture_preference=dict(base, tolerance_deg=bad)))
    assert posture.settings(dict(posture_preference=dict(base, wrist_down_weight=2., tolerance_deg=dict(wrist_down=30.))))
    c = example()
    if 'posture_preference' in c:
        native = issues(dict(c, planning_module='NATIVE', posture_preference=dict(base, tolerance_deg=dict(upper_up=20.))))
        assert any('tolerance_deg' in e for e in native)


def test_interleave_families_literal_order():
    # (name, family, class); input already in priority order.
    rows = [('a1', 'A', 0), ('a2', 'A', 0), ('a3', 'A', 0), ('b1', 'B', 0), ('c1', 'C', 0), ('b2', 'B', 0), ('x', 'A', 1)]
    order, ranks = cs.interleave_families(rows, lambda r: r[2], lambda r: r[1])
    assert [r[0] for r in order] == ['a1', 'b1', 'c1', 'a2', 'b2', 'a3', 'x']
    assert ranks == [0, 0, 0, 1, 1, 2, 3]
    same, _ = cs.interleave_families(rows[:1] + rows[3:5], lambda r: r[2], lambda r: r[1])
    assert [r[0] for r in same] == ['a1', 'b1', 'c1']          # distinct families: order unchanged


def test_family_key_groups_lift_siblings_of_one_reference_pose():
    ref = np.eye(4).tolist()
    a = dict(contact_search=dict(reference_tcp_pose=ref, lift_along_face_m=.01))
    b = dict(contact_search=dict(reference_tcp_pose=ref, lift_along_face_m=.05))
    other = np.eye(4); other[0, 3] = .02
    c = dict(contact_search=dict(reference_tcp_pose=other.tolist(), lift_along_face_m=.01))
    assert cs.family_key(0, a) == cs.family_key(1, b) != cs.family_key(2, c)
    assert cs.family_key(3, {}) != cs.family_key(4, {})        # no search record: every candidate its own family


Q0 = [1.58, .15, -1.63, 1.37, 1.67, -.16]
QA = [0.01, .49, -2.23, 1.73, 1.58, -1.57]
QC = [0.01, .51, -2.20, 1.75, 1.58, -1.57]
POLICY = cs.inheritance_policy(dict(cross_cycle_inheritance=dict(mode='IK_SEEDS_AND_FAMILY_FIRST', basis=BASIS, max_entries=2)))


def test_inheritance_store_round_trip_and_filters(tmp_path):
    task = tmp_path/'task-3'; task.mkdir()
    entries, evidence = cs.load(POLICY, task, Q0, 'FOUR_SEGMENT')
    assert entries == [] and evidence['status'] == 'NO_STORE'
    for n, (q0, mode) in enumerate(((Q0, 'FOUR_SEGMENT'), (Q0, 'START_RELEASE_SEPARATE'),
                                    ([Q0[0] + math.radians(2)] + Q0[1:], 'FOUR_SEGMENT'),
                                    (Q0, 'FOUR_SEGMENT'), (Q0, 'FOUR_SEGMENT'))):
        cs.record(POLICY, tmp_path/f'task-{n}', dict(task_id=f't{n}', q_start=q0, approach_q=QA, contact_q=QC, cycle_mode=mode))
    entries, evidence = cs.load(POLICY, task, Q0, 'FOUR_SEGMENT')
    assert [e['task_id'] for e in entries] == ['t4', 't3']            # newest first, max_entries=2
    assert evidence['status'] == 'USED'
    stored = json.loads((tmp_path/'cross-cycle-inheritance.json').read_text())
    assert stored['schema'] == cs.STORE_SCHEMA and len(stored['entries']) == 5
    # Older matching entries beyond max_entries are not read; mismatches are recorded with a reason.
    many = dict(POLICY, max_entries=5)
    entries, evidence = cs.load(many, task, Q0, 'FOUR_SEGMENT')
    assert [e['task_id'] for e in entries] == ['t4', 't3', 't0']
    assert {r['reason'] for r in evidence['rejected']} == {'Q_START_DIFFERS', 'CYCLE_STRUCTURE_DIFFERS'}
    (tmp_path/'cross-cycle-inheritance.json').write_text('{broken')
    assert cs.load(POLICY, task, Q0, 'FOUR_SEGMENT')[1]['status'] == 'STORE_UNREADABLE_IGNORED'


def test_family_first_uses_wrapped_joint_distance():
    flip = [0.01, .49, -2.23, -2.30, -1.58, 1.57]
    twin_of_inherited = QA[:5] + [QA[5] + 2*np.pi]
    order, first = cs.family_first([flip, twin_of_inherited], [dict(approach_q=QA)], .5)
    assert order == [twin_of_inherited, flip] and first == [twin_of_inherited]
    assert cs.family_first([flip], [], .5) == ([flip], [])


def native_on(p):
    from unloading_sim import moveit2_backend
    p.client = moveit2_backend.ResidentMoveItClient()


def test_inherited_approach_configurations_seed_the_ik_and_lead_the_branch_order(case):
    p, obs, box, folder, calls, opts = case
    native_on(p)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-7.,)*6, (7.,)*6, (.1,)*6, (.2,)*6)
    p.inherited = [dict(task_id='t0', approach_q=QA, contact_q=QC)]
    seeds = p.ik_seeds(Q0, [])
    assert seeds[:2] == [Q0, QA]
    flip = [0.01, .49, -2.23, -2.30, -1.58, 1.57]
    opts['ik'] = lambda r: dict(status='SUCCESS', q=r['q_start'], position_error_m=0., orientation_error_rad=0.)
    p.posture_policy = None
    p.site['cross_cycle_inheritance'] = dict(mode='IK_SEEDS', basis=BASIS)
    plain, _ = p.endpoint_ik(np.eye(4), [flip, QA], folder, 'approach-endpoint-ik', q_reference=flip)
    assert plain == [flip, QA]                                   # IK_SEEDS: order unchanged
    p.site['cross_cycle_inheritance'] = dict(mode='IK_SEEDS_AND_FAMILY_FIRST', basis=BASIS)
    first, _ = p.endpoint_ik(np.eye(4), [flip, QA], folder, 'approach-endpoint-ik', q_reference=flip)
    assert first == [QA, flip]
    record = json.loads((folder/'approach-endpoint-ik-native.json').read_text())
    assert record['inherited_family_first']['moved_first'] == [QA]
    contact, _ = p.endpoint_ik(np.eye(4), [flip, QA], folder, 'contact-endpoint-ik', q_reference=flip)
    assert contact == [flip, QA]                                 # contact IK keeps its order


def screen_setup(p, opts, monkeypatch, line_clear):
    """Two endpoint-clear branches; the joint line to ``flip`` collides unless line_clear."""
    native_on(p)
    flip = [0.01, .49, -2.23, -2.30, -1.58, 1.57]
    monkeypatch.setattr(p, 'endpoint_ik', lambda pose, seeds, folder, name, cancelled=None, q_reference=None, client=None:
                        ([flip, QA], []) if name.startswith('approach') else ([QC], []))
    lines = []

    def validate(r):
        return None
    opts['validate'] = validate

    def sent(r):
        lines.append(r['probe_path'][-1])
        ok = line_clear or r['probe_path'][-1] != flip
        failure = None if ok else dict(pair=['link_3', 'part_026'], edge=3, fraction=.5)
        return dict(native_valid=True, legacy_intersection_valid=True, path_failure=failure, failure=None)
    opts['sent'] = sent
    return flip, lines


def test_screen_prefers_the_branch_whose_approach_line_is_clear(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    flip, lines = screen_setup(p, opts, monkeypatch, line_clear=False)
    candidate = dict(precontact_pose=np.eye(4).tolist(), tcp_pose=np.eye(4).tolist())
    p.contact_geometry['connected_cup_ids'] = []
    monkeypatch.setattr(p, 'context', lambda q, world, box, candidate, stage, attachment=None, supports=(): dict(stage=stage, q=list(q)))
    monkeypatch.setattr(p, 'detach_supports', lambda world, box: [])
    off = p.endpoint_precheck(Q0, [], box, candidate, np.eye(4), folder)
    assert off['approach_q'] == flip and 'approach_path_screen' not in off and lines == []
    p.site['screen_approach_path'] = dict(mode='APPROACH_JOINT_LINE', basis=BASIS)
    on = p.endpoint_precheck(Q0, [], box, candidate, np.eye(4), folder, solutions=[])
    assert on['approach_q'] == QA and on['approach_path_screen'] == 'PASS' and on['pending_branches'] == []
    assert lines == [flip, QA]
    rejected = [r for r in on['record'] if r['stage'] == 'approach_line']
    assert [r['conclusion'] for r in rejected] == ['APPROACH_JOINT_LINE_REJECTED', 'PASS']
    assert rejected[0]['failure']['pair'] == ['link_3', 'part_026']


def test_screen_without_any_clear_line_returns_the_first_branch_as_before(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    flip, lines = screen_setup(p, opts, monkeypatch, line_clear=False)
    opts['sent'] = lambda r: (lines.append(r['probe_path'][-1]) or
                              dict(native_valid=True, legacy_intersection_valid=True, path_failure=dict(pair=['a', 'b']), failure=None))
    candidate = dict(precontact_pose=np.eye(4).tolist(), tcp_pose=np.eye(4).tolist())
    monkeypatch.setattr(p, 'context', lambda q, world, box, candidate, stage, attachment=None, supports=(): dict(stage=stage, q=list(q)))
    monkeypatch.setattr(p, 'detach_supports', lambda world, box: [])
    p.site['screen_approach_path'] = dict(mode='APPROACH_JOINT_LINE', basis=BASIS)
    off = dict(p.site); del off['screen_approach_path']
    on = p.endpoint_precheck(Q0, [], box, candidate, np.eye(4), folder, solutions=[])
    assert on['conclusion'] == 'CLEAR' and on['approach_q'] == flip and on['approach_path_screen'] == 'FAIL'
    assert on['pending_branches'] == [QA]                         # same as without the switch
    assert lines == [flip, QA]


def test_full_plan_with_every_switch_records_and_inherits(case):
    p, obs, box, folder, calls, opts = case
    for key, value in (('candidate_family_order', dict(mode='INTERLEAVE_LIFT_SIBLINGS', basis=BASIS)),
                       ('screen_approach_path', dict(mode='APPROACH_JOINT_LINE', basis=BASIS)),
                       ('cross_cycle_inheritance', dict(mode='IK_SEEDS_AND_FAMILY_FIRST', basis=BASIS)),
                       ('ik_winding_twins', dict(mode='AFTER_DISTINCT_CONFIGURATIONS', basis=BASIS))):
        p.site[key] = value
    opts['ik'] = lambda r: dict(status='SUCCESS', q=[.01]*6 if r['goal_pose'] else [.02]*6, position_error_m=0., orientation_error_rad=0.)
    plan = p.plan_cycle(obs, [box], folder)
    assert plan['full_path_validated']
    priority = json.loads((folder/'candidate-priority.json').read_text())
    assert all('approach_path_screen' in r and 'family_rank' in r for r in priority)
    inherited = plan['cross_cycle_inheritance']
    assert inherited['input']['status'] == 'NO_STORE' and inherited['recorded']['status'] == 'RECORDED'
    assert json.loads((folder/'cross-cycle-inheritance-input.json').read_text())['status'] == 'NO_STORE'
    second = folder.parent/('b'*32); second.mkdir()
    p.plan_cycle(obs, [box], second)
    used = json.loads((second/'cross-cycle-inheritance-input.json').read_text())
    assert used['status'] == 'USED' and used['entries'][0]['task_id'] == folder.name


# contact_search.single_contact (user decision 2026-11-25): one contact only.
def front_box(normal=(1., 0., 0.), half=(.15, .10)):
    n = np.asarray(normal, float); n /= np.linalg.norm(n)
    u = np.cross([0., 0., 1.], n); u /= np.linalg.norm(u); v = np.cross(n, u)   # u ~ +-Y, v ~ +Z for a front face
    uu, vv = np.meshgrid(np.arange(-half[0], half[0] + 1e-9, .002), np.arange(-half[1], half[1] + 1e-9, .002))
    face = dict(observed=True, center_m=[-.48, -.06, .38], normal=n.tolist(), u=u.tolist(), v=v.tolist(),
                size_m=[2*half[0], 2*half[1]], support_radius_m=.003, support_points_uv_m=np.c_[uu.ravel(), vv.ravel()].tolist())
    return dict(id='box', faces=[face])


def single(side='+Y', **extra):
    c = example()
    search = dict(reference_face_roll_deg=90, j6_rotation_deg=180, lift_offsets_m=[0.0], max_candidates=1, precontact_m=.06, basis=BASIS,
                  single_contact=dict(mode='FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', long_side_toward=side, basis=BASIS, **extra))
    return c, search


def tool():
    return json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())


def test_single_contact_is_one_centred_candidate_long_side_along_y_short_side_along_z():
    from unloading_sim.workbench.single_carton.geometry import contact_candidates, transform
    c, search = single()
    box = front_box()
    out = contact_candidates(box, tool(), c['topology'], search)
    assert len(out) == 1
    cand = out[0]; T = np.asarray(cand['tcp_pose']); info = cand['contact_search']['single_contact']
    assert len(cand['active_ids']) == 12 and cand['contact_search']['lift_along_face_m'] == 0.
    assert info['long_from_base_y_deg'] == pytest.approx(0., abs=1e-6) and info['short_from_base_z_deg'] == pytest.approx(0., abs=1e-6)
    assert info['long_side_world'][1] == pytest.approx(1.)
    rings = np.concatenate([transform(T, np.asarray(r)) for r in cand['target_lip_rings_tcp'].values()])
    assert np.allclose(rings.mean(0), box['faces'][0]['center_m'], atol=1e-9)     # array centre on the face centre
    ext = np.ptp(rings, axis=0)
    assert ext[1] > ext[2] > ext[0]                                               # long side Y, short side Z, flat in X
    assert np.allclose(T[:3, 2], [-1., 0., 0.])                                   # suction direction into the face
    other = contact_candidates(box, tool(), c['topology'], single('-Y')[1])[0]
    R = np.asarray(other['tcp_pose'])[:3, :3].T @ T[:3, :3]
    assert math.degrees(math.acos(np.clip((np.trace(R) - 1)/2, -1, 1))) == pytest.approx(180., abs=1e-6)


def test_single_contact_projects_onto_a_slightly_turned_face_and_rejects_a_face_not_spanning_y_z():
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    c, search = single()
    turned = contact_candidates(front_box(normal=(1., .02, .014)), tool(), c['topology'], search)[0]['contact_search']['single_contact']
    assert 0 < turned['long_from_base_y_deg'] < 1.5 and 0 < turned['short_from_base_z_deg'] < 1.5
    with pytest.raises(ValueError, match='SINGLE_CONTACT_NO_FACE_SPANNING_BASE_Y_Z'):
        contact_candidates(front_box(normal=(1., 1.5, 0.)), tool(), c['topology'], search)     # 56 deg from X
    with pytest.raises(ValueError, match='CONNECTED_CUP_SEALS_DO_NOT_FIT'):
        contact_candidates(front_box(half=(.02, .02)), tool(), c['topology'], search)


def test_single_contact_settings_are_validated():
    from unloading_sim.workbench.single_carton.geometry import contact_search_settings
    c, search = single()
    assert contact_search_settings(search)['single_contact']['long_side_toward'] == '+Y'
    for bad in (dict(search['single_contact'], long_side_toward='Y'), {k: v for k, v in search['single_contact'].items() if k != 'long_side_toward'},
                dict(search['single_contact'], max_normal_from_x_deg=80), dict(search['single_contact'], mode='CENTRE'),
                dict(search['single_contact'], basis=' ')):
        with pytest.raises(ValueError):
            contact_search_settings(dict(search, single_contact=bad))
    native = issues(dict(c, contact_search=search, planning_module='NATIVE'))
    assert any('single_contact' in e for e in native)


def test_neighbor_lip_support_basis_adds_only_the_observed_support_and_is_validated(case):
    """observed_support_basis (user decision 2026-11-25, "允许唇边碰下方箱")."""
    p, obs, box, folder, calls, opts = case
    T = np.asarray(box['pose'], float)
    under = np.array(T); under[2, 3] -= .055
    world = [dict(id=box['id'], pose=box['pose'], size=box['size_m'], category='carton'),
             dict(id='rgbd_support_' + box['id'], pose=under.tolist(), size=[.15, .1, .01], category='observed_support'),
             dict(id='other', pose=np.eye(4).tolist(), size=[.1, .1, .1], category='carton'),
             dict(id='fixed', pose=under.tolist(), size=[.1, .1, .1], category='measured_fixed')]
    lip = dict(mode='CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS', basis=BASIS)
    p.site = dict(p.site, neighbor_lip_contact=lip)
    assert p.neighbor_lip_objects(world, box) == ['other']                     # unchanged without the new basis
    p.site = dict(p.site, neighbor_lip_contact=dict(lip, observed_support_basis=BASIS))
    p._neighbor_lip_cache = None
    assert p.neighbor_lip_objects(world, box) == ['rgbd_support_' + box['id'], 'other']
    ready = dict(capabilities=dict(bounded_single_carton_planning=True, neighbor_lip_contact_v1=True))
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_NEIGHBOR_LIP_SUPPORT'):
        p.worker_ready(ready)
    c = example()
    assert not any('observed_support_basis' in e for e in issues(dict(c, neighbor_lip_contact=dict(lip, observed_support_basis=BASIS))))
    for bad in (dict(lip, observed_support_basis=' '), dict(lip, observed_support_basis=True),
                dict(mode='DISABLED', observed_support_basis=BASIS)):
        assert any('observed_support_basis' in e for e in issues(dict(c, neighbor_lip_contact=bad))), bad
