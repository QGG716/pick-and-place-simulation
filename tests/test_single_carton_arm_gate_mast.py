"""Upper-arm branch gate and mast contact record-only (user decisions 2026-11-25:
"不好的逆解为什么要尝试" / "偏竖直 >45° 不尝试"; "所有与视觉桅杆的碰撞都只记录，不否决" /
"全部只记录，不设上限")."""
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.workbench.single_carton import posture
from unloading_sim.workbench.single_carton.config import load, issues as validate

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT/'configs/workbench/single_carton.example.json'
BASE = dict(mode='SOFT_LINK_DIRECTIONS', upper_up_weight=4.0, forearm_horizontal_weight=2.0,
            joint_travel_weight=0.05, basis='TEST')


def test_gate_setting_is_validated():
    assert posture.settings(dict(posture_preference=dict(BASE, upper_arm_max_from_vertical_deg=45)))
    for bad in (0, -1, 181, float('nan'), '45'):
        with pytest.raises(ValueError):
            posture.settings(dict(posture_preference=dict(BASE, upper_arm_max_from_vertical_deg=bad)))


def test_code_defaults_turn_both_on():
    c = load(EXAMPLE)
    assert c['posture_preference']['upper_arm_max_from_vertical_deg'] == 45.0
    assert c['mast_contact']['mode'] == 'ROBOT_MAST_CONTACT_RECORD_ONLY' and c['mast_contact']['basis']


def test_gate_on_the_source_chain_literal_branches_of_ea893b88():
    """Field record ea893b88: branches 1/2 lean 59/108 deg, branches 3/4 25/20 deg."""
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    c = load(EXAMPLE)
    inst = Installation.load(c['execution']['installation'])
    policy = posture.bind(c, CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool))
    ref = np.array([1.1996346, 0.0186576, -1.6177980, -1.4952934, -1.7274396, -0.0768818])
    b3 = [-0.6407915, 0.4394642, -1.5499635, -1.9678900, -2.0687953, -3.1013180]
    b4 = [-0.6407915, 0.3462940, -2.4514065, 2.1683283, 2.0687954, 0.0402870]
    b1 = [-0.6407915, -1.0321887, 1.5499633, 2.6870214, -2.0687953, -3.1013182]
    b2 = [-0.6407915, -1.8834791, 2.4514065, -0.5047117, 2.0687954, 0.0402869]
    kept, removed = posture.upper_arm_gate(policy, [b3, b4, b1, b2], ref)
    assert kept == [b3, b4]
    assert [round(r['upper_from_up_deg']) for r in removed] == [59, 108]
    # Absent limit: nothing removed.
    loose = {k: v for k, v in policy.items() if k != 'upper_arm_max_from_vertical_deg'}
    assert posture.upper_arm_gate(loose, [b3, b1], ref) == ([b3, b1], [])


def test_mast_contact_config_and_native_only_scope():
    c = load(EXAMPLE)
    assert not [e for e in validate(c) if 'mast_contact' in e]
    for bad in (dict(mode='ROBOT_MAST_CONTACT_RECORD_ONLY'), dict(mode='ALLOW', basis='x'),
                dict(mode='ROBOT_MAST_CONTACT_RECORD_ONLY', basis='x', depth_m=0.002)):
        assert [e for e in validate(dict(c, mast_contact=bad)) if 'mast_contact' in e]
    assert not [e for e in validate(dict(c, mast_contact=dict(mode='DISABLED'))) if 'mast_contact' in e]


def test_mast_permission_rides_in_the_clearance_policy_and_the_record_drops_it(monkeypatch):
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    planner = SinglePlanner.__new__(SinglePlanner)
    planner.site = dict(mast_contact=dict(mode='ROBOT_MAST_CONTACT_RECORD_ONLY', basis='B'))
    assert planner.mast_contact_setting()['basis'] == 'B'
    planner.site = dict(mast_contact=dict(mode='DISABLED'))
    assert planner.mast_contact_setting() is None
    # The record re-checks without the permission and never decides.
    planner.site = dict(mast_contact=dict(mode='ROBOT_MAST_CONTACT_RECORD_ONLY', basis='B'))
    seen = {}
    def chunks(segment, folder, n):
        seen['policy'] = segment['context']['clearance_policy']
        return 'ADAPTIVE', [(0, 10, 'r', {}, {})]
    monkeypatch.setattr(planner, 'sent_check_chunks', chunks, raising=False)
    monkeypatch.setattr(planner, 'checkers', lambda: [object()], raising=False)
    monkeypatch.setattr(planner, 'native_call', lambda *a, **k: dict(
        native_valid=False, legacy_intersection_valid=True,
        path_failure=dict(pair=['link_3', 'eco65_camera_pitch_riser_left'], surface_distance_m=-1, edge=3, q_rad=[0]*6)), raising=False)
    written = {}
    monkeypatch.setattr(planner, 'write_evidence', lambda path, value, name, **o: written.update(value=value), raising=False)
    segment = dict(stage='approach', context=dict(clearance_policy=dict(schema='s', mast_contact_record_only=dict(basis='B')),
                                                  allowed_pairs=[]),
                   prepared=dict(collision_path=[[0]*6]*400))
    out = planner.record_mast_contacts(segment, Path('/nonexistent'))
    assert 'mast_contact_record_only' not in seen['policy'] and seen['policy']['schema'] == 's'
    assert out['status'] == 'RECORD_ONLY_NOT_A_DECISION' and out['any_contact']
    assert out['chunks'][0]['first']['pair'] == ['link_3', 'eco65_camera_pitch_riser_left'] and out['chunks'][0]['first']['state'] == 3
    # No permission in the request: nothing to record.
    assert planner.record_mast_contacts(dict(segment, context=dict(clearance_policy={}, allowed_pairs=[])), Path('/x')) is None


def test_thin_second_plane_policy_is_validated_and_on_by_default():
    """single_face_depth_policy.thin_second_plane (user decision 2026-11-25, "改成按单面处理")."""
    from unloading_sim.workbench.single_carton.measured_envelope import single_face_policy
    c = load(EXAMPLE)
    assert c['geometry']['single_face_depth_policy']['thin_second_plane'] == 'AS_SINGLE_FACE'
    base = dict(mode='BOUND_BY_MAX_CARTON_EXTENT', basis='B')
    assert single_face_policy(dict(single_face_depth_policy=base))
    assert single_face_policy(dict(single_face_depth_policy=dict(base, thin_second_plane='AS_SINGLE_FACE', thin_second_plane_basis='B')))
    for bad in (dict(base, thin_second_plane='AS_SINGLE_FACE'), dict(base, thin_second_plane='KEEP', thin_second_plane_basis='B'),
                dict(base, extra=1)):
        with pytest.raises(ValueError):
            single_face_policy(dict(single_face_depth_policy=bad))


def test_pair_clearance_setting():
    """pair_clearance (user decision 2026-11-25, "把要求2.4mm，降低为1.5mm")."""
    from unloading_sim.workbench.single_carton.config import pair_clearance
    c = load(EXAMPLE)
    assert pair_clearance(c) is None and not [e for e in validate(c) if 'pair_clearance' in e]
    ok = dict(c, pair_clearance=dict(clearance_m=0.0015, basis='B'))
    assert pair_clearance(ok) == dict(clearance_m=0.0015, basis='B') and not [e for e in validate(ok) if 'pair_clearance' in e]
    for bad in (dict(clearance_m=0.0015), dict(clearance_m=0.0003, basis='B'), dict(clearance_m=0.003, basis='B'),
                dict(clearance_m='1.5', basis='B'), dict(clearance_m=0.0015, basis='B', extra=1)):
        assert [e for e in validate(dict(c, pair_clearance=bad)) if 'pair_clearance' in e]
    assert [e for e in validate(dict(ok, planning_module='NATIVE')) if 'pair_clearance' in e]


def test_absent_tool_parts_setting():
    """absent_tool_parts (user decision 2026-11-25, "把吸具零件 part_001从layout中去掉"; "实物没有，去掉")."""
    from unloading_sim.workbench.single_carton.config import absent_tool_parts
    c = load(EXAMPLE)
    assert absent_tool_parts(c) is None
    ok = dict(c, absent_tool_parts=dict(parts=['part_001'], basis='B'))
    assert absent_tool_parts(ok, ['part_000', 'part_001']) == dict(parts=['part_001'], basis='B')
    assert not [e for e in validate(ok) if 'absent_tool_parts' in e]
    for bad in (dict(parts=['part_001']), dict(parts=[], basis='B'), dict(parts=['part_002_lip'], basis='B'),
                dict(parts=['link_3'], basis='B'), dict(parts=['part_001', 'part_001'], basis='B')):
        assert [e for e in validate(dict(c, absent_tool_parts=bad)) if 'absent_tool_parts' in e]
    with pytest.raises(ValueError):
        absent_tool_parts(ok, ['part_000'])
    assert [e for e in validate(dict(ok, planning_module='NATIVE')) if 'absent_tool_parts' in e]


def test_front_branch_first_setting_and_default():
    """front_branch_first (user decision 2026-11-26, "先规划朝前分支")."""
    from unloading_sim.workbench.single_carton.planner import front_branch_first
    c = load(EXAMPLE)
    assert c['front_branch_first']['mode'] == 'PLAN_BLOCKED_FRONT_BRANCH_FIRST' and c['front_branch_first']['basis']
    assert not [e for e in validate(c) if 'front_branch_first' in e]
    for bad in (dict(mode='PLAN_BLOCKED_FRONT_BRANCH_FIRST'), dict(mode='ON', basis='B'),
                dict(mode='PLAN_BLOCKED_FRONT_BRANCH_FIRST', basis='B', extra=1)):
        assert [e for e in validate(dict(c, front_branch_first=bad)) if 'front_branch_first' in e]
    native = dict(c, planning_module='NATIVE', front_branch_first=dict(mode='PLAN_BLOCKED_FRONT_BRANCH_FIRST', basis='B'))
    assert [e for e in validate(native) if 'front_branch_first' in e]
    facing = dict(arm_facing='FRONT_FIRST')
    on = dict(screen_approach_path=dict(mode='APPROACH_JOINT_LINE', basis='B'),
              front_branch_first=dict(mode='PLAN_BLOCKED_FRONT_BRANCH_FIRST', basis='B'))
    assert front_branch_first(on, facing)
    assert not front_branch_first(on, dict(arm_facing='NONE'))
    assert not front_branch_first(dict(on, front_branch_first=dict(mode='DISABLED')), facing)
    assert not front_branch_first(dict(on, screen_approach_path=dict(mode='ENDPOINTS_ONLY', basis='B')), facing)


def test_front_branch_first_classifies_the_7b07fd06_branches():
    """Field record 7b07fd06 (new start): the two J1 -47.9 deg branches face the
    front, the J1 164.6 deg branch reaches over the back."""
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    from unloading_sim.workbench.single_carton.planner import faces_front
    c = load(EXAMPLE)
    inst = Installation.load(c['execution']['installation'])
    policy = posture.bind(c, CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool))
    d = np.radians
    start = d([64.2, 2.5, -89.4, 5.9, 86.3, -6.4])
    assert faces_front(policy, d([-47.9, 26.6, -96.0, -106.3, -135.7, -176.7]), start)
    assert faces_front(policy, d([-47.9, 12.9, -143.4, 134.8, 135.7, 3.3]), start)
    assert not faces_front(policy, d([164.6, 22.8, 103.5, -129.4, -76.7, 0.9]), start)


def test_front_only_gate_and_via_points():
    """arm_facing=FRONT_ONLY ("取消掉朝后分支") and approach_via_point ("加入流程"), 2026-11-26."""
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    from unloading_sim.workbench.single_carton.planner import approach_vias, approach_via_enabled
    c = load(EXAMPLE)
    assert c['posture_preference']['arm_facing'] == 'FRONT_ONLY'
    assert c['approach_via_point']['mode'] == 'J1_AND_WRIST_FIRST' and approach_via_enabled(c)
    assert not [e for e in validate(c) if 'approach_via_point' in e]
    for bad in (dict(mode='J1_AND_WRIST_FIRST'), dict(mode='VIA', basis='B')):
        assert [e for e in validate(dict(c, approach_via_point=bad)) if 'approach_via_point' in e]
    assert not approach_via_enabled(dict(c, approach_via_point=dict(mode='DISABLED')))
    inst = Installation.load(c['execution']['installation'])
    policy = posture.bind(c, CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool))
    d = np.radians
    start = d([64.2, 2.5, -89.4, 5.9, 86.3, -6.4])
    front = list(d([-47.9, 12.9, -143.4, 134.8, 135.7, 3.3])); back = list(d([164.6, 22.8, 103.5, -129.4, -76.7, 0.9]))
    kept, removed = posture.front_gate(policy, [back, front], start)
    assert kept == [front] and len(removed) == 1 and removed[0]['arm_facing_deg'] > 90
    assert posture.front_gate(dict(policy, arm_facing='FRONT_FIRST'), [back, front], start) == ([back, front], [])
    vias = approach_vias(start, front)
    assert [b for b, _ in vias] == [0., 15., 30.]
    v0 = np.degrees(vias[0][1]); v30 = np.degrees(vias[2][1])
    assert np.allclose(v0, [-47.9, 2.5, -89.4, 134.8, 135.7, 3.3], atol=1e-9)
    # Tilt back = J2 +, J3 - (fix 2026-11-26; the sign was inverted before).
    assert np.allclose(v30, [-47.9, 32.5, -119.4, 134.8, 135.7, 3.3], atol=1e-9)


def _link_origins(policy, q):
    """Source-URDF FK of the bound chain (same transforms as posture.metrics)."""
    from unloading_sim.robot import _axis_angle_transform
    t = np.array(policy['base'], float); out = {}
    for i, j in enumerate(policy['chain']):
        t = t@np.asarray(j['origin'])@_axis_angle_transform(np.asarray(j['axis']), q[i], j['type'])
        out[j['child']] = t.copy()
    return out


def test_via_tilt_back_moves_arm_away_from_front():
    """Checked by forward kinematics, not by the via formula (fix 2026-11-26):
    J2 > 0 leans the upper arm toward link_1 +X, away from the joint_2 shoulder
    offset (arm front, link_1 -X); a larger "arm back" tilt brings the wrist and
    the flange closer to the J1 axis, never farther out toward the stack."""
    from unloading_sim.workbench.coordinates import CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    from unloading_sim.workbench.single_carton.planner import approach_vias
    # posture.bind reads only the pinned source chain (directions in robot
    # base), so no local field installation file is needed here.
    policy = posture.bind(load(EXAMPLE), CoordinateModel(URDF, URDF_SHA, np.eye(4), None))
    d = np.radians
    # Shoulder offset sits on link_1 -X; J2 = +20 deg leans the upper arm to +X.
    shoulder = _link_origins(policy, np.zeros(6))['link_2'][:3, 3]
    assert shoulder[0] < -0.05
    lean = lambda q: (np.linalg.inv(_link_origins(policy, q)['link_1'])[:3, :3]
                      @ (_link_origins(policy, q)['link_3'][:3, 3]-_link_origins(policy, q)['link_2'][:3, 3]))[0]
    assert lean(d([0, 20, 0, 0, 0, 0])) > 0 > lean(d([0, -20, 0, 0, 0, 0]))
    # Field start (observation start 2026-11-25) and two saved approach ends.
    start = d([68.7, 8.6, -93.4, 78.3, 95.8, -8.9])
    for end in (d([0.6, 23.7, -138.6, 114.0, 90.5, 0.1]), d([-47.0, 10.7, -134.2, 128.5, 129.8, 3.8])):
        rows = []
        for back, v in approach_vias(start, end):
            o = _link_origins(policy, np.asarray(v))
            rows.append((back, lean(np.asarray(v)), np.hypot(*o['link_5'][:2, 3]), np.hypot(*o['link_6'][:2, 3])))
        (_, l0, w0, f0), (_, l15, w15, f15), (_, l30, w30, f30) = rows
        assert l0 < l15 < l30                       # upper arm leans further toward link_1 +X
        assert w30 < w15 < w0 and f30 < f15 < f0    # wrist and flange come back toward the J1 axis
        assert f0-f30 > 0.05
