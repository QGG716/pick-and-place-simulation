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
