"""wrist_down_priority=SECONDARY (user decision 2026-11-25, "降为次要")."""
import pytest
from unloading_sim.workbench.single_carton import posture

BASE = dict(mode='SOFT_LINK_DIRECTIONS', upper_up_weight=4.0, forearm_horizontal_weight=2.0,
            joint_travel_weight=0.05, wrist_down_weight=2.0, basis='TEST')


def test_settings():
    assert posture.settings(dict(posture_preference=dict(BASE, wrist_down_priority='SECONDARY', wrist_down_secondary_band=0.05)))
    for bad in (dict(BASE, wrist_down_priority='FIRST'), dict(BASE, wrist_down_secondary_band=0.05),
                dict({k: v for k, v in BASE.items() if k != 'wrist_down_weight'}, wrist_down_priority='SECONDARY'),
                dict(BASE, wrist_down_priority='SECONDARY', wrist_down_secondary_band=2)):
        with pytest.raises(ValueError):
            posture.settings(dict(posture_preference=bad))


def test_order_groups_arm_scores_within_the_band(monkeypatch):
    table = {  # name: (arm, wrist, travel)
        'backstroke': (2.27, 0.0, 6.5), 'normal_up': (0.21, 3.81, 2.4), 'normal_down': (0.24, 0.0, 3.0),
        'far': (0.33, 0.1, 5.9)}
    def metrics(policy, q, reference):
        arm, wrist, travel = table[q]
        return dict(score=arm+wrist, arm_score=arm, wrist_score=wrist, raw_joint_travel_rad=travel)
    monkeypatch.setattr(posture, 'metrics', metrics)
    policy = dict(BASE, wrist_down_priority='SECONDARY', wrist_down_secondary_band=0.05)
    # 0.21 and 0.24 are one group (wrist decides), 0.33 and 2.27 follow by arm score.
    assert posture.order(policy, list(table), None) == ['normal_down', 'normal_up', 'far', 'backstroke']
    assert posture.order(dict(policy, wrist_down_secondary_band=0.0), list(table), None) == ['normal_up', 'normal_down', 'far', 'backstroke']
    # WEIGHTED keeps the summed score.
    weighted = dict(BASE)
    monkeypatch.setattr(posture, 'key', lambda p, q, r: (table[q][0]+table[q][1],))
    assert posture.order(weighted, list(table), None) == ['normal_down', 'far', 'backstroke', 'normal_up']


def test_arm_facing_front_first(monkeypatch):
    """arm_facing=FRONT_FIRST (user decision 2026-11-25, "加上朝向偏好")."""
    table = {'reach_over': (0.23, 3.8, 3.7, 166.), 'normal': (0.39, 3.8, 5.9, 14.), 'backstroke': (2.27, 0.0, 6.5, 14.)}
    def metrics(policy, q, reference):
        arm, wrist, travel, face = table[q]
        return dict(score=arm+wrist, arm_score=arm, wrist_score=wrist, raw_joint_travel_rad=travel,
                    arm_facing_deg=face, arm_faces_front=face <= 90.)
    monkeypatch.setattr(posture, 'metrics', metrics)
    policy = dict(BASE, wrist_down_priority='SECONDARY', wrist_down_secondary_band=0.05, arm_facing='FRONT_FIRST')
    assert posture.order(policy, list(table), None) == ['normal', 'backstroke', 'reach_over']
    assert posture.order(dict(policy, arm_facing='ANY'), list(table), None) == ['reach_over', 'normal', 'backstroke']
    assert posture.key(policy, 'reach_over', None)[0] == 1 and posture.key(policy, 'normal', None)[0] == 0
    with pytest.raises(ValueError):
        posture.settings(dict(posture_preference=dict(BASE, arm_facing='BACK')))


def test_arm_facing_metric_on_the_source_chain():
    """Literal ECO65 branches of record 649adfc8: front 14 deg, reach-over 166 deg."""
    import numpy as np
    from pathlib import Path
    from unloading_sim.workbench.single_carton.config import load
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    root = Path(__file__).resolve().parents[1]
    c = load(root/'configs/workbench/single_carton.example.json')
    inst = Installation.load(c['execution']['installation'])
    policy = posture.bind(c, CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool))
    assert policy['arm_facing'] == 'FRONT_FIRST'
    ref = np.radians([68.6, 8.6, -93.4, 78.3, 95.8, -8.9])
    front = posture.metrics(policy, np.radians([-47.0, 10.7, -134.2, 128.5, 129.8, 3.8]), ref)
    back = posture.metrics(policy, np.radians([161.9, 25.0, 93.0, -121.9, -79.0, 1.3]), ref)
    assert front['arm_faces_front'] and front['arm_facing_deg'] < 20
    assert not back['arm_faces_front'] and back['arm_facing_deg'] > 160
