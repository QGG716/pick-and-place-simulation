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
