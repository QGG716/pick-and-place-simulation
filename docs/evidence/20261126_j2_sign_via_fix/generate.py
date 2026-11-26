"""Evidence for the 2026-11-26 J2-sign / via fix: flange (link_6 origin) position of every
approach and loaded-return via, old formula (J2 -, J3 +) against the fixed one (J2 +, J3 -),
by the pinned source URDF FK. TCP-independent (no tool length used)."""
import json, sys
import numpy as np
sys.path.insert(0, 'src')
from unloading_sim.workbench.coordinates import CoordinateModel
from unloading_sim.workbench.ui import URDF, URDF_SHA
from unloading_sim.workbench.single_carton.planner import approach_vias, APPROACH_VIA_ARM_BACK_DEG

m = CoordinateModel(URDF, URDF_SHA, np.eye(4), None)
d = np.radians


def old_vias(q, qa):
    out = []
    for back in APPROACH_VIA_ARM_BACK_DEG:
        v = np.array(q, float); v[0] = qa[0]; v[3:] = qa[3:]; v[1] -= d(back); v[2] += d(back)
        out.append((back, v))
    return out


def state(q):
    F = m.robot.named_link_frames(np.asarray(q, float))
    up = np.linalg.inv(F['link_1'])[:3, :3]@(F['link_3'][:3, 3]-F['link_2'][:3, 3])
    p = F['link_6'][:3, 3]
    return dict(q_deg=[round(float(x), 3) for x in np.degrees(q)],
                flange_radial_m=round(float(np.hypot(*p[:2])), 4), flange_z_m=round(float(p[2]), 4),
                upper_arm_toward=('LINK1_PLUS_X_BACK' if up[0] > 0 else 'LINK1_MINUS_X_FRONT'),
                upper_arm_from_vertical_deg=round(float(np.degrees(np.arccos(up[2]/np.linalg.norm(up)))), 2))


start = d([68.7, 8.6, -93.4, 78.3, 95.8, -8.9])   # observation start, user decision 2026-11-25
cases = {
    '38ab81b1': dict(approach_end=d([0.6, 23.7, -138.6, 114.0, 90.5, 0.1]), detach_end=d([0.6, 22.1, -127.6, 104.6, 90.5, 0.1])),
    '649adfc8-box3wrist': dict(approach_end=d([-47.0, 10.7, -134.2, 128.5, 129.8, 3.8]), detach_end=d([-44.0, 11.5, -123.7, 116.9, 126.9, 3.5])),
}
report = dict(scope='OFFLINE_KINEMATICS_ONLY; no planning, no collision check; physical NOT_RUN',
              decision='USER_DECISION_2026_11_26 "先改两处事实错误"',
              fk='pinned ECO65-B source URDF '+URDF_SHA, start_deg=[68.7, 8.6, -93.4, 78.3, 95.8, -8.9],
              sign_check=dict(j2_plus_20=state(d([0, 20, 0, 0, 0, 0]))['upper_arm_toward'],
                              j2_minus_20=state(d([0, -20, 0, 0, 0, 0]))['upper_arm_toward']),
              note='joint angles rounded to 0.1 deg from the saved records (replay-v2.json)', cases={})
for name, c in cases.items():
    row = dict(approach_end=state(c['approach_end']), detach_end=state(c['detach_end']))
    for kind, a, b in (('approach_via', start, c['approach_end']), ('loaded_return_via', start, c['detach_end'])):
        ref = row['approach_end' if kind == 'approach_via' else 'detach_end']['flange_radial_m']
        row[kind] = {}
        for label, rows in (('old_J2_minus', old_vias(a, b)), ('fixed_J2_plus', approach_vias(a, b))):
            row[kind][label] = [dict(arm_back_deg=back, **state(v),
                                     flange_radial_minus_segment_end_m=round(state(v)['flange_radial_m']-ref, 4))
                                for back, v in rows]
    report['cases'][name] = row
print(json.dumps(report, ensure_ascii=False, indent=1))
