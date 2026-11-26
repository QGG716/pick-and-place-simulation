"""Release with the tool facing down at the unchanged release position
(user decision 2026-11-26: "释放的姿态要改为吸具朝向-Z，释放位置保持不变";
after the release "释放时相机已经面向箱堆，可以做下一箱的轨迹规划，所以机械臂
直接从释放点规划到下一箱的接近点").

``release_pose={mode: TOOL_DOWN_SAME_POSITION, basis}`` with the original
four-segment cycle (no ``cycle_structure``): the loaded return ends at
q_release instead of q_start, where

* the TCP position is the TCP position at the frozen q_start (the release
  position used so far) - same X and Y; when the tool cannot face down there
  (field 9ec45e42: 0.58 m from the base axis at 0.484 m height is out of
  reach with the tool down) the height is lowered, coarse 2 cm then 1 cm
  steps, to the highest height with an IK solution, never below the
  workspace floor (user decision 2026-11-26 "水平位置不变，自动降高度"), and
* the tool axis (TCP +Z, the suction direction) is world -Z, reached from the
  q_start orientation by the smallest rotation (no other turn about Z);

the carton is released there and the round ends there; the next round
observes and plans from that pose (its frozen q_start). When q_start already
has the tool facing down within ``ALREADY_DOWN_DEG`` the release is at q_start,
as before. q_release is a native IK solution checked by FK; its loaded end
state and the loaded return keep every check (see loaded_return_collision for
the record-only option).
"""
import numpy as np

MODE = 'TOOL_DOWN_SAME_POSITION'
ALREADY_DOWN_DEG = 1.0
FK_POSITION_TOLERANCE_M = 1e-3
FK_AXIS_TOLERANCE_DEG = 0.5


def policy(c):
    value = (c or {}).get('release_pose')
    if value is None or (isinstance(value, dict) and value.get('mode') == 'DISABLED'):
        return None
    if (not isinstance(value, dict) or value.get('mode') != MODE or set(value) - {'mode', 'basis'}
            or not isinstance(value.get('basis'), str) or not value['basis'].strip()):
        raise ValueError('RELEASE_POSE_POLICY_INVALID')
    if (c or {}).get('cycle_structure') is not None:
        raise ValueError('RELEASE_POSE_REQUIRES_THE_FOUR_SEGMENT_CYCLE')
    return dict(value)


def _rotation(a, b):
    """Smallest rotation taking unit vector a to unit vector b."""
    a = np.asarray(a, float)/np.linalg.norm(a); b = np.asarray(b, float)/np.linalg.norm(b)
    v = np.cross(a, b); c = float(a@b); s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        # Opposite: half turn about any axis normal to a.
        axis = np.cross(a, [1., 0, 0]) if abs(a[0]) < .9 else np.cross(a, [0, 1., 0])
        axis /= np.linalg.norm(axis)
        return 2*np.outer(axis, axis)-np.eye(3)
    k = v/s; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    angle = np.arctan2(s, c)
    return np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*K@K


COARSE_STEP_M = .02
FINE_STEP_M = .01


def goal(T_tcp_start, lower_by_m=0.):
    """(goal TCP pose, rotation angle deg); None when the tool already faces down."""
    T = np.asarray(T_tcp_start, float)
    z = T[:3, 2]
    angle = float(np.degrees(np.arccos(np.clip(z@np.array([0, 0, -1.]), -1, 1))))
    if angle <= ALREADY_DOWN_DEG:
        return None, angle
    G = np.eye(4); G[:3, :3] = _rotation(z, [0, 0, -1.])@T[:3, :3]; G[:3, 3] = T[:3, 3]-[0, 0, lower_by_m]
    return G, angle


def lowering_schedule(start_z, floor_z):
    """Coarse lowering steps (m) from the start height down to the floor."""
    steps = []; k = 0
    while start_z-k*COARSE_STEP_M >= floor_z-1e-9:
        steps.append(round(k*COARSE_STEP_M, 6)); k += 1
    return steps


def check(T_tcp_start, T_tcp_release, lower_by_m=0.):
    """FK evidence of a candidate q_release (target: same X, Y; height lowered by lower_by_m)."""
    A = np.asarray(T_tcp_start, float).copy(); A[2, 3] -= lower_by_m; B = np.asarray(T_tcp_release, float)
    position = float(np.linalg.norm(A[:3, 3]-B[:3, 3]))
    axis = float(np.degrees(np.arccos(np.clip(B[:3, 2]@np.array([0, 0, -1.]), -1, 1))))
    return dict(position_error_m=position, tool_axis_from_down_deg=axis, lowered_by_m=float(lower_by_m),
                valid=position <= FK_POSITION_TOLERANCE_M and axis <= FK_AXIS_TOLERANCE_DEG,
                tolerance=dict(position_m=FK_POSITION_TOLERANCE_M, axis_deg=FK_AXIS_TOLERANCE_DEG))
