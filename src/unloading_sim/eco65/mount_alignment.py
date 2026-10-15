"""Six-hole simulation alignment; no claim of unique hardware indexing."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def relative_roll(robot_xy, tool_xz):
    phases = []
    for value in (robot_xy, tool_xz):
        p = np.asarray(value, float)
        if p.shape != (6, 2) or not np.isfinite(p).all():
            raise ValueError('SIX_FINITE_HOLES_REQUIRED')
        phase = np.mean(np.exp(6j*np.arctan2(p[:, 1], p[:, 0])))
        if abs(phase) < .999: raise ValueError('REGULAR_SIX_HOLE_PATTERN_REQUIRED')
        phases.append(phase)
    return float(np.angle(phases[0]/phases[1])/6)


def check_alignment(robot_xy, tool_xz, mount_y, mount, *, tolerance=1e-6):
    robot = np.column_stack([robot_xy, np.zeros(6)])
    points = np.column_stack([np.asarray(tool_xz)[:, 0], np.full(6, mount_y), np.asarray(tool_xz)[:, 1]])
    T = np.asarray(mount, float)
    points = points@T[:3, :3].T+T[:3, 3]
    rows, cols = linear_sum_assignment(np.linalg.norm(points[:, None]-robot[None], axis=2))
    errors = np.linalg.norm(points[rows]-robot[cols], axis=1)
    center = T[:3, :3]@np.array([0, mount_y, 0])+T[:3, 3]
    normal = T[:3, :3]@np.array([0., 1., 0.])
    report = dict(max_hole_residual_m=float(errors.max()), rms_hole_residual_m=float(np.sqrt(np.mean(errors**2))),
        tool_to_robot_indices=cols.tolist(), transformed_hole_centers_F_m=points.tolist(),
        mount_center_F_m=center.tolist(), mount_normal_F=normal.tolist(),
        plane_residual_m=float(np.max(abs(points[:, 2]))), axis_offset_m=float(np.linalg.norm(center[:2])),
        opposing_normal_error=float(np.linalg.norm(normal-[0, 0, -1])), hardware_indexing='NOT_VERIFIED')
    report['status'] = 'PASS' if (errors.max() <= tolerance and np.linalg.norm(center) <= tolerance
        and report['opposing_normal_error'] <= 1e-8) else 'FAIL'
    return report
