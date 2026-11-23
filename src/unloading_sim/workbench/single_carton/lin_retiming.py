"""lin_time_parametrization PATH_TOPP (user decision D, 2026-11-23): retime a Pilz LIN path.

The Pilz LIN result fixes the geometry: its waypoints are IK solutions on the
straight TCP line. Today the line is re-planned at uniformly lower Pilz
velocity/acceleration scales (0.5 ... 0.05) and the send grid is slowed by up
to x2 until every joint stays within the execution limits; one slow region
then slows the whole line. Here the same waypoints get a time-optimal
parametrisation along the path instead:

* path parameter: cumulative TCP distance between consecutive waypoints;
  joints follow a cubic spline through the waypoints in that parameter;
* constraints at every grid point: each joint's velocity and acceleration
  within ``fraction`` of the execution limits, and the TCP translation
  velocity / acceleration and rotation rate within Pilz's own Cartesian
  limits (the values the worker is configured with), i.e. never faster than
  the full-scale Pilz profile along the line;
* backward pass (largest controllable path speed) then forward pass (largest
  admissible acceleration), constant path acceleration per grid interval;
* the result starts and ends at rest (velocity and acceleration zero).

The returned knots go through the unchanged prepare() (exact extremum checks
against the execution limits, send-slowdown factors) and the unchanged
mandatory sent-path audit. The caller falls back to the existing profile on
any failure. Not a motion threshold change: limits are only read.
"""
import numpy as np
from scipy.interpolate import CubicSpline


class RetimingUnavailable(ValueError):
    """The path cannot be retimed here; the caller keeps the existing profile."""


def _rotation_angle(a, b):
    cos = (np.trace(a.T@b)-1.)/2.
    return float(np.arccos(np.clip(cos, -1., 1.)))


def retime(waypoints, tcp, velocity, acceleration, cartesian, fraction=.95, grid_points=None):
    """Knots (t, q, v, a) of the time-optimal parametrisation of ``waypoints`` (K x 6).

    ``tcp(q)`` is the 4x4 task-TCP pose; ``velocity`` / ``acceleration`` the
    joint limits (6,); ``cartesian`` = dict(velocity_m_s, acceleration_m_s2,
    rotation_rad_s). Raises RetimingUnavailable.
    """
    q = np.asarray(waypoints, float)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
        raise RetimingUnavailable('LIN_RETIME_INVALID_WAYPOINTS')
    poses = [np.asarray(tcp(x), float) for x in q]
    step = np.array([np.linalg.norm(poses[k+1][:3, 3]-poses[k][:3, 3]) for k in range(len(q)-1)])
    turn = np.array([_rotation_angle(poses[k][:3, :3], poses[k+1][:3, :3]) for k in range(len(q)-1)])
    keep = np.r_[True, step > 1e-9]
    if keep.sum() < 2:
        raise RetimingUnavailable('LIN_RETIME_NO_TRANSLATION')
    q = q[keep]; sigma = np.r_[0., np.cumsum(step[step > 1e-9])]; turn = turn[step > 1e-9]
    length = float(sigma[-1])
    if length < 1e-4:
        raise RetimingUnavailable('LIN_RETIME_PATH_TOO_SHORT')
    spline = CubicSpline(sigma, q, axis=0)
    n = int(grid_points or min(4000, max(400, 40*len(q))))
    s = np.linspace(0., length, n+1); d = np.diff(s)
    q1 = spline(s, 1); q2 = spline(s, 2)
    rate = np.interp(s, (sigma[:-1]+sigma[1:])/2, turn/np.diff(sigma)) if len(turn) else np.zeros_like(s)
    vmax = np.asarray(velocity, float)*fraction; amax = np.asarray(acceleration, float)*fraction
    vc, ac, wc = float(cartesian['velocity_m_s']), float(cartesian['acceleration_m_s2']), float(cartesian['rotation_rad_s'])
    if not (vc > 0 and ac > 0 and wc > 0):
        raise RetimingUnavailable('LIN_RETIME_INVALID_CARTESIAN_LIMITS')

    # Path acceleration u at grid point i for x = speed**2: every constraint is
    # a pair of lines lo_c(x) = alpha_c + beta_c*x <= u <= hi_c(x) = gamma_c + delta_c*x.
    # Joint j: |q1 u + q2 x| <= amax_j  ->  alpha = -amax/|q1|, gamma = amax/|q1|,
    # beta = delta = -q2/q1 (both signs of q1); TCP: |u| <= ac. A joint with
    # q1 = 0 only bounds x <= amax/|q2|.
    small = abs(q1) < 1e-12
    with np.errstate(divide='ignore', invalid='ignore'):
        alpha = np.where(small, -np.inf, -amax/abs(q1)); gamma = -alpha
        beta = np.where(small, 0., -q2/q1)
        zero_q1 = np.where(small & (abs(q2) > 1e-12), amax/np.maximum(abs(q2), 1e-300), np.inf).min(axis=1)
        speed = np.minimum(vc, np.min(np.where(small, np.inf, vmax/abs(q1)), axis=1))
        speed = np.minimum(speed, np.where(rate > 1e-12, wc/np.maximum(rate, 1e-12), np.inf))
    alpha = np.c_[alpha, np.full(n+1, -ac)]; gamma = np.c_[gamma, np.full(n+1, ac)]; beta = np.c_[beta, np.zeros(n+1)]
    # Largest x with max_c lo_c(x) <= min_c hi_c(x): pairs whose gap closes as x grows.
    slope = beta[:, :, None]-beta[:, None, :]; gap = gamma[:, None, :]-alpha[:, :, None]
    with np.errstate(divide='ignore', invalid='ignore'):
        pair = np.where((slope > 0) & np.isfinite(gap), gap/np.where(slope > 0, slope, 1.), np.inf)
    xmax = np.minimum(np.minimum(speed**2, zero_q1), pair.reshape(n+1, -1).min(axis=1))
    if not np.all(np.isfinite(xmax)) or np.any(xmax < 0):
        raise RetimingUnavailable('LIN_RETIME_UNBOUNDED_SPEED')
    A, B, G = alpha.tolist(), beta.tolist(), gamma.tolist()
    # Backward: largest x from which the end can still be reached at rest,
    # x + 2 d lo_c(x) <= k_{i+1} for every c.
    k = [0.]*(n+1)
    for i in range(n-1, -1, -1):
        best = float(xmax[i]); two = 2*d[i]; nxt = k[i+1]
        for a_, b_ in zip(A[i], B[i]):
            if a_ == -np.inf:
                continue
            coef = 1.+two*b_
            if coef > 0:
                best = min(best, (nxt-two*a_)/coef)
        k[i] = max(best, 0.)
    # Forward: largest admissible acceleration that stays controllable.
    x = np.zeros(n+1); u = np.zeros(n)
    for i in range(n):
        hi = min(g_+b_*x[i] for g_, b_ in zip(G[i], B[i]))
        x[i+1] = max(0., min(k[i+1], x[i]+2*d[i]*hi))
        u[i] = (x[i+1]-x[i])/(2*d[i])
    x[-1] = 0.
    root = np.sqrt(np.maximum(x, 0.))
    denominators = root[:-1]+root[1:]
    if np.any(denominators <= 0):
        raise RetimingUnavailable('LIN_RETIME_STALLED')
    t = np.r_[0., np.cumsum(2*d/denominators)]
    accel = np.r_[u, 0.]
    v = q1*root[:, None]; a = q2*x[:, None]+q1*accel[:, None]
    v[[0, -1]] = 0.; a[[0, -1]] = 0.
    positions = spline(s)
    # Knots: about one per 2 % of the duration, never fewer than the waypoints, ends kept.
    count = int(min(n+1, max(len(q), 50, int(np.ceil(t[-1]/max(t[-1]/300., 1e-3))))))
    index = np.unique(np.r_[np.round(np.linspace(0, n, count)).astype(int), 0, n])
    points = [dict(t=float(t[i]), q=positions[i].tolist(), v=v[i].tolist(), a=a[i].tolist()) for i in index]
    deviation = max(_line_deviation(tcp, positions[i], poses[0][:3, 3], poses[-1][:3, 3]) for i in index)
    return points, dict(method='PATH_TOPP_CUBIC_SPLINE_IN_TCP_DISTANCE', grid_points=n+1, knots=len(points),
                        duration_s=float(t[-1]), path_length_m=length, joint_limit_fraction=fraction,
                        cartesian_limits=dict(cartesian), knot_max_line_deviation_m=deviation)


def _line_deviation(tcp, q, start, end):
    p = np.asarray(tcp(q), float)[:3, 3]; axis = end-start; length = np.linalg.norm(axis)
    if length < 1e-12:
        return float(np.linalg.norm(p-start))
    axis = axis/length; r = p-start
    return float(np.linalg.norm(r-np.dot(r, axis)*axis))
