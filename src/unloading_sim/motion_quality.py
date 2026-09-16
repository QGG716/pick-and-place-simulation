"""Geometry-only path diagnostics. Angles are never wrapped or rewritten."""
from __future__ import annotations

import numpy as np
from time import perf_counter


class QualityDeadline(TimeoutError):
    """Optional scoring stopped; this says nothing about path validity."""


def reversal_counts(positions, threshold_rad=0.02):
    """Count changes of direction only after a threshold-sized excursion."""
    q = np.asarray(positions, dtype=float)
    counts = []
    for values in q.T:
        anchor = extreme = float(values[0])
        direction = count = 0
        for value in values[1:]:
            if direction == 0:
                if abs(value - anchor) >= threshold_rad:
                    direction = 1 if value > anchor else -1
                    extreme = value
            elif direction * (value - extreme) > 0:
                extreme = value
            elif direction * (extreme - value) >= threshold_rad:
                count += 1
                direction = -direction
                extreme = value
        counts.append(count)
    return counts


def path_quality(positions, *, fk=None, velocity_limits=None, joint_limits=None, jacobian=None,
                 deadline=None, joint_names=None):
    def check():
        if deadline is not None and perf_counter() >= deadline:
            raise QualityDeadline("optional path quality deadline")
    check()
    q = np.asarray(positions, dtype=float)
    delta = np.abs(np.diff(q, axis=0))
    travel = delta.sum(axis=0)
    net = np.abs(q[-1] - q[0])
    names = list(joint_names) if joint_names is not None else []
    wrist_indices = [names.index(name) for name in ("J4", "J5", "J6") if name in names]
    # Generic callers retain the old six-axis score; named diagnostics never guess J6.
    if not names:
        wrist_indices = list(range(3, min(6, q.shape[1])))
    j6 = names.index("J6") if "J6" in names else None
    reversals = reversal_counts(q)
    orientation = None
    tcp_length = None
    if fk is not None:
        # A long joint edge can turn through more than pi and return to a
        # similar endpoint orientation. Sample the actual joint edge so SO(3)
        # endpoint distance cannot hide that travel after shortcutting.
        samples = [q[0]]
        for first, last in zip(q[:-1], q[1:]):
            check()
            count = max(1, int(np.ceil(np.max(np.abs(last-first))/.04)))
            samples.extend(first + i/count*(last-first) for i in range(1, count+1))
        rotations, translations = [], []
        for point in samples:
            check()
            pose = np.asarray(fk(point))
            rotations.append(pose[:3, :3])
            translations.append(pose[:3, 3])
            check()
        orientation = float(sum(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1))
                                for a, b in zip(rotations[:-1], rotations[1:])))
        tcp_length = float(np.linalg.norm(np.diff(translations, axis=0), axis=1).sum())
    duration = None if velocity_limits is None else float(
        np.max(delta / np.asarray(velocity_limits), axis=1).sum())
    margin = None if joint_limits is None else float(np.min(np.minimum(
        q - np.asarray(joint_limits)[:, 0], np.asarray(joint_limits)[:, 1] - q)))
    condition = None
    if jacobian is not None:
        conditions = []
        for point in q:
            check()
            conditions.append(np.linalg.cond(jacobian(point)))
            check()
        condition = float(max(conditions))
    check()
    # Soft weights convert each component to a dimensionless preference. All
    # collision, joint margin and singularity acceptance remains external.
    score = float(travel.sum() + .5 * travel[wrist_indices].sum() + .1 * sum(reversals)
                  + .25 * (orientation or 0) + .1 * (duration or 0) + .02*np.log1p(condition or 0)
                  + (.02 / max(margin, .001) if margin is not None else 0))
    return dict(nodes=len(q), joint_travel_rad=travel.tolist(), total_joint_travel_rad=float(travel.sum()),
                joint_names=names, start_q_rad=q[0].tolist(), end_q_rad=q[-1].tolist(),
                joint_net_rad=net.tolist(), joint_extra_rad=(travel-net).tolist(),
                j6_index=j6, j6_total_rad=None if j6 is None else float(travel[j6]),
                j6_net_rad=None if j6 is None else float(net[j6]),
                j6_extra_rad=None if j6 is None else float(travel[j6]-net[j6]),
                tcp_path_length_m=tcp_length,
                wrist_travel_rad=travel[wrist_indices].tolist(), wrist_total_rad=float(travel[wrist_indices].sum()),
                reversals=reversals, reversal_threshold_rad=.02, orientation_travel_so3_rad=orientation,
                orientation_max_joint_sample_step_rad=.04,
                velocity_only_time_lower_bound_s=duration, minimum_joint_margin_rad=margin,
                maximum_jacobian_condition=condition, soft_score=score,
                score_weights=dict(joint_rad=1., wrist_rad=.5, reversal=.1, orientation_rad=.25,
                                   velocity_lower_bound_s=.1, inverse_joint_margin_rad=.02, log_jacobian_condition=.02))


def quality_improves(before, after, tolerance_rad=1e-6):
    """Replacement preference, never a safety gate. No added wrist score weight.

    Require a strictly lower original soft score and no increase beyond numerical
    tolerance in J6, total wrist, or total joint travel. Unknown cannot win.
    """
    if before is None or after is None or after["soft_score"] >= before["soft_score"] - 1e-9:
        return False
    for name in ("j6_total_rad", "wrist_total_rad", "total_joint_travel_rad"):
        a, b = before.get(name), after.get(name)
        if a is not None and (b is None or b > a + tolerance_rad):
            return False
    return True


def collinear_indices(positions, protected=(), tolerance=1e-12):
    """Remove only forward collinear samples; protect process boundaries."""
    q = np.asarray(positions, dtype=float)
    keep = [0]
    protected = set(protected) | {0, len(q) - 1}
    for i in range(1, len(q) - 1):
        a, b = q[i] - q[keep[-1]], q[i + 1] - q[keep[-1]]
        fraction = float(a @ b / (b @ b)) if b @ b > tolerance**2 else -1.
        if i in protected or not (0 <= fraction <= 1 and np.max(np.abs(a - fraction*b)) <= tolerance):
            keep.append(i)
    if len(q) > 1:
        keep.append(len(q) - 1)
    return keep
