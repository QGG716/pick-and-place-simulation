"""S50 quasi-static load screening (plan item 0.3).

Pure NumPy. Estimates link mass properties from the CAD meshes with a uniform
density scaled to the published 156 kg total, then evaluates gravity joint
torques, base overturning moment and the manufacturer payload curve on actual
replay states. Every value derived from CAD volume is an ESTIMATE; the
manufacturer has not supplied link inertials or joint torque ratings.
"""
from __future__ import annotations

import numpy as np

GRAVITY_M_S2 = 9.80665
# Digitized from S-series-mechanical.pdf, PDF page 37, "Payload Diagram of S50".
# Offset is the distance from the load centre of gravity to the flange centre.
# Reading uncertainty is about +/-0.5 kg between grid lines.
S50_PAYLOAD_CURVE_MM_KG = ((0., 50.), (40., 50.), (50., 41.7), (450., 35.0), (550., 30.0),
                           (600., 28.0), (650., 26.1), (700., 24.5), (750., 23.0), (800., 21.8))
_CANONICAL = np.array([[2., 1., 1.], [1., 2., 1.], [1., 1., 2.]]) / 120.


def payload_curve_limit_kg(offset_m, curve=S50_PAYLOAD_CURVE_MM_KG):
    """Allowed mass at a centre-of-gravity offset; None beyond the published range."""
    points = np.asarray(curve, dtype=float)
    offset_mm = float(offset_m) * 1000.
    if offset_mm < 0:
        raise ValueError('NEGATIVE_PAYLOAD_OFFSET')
    if offset_mm > points[-1, 0] + 1e-9:
        return None
    return float(np.interp(offset_mm, points[:, 0], points[:, 1]))


def mesh_mass_properties(vertices, triangles):
    """Unit-density volume, centroid and inertia about the centroid of a closed mesh.

    Also reports the boundary-edge count; a non-zero count means the mesh is
    not closed and the volume is only an approximation.
    """
    v = np.asarray(vertices, dtype=float)
    t = np.asarray(triangles, dtype=np.int64)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    det = np.einsum('ij,ij->i', a, np.cross(b, c))
    volume = det.sum() / 6.
    if not np.isfinite(volume) or abs(volume) < 1e-15:
        raise ValueError('MESH_HAS_NO_VOLUME')
    centroid = (det[:, None] * (a + b + c)).sum(0) / 24. / volume
    stacked = np.stack([a, b, c], axis=2)  # columns are the three vertices
    second = np.einsum('n,nij,jk,nlk->il', det, stacked, _CANONICAL, stacked)
    if volume < 0:
        volume, second = -volume, -second
    origin_inertia = np.trace(second) * np.eye(3) - second
    inertia = origin_inertia - volume * (centroid @ centroid * np.eye(3) - np.outer(centroid, centroid))
    edges = np.sort(np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return dict(volume_m3=float(volume), centroid_m=centroid, inertia_unit_density=0.5 * (inertia + inertia.T),
                boundary_edges=int((counts == 1).sum()), nonmanifold_edges=int((counts > 2).sum()))


def scale_to_total_mass(properties, total_mass_kg):
    """Uniform density over all links so the summed mass equals the published total."""
    density = float(total_mass_kg) / sum(p['volume_m3'] for p in properties.values())
    return {name: dict(mass_kg=p['volume_m3'] * density, com_local_m=np.asarray(p['centroid_m']),
                       inertia_com_kg_m2=p['inertia_unit_density'] * density)
            for name, p in properties.items()}, density


def gravity_torques(points, axes, masses_and_positions):
    """Static joint torque needed to hold point masses against gravity.

    ``points``/``axes`` are per-joint world origins and unit axes;
    ``masses_and_positions`` is a list of (joint_index_first_moved, mass, world_position).
    A mass is carried by every joint whose index is <= its first moved joint.
    Returned torques are the actuator torques that balance gravity.
    """
    weight = np.array([0., 0., -GRAVITY_M_S2])
    torque = np.zeros(len(axes))
    for carried_from, mass, position in masses_and_positions:
        for j in range(carried_from + 1):
            torque[j] -= axes[j] @ np.cross(np.asarray(position) - points[j], mass * weight)
    return torque


def base_moment(mount_point, masses_and_positions):
    """Gravity moment of the moving masses about the mount point, and their weight."""
    weight = np.array([0., 0., -GRAVITY_M_S2])
    moment = np.zeros(3)
    total = 0.
    for _, mass, position in masses_and_positions:
        moment += np.cross(np.asarray(position) - mount_point, mass * weight)
        total += mass
    return moment, total * GRAVITY_M_S2
