"""Explicit CAD polylines; asset preparation is never a search-state callback."""
from __future__ import annotations

import hashlib
from pathlib import Path
import numpy as np

from .model import check_transform

TYPE = 'cad_seal_contours_v1'


def validate_geometry(geometry):
    if geometry.get('geometry_type') != TYPE:
        raise ValueError('CONTACT_GEOMETRY_TYPE_MISMATCH')
    rows = geometry.get('contours', [])
    ids = [row['id'] for row in rows]
    if len(rows) != 12 or not all(isinstance(i, str) and i for i in ids) or len(set(ids)) != 12 or ids != geometry['expected_contour_ids']:
        raise ValueError('CONTACT_CONTOUR_IDENTITY_OR_COMPLETENESS')
    check_transform(geometry['flange_from_contact_frame'])
    for row in rows:
        p = np.asarray(row['points_m'], float)
        n = np.asarray(row['normal'], float)
        if (p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or
                not np.isfinite(p).all() or n.shape != (3,) or
                not np.isfinite(n).all() or abs(np.linalg.norm(n)-1) > 1e-8 or
                len(np.unique(p, axis=0)) != len(p) or
                np.linalg.matrix_rank(p-p[0], tol=1e-9) != 2 or
                np.max(np.abs((p-p[0]) @ n)) > 1e-7):
            raise ValueError('CONTACT_CONTOUR_GEOMETRY_INVALID')
        if np.linalg.norm(np.cross(p-p[0], np.roll(p, -1, axis=0)-p[0]).sum(axis=0)) < 1e-12:
            raise ValueError('CONTACT_CONTOUR_DEGENERATE')
    return geometry


def check_contours(geometry, flange, target, size, *, gap, penetration,
                   margin=.003, angle=.01):
    """Independent array implementation, top face only, metres/radians.

    A rectangular target face is convex: testing every polygon vertex proves
    containment of every straight boundary edge. CAD tessellation error is
    reserved separately by the collision policy, not added a second time here.
    """
    validate_geometry(geometry)
    tolerances = np.asarray([gap, penetration, margin, angle])
    if not np.isfinite(tolerances).all() or np.any(tolerances < 0) or angle >= np.pi/2:
        raise ValueError('CONTACT_TOLERANCE_INVALID')
    half = np.asarray(size, float)/2
    if half.shape != (3,) or not np.isfinite(half).all() or np.any(half <= 0):
        raise ValueError('CONTACT_TARGET_INVALID')
    local = np.linalg.inv(check_transform(target)) @ check_transform(flange) @ check_transform(
        geometry['flange_from_contact_frame'])
    results = []
    for row in geometry['contours']:
        pts = np.asarray(row['points_m']) @ local[:3, :3].T + local[:3, 3]
        n = local[:3, :3] @ row['normal']
        errors = pts[:, 2]-half[2]
        reserve = float(np.min(half[:2]-np.abs(pts[:, :2])))
        good = bool(-n[2] >= np.cos(angle) and errors.min() >= -penetration-1e-12
                    and errors.max() <= gap+1e-12 and reserve >= margin-1e-12)
        results.append(dict(id=row['id'], valid=good, min_gap_m=float(errors.min()),
                            max_gap_m=float(errors.max()), edge_margin_m=reserve))
    return dict(status='PASS' if all(r['valid'] for r in results) else 'FAIL',
                contours=results, checked_count=12,
                contact_observation='predicted_geometry_not_physical_contact')


def extract_cad_contours(tool, root):
    """Read all outer terminal loops from existing triangulated CAD surfaces.

    Never convert an unverified fitted radius into a claimed CAD contour.
    The original tool config and CAD meshes remain immutable.
    """
    import trimesh
    if not tool['contact'].get('require_all_seals_fit') or len(tool['seals']) != 12:
        raise ValueError('ALL_12_CAD_SEALS_REQUIRED')
    components = {s['component'] for s in tool['seals']}
    bands = [p for p in tool['collisions'] if p['semantic'] == 'terminal_contact_band']
    if len(components) != 12 or len(bands) != 12 or {b['source_part'] for b in bands} != components:
        raise ValueError('SEAL_TO_CONTACT_BAND_OWNERSHIP_MISMATCH')
    parts = {p['id']: p for p in tool['visual_parts']}
    rows = []
    for seal in tool['seals']:
        path = Path(root)/parts[seal['component']]['visual']
        mesh = trimesh.load(path, force='mesh', process=True)
        vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.faces)
        normal = np.asarray(seal['normal_cad'], float)
        if not np.array_equal(normal, [0, -1, 0]):
            raise ValueError('PINNED_CAD_TERMINAL_NORMAL_CHANGED')
        y = vertices[:, 1].min()
        terminal = faces[np.all(np.abs(vertices[faces, 1]-y) <= 1e-7, axis=1)]
        if not len(terminal):
            raise ValueError('CAD_TERMINAL_PLANAR_FACES_MISSING:'+seal['id'])
        edges = np.sort(np.concatenate([terminal[:, [0, 1]], terminal[:, [1, 2]],
                                        terminal[:, [2, 0]]]), axis=1)
        edges, counts = np.unique(edges, axis=0, return_counts=True)
        adjacency = {}
        for a, b in edges[counts == 1]:
            adjacency.setdefault(int(a), []).append(int(b))
            adjacency.setdefault(int(b), []).append(int(a))
        if not adjacency or any(len(v) != 2 for v in adjacency.values()):
            raise ValueError('CAD_TERMINAL_BOUNDARY_NOT_CLOSED:'+seal['id'])
        unused, loops = set(adjacency), []
        while unused:
            start = min(unused); current = start; previous = None; loop = []
            while True:
                if current in loop:
                    if current != start: raise ValueError('CAD_TERMINAL_BOUNDARY_INVALID')
                    break
                loop.append(current); unused.remove(current)
                nxt = next(v for v in adjacency[current] if v != previous)
                previous, current = current, nxt
            loops.append(vertices[loop])
        def area(p):
            x, z = p[:, 0], p[:, 2]
            return abs(float(x @ np.roll(z, -1)-z @ np.roll(x, -1)))/2
        outer = max(loops, key=area)
        # An annular terminal face has exactly one outer and one inner loop.
        # Ambiguous disconnected terminal patches are rejected for inspection.
        if len(loops) != 2 or min(map(area, loops)) <= 0 or area(outer) <= 0:
            raise ValueError('CAD_TERMINAL_ANNULUS_AMBIGUOUS:'+seal['id'])
        inner = next(p for p in loops if p is not outer)
        polygon = outer[:, [0, 2]]
        for point in inner[:, [0, 2]]:
            inside = False
            for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                if (a[1] > point[1]) != (b[1] > point[1]):
                    crossing = a[0]+(point[1]-a[1])*(b[0]-a[0])/(b[1]-a[1])
                    if point[0] < crossing: inside = not inside
            if not inside: raise ValueError('CAD_TERMINAL_LOOPS_NOT_NESTED:'+seal['id'])
        rows.append(dict(id=seal['id'], component=seal['component'], points_m=outer.tolist(),
                         normal=normal.tolist(), mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                         source='existing_CAD_terminal_face_outer_boundary',
                         original_fitted_ring_used=False))
    result = dict(geometry_type=TYPE, max_normal_misalignment_rad=tool['contact']['normal_tolerance_rad'],
                  expected_contour_ids=[r['id'] for r in rows],
                  contours=rows, flange_from_contact_frame=tool['T_flange_tool_cad'])
    return validate_geometry(result)
