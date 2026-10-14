"""Build a private measured tool from the supplied flange-bearing STEP assembly.

This adapter recognizes the reviewed component families and fails on ambiguity.
It creates no supplementary mechanical parts and makes no hardware claim.
"""
from pathlib import Path
import hashlib
import json
import shutil
from datetime import datetime, timezone
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
D = ROOT/'assets/tools/desktop_suction/derived'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plane_loops(mesh, normal, offset, tolerance=1e-7):
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    f = f[np.all(abs(v[f]@normal-offset) <= tolerance, axis=1)]
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    edges, count = np.unique(edges, axis=0, return_counts=True)
    adjacency = {}
    for a, b in edges[count == 1]:
        adjacency.setdefault(int(a), []).append(int(b)); adjacency.setdefault(int(b), []).append(int(a))
    if not adjacency or any(len(x) != 2 for x in adjacency.values()): raise ValueError('OPEN_INTERFACE_BOUNDARY')
    left, loops = set(adjacency), []
    while left:
        cur = min(left); prev = None; loop = []
        while cur not in loop:
            loop.append(cur); left.remove(cur)
            nxt = next(x for x in adjacency[cur] if x != prev); prev, cur = cur, nxt
        if cur != loop[0]: raise ValueError('INVALID_INTERFACE_LOOP')
        loops.append(v[loop])
    return loops


def clipped_hull(mesh, lo, hi):
    """Cover the exact triangle/slab intersections, preserving axial recesses."""
    v = np.asarray(mesh.vertices); triangles = v[np.asarray(mesh.faces)]
    points = list(v[(v[:, 1] >= lo-1e-12) & (v[:, 1] <= hi+1e-12)])
    for cut in (lo, hi):
        for ia, ib in ((0, 1), (1, 2), (2, 0)):
            a, b = triangles[:, ia], triangles[:, ib]
            mask = (a[:, 1]-cut)*(b[:, 1]-cut) < 0; a, b = a[mask], b[mask]
            if len(a): points.extend(a+(cut-a[:, 1])[:, None]/(b[:, 1]-a[:, 1])[:, None]*(b-a))
    hull = trimesh.convex.convex_hull(np.asarray(points))
    from scipy.spatial import ConvexHull
    eq = ConvexHull(hull.vertices).equations
    pts = np.asarray(points)
    error = max(float(np.max(pts@e[:, :3].T+e[:, 3])) for e in np.array_split(eq, max(1, len(eq)//64)))
    if error > 1e-9: raise ValueError('CLIPPED_HULL_UNDERCOVERAGE')
    return hull, error


def build():
    cad_path = D/'manifests/cad.json'; cad = json.loads(cad_path.read_text(encoding='utf-8'))
    if sha(ROOT/cad['source']) != cad['source_sha256']: raise ValueError('STEP_CHANGED')
    out = ROOT/'configs/local/eco65_desktop/tool.json'
    if out.exists():
        backup = ROOT/'outputs/eco65_native/asset_backups'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup.mkdir(parents=True); shutil.copy2(out, backup/'tool.json')
        shutil.copytree(D/'collision', backup/'collision')
    parts = cad['parts']
    flange, = [p for p in parts if p['name'].startswith('L57-')]
    cups = [p for p in parts if p['name'].startswith('FG42S-G2M') and p['solid_index'] == 0]
    if len(cups) != 12 or len({p['instance_id'] for p in cups}) != 12: raise ValueError('ACTUAL_CUP_COUNT_CHANGED')
    surfaces = flange['surface_features']
    faces = [f for f in surfaces if f['type'] == 'plane' and np.allclose(f['outward_normal'], [0, 1, 0])]
    mount_face = max(faces, key=lambda f: f['point_m'][1]); mount_y = mount_face['point_m'][1]
    outer, = [w for w in mount_face['wires'] if w['outer']]
    holes = [f for f in surfaces if f['type'] == 'cylinder' and abs(f['axis'][1]) > .999
             and abs(f['bounds_m'][1][1]-mount_y) < 1e-8 and f['radius_m'] < .005]
    centers = np.unique(np.round([np.array(f['point_m'])[[0, 2]] for f in holes], 10), axis=0)
    if len(centers) != 6 or np.max(np.ptp(np.linalg.norm(centers, axis=1))) > 1e-8:
        raise ValueError('FLANGE_BOLT_CIRCLE_AMBIGUOUS')
    center = centers.mean(axis=0)
    if np.linalg.norm(center) > 1e-8: raise ValueError('FLANGE_AXIS_NOT_AT_CAD_XZ_ZERO')
    # Read the actual official end face; do not substitute a drawing origin.
    official = ROOT/'assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/meshes/link_6.STL'
    mesh = trimesh.load(official, force='mesh', process=True)
    if abs(mesh.bounds[1, 2]) > 1e-7: raise ValueError('OFFICIAL_FLANGE_PLANE_CHANGED')
    loops = plane_loops(mesh, np.array([0, 0, 1]), 0.)
    # Six identical smallest holes on the bolt circle; the seventh off-angle
    # hole is a distinct locating feature, absent on the supplied tool flange.
    radii = [np.mean(np.linalg.norm(p[:, :2]-p[:, :2].mean(axis=0), axis=1)) for p in loops]
    robot_holes = np.array([p[:, :2].mean(axis=0) for p, r in zip(loops, radii) if abs(r-min(radii)) < 1e-6])
    if len(robot_holes) != 6 or np.max(abs(np.linalg.norm(robot_holes, axis=1)-np.linalg.norm(centers, axis=1).mean())) > 1e-6:
        raise ValueError('OFFICIAL_TOOL_BOLT_CIRCLE_MISMATCH')
    angle = float(np.angle(np.mean(np.exp(6j*np.arctan2(robot_holes[:, 1], robot_holes[:, 0]))))/6)
    c, s = np.cos(angle), np.sin(angle); yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    basis = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.]])
    mount = np.eye(4); mount[:3, :3] = yaw@basis; mount[:3, 3] = -mount[:3, :3]@np.array([0, mount_y, 0])
    seals, collisions = [], []
    for p in parts:
        if sha(ROOT/p['visual']) != p['visual_sha256']: raise ValueError('VISUAL_CHANGED')
        m = trimesh.load(ROOT/p['visual'], force='mesh', process=True)
        regions = [(p['id'], -np.inf, np.inf, 'conservative_rigid')]
        if p in cups:
            torus = [f for f in p['surface_features'] if f['type'] == 'torus' and
                     'axial_tangent_curve_m' in f and np.allclose(f['axis'], [0, -1, 0]) and
                     abs(f['bounds_m'][0][1]-p['bounds_m'][0][1]) < 2e-7]
            torus.sort(key=lambda f: f['uv_bounds'][0])
            if len(torus) != 2 or abs(torus[0]['uv_bounds'][0]) > 1e-8 or abs(torus[-1]['uv_bounds'][1]-2*np.pi) > 1e-8:
                raise ValueError('TERMINAL_TORUS_NOT_COMPLETE')
            a, b = [np.array(f['axial_tangent_curve_m']) for f in torus]
            if np.linalg.norm(a[-1]-b[0]) > 1e-9 or np.linalg.norm(b[-1]-a[0]) > 1e-9: raise ValueError('OPEN_CAD_TANGENT_CONTOUR')
            ring = np.concatenate([a[:-1], b[:-1]]); lip_y = float(ring[:, 1].mean())
            # Existing terminal-skirt semantics: split at the actual nearest
            # substantial annular plane behind the rounded lip, never whole cup.
            planes = [f for f in p['surface_features'] if f['type'] == 'plane' and abs(f['axis'][1]) > .999
                      and f['area_m2'] > .0001 and lip_y+.001 < f['point_m'][1] < lip_y+.01]
            if not planes: raise ValueError('TERMINAL_SKIRT_SPLIT_UNRESOLVED')
            cut = min(f['point_m'][1] for f in planes)
            seals.append(dict(id=p['id']+'_seal', component=p['id'], center_cad_m=ring.mean(axis=0).tolist(),
                normal_cad=[0, -1, 0], ring_cad_m=ring.tolist(), cad_face_indices=[f['face_index'] for f in torus],
                contour_method='exact_trimmed_CAD_torus_axial_tangent', original_fitted_ring_used=False,
                max_chord_error_m=float(torus[0]['major_radius_m']*(1-np.cos(.02/2))),
                contact_band=p['id']+'_lip', split_plane_cad_y_m=cut, real_compression_stroke_m=None))
            regions = [(p['id']+'_rigid', cut, np.inf, 'conservative_rigid'),
                       (p['id']+'_lip', -np.inf, cut, 'terminal_contact_band')]
        if p is flange:
            cuts = sorted(set(round(f['point_m'][1], 10) for f in surfaces if f['type'] == 'plane' and abs(f['axis'][1]) > .999))
            regions = [(p['id']+'_region_'+str(i), lo, hi, 'conservative_rigid') for i, (lo, hi) in enumerate(zip(cuts, cuts[1:]))]
        for ident, lo, hi, semantic in regions:
            hull, error = clipped_hull(m, lo, hi)
            path = D/'collision'/(ident+'.obj'); hull.export(path)
            collisions.append(dict(id=ident, source_part=p['id'], source_instance=p['instance_id'], mesh=path.relative_to(ROOT).as_posix(),
                mesh_sha256=sha(path), visual_sha256=p['visual_sha256'], semantic=semantic,
                axial_slab_m=[None if np.isinf(x) else x for x in (lo, hi)],
                coverage='convex hull of triangle/slab intersections', max_vertex_outside_hull_m=error))
    if np.ptp([s['center_cad_m'][1] for s in seals]) > 1e-9: raise ValueError('NONCOPLANAR_SEALS')
    tcp = np.eye(4); tcp[:3, :3] = basis.T; tcp[:3, 3] = np.mean([s['center_cad_m'] for s in seals], axis=0)
    contact_part = next(x['id'] for x in collisions if x['source_part'] == flange['id'] and x['axial_slab_m'][1] == round(mount_y, 10))
    result = dict(schema='eco65_tool_v1', source_sha256=cad['source_sha256'], cad_manifest_sha256=sha(cad_path),
        transform_convention='T_A_B maps B to A', length_unit='m', T_flange_tool_cad=mount.tolist(),
        T_tool_cad_tcp=tcp.tolist(), T_flange_tcp=(mount@tcp).tolist(), visual_parts=parts, collisions=collisions, seals=seals,
        adapter=[], mounting_contacts=[dict(robot_link='link_6', tool_collision=contact_part, source_part=flange['id'],
            interface='actual_STEP_flange_outer_installation_face', maximum_penetration_m=.0004)],
        assembly_status='SIMULATION_MOUNT_ASSUMPTION', assembly_assumption='CAD_installation_plane_and_official_mesh_bolt_circle',
        mounting_evidence=dict(flange_part=flange['id'], flange_instance=flange['instance_id'],
            mount_face_cad_index=mount_face['face_index'], mount_point_cad_m=[0, mount_y, 0],
            mount_normal_cad=[0, 1, 0], bolt_centers_cad_xz_m=centers.tolist(), official_bolt_centers_xy_m=robot_holes.tolist(),
            official_flange_mesh_sha256=sha(official), selected_roll_rad=angle,
            candidate_rolls_rad=[angle+k*np.pi/3 for k in range(6)],
            ambiguity='Six bolt-pattern-equivalent rolls; no matching tool-side locating-pin feature. Choose nearest zero for simulation only.',
            duplicate_robot_geometry='None: STEP flange occupies positive F Z; official link_6 occupies negative F Z.',
            generated_adapter_disposition='disk/cross_plate/four_legs all removed; actual supplied flange retained at measured face, no added standoff'),
        tcp_evidence='Mean of twelve exact CAD torus tangent loci; TCP +X CAD X, +Y CAD Z, +Z CAD -Y; contact frame is original CAD.',
        contact=dict(ideal_suction=True, geometric_position_tolerance_m=.0004, normal_tolerance_rad=.01,
            require_all_seals_fit=True, independent_control_verified=False,
            terminal_band_thickness_m=max(s['split_plane_cad_y_m']-s['center_cad_m'][1] for s in seals),
            terminal_band_semantics='Conservative terminal skirt, rounded CAD lip tangent; compliance not measured'),
        physics=dict(tool_mass_kg=1., adapter_mass_kg=0., actual_mass_kg=None, actual_material=None,
            actual_tcp_calibration=None, vacuum_circuit=None, source='Finite display assumption only; Dynamics NOT_EVALUATED'))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(solids=len(parts), seals=len(seals), flange=flange['id'], collisions=len(collisions),
                         tcp_F=(mount@tcp).tolist(), mounting_contact=contact_part)))
    return result


if __name__ == '__main__': build()
