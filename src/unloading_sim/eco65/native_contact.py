"""Explicit CAD polylines; asset preparation is never a search-state callback."""
from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from pathlib import Path
import numpy as np

from .model import check_transform

TYPE = 'cad_seal_contours_v1'
TASK_TYPE = 'eco65_single_carton_task_v1'
INDEPENDENT = 'SIMULATED_INDEPENDENT_CUPS'
FACE_AXES = {'top': (2, 1.), 'front': (0, -1.), 'left': (1, 1.), 'right': (1, -1.)}
EDGE_ALIGNMENT = dict(schema='rigid_tool_face_edge_alignment_v1',
    axes_contact_frame=[[1.,0.,0.],[0.,0.,1.]], tolerance_rad=float(np.radians(1.)),
    source='RIGID_PLATE_CAD_X_Z_EQUIVALENT_TO_FIXED_TCP_X_Y',
    allowed_matching='DIRECT_OR_SWAPPED_ABSOLUTE_DOT')


def check_edge_alignment(geometry, local, face):
    rule = geometry.get('edge_alignment')
    if rule is None:
        return dict(status='NOT_REQUIRED_LEGACY_GEOMETRY')
    if rule != EDGE_ALIGNMENT:
        raise ValueError('CONTACT_EDGE_ALIGNMENT_CONTRACT_INVALID')
    axis, _ = FACE_AXES[face]
    others = [k for k in range(3) if k != axis]
    axes = np.asarray(rule['axes_contact_frame'])@local[:3,:3].T
    errors = np.arccos(np.clip(np.abs(axes[:,others]),0.,1.))
    direct = [float(errors[0,0]),float(errors[1,1])]
    swapped = [float(errors[0,1]),float(errors[1,0])]
    chosen = direct if max(direct)<=max(swapped) else swapped
    passed = max(chosen)<=rule['tolerance_rad']+1e-12
    return dict(status='PASS' if passed else 'FAIL',
        reason=None if passed else 'TOOL_FACE_EDGES_NOT_ALIGNED',
        matching='DIRECT' if chosen is direct else 'SWAPPED',
        angle_errors_rad=chosen,maximum_angle_error_rad=max(chosen),
        tolerance_rad=rule['tolerance_rad'],tool_axes_target_local=axes.tolist(),
        face_axes_target_local=np.eye(3)[others].tolist(),
        source='ACTUAL_FLANGE_FK_AND_FIXED_RIGID_CAD_AXES')


def selected_ids(geometry, active_ids=None):
    expected = geometry['expected_contour_ids']
    active = list(expected) if active_ids is None else active_ids
    if (not isinstance(active, list) or not active or
            any(not isinstance(i, str) or i not in expected for i in active) or
            len(set(active)) != len(active)):
        raise ValueError('CONTACT_ACTIVE_SELECTION_INVALID')
    return [i for i in expected if i in active]


def face_outward(target, face):
    if face not in FACE_AXES: raise ValueError('CONTACT_TARGET_FACE_INVALID')
    axis, sign = FACE_AXES[face]
    return check_transform(target)[:3, axis]*sign


def validate_task_contract(scene):
    """Bind the new twelve-carton task to measured scene geometry, not ID parsing.

    The original four-carton entry has no such contract and remains separate.
    Receiving bounds describe a verified flat support rectangle, not a collision
    replacement for the conveyor's original connected-component convex solids.
    """
    contract = scene.get('native_task')
    if contract is None:
        return None
    if contract.get('schema') != TASK_TYPE or contract.get('target_face') not in FACE_AXES:
        raise ValueError('NATIVE_TASK_CONTRACT_OR_FACE_INVALID')
    if contract.get('cup_selection_mode') == INDEPENDENT:
        if contract.get('allowed_faces') not in (['top', 'front'], ['top', 'front', 'left', 'right']):
            raise ValueError('NATIVE_TASK_ALLOWED_FACES_INVALID')
    elif contract.get('target_face') != 'top':
        raise ValueError('NATIVE_TASK_LEGACY_TOP_FACE_REQUIRED')
    boxes = scene['boxes']; by_id = {b['id']: b for b in boxes}
    coverage=contract.get('coverage_context')
    expected_count=12 if coverage is None else len(coverage['remaining_carton_ids'])
    if len(boxes) != expected_count or len(by_id) != expected_count:
        raise ValueError('NATIVE_TASK_TWELVE_CARTONS_REQUIRED')
    declared = contract.get('stack_carton_ids', [])
    if len(declared) != expected_count or len(set(declared)) != expected_count or set(declared) != set(by_id):
        raise ValueError('NATIVE_TASK_CARTON_IDENTITIES_CHANGED')
    positions = np.asarray([b['pose'] for b in boxes], float)
    sizes = np.asarray([b['size_m'] for b in boxes], float)
    if (positions.shape != (expected_count, 3) or sizes.shape != (expected_count, 3) or
            not np.isfinite(positions).all() or not np.isfinite(sizes).all() or np.any(sizes <= 0)):
        raise ValueError('NATIVE_TASK_CARTON_GEOMETRY_INVALID')
    tops = positions[:, 2]+sizes[:, 2]/2
    middle_y = (np.min(positions[:, 1]-sizes[:, 1]/2)+np.max(positions[:, 1]+sizes[:, 1]/2))/2
    highest = np.flatnonzero(np.abs(tops-np.max(tops)) <= 1e-8)
    centered = np.abs(positions[highest, 1]-middle_y)
    selected = highest[np.flatnonzero(np.abs(centered-np.min(centered)) <= 1e-8)]
    if len(selected) != 1:
        raise ValueError('NATIVE_TASK_TOP_CENTER_AMBIGUOUS')
    explicit = contract.get('target_selection')
    if explicit is not None:
        if (coverage is not None or explicit.get('schema')!='eco65_explicit_exposed_target_v1'
                or explicit.get('target_id')!=contract['target_id']
                or explicit.get('inventory_scope')!='UNCHANGED_TWELVE_CARTONS'
                or (explicit.get('source')!='USER_REQUEST' and not
                    (explicit.get('source')=='TRANSVERSE_PROCESS_DISPATCH' and scene.get('transverse_process_policy')))
                or contract['target_id'] not in {boxes[int(i)]['id'] for i in highest}):
            raise ValueError('NATIVE_TASK_EXPLICIT_TARGET_NOT_EXPOSED')
    target = boxes[int(selected[0])] if coverage is None and explicit is None else by_id[contract['target_id']]
    if (contract.get('target_id') != target['id'] or scene.get('target_box_id') != target['id']
            or scene.get('box') != target or target.get('supports')):
        raise ValueError('NATIVE_TASK_TARGET_IS_NOT_TOP_CENTER')
    if coverage is not None:
        universe={f'carton_r00_l{l:02d}_c{c:02d}' for l in range(4) for c in range(3)}
        removed={f'carton_r00_l{l:02d}_c{c:02d}' for l in range(target['layer']+1,4) for c in range(3)}
        if (set(coverage['removed_carton_ids'])!=removed or set(declared)!=universe-removed or
                coverage['remaining_carton_ids']!=declared or not coverage.get('source_snapshot_fingerprint') or
                coverage['scope']!='CONDITIONAL_COVERAGE_DIAGNOSTIC_NOT_COMPLETE_TASK'):
            raise ValueError('COVERAGE_EXPOSURE_STATE_INVALID')
    supports = contract.get('original_support_names', [])
    if (not supports or len(set(supports)) != len(supports) or
            set(supports) != set(target.get('supported_by', [])) or target['id'] in supports):
        raise ValueError('NATIVE_TASK_ORIGINAL_SUPPORT_CHANGED')
    objects = {o['id']: o for o in scene['obstacles']}
    mesh_objects = {o['id']: o for o in scene.get('native_world_objects', [])}
    all_ids = set(objects) | set(mesh_objects) | set(by_id)
    if not set(supports) <= all_ids:
        raise ValueError('NATIVE_TASK_ORIGINAL_SUPPORT_MISSING')
    # Initial carton supports must be actual lower top faces, never a neighbor
    # permission inferred solely from the supplied support name.
    for name in supports:
        below = by_id.get(name, objects.get(name))
        if below is None:
            raise ValueError('NATIVE_TASK_INITIAL_SUPPORT_REQUIRES_BOX_GEOMETRY')
        p = np.asarray(below.get('pose', below.get('position_m')), float)
        half = np.asarray(below['size_m'], float)/2
        target_half = np.asarray(target['size_m'])/2; target_p = np.asarray(target['pose'])
        overlap = np.minimum(p[:2]+half[:2], target_p[:2]+target_half[:2])-np.maximum(
            p[:2]-half[:2], target_p[:2]-target_half[:2])
        if abs(target_p[2]-target_half[2]-p[2]-half[2]) > .0004 or not np.all(overlap > 0):
            raise ValueError('NATIVE_TASK_INITIAL_SUPPORT_GEOMETRY_INVALID')
    receiving = contract.get('receiving', {})
    receiver_ids = receiving.get('support_ids', [])
    if (not isinstance(receiving.get('region_id'), str) or not receiving['region_id'] or
            not receiver_ids or len(set(receiver_ids)) != len(receiver_ids) or
            not set(receiver_ids) <= (set(objects) | set(mesh_objects)) or set(receiver_ids) & set(by_id)):
        raise ValueError('NATIVE_TASK_RECEIVING_SUPPORT_INVALID')
    limits = np.asarray(receiving.get('bounds_D_m'), float)
    if (limits.shape != (2, 3) or not np.isfinite(limits).all() or
            np.any(limits[1, :2] <= limits[0, :2]) or limits[1, 2] < limits[0, 2]):
        raise ValueError('NATIVE_TASK_RECEIVING_BOUNDS_INVALID')
    pose = check_transform(receiving['box_pose_D'])
    half = np.asarray(target['size_m'])/2
    bottom = np.array([[x, y, -half[2]] for x in (-half[0], half[0]) for y in (-half[1], half[1])])
    bottom = bottom@pose[:3, :3].T+pose[:3, 3]
    if (np.max(np.abs(bottom[:, 2]-limits[1, 2])) > .0004 or
            np.any(bottom[:, :2] < limits[0, :2]-1e-9) or
            np.any(bottom[:, :2] > limits[1, :2]+1e-9)):
        raise ValueError('NATIVE_TASK_RECEIVING_BOX_NOT_SUPPORTED')
    for name in ('preapproach_m', 'preplace_m', 'withdrawal_m', 'minimum_lift_m'):
        value = contract.get(name)
        if not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError('NATIVE_TASK_MOTION_DISTANCE_INVALID:'+name)
    return deepcopy(contract)


def bind_task_contract(geometry, scene):
    """Return contact geometry carrying the immutable twelve-carton contract."""
    result = deepcopy(validate_geometry(geometry))
    contract = validate_task_contract(scene)
    if contract is None:
        if 'task_contract' in result:
            raise ValueError('NATIVE_TASK_CONTRACT_WITHOUT_SCENE')
    else:
        result['task_contract'] = contract
        if contract.get('cup_selection_mode') == INDEPENDENT:
            result['cup_selection_mode'] = INDEPENDENT
    return result


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
                   margin=.003, angle=.01, face='top', active_ids=None):
    """All physical contours evaluated; only the explicit enabled subset must seal.

    A rectangular target face is convex: testing every polygon vertex proves
    containment of every straight boundary edge. CAD tessellation error is
    reserved separately by the collision policy, not added a second time here.
    """
    validate_geometry(geometry)
    active = selected_ids(geometry, active_ids)
    if face not in FACE_AXES: raise ValueError('CONTACT_TARGET_FACE_INVALID')
    axis, sign = FACE_AXES[face]; others = [i for i in range(3) if i != axis]
    tolerances = np.asarray([gap, penetration, margin, angle])
    if not np.isfinite(tolerances).all() or np.any(tolerances < 0) or angle >= np.pi/2:
        raise ValueError('CONTACT_TOLERANCE_INVALID')
    half = np.asarray(size, float)/2
    if half.shape != (3,) or not np.isfinite(half).all() or np.any(half <= 0):
        raise ValueError('CONTACT_TARGET_INVALID')
    local = np.linalg.inv(check_transform(target)) @ check_transform(flange) @ check_transform(
        geometry['flange_from_contact_frame'])
    alignment = check_edge_alignment(geometry, local, face)
    results = []
    for row in geometry['contours']:
        pts = np.asarray(row['points_m']) @ local[:3, :3].T + local[:3, 3]
        n = local[:3, :3] @ row['normal']
        errors = sign*pts[:, axis]-half[axis]
        reserve = float(np.min(half[others]-np.abs(pts[:, others])))
        face_contact = bool(-sign*n[axis] >= np.cos(angle) and errors.min() >= -penetration-1e-12
                            and errors.max() <= gap+1e-12)
        good = bool(face_contact and reserve >= margin-1e-12)
        results.append(dict(id=row['id'], valid=good, min_gap_m=float(errors.min()),
                            max_gap_m=float(errors.max()), edge_margin_m=reserve,
                            face_contact=face_contact, active=row['id'] in active))
    actual = [r['id'] for r in results if r['valid']]
    return dict(status='PASS' if set(active) <= set(actual) and alignment['status']!='FAIL' else 'FAIL',
                edge_alignment=alignment,
                contours=results, checked_count=12,
                target_face=face, active_ids=active, geometrically_eligible_ids=actual,
                actual_sealed_ids=actual,
                contact_observation='predicted_geometry_not_physical_contact')


def top_grasp_candidates(geometry, target, size, flange_from_tcp, base_position,
                         *, margin=.003, angle=.01, candidate_inset_m=.0001):
    """Finite top-face translations/rolls with every real contour preserved.

    Choosing the middle carton does not force the TCP to its face centre. The
    allowed translation rectangle follows directly from all CAD vertices and
    the required edge margin. Flange distance is a ranking heuristic only;
    native IK, collision checks and complete motion remain mandatory.
    """
    validate_geometry(geometry)
    target = check_transform(target); flange_from_tcp = check_transform(flange_from_tcp)
    half = np.asarray(size, float)/2
    base = np.asarray(base_position, float)
    if (half.shape != (3,) or not np.isfinite(half).all() or np.any(half <= 0) or
            base.shape != (3,) or not np.isfinite(base).all() or not np.isfinite(margin) or margin < 0 or
            not np.isfinite(candidate_inset_m) or candidate_inset_m < 0):
        raise ValueError('TOP_GRASP_INPUT_INVALID')
    tcp_from_cad = np.linalg.inv(flange_from_tcp)@check_transform(geometry['flange_from_contact_frame'])
    points = np.concatenate([row['points_m'] for row in geometry['contours']])
    points = points@tcp_from_cad[:3, :3].T+tcp_from_cad[:3, 3]
    base_local = target[:3, :3].T@(base-target[:3, 3])
    candidates = []; rejected = []
    top_rotation = np.diag([1., -1., -1.])
    for roll in (0., np.pi/2, -np.pi/2, np.pi):
        co, si = np.cos(roll), np.sin(roll)
        rotation = top_rotation@np.array([[co,-si,0.],[si,co,0.],[0.,0.,1.]])
        rotated = points@rotation.T
        lower = -half[:2]+margin-rotated[:, :2].min(axis=0)
        upper = half[:2]-margin-rotated[:, :2].max(axis=0)
        if np.any(lower > upper+1e-12):
            rejected.append(dict(roll_rad=float(roll), reason='TWELVE_CONTOURS_EXCEED_FACE',
                                 allowed_tcp_xy_m=[lower.tolist(),upper.tolist()]))
            continue
        # Keep boundary proposals slightly inside the valid rectangle so a
        # finite native IK residual does not spend the entire seal-edge margin.
        # This changes proposals only; every actual contour still needs 3 mm.
        centre = (lower+upper)/2
        proposal_lower = np.minimum(lower+candidate_inset_m, centre)
        proposal_upper = np.maximum(upper-candidate_inset_m, centre)
        options = [('nearest_base', np.clip(base_local[:2], proposal_lower, proposal_upper)),
                   ('front_center', np.array([proposal_lower[0], np.clip(0.,proposal_lower[1],proposal_upper[1])])),
                   ('allowed_region_center', (lower+upper)/2)]
        if np.all(lower <= 0) and np.all(upper >= 0): options.append(('face_center', np.zeros(2)))
        seen = []
        for label, xy in options:
            if any(np.max(np.abs(xy-old)) <= 1e-10 for old in seen): continue
            seen.append(xy.copy())
            local = np.eye(4); local[:3, :3] = rotation; local[:3, 3] = [*xy, half[2]]
            tcp = target@local; flange = tcp@np.linalg.inv(flange_from_tcp)
            seal = check_contours(geometry, flange, target, size, gap=.0004, penetration=.0004,
                                  margin=margin, angle=angle)
            if seal['status'] != 'PASS':
                rejected.append(dict(roll_rad=float(roll), offset_tcp_xy_m=xy.tolist(),
                                     reason='NOMINAL_CONTOURS_REJECTED', contour_report=seal))
                continue
            candidates.append(dict(kind=label, roll_rad=float(roll), offset_tcp_xy_m=xy.tolist(),
                allowed_tcp_xy_m=[lower.tolist(),upper.tolist()], tcp_pose_D=tcp.tolist(),
                flange_distance_from_base_m_heuristic=float(np.linalg.norm(flange[:3, 3]-base)),
                nominal_contact_geometry=seal, qualification='GEOMETRY_ONLY_NOT_IK_OR_PATH_VALIDATED'))
    candidates.sort(key=lambda item: (item['flange_distance_from_base_m_heuristic'],
                                     abs(item['roll_rad']), item['kind']))
    return dict(candidates=candidates, rejected=rejected, checked_contour_count=12,
                edge_margin_m=margin, proposal_inset_m=candidate_inset_m,
                method='ALL_CAD_CONTOURS_RECTANGULAR_FACE_CONTAINMENT')


def extract_cad_contours(tool, root):
    """Read all outer terminal loops from existing triangulated CAD surfaces.

    Never convert an unverified fitted radius into a claimed CAD contour.
    The original tool config and CAD meshes remain immutable.
    """
    import trimesh
    from .asset_paths import resolve_asset
    if len(tool['seals']) != 12:
        raise ValueError('ALL_12_CAD_SEALS_REQUIRED')
    components = {s['component'] for s in tool['seals']}
    bands = [p for p in tool['collisions'] if p['semantic'] == 'terminal_contact_band']
    if len(components) != 12 or len(bands) != 12 or {b['source_part'] for b in bands} != components:
        raise ValueError('SEAL_TO_CONTACT_BAND_OWNERSHIP_MISMATCH')
    parts = {p['id']: p for p in tool['visual_parts']}
    rows = []
    for seal in tool['seals']:
        path = resolve_asset(root, parts[seal['component']]['visual'])
        if seal.get('contour_method') == 'exact_trimmed_CAD_torus_axial_tangent':
            manifest = Path(root)/'assets/tools/desktop_suction/derived/manifests/cad.json'
            if hashlib.sha256(manifest.read_bytes()).hexdigest() != tool['cad_manifest_sha256']:
                raise ValueError('CONTACT_CAD_MANIFEST_CHANGED')
            cad = json.loads(manifest.read_text(encoding='utf-8'))
            if cad['source_sha256'] != tool['source_sha256']:
                raise ValueError('CONTACT_CAD_SOURCE_CHANGED')
            part = next(p for p in cad['parts'] if p['id'] == seal['component'])
            if hashlib.sha256(path.read_bytes()).hexdigest() != part['visual_sha256']:
                raise ValueError('CONTACT_VISUAL_CHANGED')
            faces = [part['surface_features'][i] for i in seal['cad_face_indices']]
            if len(faces) != 2 or any(f['type'] != 'torus' for f in faces):
                raise ValueError('CONTACT_CAD_FACE_IDENTITY')
            a, b = [np.asarray(f['axial_tangent_curve_m']) for f in faces]
            if not np.allclose(a[-1], b[0], atol=1e-10) or not np.allclose(b[-1], a[0], atol=1e-10):
                raise ValueError('CONTACT_CAD_CURVE_OPEN')
            points = np.concatenate([a[:-1], b[:-1]])
            if not np.array_equal(points, seal['ring_cad_m']): raise ValueError('CONTACT_CAD_CURVE_CHANGED')
            if not np.allclose(seal['normal_cad'], faces[0]['axis'], atol=1e-12):
                raise ValueError('CONTACT_CAD_NORMAL_CHANGED')
            rows.append(dict(id=seal['id'], component=seal['component'], points_m=points.tolist(),
                normal=seal['normal_cad'], mesh_sha256=part['visual_sha256'],
                source='exact_trimmed_CAD_torus_axial_tangent', original_fitted_ring_used=False,
                source_sha256=cad['source_sha256'], cad_manifest_sha256=tool['cad_manifest_sha256'],
                cad_face_indices=seal['cad_face_indices'], max_chord_error_m=seal['max_chord_error_m']))
            continue
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
    result['terminal_band_by_contour'] = {s['id']: next(b['id'] for b in bands
        if b['source_part'] == s['component']) for s in tool['seals']}
    if 'mounting_contacts' in tool:
        result['mounting_contacts'] = tool['mounting_contacts']
    return validate_geometry(result)
