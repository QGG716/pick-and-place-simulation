"""Bind an existing twelve-carton scene to native planning without calibration fixtures."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
from pathlib import Path
import re
from time import perf_counter

import numpy as np
import yaml

from unloading_sim.eco65.model import ROOT, check_transform, load_json, save_json, transform
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.eco65.native_io import file_hash, verify_snapshot
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.native_scene import TWELVE_CONFIG, adapt_static_scene
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.moveit2_backend import digest


def _convex(mesh):
    """Keep even planar source parts with an explicit one-micron envelope."""
    import trimesh
    vertices = np.asarray(mesh.vertices, float)
    if not len(mesh.faces) or not np.isfinite(vertices).all():
        raise ValueError('EMPTY_OR_NONFINITE_SOURCE_COMPONENT')
    centered = vertices-vertices.mean(axis=0)
    rank = np.linalg.matrix_rank(centered)
    thickness = 0.
    if rank == 2:
        normal = np.linalg.svd(centered, full_matrices=False)[2][-1]
        thickness = 1e-6
        vertices = np.vstack([vertices-normal*thickness/2, vertices+normal*thickness/2])
    elif rank < 2:
        raise ValueError('SOURCE_COMPONENT_HAS_NO_SURFACE')
    # Keep Qhull's closed topology. Trimesh's default hull postprocessing
    # drops very small seat facets and can leave this real mesh open.
    from scipy.spatial import ConvexHull
    vertices = np.unique(vertices, axis=0)
    boundary = ConvexHull(vertices)
    faces = boundary.simplices.copy(); triangles = vertices[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    reverse = np.einsum('ij,ij->i', normals, boundary.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, ::-1]
    hull = trimesh.Trimesh(vertices, faces, process=False)
    hull.remove_unreferenced_vertices()
    if not hull.is_watertight or not hull.is_convex:
        raise ValueError('INVALID_DERIVED_CONVEX_COMPONENT')
    return hull, thickness


def prepare_conveyors(scene):
    """Use every collision component of the existing derived conveyor USDs."""
    import trimesh
    from pxr import Usd, UsdGeom
    objects, records, surfaces = [], [], {}
    belt_source = scene['conveyors']['source']['belt_prim']
    for item in scene['conveyors']['items']:
        path = resolve_asset(ROOT, item['usd_path'])
        if file_hash(path) != item['usd_sha256']: raise ValueError('SOURCE_CONVEYOR_CHANGED')
        stage = Usd.Stage.Open(str(path))
        if not stage or stage.GetCompositionErrors(): raise ValueError('SOURCE_CONVEYOR_COMPOSITION_INVALID')
        cache = UsdGeom.XformCache(); support_ids = []
        for mesh_index, row in enumerate(item['meshes']):
            if not row['collision']: continue
            prim = stage.GetPrimAtPath(row['prim'])
            if not prim or not prim.IsA(UsdGeom.Mesh): raise ValueError('SOURCE_CONVEYOR_MESH_MISSING')
            mesh = UsdGeom.Mesh(prim); raw = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
            if hashlib.sha256(raw.tobytes()).hexdigest() != row['points_sha256']:
                raise ValueError('SOURCE_CONVEYOR_POINTS_CHANGED')
            T = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices = raw.astype(float)@T[:3, :3].T+T[:3, 3]
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
            faces = []; offset = 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                polygon = indices[offset:offset+count]; offset += count
                if count < 3: raise ValueError('CONVEYOR_FACE_INVALID')
                faces.extend([[polygon[0], polygon[k], polygon[k+1]] for k in range(1, count-1)])
            original = trimesh.Trimesh(vertices, np.asarray(faces), process=True)
            components = original.split(only_watertight=False)
            if not len(components): raise ValueError('CONVEYOR_COMPONENTS_EMPTY')
            for component_index, component in enumerate(components):
                hull, thickness = _convex(component)
                ident = f"{item['id']}__mesh{mesh_index:03d}_component{component_index:03d}"
                is_belt = row['source_prim'] == belt_source
                objects.append(dict(id=ident, pose=np.eye(4).tolist(), category='environment',
                    owner=item['id'], role='conveyor_belt' if is_belt else 'conveyor_structure',
                    convex_mesh=dict(vertices=hull.vertices.tolist(), triangles=hull.faces.tolist()),
                    bounds_D_m=[hull.vertices.min(0).tolist(), hull.vertices.max(0).tolist()]))
                if is_belt: support_ids.append(ident)
                records.append(dict(id=ident, source_usd=item['usd_path'], source_sha256=item['usd_sha256'],
                    source_prim=row['source_prim'], source_points_sha256=row['points_sha256'],
                    source_component=component_index, source_triangle_count=len(component.faces),
                    representation='connected_component_convex_hull', planar_envelope_thickness_m=thickness))
        if not support_ids: raise ValueError('CONVEYOR_BEARING_SURFACE_MISSING:'+item['id'])
        surfaces[item['id']] = dict(region_id=item['id'], support_ids=support_ids,
            bounds_D_m=deepcopy(item['receiving_surface_bounds_D_m']),
            source_belt_prim=belt_source, source_usd=item['usd_path'])
    if set(surfaces) != {'conveyor_transverse', 'conveyor_longitudinal'}:
        raise ValueError('BOTH_EXISTING_CONVEYORS_REQUIRED')
    return objects, records, surfaces


def prepare_j1_collision(output):
    """Decompose the unchanged official J1 surface for its conformal seat."""
    import coacd
    import trimesh
    import xml.etree.ElementTree as ET
    from unloading_sim.eco65.asset_inventory import load_robot_config
    config = load_robot_config()
    visual = ET.parse(ROOT/config['urdf']).find("link[@name='link_1']/visual")
    uri = visual.find('geometry/mesh').get('filename')
    path = ROOT/config['package_root']/uri.removeprefix('package://ECO65-B/')
    original = trimesh.load(path, force='mesh', process=True)
    folder = Path(output); folder.mkdir(parents=True, exist_ok=False)
    rows, assets = [], {}
    shells = original.split(only_watertight=False)
    outer = [m for m in shells if m.volume > 0]
    cavities = [m for m in shells if m.volume <= 0]
    if any(not m.is_watertight for m in shells) or not outer:
        raise ValueError('OFFICIAL_J1_CLOSED_SURFACE_REQUIRED')
    # Negative-wound, completely enclosed shells describe voids, not separate
    # solids. Filling only those enclosed voids is conservative; decomposing
    # them as solids is invalid and can make CoACD subdivide indefinitely.
    for cavity in cavities:
        samples = np.vstack([cavity.vertices, cavity.triangles_center])
        if not np.all(np.logical_or.reduce([m.contains(samples) for m in outer])):
            raise ValueError('J1_NEGATIVE_SHELL_NOT_ENCLOSED')
    for component in outer:
        pieces = ([(component.vertices, component.faces)] if component.is_convex else
            coacd.run_coacd(coacd.Mesh(component.vertices, component.faces), threshold=.01,
                preprocess_mode='off', resolution=2000, mcts_iterations=100,
                merge=True, decimate=False, seed=6518))
        for vertices, faces in pieces:
            hull, _ = _convex(trimesh.Trimesh(vertices, faces, process=False))
            name = f'robot_link_1__convex_{len(rows):03d}'; dest = folder/(name+'.obj')
            hull.export(dest, digits=17)
            restored = trimesh.load(dest, force='mesh', process=False)
            if not restored.is_watertight or not restored.is_convex:
                raise ValueError('J1_CONVEX_ROUNDTRIP_INVALID')
            relative = dest.relative_to(ROOT).as_posix(); sha = file_hash(dest)
            assets[relative] = sha
            rows.append(dict(id=name, mesh=relative, mesh_sha256=sha,
                source=path.relative_to(ROOT).as_posix(), source_sha256=file_hash(path),
                representation='official_outer_shell_CoACD_enclosed_voids_conservatively_filled',
                enclosed_void_count=len(cavities)))
    if not rows: raise ValueError('OFFICIAL_J1_DECOMPOSITION_EMPTY')
    return rows, assets


def prepare_attachments(scene, camera, output):
    """Retain source CAD, J1 mounting seat and optional current camera installation."""
    import trimesh
    folder = Path(output); folder.mkdir(parents=True, exist_ok=False)
    mast = scene['mast']; M = check_transform(mast['T_link1_mast'])
    rows, assets, records, interfaces = [], {}, [], []
    replaced = set(camera['replaced_mast_parts']) if camera else set()
    source_ids = {p['id'] for p in mast['parts']}
    if not replaced <= source_ids: raise ValueError('CAMERA_REPLACEMENT_PART_NOT_IN_SOURCE_MAST')
    def add(ident, mesh, T, role, source):
        ident = re.sub(r'[^A-Za-z0-9_]', '_', ident)
        if ident in {r['id'] for r in rows}: raise ValueError('PERMANENT_ATTACHMENT_ID_DUPLICATE')
        hull, thickness = _convex(mesh)
        # Default OBJ precision can turn nearly coplanar hull facets concave
        # after reload. Preserve the computed boundary and validate its bytes.
        path = folder/(ident+'.obj'); hull.export(path, digits=17)
        restored = trimesh.load(path, force='mesh', process=False)
        if not restored.is_watertight or not restored.is_convex:
            raise ValueError('DERIVED_ATTACHMENT_ROUNDTRIP_INVALID:'+ident)
        relative = path.relative_to(ROOT).as_posix(); sha = file_hash(path)
        assets[relative] = sha
        row = dict(id=ident, parent_link='link_1', T_parent_local=check_transform(T).tolist(),
            mesh=relative, mesh_sha256=sha, role=role, source=deepcopy(source))
        rows.append(row)
        records.append(dict(id=ident, source=deepcopy(source), source_triangles=len(mesh.faces),
            planar_envelope_thickness_m=thickness, representation='source_solid_conservative_convex_hull'))
        return ident
    def interface(ident):
        interfaces.append(dict(parent_link='link_1', attachment_id=ident,
            maximum_penetration_m=.0004, source=deepcopy(mast['mount_interface'])))
    for part in mast['parts']:
        if part['id'] in replaced: continue
        # The connected silver bracket is concave. Decompose its original
        # surface, never its already-filled single-hull collision proxy.
        kind = 'visual' if part['id'] == 'mast_part_001' else 'collision'
        path = resolve_asset(ROOT, part[kind])
        if file_hash(path) != part[kind+'_sha256']: raise ValueError('SOURCE_MAST_MESH_CHANGED')
        original = trimesh.load(path, force='mesh', process=True)
        if part['id'] == 'mast_part_001' and not original.is_convex:
            import coacd
            pieces = coacd.run_coacd(coacd.Mesh(original.vertices, original.faces),
                threshold=.01, preprocess_mode='off', resolution=2000,
                mcts_iterations=100, merge=True, decimate=False, seed=6518)
            if not pieces: raise ValueError('MOUNT_DECOMPOSITION_EMPTY')
            for index, (vertices, faces) in enumerate(pieces):
                ident = add(f'eco65_mast_mast_part_001__convex_{index:03d}',
                    trimesh.Trimesh(vertices, faces, process=True), M, 'permanent_mast',
                    dict(part_id=part['id'], mesh=part[kind], sha256=part[kind+'_sha256'],
                         decomposition='CoACD_no_preprocessing_no_decimation', component=index))
                interface(ident)
            continue
        ident = add('eco65_mast_'+part['id'], trimesh.load(path, force='mesh', process=False), M,
            'permanent_mast', dict(part_id=part['id'], mesh=part[kind], sha256=part[kind+'_sha256']))
        if part['id'] == 'mast_part_001':
            interface(ident)
    seat = mast['mount_interface']['seat']; path = resolve_asset(ROOT, seat['mesh'])
    if file_hash(path) != seat['sha256']: raise ValueError('SOURCE_MOUNTING_SEAT_CHANGED')
    seat_mesh = trimesh.load(path, force='mesh', process=False)
    if seat_mesh.is_convex:
        interface(add('eco65_mast_mounting_seat', seat_mesh, np.eye(4),
            'permanent_mounting_seat', dict(mesh=seat['mesh'], sha256=seat['sha256'])))
    else:
        # The source is a collection of closed conformal cells. Keep every
        # cell; spatially local conservative hulls avoid filling the full seat
        # cavity. The unchanged 0.4 mm FCL gate checks every resulting piece.
        cells = seat_mesh.split(only_watertight=False); groups = {}
        for index, cell in enumerate(cells):
            if not cell.is_watertight or not cell.is_convex:
                raise ValueError('SOURCE_SEAT_CLOSED_CONVEX_CELL_REQUIRED')
            key = tuple(np.floor(cell.vertices.mean(0)[:2]/.005).astype(int))
            groups.setdefault(key, []).append(index)
        for index, key in enumerate(sorted(groups)):
            indices = groups[key]
            joined = trimesh.util.concatenate([cells[i] for i in indices])
            ident = add(f'eco65_mast_mounting_seat__convex_{index:03d}', joined, np.eye(4),
                'permanent_mounting_seat', dict(mesh=seat['mesh'], sha256=seat['sha256'],
                    decomposition='source_cells_local_conservative_hulls', cell_indices=indices,
                    source_cell_count=len(cells), grouping_pitch_m=.005))
            interface(ident)
    if camera:
        for name in ('source', 'cad_manifest'):
            path = resolve_asset(ROOT, camera[name]); expected = camera[name+'_sha256']
            if file_hash(path) != expected: raise ValueError('PERMANENT_CAMERA_SOURCE_CHANGED')
            assets[camera[name]] = expected
        C = check_transform(camera['T_mast_CAD'])
        for part in camera['parts']:
            for kind in ('visual', 'collision'):
                path = resolve_asset(ROOT, part[kind])
                expected = part[kind+'_sha256']
                if file_hash(path) != expected: raise ValueError('PERMANENT_CAMERA_ASSET_CHANGED')
                assets[part[kind]] = expected
            add('eco65_camera_'+part['id'], trimesh.load(resolve_asset(ROOT, part['collision']), force='mesh', process=False),
                M@C, 'permanent_camera', dict(part_id=part['id'], mesh=part['collision'], sha256=part['collision_sha256']))
        for part in camera.get('pitch_adapter', []):
            add('eco65_camera_'+part['name'], trimesh.creation.box(part['size_m']), M@check_transform(part['T_mast']),
                'permanent_camera_adapter', dict(definition=deepcopy(part), camera_mount=deepcopy(camera['mount'])))
    if not rows or not interfaces: raise ValueError('PERMANENT_J1_ASSEMBLY_INCOMPLETE')
    return rows, assets, records, interfaces


def build_from_static(snapshot_path, home_path, output, *, camera_configuration=None,
                      native_asset_root=None, config_path=TWELVE_CONFIG):
    """Create fresh assets, native snapshot and an unaccepted current HOME proposal."""
    from build_eco65_full_face_layout import verify_static
    started = perf_counter(); out = Path(output).resolve(); out.relative_to(ROOT/'outputs')
    out.mkdir(parents=True, exist_ok=False)
    snapshot_path, home_path, config_path = map(lambda p: Path(p).resolve(), [snapshot_path, home_path, config_path])
    source = verify_static(load_json(snapshot_path)); home = load_json(home_path)
    if (home.get('source_snapshot_fingerprint') != source['fingerprint']
            or home.get('status') != 'STATIC_POSE_ONLY'
            or home.get('identity') != source['native_model']['init']['identity']):
        raise ValueError('SOURCE_DISPLAY_POSE_SNAPSHOT_MISMATCH')
    q = home.get('home_q', home.get('q_rad'))
    if q is None: raise ValueError('SOURCE_DISPLAY_POSE_REQUIRED')
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    assets = deepcopy(source['asset_sha256']); hashes = deepcopy(source['config_sha256'])
    hashes[config_path.relative_to(ROOT).as_posix()] = file_hash(config_path)
    camera = None
    if camera_configuration is not None:
        path = Path(camera_configuration).resolve(); c = load_json(path)
        from eco65_touch_sources import verify_scene_sources
        binding = verify_scene_sources(c)
        if binding['files']['snapshot']['fingerprint'] != source['fingerprint']:
            raise ValueError('PERMANENT_CAMERA_SOURCE_SNAPSHOT_MISMATCH')
        camera = deepcopy(c['camera'])
        hashes[path.relative_to(ROOT).as_posix()] = file_hash(path)
    native_world, conveyor_records, surfaces = prepare_conveyors(source['scene'])
    attachments, extra_assets, attachment_records, interfaces = prepare_attachments(source['scene'], camera, out/'permanent_attachments')
    assets.update(extra_assets)
    receiver = deepcopy(surfaces[config['receiving_region']]); bounds = np.asarray(receiver['bounds_D_m'])
    size = np.asarray(next(b for b in source['scene']['boxes'] if b['id'] == config['target_id'])['size_m'])
    receiver['box_pose_D'] = transform([*((bounds[0, :2]+bounds[1, :2])/2), bounds[1, 2]+size[2]/2]).tolist()
    scene = adapt_static_scene(source, native_world, attachments, receiver, q, config=config,
        config_sha256=hashes, permanent_camera=camera, fixed_attachment_interfaces=interfaces)
    j1, j1_assets = prepare_j1_collision(out/'robot_collision')
    scene['robot_collision_components'] = {'link_1': j1}
    assets.update(j1_assets)
    from eco65_mount_convex import refine_seat
    refined_assets, refined_records, refinement = refine_seat(scene, out/'refined_seat')
    assets.update(refined_assets)
    current_ids = {r['id'] for r in scene['moving_attachments'] if r['role'] != 'permanent_mounting_seat'}
    attachment_records = [r for r in attachment_records if r['id'] in current_ids]+refined_records
    scene['receiving_surfaces'] = surfaces
    scene['scene_fingerprint'] = digest({k: v for k, v in scene.items() if k != 'scene_fingerprint'})
    model = build_native_model(scene, out/'model', native_asset_root=native_asset_root)
    assets.update(model['asset_audit']['asset_sha256'])
    assets.update({b['convex_path']: b['convex_sha256'] for b in model['convex_bindings']})
    snapshot = dict(schema='eco65_native_snapshot_v1', scene=scene, tool=model['tool'],
        native_model=model, asset_sha256=assets, config_sha256=hashes)
    snapshot['fingerprint'] = digest(snapshot); verify_snapshot(snapshot)
    save_json(out/'snapshot.json', snapshot)
    # Keep the existing ROS rootfs packaging/URI-verification workflow usable
    # with the fresh model, including the permanent J1 collision links.
    save_json(out/'native_model_prepared.json', model)
    candidate = make_candidate(q, snapshot['tool'], scene,
        dict(source='SOURCE_STATIC_DISPLAY_POSE_REQUIRES_CURRENT_NATIVE_ACCEPTANCE',
             source_snapshot_fingerprint=source['fingerprint'], source_home_sha256=file_hash(home_path)))
    save_json(out/'home_candidate.json', candidate)
    save_json(out/'scene_geometry.json', dict(conveyor_components=conveyor_records, seat_refinement=refinement,
        permanent_attachments=attachment_records, interfaces=scene['fixed_attachment_interfaces'], receiving_surfaces=surfaces,
        source_snapshot_path=snapshot_path.relative_to(ROOT).as_posix(), source_snapshot_sha256=file_hash(snapshot_path),
        target=scene['box'], all_carton_ids=[b['id'] for b in scene['boxes']],
        calibration_fixtures_included=False, permanent_camera_mode='CURRENT_SOURCE_CONFIGURATION' if camera else 'ORIGINAL_MAST_ASSEMBLY'))
    report = dict(status='BUILT_NOT_RUNTIME_VALIDATED', snapshot_fingerprint=snapshot['fingerprint'],
        snapshot=str(out/'snapshot.json'), native_model_prepared=str(out/'native_model_prepared.json'),
        home_candidate=str(out/'home_candidate.json'),
        target_id=scene['target_box_id'], cartons=12, conveyor_components=len(native_world),
        permanent_attachment_count=len(scene['moving_attachments']), native_initialization='NOT_RUN',
        planning='NOT_RUN', independent_review='NOT_RUN', replay='NOT_RUN', dynamics='NOT_EVALUATED',
        wall_s=perf_counter()-started)
    save_json(out/'build_report.json', report)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene-snapshot', type=Path, required=True)
    p.add_argument('--home', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--camera-configuration', type=Path)
    p.add_argument('--native-asset-root')
    p.add_argument('--config', type=Path, default=TWELVE_CONFIG)
    a = p.parse_args()
    print(build_from_static(a.scene_snapshot, a.home, a.output,
        camera_configuration=a.camera_configuration, native_asset_root=a.native_asset_root, config_path=a.config))
