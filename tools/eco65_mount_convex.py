"""Refine seat hull bridges without removing any original conformal cell."""
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import xml.etree.ElementTree as ET

import numpy as np

from unloading_sim.eco65.model import ROOT
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.eco65.native_io import file_hash


def refine_seat(scene, output):
    import mujoco as mj
    import trimesh
    from build_eco65_native_12carton import _convex
    started = perf_counter(); folder = Path(output); folder.mkdir(parents=True, exist_ok=False)
    source = scene['mast']['mount_interface']['seat']; path = resolve_asset(ROOT, source['mesh'])
    if file_hash(path) != source['sha256']: raise ValueError('REFINEMENT_SOURCE_SEAT_CHANGED')
    cells = trimesh.load(path, force='mesh', process=False).split(only_watertight=False)
    old = [r for r in scene['moving_attachments'] if r['role'] == 'permanent_mounting_seat']
    groups = [list(r['source']['cell_indices']) for r in old]
    if (sorted(i for g in groups for i in g) != list(range(len(cells))) or
            any(not np.array_equal(r['T_parent_local'], np.eye(4)) for r in old)):
        raise ValueError('REFINEMENT_MUST_PRESERVE_EVERY_SOURCE_CELL_IN_LINK1')
    robot = []
    for row in scene['robot_collision_components']['link_1']:
        path = resolve_asset(ROOT, row['mesh'])
        if file_hash(path) != row['mesh_sha256']: raise ValueError('REFINEMENT_ROBOT_COMPONENT_CHANGED')
        robot.append(trimesh.load(path, force='mesh', process=False))
    centers = np.array([m.vertices.mean(0) for m in cells]); passes = []
    for iteration in range(16):
        hulls = [_convex(trimesh.util.concatenate([cells[i] for i in group]))[0] for group in groups]
        xml = ET.Element('mujoco'); asset = ET.SubElement(xml, 'asset'); world = ET.SubElement(xml, 'worldbody')
        for index, mesh in enumerate(robot+hulls):
            name = 'g'+str(index)
            ET.SubElement(asset, 'mesh', name=name, vertex=' '.join(map(str, mesh.vertices.reshape(-1))))
            body = ET.SubElement(world, 'body', name='b'+str(index))
            ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, contype='0', conaffinity='0')
        model = mj.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
        data = mj.MjData(model); mj.mj_forward(model, data)  # no mj_step
        distances = [min(float(mj.mj_geomDistance(model, data, i, len(robot)+j, .001, None))
                         for i in range(len(robot))) for j in range(len(groups))]
        if not np.isfinite(distances).all(): raise ValueError('REFINEMENT_DISTANCE_UNKNOWN')
        split = [j for j, d in enumerate(distances) if d < -.00035 and len(groups[j]) > 1]
        passes.append(dict(iteration=iteration, hull_count=len(groups), minimum_distance_m=min(distances), split_count=len(split)))
        if not split: break
        next_groups = []
        for j, group in enumerate(groups):
            if j not in split: next_groups.append(group); continue
            axis = int(np.argmax(np.ptp(centers[group], axis=0)))
            ordered = sorted(group, key=lambda i: (centers[i, axis], i)); middle = len(ordered)//2
            next_groups.extend([ordered[:middle], ordered[middle:]])
        groups = next_groups
        if len(groups) > 999: raise ValueError('REFINEMENT_COMPONENT_BUDGET_EXHAUSTED')
    else: raise ValueError('REFINEMENT_DEPTH_EXHAUSTED')
    if min(distances) < -.0004-1e-9:
        raise ValueError('EXACT_SOURCE_SEAT_CELL_REQUIRES_ROBOT_REFINEMENT:'+str(min(distances)))
    old_ids = {r['id'] for r in old}; rows, assets, records = [], {}, []
    for index, (group, hull) in enumerate(zip(groups, hulls)):
        ident = f'eco65_mast_mounting_seat__convex_{index:03d}'; path = folder/(ident+'.obj')
        hull.export(path, digits=17); restored = trimesh.load(path, force='mesh', process=False)
        if not restored.is_convex or not restored.is_watertight: raise ValueError('REFINED_SEAT_ROUNDTRIP_INVALID')
        relative = path.relative_to(ROOT).as_posix(); sha = file_hash(path); assets[relative] = sha
        binding = dict(mesh=source['mesh'], sha256=source['sha256'], cell_indices=group,
            source_cell_count=len(cells), decomposition='source_cell_subdivision_no_cells_removed')
        rows.append(dict(id=ident, parent_link='link_1', T_parent_local=np.eye(4).tolist(), mesh=relative,
            mesh_sha256=sha, role='permanent_mounting_seat', source=binding))
        records.append(dict(id=ident, source=binding, representation='source_cell_conservative_convex_hull'))
    scene['moving_attachments'] = [r for r in scene['moving_attachments'] if r['id'] not in old_ids]+rows
    scene['fixed_attachment_interfaces'] = [r for r in scene['fixed_attachment_interfaces'] if r['attachment_id'] not in old_ids]+[
        dict(parent_link='link_1', attachment_id=r['id'], maximum_penetration_m=.0004,
             source=deepcopy(scene['mast']['mount_interface'])) for r in rows]
    return assets, records, dict(scope='DECOMPOSITION_ONLY_REQUIRES_FRESH_NATIVE_ACCEPTANCE',
        source_cell_count=len(cells), retained_source_cell_count=sum(map(len, groups)),
        old_hull_count=len(old), new_hull_count=len(rows), passes=passes,
        maximum_penetration_m=.0004, wall_s=perf_counter()-started)
