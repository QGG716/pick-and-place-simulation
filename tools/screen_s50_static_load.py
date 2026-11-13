"""S50 quasi-static load screening on an actual replay (plan item 0.3).

Example:
  python tools/screen_s50_static_load.py \
      --cycle docs/experiments/s50_spacer80_fixed_face_20261112/cycle.json.gz \
      --output outputs/s50_static_load

Link masses are CAD-volume estimates scaled to the published 156 kg total.
The manufacturer supplies neither link inertials nor joint torque ratings, so
joint torques are compared with the static torque that the manufacturer's own
payload curve produces over a configuration grid (rated-load reference). This
is screening, not payload qualification.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
import yaml

from unloading_sim.s50_cad_model import read_obj
from unloading_sim.s50_kinematic_sim import make_context
from unloading_sim.s50_static_load import (GRAVITY_M_S2, S50_PAYLOAD_CURVE_MM_KG, base_moment,
    gravity_torques, mesh_mass_properties, payload_curve_limit_kg, scale_to_total_mass)

DEFAULT_CONFIG = ROOT / 'configs/robots/huayan_s50_spacer80_kinematic_sim.yaml'
PUBLISHED_TOTAL_MASS_KG = 156.0
SENSITIVITY = (0.7, 1.0, 1.3)


def _load_json(path):
    raw = Path(path).read_bytes()
    text = gzip.decompress(raw) if str(path).endswith('.gz') else raw
    return json.loads(text), hashlib.sha256(text).hexdigest()


def link_inertials(context):
    props = {}
    for link in context.definition['links']:
        mesh = link['visual_mesh']
        vertices, triangles = read_obj(context.root / mesh['path'], mesh['sha256'])
        props[link['name']] = mesh_mass_properties(vertices, triangles)
    scaled, density = scale_to_total_mass(props, PUBLISHED_TOTAL_MASS_KG)
    record = {name: dict(mass_kg=v['mass_kg'], com_local_m=v['com_local_m'].tolist(),
                         inertia_com_kg_m2=v['inertia_com_kg_m2'].tolist(),
                         volume_m3=props[name]['volume_m3'], boundary_edges=props[name]['boundary_edges'],
                         nonmanifold_edges=props[name]['nonmanifold_edges'])
              for name, v in scaled.items()}
    return scaled, density, record


def tool_mass_properties(context):
    """Tool COM/inertia in the S50 flange frame from the unchanged 20 kg tool record.

    The record is in the original flange frame whose virtual TCP is the
    POC contract frame; the S50 binding moves that virtual TCP, so the COM is
    carried through the virtual TCP rather than copied numerically.
    """
    load = context.intrinsic['load']
    contract = yaml.safe_load((context.root / 'configs/validation/m710id70_proof_of_concept.yaml')
                              .read_text(encoding='utf-8'))['tool_frame_contract']
    original_flange_from_vt = np.asarray(contract['T_flange_virtual_task_tcp'], dtype=float)
    s50_flange_from_original_flange = np.asarray(context.frames['virtual_task_tcp']) @ np.linalg.inv(original_flange_from_vt)
    com = (s50_flange_from_original_flange @ np.r_[np.asarray(load['com_xyz_m'], float), 1.])[:3]
    rotation = s50_flange_from_original_flange[:3, :3]
    inertia = rotation @ np.asarray(load['inertia_tensor_com_kg_m2'], float) @ rotation.T
    return float(load['mass_kg']), com, inertia


def _bodies(context, q, links, tool, carton_center, carton_mass, link_scale=1.):
    frames = context.robot.named_link_frames(q)
    names = [link['name'] for link in context.definition['links']]
    bodies = []
    for index, name in enumerate(names[1:], start=0):
        frame = frames[name]
        bodies.append((index, links[name]['mass_kg'] * link_scale,
                       (frame @ np.r_[links[name]['com_local_m'], 1.])[:3]))
    flange = frames[context.definition['flange_link']]
    tool_mass, tool_com, _ = tool
    bodies.append((5, tool_mass, (flange @ np.r_[tool_com, 1.])[:3]))
    if carton_center is not None:
        bodies.append((5, carton_mass, np.asarray(carton_center, float)))
    return frames, flange, bodies


def _joint_frames(context, q):
    axes = context.robot.joint_axis_frames(q)
    names = context.robot.active_joint_names
    return np.array([axes[n][0] for n in names]), np.array([axes[n][1] for n in names])


def rated_reference(context, links, link_scale=1.):
    """Max static torque per joint from the manufacturer curve loads over a grid.

    Each curve point (mass m at distance d along the flange axis) is a load the
    manufacturer rates; the robot must hold its static torque somewhere in the
    workspace. q1 is irrelevant to gravity; q6 does not move an on-axis load.
    """
    lo, hi = context.robot.joint_limits.T
    grid2 = np.deg2rad(np.arange(-190., 10.1, 5.))
    grid3 = np.deg2rad(np.arange(-160., 160.1, 5.))
    grid5 = np.deg2rad(np.arange(-180., 180.1, 15.))
    curve = np.asarray(S50_PAYLOAD_CURVE_MM_KG, float)
    masses, distances = curve[:, 1], curve[:, 0] / 1000.
    best = np.zeros(6)
    best_q = [None] * 6
    for q4 in np.deg2rad([0., 90.]):
        for q2 in grid2:
            for q3 in grid3:
                for q5 in grid5:
                    q = np.array([0., q2, q3, q4, q5, 0.])
                    if np.any(q < lo - 1e-12) or np.any(q > hi + 1e-12):
                        continue
                    points, axes = _joint_frames(context, q)
                    frames, flange, bodies = _bodies(context, q, links, (0., np.zeros(3), None), None, 0., link_scale)
                    base = gravity_torques(points, axes, bodies[:-1])
                    unit0 = gravity_torques(points, axes, [(5, 1., flange[:3, 3])])
                    unit1 = gravity_torques(points, axes, [(5, 1., flange[:3, 3] + flange[:3, 2])])
                    loads = masses[:, None] * ((1 - distances)[:, None] * unit0 + distances[:, None] * unit1)
                    peak = np.abs(base[None, :] + loads).max(axis=0)
                    for j in range(6):
                        if peak[j] > best[j]:
                            best[j], best_q[j] = peak[j], q.tolist()
    return best, best_q


def screen(cycle, context, links, tool, carton_mass):
    samples = cycle['samples']
    names = context.robot.active_joint_names
    mount = np.asarray(context.robot.base_transform)[:3, 3]
    flange_from_contact = np.asarray(context.frames['nominal_compressed_contact'])
    per_target = {}
    fk_error = 0.
    rows = dict(torque=[], torque_payload_only=[], overturning=[], offset=[], margin=[])
    tool_mass, tool_com, _ = tool
    for sample in samples:
        q = np.asarray(sample['q_rad'], float)
        attached = bool(sample.get('attached'))
        carton = np.asarray(sample['world_from_carton'], float)[:3, 3] if attached else None
        points, axes = _joint_frames(context, q)
        frames, flange, bodies = _bodies(context, q, links, tool, carton, carton_mass)
        recorded = sample.get('world_from_physical_contact')
        if recorded is not None:
            fk_error = max(fk_error, float(np.abs(flange @ flange_from_contact - np.asarray(recorded)).max()))
        torque = gravity_torques(points, axes, bodies)
        payload = gravity_torques(points, axes, bodies[6:])
        moment, weight = base_moment(mount, bodies)
        local_tool = tool_com
        if attached:
            local_carton = (np.linalg.inv(flange) @ np.r_[carton, 1.])[:3]
            combined_mass = tool_mass + carton_mass
            combined = (tool_mass * local_tool + carton_mass * local_carton) / combined_mass
        else:
            combined_mass, combined = tool_mass, local_tool
        offset = float(np.linalg.norm(combined))
        limit = payload_curve_limit_kg(offset)
        margin = None if limit is None else limit - combined_mass
        overturning = float(np.hypot(moment[0], moment[1]))
        rows['torque'].append(torque); rows['torque_payload_only'].append(payload)
        rows['overturning'].append(overturning); rows['offset'].append(offset); rows['margin'].append(margin)
        target = per_target.setdefault(sample['target'], dict(peak_abs_torque_nm=np.zeros(6), peak_overturning_nm=0.,
            max_attached_payload_offset_m=0., min_curve_margin_kg=None, attached_samples=0, samples=0,
            peak_phase=[None] * 6))
        target['samples'] += 1
        larger = np.abs(torque) > target['peak_abs_torque_nm']
        for j in np.flatnonzero(larger):
            target['peak_phase'][j] = sample['phase']
        target['peak_abs_torque_nm'] = np.maximum(target['peak_abs_torque_nm'], np.abs(torque))
        target['peak_overturning_nm'] = max(target['peak_overturning_nm'], overturning)
        if attached:
            target['attached_samples'] += 1
            target['max_attached_payload_offset_m'] = max(target['max_attached_payload_offset_m'], offset)
            if margin is not None and (target['min_curve_margin_kg'] is None or margin < target['min_curve_margin_kg']):
                target['min_curve_margin_kg'] = margin
    torque = np.asarray(rows['torque'])
    payload_only = np.asarray(rows['torque_payload_only'])
    worst = np.abs(torque).argmax(axis=0)
    margins = [m for m in rows['margin'] if m is not None]
    return dict(joint_names=list(names), sample_count=len(samples), fk_replay_max_error_m=fk_error,
        peak_abs_torque_nm=np.abs(torque).max(axis=0).tolist(),
        peak_abs_payload_only_torque_nm=np.abs(payload_only).max(axis=0).tolist(),
        rms_torque_nm=np.sqrt((torque ** 2).mean(axis=0)).tolist(),
        peak_sample=[dict(index=int(i), time_s=samples[i]['time_s'], target=samples[i]['target'],
            phase=samples[i]['phase'], q_rad=samples[i]['q_rad']) for i in worst],
        peak_overturning_moment_nm=float(max(rows['overturning'])),
        max_payload_offset_m=float(max(rows['offset'])),
        min_payload_curve_margin_kg=None if not margins else float(min(margins)),
        samples_over_payload_curve=int(sum(m is not None and m < 0 for m in rows['margin'])),
        samples_beyond_curve_range=int(sum(m is None for m in rows['margin'])),
        per_target={k: dict(v, peak_abs_torque_nm=v['peak_abs_torque_nm'].tolist()) for k, v in sorted(per_target.items())})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cycle', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/s50_static_load')
    parser.add_argument('--skip-reference', action='store_true')
    args = parser.parse_args(argv)
    started = time.time()
    args.output.mkdir(parents=True, exist_ok=True)
    context = make_context(ROOT, args.output / 'model', args.config)
    cycle, cycle_sha = _load_json(args.cycle)
    if cycle.get('schema') != 's50_full_kinematic_cycle_v1':
        raise ValueError('S50_FULL_KINEMATIC_CYCLE_REQUIRED')
    if cycle['source_model_sha256'] != context.definition['definition_sha256']:
        raise ValueError('CYCLE_MODEL_DIFFERS_FROM_SCREENING_MODEL')
    geometry_keys = ('model_definition', 'source_urdf', 'joint_coordinate_convention', 'installation_reference_from_model_base',
        'flange_from_original_tool_mount', 'tool_mount_profile', 'tool_mount_standoff_m', 'tool_flange_spacer_m', 'layout_gap_m')
    changed = [k for k in geometry_keys if cycle['settings'].get(k) != context.settings.get(k)]
    if changed:
        raise ValueError('CYCLE_GEOMETRY_SETTINGS_DIFFER:' + ','.join(changed))
    for key in ('flange_from_physical_contact', 'flange_from_virtual_task_tcp'):
        expected = context.frames['nominal_compressed_contact' if 'contact' in key else 'virtual_task_tcp']
        if not np.allclose(cycle[key], expected, atol=1e-9):
            raise ValueError('CYCLE_TOOL_FRAME_DIFFERS:' + key)
    links, density, link_record = link_inertials(context)
    tool = tool_mass_properties(context)
    carton_mass = float(context.intrinsic['carton_mass_kg'])
    cases = {}
    for scale in SENSITIVITY:
        scaled = {k: dict(v, mass_kg=v['mass_kg'] * (1. if k == context.definition['base_link'] else scale))
                  for k, v in links.items()}
        result = screen(cycle, context, scaled, tool, carton_mass)
        if not args.skip_reference:
            reference, reference_q = rated_reference(context, scaled)
            result['rated_reference_abs_torque_nm'] = reference.tolist()
            result['rated_reference_q_rad'] = reference_q
            result['peak_to_rated_reference_ratio'] = (np.asarray(result['peak_abs_torque_nm']) / reference).tolist()
        cases[f'link_mass_x{scale:.1f}'] = result
    historical = screen(cycle, context, links, tool, 42.5)
    report = dict(schema='s50_quasi_static_load_screening_v1', status='SCREENING_ESTIMATE_NOT_QUALIFICATION',
        cycle=dict(path=str(args.cycle), uncompressed_sha256=cycle_sha, status=cycle.get('status'),
                   completed_cartons=cycle.get('completed_cartons')),
        config=str(args.config), model_definition_sha256=context.definition['definition_sha256'],
        environment_fingerprint=context.frozen['environment_fingerprint'],
        gravity_m_s2=GRAVITY_M_S2, carton_mass_kg=carton_mass, tool=dict(mass_kg=tool[0], com_flange_m=tool[1].tolist(),
            inertia_com_kg_m2=tool[2].tolist(), source='configs/tools/unloading_gripper_20kg.yaml (scaled STEP distribution; S50 spacer/mount mass not added)'),
        link_inertials=dict(method='CAD_VISUAL_MESH_VOLUME_UNIFORM_DENSITY_SCALED_TO_PUBLISHED_TOTAL',
            published_total_mass_kg=PUBLISHED_TOTAL_MASS_KG, density_kg_m3=density, links=link_record),
        payload_curve=dict(source='assets/robots/huayan_s50_2000/source/S-series-mechanical.pdf PDF page 37',
            points_mm_kg=S50_PAYLOAD_CURVE_MM_KG, reading_uncertainty_kg=0.5,
            note='Manual 2.3.4 states larger payloads are allowed with the last joint facing down, but gives no S50 curve for that case; not used.'),
        sensitivity_cases=cases, historical_42p5kg_carton=historical,
        manufacturer_joint_torque_ratings='NOT_SUPPLIED', elapsed_s=time.time() - started)
    (args.output / 'static-load-screening.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    nominal = cases['link_mass_x1.0']
    print(json.dumps(dict(peak_abs_torque_nm=nominal['peak_abs_torque_nm'],
        ratio=nominal.get('peak_to_rated_reference_ratio'), max_payload_offset_m=nominal['max_payload_offset_m'],
        min_payload_curve_margin_kg=nominal['min_payload_curve_margin_kg'],
        samples_over_payload_curve=nominal['samples_over_payload_curve'],
        peak_overturning_moment_nm=nominal['peak_overturning_moment_nm'],
        fk_replay_max_error_m=nominal['fk_replay_max_error_m'], elapsed_s=report['elapsed_s'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
