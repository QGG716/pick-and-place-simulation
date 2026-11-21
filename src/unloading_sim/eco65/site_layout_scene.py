"""Simulation scene from the user's site layout STEP (base_link frame).

Same manifest, roles and lift mechanism as the real-site static layer
(``workbench.single_carton.site_layout``). Simulation additionally receives
the CAD cartons as the initial carton set; the real site never does.
Execution mode is simulation only; nothing here touches hardware.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
import yaml

from .site_mechanism import LiftMechanism, cad_robot_base, load_manifest, load_roles, translation

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'configs/workcells/eco65_site_layout_sim_v1.yaml'


def simulation_lift(config, mechanism):
    """Simulation h needs a basis; the field range is recorded if known."""
    c = config.get('lift')
    if not isinstance(c, dict) or type(c.get('h_m')) not in (int, float) or not np.isfinite(c['h_m']):
        raise ValueError('SIM_SITE_LIFT_H_REQUIRED')
    if not isinstance(c.get('h_basis'), str) or not c['h_basis'].strip():
        raise ValueError('SIM_SITE_LIFT_H_BASIS_REQUIRED')
    state = mechanism.state(c['h_m'])
    r = c.get('range_m')
    if r is not None and not (r[0] <= c['h_m'] <= r[1]):
        raise ValueError('SIM_SITE_LIFT_H_OUTSIDE_FIELD_RANGE')
    return dict(h_m=float(c['h_m']), h_basis=c['h_basis'], range_m=r,
                range_basis=c.get('range_basis', 'FIELD_RANGE_NOT_PROVIDED'), state=state)


def build_site_scene(config=CONFIG, root=ROOT):
    root = Path(root)
    if not isinstance(config, dict):
        config = yaml.safe_load(Path(config).read_text(encoding='utf-8'))
    if config.get('schema') != 'eco65_site_layout_sim_v1' or config.get('execution_mode') != 'simulation':
        raise ValueError('SIM_SITE_CONFIG_INVALID')
    manifest_path, roles_path = root / config['manifest'], root / config['roles']
    manifest, roles = load_manifest(manifest_path), load_roles(roles_path)
    mechanism = LiftMechanism(manifest, roles)
    lift = simulation_lift(config, mechanism)
    T = cad_robot_base(roles)
    h = lift['h_m']
    world = [dict(id='site_' + b['id'], size=b['size'], pose=(T @ np.asarray(b['pose'])).tolist(),
                  category='environment', role=b['role'], name=b['name'], part_ids=b['part_ids'])
             for b in mechanism.boxes(h)]
    cartons = []
    for p in manifest['parts']:
        if mechanism.classification[p['id']]['role'] != 'CARTON':
            continue
        lo, hi = np.asarray(p['collision_bounds_m'], float)
        cartons.append(dict(id='carton_' + p['id'], size_m=(hi - lo).tolist(), pose=(T @ translation((lo + hi) / 2)).tolist(),
                            mass_kg=config['carton_mass_kg'], source_part=p['id'],
                            evidence='simulation initial carton from site layout CAD; not a camera measurement'))
    cartons.sort(key=lambda c: (-c['pose'][2][3], c['pose'][1][3], -c['pose'][0][3]))
    excluded = {r: sum(1 for v in mechanism.classification.values() if v['role'] == r) for r in roles['excluded_roles']}
    return dict(schema='eco65_site_layout_scene_v1', execution_mode='simulation', enable_hardware=False,
                frame='base_link', world_objects=world, cartons=cartons, lift=lift, T_base_cad=T.tolist(),
                excluded_roles=excluded, mechanism=mechanism.summary(),
                identity=dict(manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                              roles_sha256=roles['_sha256'], source_sha256=manifest['source_sha256']),
                notes=['CAD design geometry; dimensions and cartons are simulation inputs, not field measurements.',
                       'Robot links, camera mast and tool come from the official URDF and existing tool/mast assets.'])
