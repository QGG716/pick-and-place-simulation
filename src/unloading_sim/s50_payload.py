"""S50-only carton payload, independent of the inherited FANUC carton record.

The robot-independent frozen environment keeps the FANUC carton value from
``configs/simulation/m710id70_official_dynamics_v2.yaml`` (42.5 kg), so FANUC
runs and the S50-versus-baseline geometry comparison are unchanged. Every S50
environment replaces it with the single value in
``configs/robots/huayan_s50_2000.yaml#carton_payload`` after that comparison.

Kinematic run settings select it with ``carton_payload_profile:
S50_ROBOT_CONFIG``. Historical run settings recorded without the profile keep
the inherited value, so their recorded environment fingerprints remain
reproducible.

This is a bookkeeping input for kinematic records and later load screening; it
is not payload qualification.
"""
from copy import deepcopy
from pathlib import Path
import math

import yaml

from .workcell_layout import canonical_digest

ROBOT_CONFIG = 'configs/robots/huayan_s50_2000.yaml'
INHERITED_PROFILE = 'INHERITED_FANUC_CARTON_RECORD'
CONFIGURED_PROFILE = 'S50_ROBOT_CONFIG'


def configured_carton_mass_kg(data):
    """Return the S50 carton mass from a loaded S50 robot or task config."""
    payload = (data or {}).get('carton_payload')
    mass = payload.get('carton_mass_kg') if isinstance(payload, dict) else None
    if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
        raise ValueError('S50_CARTON_MASS_NOT_CONFIGURED')
    return float(mass)


def apply_s50_carton_mass(frozen, root):
    """Return a rehashed copy of a frozen scene carrying the S50 carton mass."""
    if 'inherited_carton_mass_kg' in frozen['tool_intrinsic']:
        raise ValueError('S50_CARTON_MASS_ALREADY_APPLIED')
    data = yaml.safe_load((Path(root) / ROBOT_CONFIG).read_text(encoding='utf8'))
    mass = configured_carton_mass_kg(data)
    scene = deepcopy(frozen)
    intrinsic = scene['tool_intrinsic']
    intrinsic.update(carton_mass_kg=mass, inherited_carton_mass_kg=float(intrinsic['carton_mass_kg']),
        carton_payload_profile=CONFIGURED_PROFILE, carton_mass_source=ROBOT_CONFIG + '#carton_payload',
        carton_mass_basis=data['carton_payload'].get('source'))
    scene['source_environment_fingerprint_before_s50_payload_change'] = scene.pop('environment_fingerprint')
    scene['environment_fingerprint'] = canonical_digest(scene)
    return scene


def bind_s50_carton_payload(frozen, settings, root):
    profile = settings.get('carton_payload_profile', INHERITED_PROFILE)
    if profile == INHERITED_PROFILE:
        return frozen
    if profile != CONFIGURED_PROFILE:
        raise ValueError('UNKNOWN_S50_CARTON_PAYLOAD_PROFILE:' + str(profile))
    return apply_s50_carton_mass(frozen, root)


def freeze_s50_environment(layout):
    """Frozen S50 environment: shared non-robot geometry plus the S50 carton mass."""
    from .robot_replacement_layout import freeze_non_robot_scene
    return apply_s50_carton_mass(freeze_non_robot_scene(layout), Path(layout.config_path).parents[2])
