"""S50-only carton payload binding; FANUC carton records stay unchanged."""
from pathlib import Path

import pytest
import yaml

from unloading_sim.robot_replacement_layout import (
    compare_non_robot_scenes, freeze_non_robot_scene, frozen_baseline_layout)
from unloading_sim.s50_payload import (
    apply_s50_carton_mass, bind_s50_carton_payload, configured_carton_mass_kg, freeze_s50_environment)
from unloading_sim.workcell_layout import canonical_digest

ROOT = Path(__file__).resolve().parents[1]


def _load(name):
    return yaml.safe_load((ROOT / name).read_text(encoding='utf8'))


def _scene(mass=42.5):
    scene = dict(tool_intrinsic=dict(tool_mass_kg=20.0, carton_mass_kg=mass), initial_carton_ids=['a'])
    scene['environment_fingerprint'] = canonical_digest(scene)
    return scene


def test_literal_s50_and_fanuc_carton_masses():
    assert configured_carton_mass_kg(_load('configs/robots/huayan_s50_2000.yaml')) == 30.0
    assert _load('configs/robots/huayan_s50_2000.yaml')['rated_payload_kg'] == 50.0
    assert _load('configs/simulation/m710id70_official_dynamics_v2.yaml')['cartons']['mass_kg_each'] == 42.5
    names = sorted(p.name for p in (ROOT / 'configs/robots').glob('huayan_s50_*kinematic_sim.yaml'))
    assert names == ['huayan_s50_150mm_kinematic_sim.yaml', 'huayan_s50_2000_kinematic_sim.yaml',
                     'huayan_s50_short105_kinematic_sim.yaml', 'huayan_s50_spacer50_kinematic_sim.yaml',
                     'huayan_s50_spacer80_kinematic_sim.yaml']
    for name in names:
        assert _load('configs/robots/' + name)['carton_payload_profile'] == 'S50_ROBOT_CONFIG'


@pytest.mark.parametrize('payload', [None, {}, dict(carton_mass_kg=0), dict(carton_mass_kg=-1.),
                                     dict(carton_mass_kg=float('nan')), dict(carton_mass_kg=True),
                                     dict(carton_mass_kg='30')])
def test_missing_or_invalid_s50_carton_mass_fails_closed(payload):
    with pytest.raises(ValueError, match='S50_CARTON_MASS_NOT_CONFIGURED'):
        configured_carton_mass_kg({} if payload is None else dict(carton_payload=payload))


def test_binding_replaces_mass_and_rehashes_scene():
    source = _scene()
    bound = bind_s50_carton_payload(source, dict(carton_payload_profile='S50_ROBOT_CONFIG'), ROOT)
    assert source['tool_intrinsic']['carton_mass_kg'] == 42.5
    assert bound['tool_intrinsic']['carton_mass_kg'] == 30.0
    assert bound['tool_intrinsic']['inherited_carton_mass_kg'] == 42.5
    assert bound['tool_intrinsic']['tool_mass_kg'] == 20.0
    assert bound['source_environment_fingerprint_before_s50_payload_change'] == source['environment_fingerprint']
    assert bound['environment_fingerprint'] != source['environment_fingerprint']
    assert bound['environment_fingerprint'] == canonical_digest(
        {k: v for k, v in bound.items() if k != 'environment_fingerprint'})


def test_mass_is_applied_once():
    bound = apply_s50_carton_mass(_scene(), ROOT)
    with pytest.raises(ValueError, match='S50_CARTON_MASS_ALREADY_APPLIED'):
        apply_s50_carton_mass(bound, ROOT)


def test_real_s50_environment_changes_only_carton_mass_after_geometry_comparison():
    layout = frozen_baseline_layout(ROOT)
    shared = freeze_non_robot_scene(layout)
    s50 = freeze_s50_environment(layout)
    assert shared['tool_intrinsic']['carton_mass_kg'] == 42.5
    assert s50['tool_intrinsic']['carton_mass_kg'] == 30.0
    assert s50['source_environment_fingerprint_before_s50_payload_change'] == shared['environment_fingerprint']
    assert compare_non_robot_scenes(shared, freeze_non_robot_scene(layout))['passed']
    for key in ('initial_cartons', 'height_samples', 'parameters', 'layout_fields'):
        assert s50[key] == shared[key]


def test_historical_settings_keep_inherited_scene_identity():
    source = _scene()
    assert bind_s50_carton_payload(source, {}, ROOT) is source
    with pytest.raises(ValueError, match='UNKNOWN_S50_CARTON_PAYLOAD_PROFILE'):
        bind_s50_carton_payload(source, dict(carton_payload_profile='S50_30KG'), ROOT)
