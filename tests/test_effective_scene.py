import copy
import json
from pathlib import Path

import pytest

from unloading_perception.effective_scene import derive_effective_scene, trailer_solids
from unloading_perception.isaac_validation import load_validation_config, feasibility_digest

ROOT = Path(__file__).resolve().parents[1]


def test_roof_and_floor_thickness_do_not_consume_clearance():
    solids = trailer_solids({'local_x_range_m': [-3.8, 4.], 'inside_width_m': 2.3,
                             'inside_height_m': 2.7, 'wall_thickness_m': .06})
    by_name = {s['name']: s for s in solids}
    assert by_name['Roof']['center_m'][2] - by_name['Roof']['size_xyz_m'][2] / 2 == pytest.approx(2.7)
    assert by_name['Floor']['center_m'][2] + by_name['Floor']['size_xyz_m'][2] / 2 == pytest.approx(0.)
    assert by_name['LeftWall']['center_m'][1] - .03 == pytest.approx(1.15)
    assert by_name['RightWall']['center_m'][1] + .03 == pytest.approx(-1.15)
    assert all(s['collision_enabled'] for s in solids)
    with pytest.raises(ValueError):
        trailer_solids({'local_x_range_m': [4., -3.8], 'inside_width_m': 2.3,
                        'inside_height_m': 2.7, 'wall_thickness_m': .06})


def test_derived_scene_preserves_parent_cartons_and_fixed_layout():
    config = load_validation_config(ROOT / 'configs/isaac/perception_validation.yaml')
    snapshot = json.loads((ROOT / config['layout_bundle']['snapshot']).read_text())
    contract = json.loads((ROOT / config['layout_bundle']['isaac_contract']).read_text())
    before = copy.deepcopy((snapshot, contract))
    derived, effective = derive_effective_scene(snapshot, contract, config, ROOT)
    assert (snapshot, contract) == before
    assert derived['cartons'] == snapshot['cartons']
    assert derived['assembly'] == snapshot['assembly']
    assert effective['primitives'][:len(contract['primitives'])] == contract['primitives']
    assert derived['robot']['world_from_mount'][0][3] == pytest.approx(-1.325)
    assert derived['trailer']['inside_length_m'] == 9.6
    assert derived['trailer']['door_world_x_m'] is None
    assert derived['layout_id'] != snapshot['layout_id']
    assert derived['layout_fingerprint'] != snapshot['layout_fingerprint']
    value = dict(effective); recorded = value.pop('contract_fingerprint')
    assert feasibility_digest(value) == recorded
    assert not effective['claims']['other_branch_execution_certification']
