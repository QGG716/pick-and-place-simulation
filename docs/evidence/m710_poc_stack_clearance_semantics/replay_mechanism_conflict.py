"""CPU replay of saved measured frames; no Isaac launch or reconstructed pose.

Run from repository root with PYTHONPATH=src. These are 5 fps samples around,
not an invented full-rate reconstruction of, the first conflict at step 3027.
"""
import json
from copy import deepcopy
from pathlib import Path

import yaml

from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.m710_replay_physics import ActualStackContactMonitor
from unloading_sim.serial_unloading import rotation_from_actual_quaternion
from unloading_sim.stack_clearance import StackClearanceStep


def replay():
    fixture = json.loads(Path(__file__).with_name('mechanism_conflict_fixture.json').read_text())
    policy = SimulationCollisionPolicy.from_mapping(yaml.safe_load(Path(
        'configs/validation/m710id70_layout_poc_pair_clearance.yaml').read_text())['collision_policy'])
    initial = {x['name']: OBB(x['center_m'], [v / 2 for v in x['size_m']],
        rotation_from_actual_quaternion(x['quaternion_wxyz']), x['name'])
        for x in fixture['initial_states']}
    # The mechanism revision stored backend offsets in result.json separately.
    # Explicitly join that original record in the original body-view order;
    # this does not edit the measured frames or invent a geometry inflation.
    shapes = deepcopy(fixture['shapes'])
    for i, state in enumerate(fixture['initial_states']):
        assert shapes[state['name']]['actor'] == state['prim_path']
        offsets = fixture['effective_offsets']['contact_offsets_m'][i]
        assert len(offsets) == 1
        shapes[state['name']]['contact_generation_offset_m'] = offsets[0]
    results = []
    for row in fixture['samples']:
        # Each sample starts with its recorded pre-step phase, before free-space
        # entry. There is no claimed interpolation across unrecorded steps.
        assert not row['geometry_free_before']
        monitor = ActualStackContactMonitor(initial['target'], [initial['neighbor']], policy)
        gate = StackClearanceStep(shapes=shapes, target='target', neighbors=['neighbor'],
            policy=policy, world_id=fixture['source_world'], task_id='offline-recorded-step', maximum_wait_s=1.)
        gate.begin_step(step=row['step'], time_s=row['pre_time_s'],
            trajectory_time_s=row['trajectory_time_s'], context=row['context'],
            states=list(row['pre_states'].values()))
        for contact in row['contacts']:
            gate.collect(**{k: contact[k] for k in
                ('actor0', 'actor1', 'collider0', 'collider1', 'event', 'points')})
        outcome = gate.finish_step(states=list(row['post_states'].values()), time_s=row['time_s'],
            monitor=monitor, commanded_motion=not row['hold'])
        results.append(dict(step=row['step'], geometry=gate.last['pair_geometry'],
            evidence_status=gate.last['evidence_status'], hold=outcome['hold'],
            free_space_reached=monitor.free_space_reached))
    assert results[0]['evidence_status'] == 'RESOLVED'
    assert results[1]['evidence_status'] == 'STACK_CONTACT_GEOMETRY_EVIDENCE_CONFLICT'
    assert results[1]['hold'] and not results[1]['free_space_reached']
    return results


if __name__ == '__main__':
    print(json.dumps(replay(), indent=2))
