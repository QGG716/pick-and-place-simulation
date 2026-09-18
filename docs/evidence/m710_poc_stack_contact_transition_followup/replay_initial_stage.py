"""Replay the recorded first-step pair under the corrected production stage resolver.

This is an offline classification check, never a resumed Isaac world.
Run from repository root with PYTHONPATH=src.
"""
import json
from pathlib import Path

import yaml

from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.m710_replay_physics import ActualStackContactMonitor, resolve_actual_task_stage
from unloading_sim.serial_unloading import rotation_from_actual_quaternion
from unloading_sim.stack_clearance import StackClearanceStep


def replay():
    fixture=json.loads(Path(__file__).with_name('initial_stage_failure_fixture.json').read_text())
    row=fixture['row'];target=fixture['target'];neighbors=set(row['pre_states'])-{target}
    policy=SimulationCollisionPolicy.from_mapping(yaml.safe_load(Path(
        'configs/validation/m710id70_layout_poc_pair_clearance.yaml').read_text())['collision_policy'])
    boxes={n:OBB(x['center_m'],fixture['shapes'][n]['half_extents_m'],
        rotation_from_actual_quaternion(x['quaternion_wxyz']),n) for n,x in row['pre_states'].items()}
    monitor=ActualStackContactMonitor(boxes[target],[boxes[n] for n in neighbors],policy)
    stage=resolve_actual_task_stage(fixture['stage_windows'],row['trajectory_time_s'],
        grasp_commanded=False,grasp_event_time_s=fixture['grasp_time_seconds'],
        contact_wait_started_s=None,attached=False)
    gate=StackClearanceStep(shapes=fixture['shapes'],target=target,neighbors=neighbors,policy=policy,
        world_id=fixture['world'],task_id=fixture['task'],maximum_wait_s=1.)
    context={**row['context'],'stage':stage}
    gate.begin_step(step=row['step'],time_s=row['pre_time_s'],trajectory_time_s=row['trajectory_time_s'],
        context=context,states=list(row['pre_states'].values()),world_id=fixture['world'],task_id=fixture['task'])
    assert not row['contacts']
    result=gate.finish_step(states=list(row['post_states'].values()),time_s=row['time_s'],
        monitor=monitor,commanded_motion=False)
    assert row['context']['stage']=='home' and stage=='pregrasp'
    assert result['stop_reason'] is None and not result['hold'] and not monitor.free_space_reached
    return dict(scope=fixture['scope'],old_stage=row['context']['stage'],new_stage=stage,
        old_stop=row['stop_reason'],new_stop=result['stop_reason'],actual_geometry=gate.last['pair_geometry'],
        historical_physical_result_unchanged=True)


if __name__=='__main__':
    print(json.dumps(replay(),indent=2))
