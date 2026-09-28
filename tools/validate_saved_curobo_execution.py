"""Finalize/recheck a saved complete task without invoking any planner or Isaac."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
from time import perf_counter
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState, RowSequencePolicy
from unloading_sim.m710_execution import build_m710_execution_preflight, write_m710_execution_preflight
from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.m710_replay_contract import verify_m710_replay_bundle
from unloading_sim.curobo_execution import validate_execution_handoff
from unloading_sim.stage_export import file_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion',required=True,type=Path)
    parser.add_argument('--state',required=True,type=Path)
    parser.add_argument('--config',default='configs/validation/m710id70_handoff_continuation.yaml')
    parser.add_argument('--execution-config',default='configs/simulation/m710id70_handoff_continuation.yaml')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    started=perf_counter()
    def write(name,value):
        (args.output/name).write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')
    report=dict(planning_invoked=False,isaac_executed=False,motion_sha256=file_hash(args.motion),
                state_sha256=file_hash(args.state),status='NOT_COMPLETED')
    try:
        motion=json.loads(args.motion.read_text())
        policy=load_layout_motion_policy(args.config)
        policy=replace(policy,data=motion['effective_motion_policy'])
        initial=build_verified_motion_input(policy)
        rows=RowUnloadingState(RowSequencePolicy(row_height_fraction=float(
            policy.data['search_strategy'].get('row_height_fraction',.05))))
        rows.rank(initial.cartons,support_graph=initial.support_graph)
        scene=apply_actual_motion_state(initial,json.loads(args.state.read_text()),row_state=rows)
        if scene.snapshot['scene_fingerprint']!=motion['scene_fingerprint']:
            raise ValueError('saved complete task belongs to a different actual scene')
        t=perf_counter()
        preflight=build_m710_execution_preflight(args.execution_config,motion_result=motion,motion_input=scene)
        write_m710_execution_preflight(preflight,args.output/'preflight.json')
        report['preflight_s']=perf_counter()-t
        if not preflight['simulation_execution_ready']:
            report.update(status='PREFLIGHT_REJECTED',failure=preflight['blockers'])
            return 3
        t=perf_counter();adapter=preflight['replay_adapter_inputs']
        plan=dict(adapter['plan_common'],segments=[adapter['trajectory_segment']])
        bundle=build_fanuc_isaac_replay_bundle(plan,adapter['configuration'],preflight=preflight).to_dict()
        write('replay_bundle.json',bundle)
        report['export_s']=perf_counter()-t
        report['readback']=verify_m710_replay_bundle(bundle,project_root=policy.project_root)
        built=_build_automatic_trajectory_connector(scene,scene.policy.layout_validation.layout.robot())
        if built.connector is None:raise ValueError('exact execution validator unavailable: '+str(built))
        connector=built.connector
        connector.stack_carton_names=({b.name for b in scene.cartons}
            if scene.remaining_stack_names is None else set(scene.remaining_stack_names))
        handoff=validate_execution_handoff(scene,connector,motion,bundle)
        write('execution_handoff.json',handoff)
        report.update(status='PASS' if handoff['accepted'] else 'EXECUTION_HANDOFF_REJECTED',
                      failure=handoff['failure'],handoff_wall_s=handoff['wall_s'])
        return 0 if handoff['accepted'] else 4
    except (ValueError,RuntimeError,KeyError,TypeError) as exc:
        report.update(status='REJECTED',failure=dict(reason=type(exc).__name__,detail=str(exc)))
        return 2
    finally:
        report['wall_s']=perf_counter()-started
        write('delivery.json',report)
        print(json.dumps(report),flush=True)


if __name__=='__main__':raise SystemExit(main())
