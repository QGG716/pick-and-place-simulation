"""Cold native one-carton cycle plus actual next-carton contact; no hardware."""
import argparse
from pathlib import Path
import traceback
from unloading_sim.eco65.model import ROOT,load_json,save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.coupled_task import CoupledTask


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--home-candidate',type=Path,required=True)
    p.add_argument('--worker-command',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--task-seconds',type=float,default=900.)
    p.add_argument('--connection-seconds',type=float,default=100.)
    p.add_argument('--solver-seconds',type=float,default=8.)
    p.add_argument('--rpc-seconds',type=float,default=100.)
    p.add_argument('--ik-seeds',type=int,default=8)
    p.add_argument('--max-grasps',type=int,default=24)
    p.add_argument('--search-trace',action='store_true')
    a=p.parse_args()
    output=a.output.resolve()
    output.relative_to(ROOT/'outputs')
    output.mkdir(parents=True,exist_ok=False)
    if not 1<=a.ik_seeds<=32 or not 1<=a.max_grasps<=64:
        p.error('bounded input requires 1..32 IK seeds, 1..64 grasp candidates')
    snapshot=verify_snapshot(load_json(a.snapshot))
    task=None
    try:
        task=CoupledTask(snapshot,a.worker_command,home_candidate=load_json(a.home_candidate),
            task_seconds=a.task_seconds,connection_seconds=a.connection_seconds,
            stage_seconds=a.solver_seconds,rpc_seconds=a.rpc_seconds,ik_seeds=a.ik_seeds,
            max_grasps=a.max_grasps,search_trace=a.search_trace,
            trace_stages=('pregrasp','transit','residence'),progress_path=output/'native_progress.jsonl')
        plan=task.run()
        save_json(output/'plan.json',plan)
        report=dict(status='COMPLETE_ONE_CARTON_AND_NEXT_CONTACT',complete=True,
            first_target_id=plan['first_target_id'],next_target_id=plan['next_target_id'],
            validation=plan['validation'],planning_wall_s=plan['planning_wall_s'],
            robot_motion_s=plan['robot_motion_s'],mechanism_display_s=plan['mechanism_display_s'],
            event_display_s=plan['event_display_s'],physical_execution='NOT_RUN',execution_ready=False)
        save_json(output/'run_report.json',report)
        print(report)
        return 0
    except Exception as error:
        if task:
            save_json(output/'partial_plan.json',task.result())
        report=dict(status='INCOMPLETE',complete=False,error=str(error),traceback=traceback.format_exc(),
                    physical_execution='NOT_RUN',execution_ready=False)
        save_json(output/'run_report.json',report)
        print(report)
        return 2
    finally:
        if task:task.close()


if __name__=='__main__':
    raise SystemExit(main())
