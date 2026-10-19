"""Explicit MoveIt2 entry. No fallback planner and no executable artifact export."""
import argparse
from pathlib import Path
from time import perf_counter
import traceback
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import build_snapshot, verify_snapshot, warehouse_bounds
from unloading_sim.eco65.native_task import NativeTask, review_plan
from unloading_sim.eco65.evaluation import PROFILES, STRICT, CHAIN


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['build', 'bounds', 'plan', 'review', 'export'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--snapshot', type=Path); p.add_argument('--plan', type=Path)
    p.add_argument('--environment', type=Path); p.add_argument('--assets', type=Path)
    p.add_argument('--static-snapshot', type=Path,
                   help='Frozen current twelve-carton scene, including its J1-mounted accessories')
    p.add_argument('--static-home', type=Path,
                   help='Same-source display pose used only as a freshly checked initial-state proposal')
    p.add_argument('--camera-configuration', type=Path,
                   help='Source-bound permanent camera installation; calibration board/probe are not task equipment')
    p.add_argument('--worker-command', help='Existing resident native worker launcher; never a core fallback')
    p.add_argument('--evaluation-profile', choices=PROFILES,
                   help='Default: strict for planning; export/review inherit the fingerprinted plan profile')
    p.add_argument('--home-candidate', type=Path, help='Identity-bound simulation initial state; rechecked by the worker')
    p.add_argument('--search-trace',action='store_true',help='Record actual selected OMPL solves; ordinary runs default off')
    p.add_argument('--trace-stage',action='append',help='Record these semantic stages (default transit)')
    p.add_argument('--trace-request',action='append',help='Optional exact stage_id filter')
    p.add_argument('--preferred-candidate-id',help='Prioritize a current geometric candidate; regenerate every motion')
    p.add_argument('--preferred-grasp-face',choices=['top','front'],
                   help='Stable face priority for independent cups; retains all candidates and regenerates every path')
    p.add_argument('--native-asset-root', help='Repository path as seen inside the existing ROS rootfs')
    p.add_argument('--task-seconds', type=float, default=900.)
    p.add_argument('--stage-seconds', type=float, default=60.)
    p.add_argument('--rpc-seconds',type=float,default=180.,help='Bounded watchdog including scene import and output path checking; does not extend solver time')
    p.add_argument('--attempts', type=int, default=2, help='Bounded cold retries after search/review failure (1..3)')
    a = p.parse_args(); out = a.output.resolve(); out.relative_to(ROOT/'outputs')
    if out.exists(): raise FileExistsError('Use a fresh output directory')
    if not 1 <= a.attempts <= 3: p.error('--attempts must be 1..3')
    if a.preferred_grasp_face and a.action!='plan':p.error('--preferred-grasp-face requires plan')
    if a.static_snapshot and (a.action != 'build' or not a.static_home):
        p.error('--static-snapshot requires build and --static-home')
    if (a.static_home or a.camera_configuration) and not a.static_snapshot:
        p.error('--static-home/--camera-configuration require --static-snapshot')
    if a.static_snapshot and a.environment:
        p.error('The static snapshot already binds its complete warehouse environment')
    started = perf_counter()
    try:
        if a.action == 'build':
            if a.static_snapshot:
                from build_eco65_native_12carton import build_from_static
                result = build_from_static(a.static_snapshot, a.static_home, out,
                    camera_configuration=a.camera_configuration,
                    native_asset_root=a.native_asset_root)
            else:
                result = build_snapshot(out, load_json(a.environment) if a.environment else None, a.native_asset_root)
            print(result); return 0 if result['status'] != 'BLOCKED' else 2
        out.mkdir(parents=True)
        if a.action == 'bounds':
            if not a.assets: p.error('bounds requires --assets')
            save_json(out/'environment_bounds.json', warehouse_bounds(a.assets)); return 0
        if not a.snapshot: p.error('--snapshot required')
        snapshot = verify_snapshot(load_json(a.snapshot))
        if a.action == 'plan':
            if not a.worker_command: raise ValueError('MOVEIT2_WORKER_REQUIRED_NO_FALLBACK')
            remaining = a.task_seconds; failures = []
            for attempt in range(a.attempts):
                task = None; begin = perf_counter(); attempt_out = out/f'attempt_{attempt:02d}'
                try:
                    task = NativeTask(snapshot, a.worker_command, task_seconds=remaining,
                                      evaluation_profile=a.evaluation_profile or STRICT,
                                      search_trace=a.search_trace,trace_stages=a.trace_stage or ('transit',),
                                      trace_requests=a.trace_request or (),preferred_candidate_id=a.preferred_candidate_id,
                                      stage_seconds=a.stage_seconds, attempt=attempt,rpc_seconds=a.rpc_seconds,
                                      progress_path=attempt_out/'native_progress.jsonl',
                                      preferred_grasp_face=a.preferred_grasp_face,
                                      home_candidate=load_json(a.home_candidate) if a.home_candidate else None)
                    plan = task.run(); save_json(attempt_out/'plan.json', plan)
                    if plan['evaluation_profile'] == CHAIN:
                        save_json(out/'plan.json', plan)
                        save_json(out/'run_report.json', dict(status='PLANNING_ONLY_NOT_EXECUTABLE',
                            evaluation_profile=CHAIN, native_output_status='BASIC_SAMPLED_PASS',
                            independent_geometry='NOT_RUN_PROFILE', dense_output_review='NOT_RUN_PROFILE',
                            python_linear_audit='NOT_RUN_PROFILE', attempts=attempt+1, failures=failures,
                            stage_checks=[dict(stage=s['stage'], basic_path_check=s['native_result']['basic_path_check'],
                                planner_state_checks_enabled=s['native_result']['planner_internal_process_checks'],
                                planner_clearance_checks=s['native_result']['clearance']['counts'].get('search',{}).get('queries',0),
                                planner_state_checks=s['native_result']['process_policy']['planner_state_checks'],
                                start_state_status=s['native_result']['start_state_status'],
                                endpoint_fk=s['native_result'].get('endpoint_fk'),
                                native_solver_s=s['native_result']['native_solver_s']) for s in plan['stages']],
                            timings=dict(native_initialization_s=plan['native_initialization_s'],
                                planning_wall_s=plan['planning_wall_s'], total_wall_s=perf_counter()-started,
                                simulated_duration_s=plan['simulated_duration_s']),
                            dynamics='NOT_EVALUATED', replay='NOT_EVALUATED'))
                        return 0
                    review = review_plan(plan, snapshot,progress_path=attempt_out/'review_progress.jsonl'); save_json(attempt_out/'review.json', review)
                    if review['status'] == 'PASS':
                        save_json(out/'plan.json', plan); save_json(out/'review.json', review)
                        save_json(out/'run_report.json', dict(status='PLANNING_ONLY_NOT_EXECUTABLE',
                            evaluation_profile=STRICT,
                            independent_geometry='PASS', attempts=attempt+1, failures=failures,
                            timings=dict(native_initialization_s=plan['native_initialization_s'],
                                planning_wall_s=plan['planning_wall_s'], independent_review_s=review['wall_s'],
                                simulated_duration_s=plan['simulated_duration_s']),
                            dynamics='NOT_EVALUATED', replay='NOT_EVALUATED'))
                        return 0
                    failures.append(review)
                except Exception as error:
                    failure = dict(error=str(error), traceback=traceback.format_exc(),
                                   evaluation_profile=a.evaluation_profile or STRICT)
                    if hasattr(error, 'native_calls'): failure['native_calls'] = error.native_calls
                    if task:
                        failure.update(native_records=task.records, native_ik_records=task.ik_records, partial_stages=task.stages)
                        failure.update(native_calls=task.native_calls, initial_state_checks=task.home_checks)
                        failure['native_calls_archive']=task.call_archive_info()
                        failure['extraction_geometry'] = task.extraction_evidence
                        for name in ('grasp_geometry', 'grasp_search', 'grasp_failures', 'grasp_home_rejections'):
                            if hasattr(task, name): failure[name] = getattr(task, name)
                    save_json(attempt_out/'failure.json', failure); failures.append(failure)
                finally:
                    if task: task.close()
                remaining -= perf_counter()-begin
                if remaining <= 0 or task is None: break
            save_json(out/'run_report.json', dict(status='NO_BASIC_CHECKED_COMPLETE_PLAN' if a.evaluation_profile == CHAIN else 'NO_REVIEWED_COMPLETE_PLAN',
                evaluation_profile=a.evaluation_profile or STRICT, failures=failures,
                independent_geometry='NOT_RUN_PROFILE' if a.evaluation_profile == CHAIN else 'NOT_EVALUATED',
                total_wall_s=perf_counter()-started, dynamics='NOT_EVALUATED', replay='NOT_EVALUATED'))
            return 2
        if not a.plan: p.error('--plan required')
        plan = load_json(a.plan)
        if a.evaluation_profile and a.evaluation_profile != plan.get('evaluation_profile', STRICT):
            raise ValueError('EVALUATION_PROFILE_MISMATCH')
        if a.action == 'review':
            report = review_plan(plan, snapshot,progress_path=out/'review_progress.jsonl'); save_json(out/'review.json', report)
            return 0 if report['status'] == 'PASS' else 2
        if a.action == 'export':
            from unloading_sim.eco65.native_task import validate_plan_contract
            validate_plan_contract(plan, snapshot)
            from export_eco65_warehouse_workstation import export
            export(a.snapshot, a.plan, None, out)
            save_json(out/'export_timing.json', dict(usd_export_s=perf_counter()-started, dynamics='NOT_EVALUATED'))
            return 0
    except Exception as error:
        save_json(out/'failure.json', dict(status='BLOCKED_OR_FAILED', error=str(error),
            evaluation_profile=a.evaluation_profile or (plan.get('evaluation_profile', STRICT) if 'plan' in locals() else STRICT),
            traceback=traceback.format_exc(), wall_s=perf_counter()-started, dynamics='NOT_EVALUATED'))
        print(type(error).__name__, error); return 2


if __name__ == '__main__': raise SystemExit(main())
