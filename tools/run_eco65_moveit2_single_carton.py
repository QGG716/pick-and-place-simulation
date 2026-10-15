"""Explicit MoveIt2 entry. No fallback planner and no executable artifact export."""
import argparse
from pathlib import Path
from time import perf_counter
import traceback
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import build_snapshot, verify_snapshot, warehouse_bounds
from unloading_sim.eco65.native_task import NativeTask, review_plan


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['build', 'bounds', 'plan', 'review', 'export'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--snapshot', type=Path); p.add_argument('--plan', type=Path)
    p.add_argument('--environment', type=Path); p.add_argument('--assets', type=Path)
    p.add_argument('--worker-command', help='Existing resident native worker launcher; never a core fallback')
    p.add_argument('--home-candidate', type=Path, help='Identity-bound simulation initial state; rechecked by the worker')
    p.add_argument('--native-asset-root', help='Repository path as seen inside the existing ROS rootfs')
    p.add_argument('--task-seconds', type=float, default=900.)
    p.add_argument('--stage-seconds', type=float, default=60.)
    p.add_argument('--attempts', type=int, default=2, help='Bounded cold retries after search/review failure (1..3)')
    a = p.parse_args(); out = a.output.resolve(); out.relative_to(ROOT/'outputs')
    if out.exists(): raise FileExistsError('Use a fresh output directory')
    if not 1 <= a.attempts <= 3: p.error('--attempts must be 1..3')
    started = perf_counter()
    try:
        if a.action == 'build':
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
                                      stage_seconds=a.stage_seconds, attempt=attempt,
                                      home_candidate=load_json(a.home_candidate) if a.home_candidate else None)
                    plan = task.run(); save_json(attempt_out/'plan.json', plan)
                    review = review_plan(plan, snapshot); save_json(attempt_out/'review.json', review)
                    if review['status'] == 'PASS':
                        save_json(out/'plan.json', plan); save_json(out/'review.json', review)
                        save_json(out/'run_report.json', dict(status='PLANNING_ONLY_NOT_EXECUTABLE',
                            independent_geometry='PASS', attempts=attempt+1, failures=failures,
                            timings=dict(native_initialization_s=plan['native_initialization_s'],
                                planning_wall_s=plan['planning_wall_s'], independent_review_s=review['wall_s'],
                                simulated_duration_s=plan['simulated_duration_s']),
                            dynamics='NOT_EVALUATED', replay='NOT_EVALUATED'))
                        return 0
                    failures.append(review)
                except Exception as error:
                    failure = dict(error=str(error), traceback=traceback.format_exc())
                    if hasattr(error, 'native_calls'): failure['native_calls'] = error.native_calls
                    if task:
                        failure.update(native_records=task.records, native_ik_records=task.ik_records, partial_stages=task.stages)
                        failure.update(native_calls=task.native_calls, initial_state_checks=task.home_checks)
                    save_json(attempt_out/'failure.json', failure); failures.append(failure)
                finally:
                    if task: task.close()
                remaining -= perf_counter()-begin
                if remaining <= 0 or task is None: break
            save_json(out/'run_report.json', dict(status='NO_REVIEWED_COMPLETE_PLAN', failures=failures,
                total_wall_s=perf_counter()-started, dynamics='NOT_EVALUATED', replay='NOT_EVALUATED'))
            return 2
        if not a.plan: p.error('--plan required')
        plan = load_json(a.plan)
        if a.action == 'review':
            report = review_plan(plan, snapshot); save_json(out/'review.json', report)
            return 0 if report['status'] == 'PASS' else 2
        if a.action == 'export':
            from unloading_sim.eco65.native_task import validate_plan_contract
            validate_plan_contract(plan, snapshot)
            from export_eco65_warehouse_workstation import export
            export(a.snapshot, a.plan, ROOT/'configs/workcells/eco65_desktop_warehouse_v1.yaml', out)
            save_json(out/'export_timing.json', dict(usd_export_s=perf_counter()-started, dynamics='NOT_EVALUATED'))
            return 0
    except Exception as error:
        save_json(out/'failure.json', dict(status='BLOCKED_OR_FAILED', error=str(error),
            traceback=traceback.format_exc(), wall_s=perf_counter()-started, dynamics='NOT_EVALUATED'))
        print(type(error).__name__, error); return 2


if __name__ == '__main__': raise SystemExit(main())
