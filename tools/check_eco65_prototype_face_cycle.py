"""Bounded representative face diagnostic in an unchanged frozen inventory.

This uses the same native CoupledTask and preserves rejected requests. A result
is an independent diagnostic, never added to the continuous batch's count.
"""
import argparse
import gzip
import json
from pathlib import Path
import traceback
from unloading_sim.eco65.coupled_task import CoupledTask
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.moveit2_backend import digest


class FaceDiagnostic(CoupledTask):
    def __init__(self, *args, diagnostic_face, **kwargs):
        self.diagnostic_face = diagnostic_face
        super().__init__(*args, **kwargs)

    def geometry_candidates(self):
        result = super().geometry_candidates()
        if self.target['id'] == self.first_target_id and not self.completed_ids:
            original = result['candidates']
            result['candidates'] = [g for g in original if g['target_face'] == self.diagnostic_face]
            result['diagnostic_face_filter'] = dict(face=self.diagnostic_face,
                excluded_candidate_count=len(original)-len(result['candidates']),
                excluded_reason='EXPLICIT_REPRESENTATIVE_FACE_DIAGNOSTIC_NOT_INFEASIBILITY')
        return result

    def result(self, *, complete=False):
        result = super().result(complete=complete)
        result['diagnostic_scope'] = dict(first_face=self.diagnostic_face,
            inventory='UNCHANGED_FROZEN_SCENE', independent_run=True,
            count_toward_continuous_batch=False)
        result.pop('fingerprint'); result['fingerprint'] = digest(result)
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot', 'home-candidate', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--worker-command', required=True)
    p.add_argument('--face', choices=('front','top','left','right'), required=True)
    p.add_argument('--task-seconds', type=float, default=300.)
    a = p.parse_args()
    if not 30 <= a.task_seconds <= 600:
        p.error('representative diagnostic requires a 30..600 second absolute budget')
    out = a.output.resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True, exist_ok=False)
    snapshot = verify_snapshot(load_json(a.snapshot)); task = None
    try:
        task = FaceDiagnostic(snapshot, a.worker_command, diagnostic_face=a.face,
            home_candidate=load_json(a.home_candidate), task_seconds=a.task_seconds,
            connection_seconds=100., stage_seconds=6., rpc_seconds=90., ik_seeds=8,
            max_grasps=24, search_trace=True, trace_stages=('pregrasp','transit','residence'),
            progress_path=out/'native_progress.jsonl')
        plan = task.run(); error = None
    except Exception as exc:
        error = dict(message=str(exc), traceback=traceback.format_exc())
        plan = task.result() if task else None
    finally:
        if task: task.close()
    if plan:
        with gzip.open(out/'plan.json.gz','wt',encoding='utf-8',compresslevel=3) as stream:
            json.dump(plan,stream,separators=(',',':'),allow_nan=False)
    report = dict(status='COMPLETE_REPRESENTATIVE_CYCLE_AND_NEXT_CONTACT' if plan and plan['complete'] else 'INCOMPLETE',
        complete=bool(plan and plan['complete']), face=a.face, error=error,
        plan_file='plan.json.gz' if plan else None, snapshot_fingerprint=snapshot['fingerprint'],
        completed_ids=(plan.get('terminal_state') or {}).get('completed_ids',[]) if plan else [],
        validation=plan.get('validation') if plan else None,
        planning_wall_s=plan['planning_wall_s'] if plan else None,
        inventory='UNCHANGED_FROZEN_SCENE', count_toward_continuous_batch=False,
        physical_execution='NOT_RUN', execution_ready=False)
    save_json(out/'run_report.json', report); print(report)
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
