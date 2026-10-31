"""Check the actual current-layout capture pose with the resident native worker."""
import argparse
import numpy as np
from unloading_sim.eco65.model import load_json, save_json
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.coupled_task import CoupledTask


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', required=True)
    p.add_argument('--worker-command', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--candidate-q',nargs=6,type=float,action='append',help='Finite static acquisition poses, checked in order')
    a = p.parse_args()
    from pathlib import Path
    out = Path(a.output); out.mkdir(parents=True, exist_ok=False)
    s = load_json(a.snapshot)
    candidates=a.candidate_q or [[-np.pi/2, 0., 0., 0., 0., 0.]]
    q = candidates[0]
    home = make_candidate(q, s['tool'], s['scene'], dict(source='CURRENT_LAYOUT_CAMERA_CAPTURE_STATE'))
    save_json(out/'capture_home_candidate.json', home)
    task = CoupledTask(s, a.worker_command, home_candidate=home, initial_mode='inherited_state',
                       task_seconds=180., stage_seconds=5., rpc_seconds=100.,
                       progress_path=out/'native_progress.jsonl')
    try:
        # The transverse worker requires a bound process option even for an
        # unattached stopped-state check. Selecting this context does not claim
        # contact or attachment at the independent camera capture q.
        geometry=task.geometry_candidates()
        if not geometry['candidates']:
            raise ValueError('CAPTURE_CHECK_NO_LEGAL_TRANSVERSE_PROCESS_CONTEXT')
        task.select_grasp(geometry['candidates'][0])
        attempts=[]
        for q in candidates:
            check = task.call(task.request(np.asarray(q), 'pregrasp'))
            attempts.append(dict(q_rad=q,native_check=check))
            if check['status']=='SUCCESS':break
        save_json(out/'capture_home_candidate.json',make_candidate(q,s['tool'],s['scene'],
            dict(source='CURRENT_LAYOUT_CAMERA_CAPTURE_STATE_NATIVE_CHECKED',native_status=check['status'])))
        fk = task.check_fk(np.asarray(q)) if check['status'] == 'SUCCESS' else None
        save_json(out/'report.json', dict(status=check['status'], q_rad=q, native_check=check,
            fk=fk, attempts=attempts, snapshot_fingerprint=s['fingerprint'], physical_execution='NOT_RUN',
            acquisition='INDEPENDENT_STATIC_SIMULATION_STATE_NO_CLAIMED_APPROACH_MOTION'))
        if check['status'] != 'SUCCESS':
            raise ValueError('CAPTURE_STATE_REJECTED')
    finally:
        task.close()


if __name__ == '__main__': main()
