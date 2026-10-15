"""Freeze an already accepted initial state; no IK, planning or motion."""
import argparse
import hashlib
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.home_candidate import verify_candidate


def prepare(snapshot_path, candidate_path, failure_path, output):
    snapshot=verify_snapshot(load_json(snapshot_path))
    candidate=verify_candidate(load_json(candidate_path),snapshot['tool'],snapshot['scene'])
    failure=load_json(failure_path)
    accepted=[r for r in failure['initial_state_checks'] if r['status']=='ACCEPTED_SIMULATION_INITIAL_STATE']
    if len(accepted)!=1: raise ValueError('ONE_ACTUAL_ACCEPTED_HOME_REQUIRED')
    row=accepted[0]; q=row['q_rad']
    first=failure['partial_stages'][0]
    if first['stage']!='pregrasp' or not np.array_equal(first['q'][0],q):
        raise ValueError('INITIAL_STATE_DIFFERS_FROM_ACTUAL_TASK')
    if first['native_request']['identity']!=snapshot['native_model']['init']['identity']:
        raise ValueError('ACTUAL_TASK_IDENTITY_MISMATCH')
    output=Path(output)
    if output.exists(): raise FileExistsError(output)
    save_json(output,dict(status='STATIC_POSE_ONLY',home_q=q,
        source_files={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [snapshot_path,candidate_path,failure_path]},
        source='actual accepted initial_state_checks and first pregrasp start',
        proposal_matches_accepted=bool(np.array_equal(candidate['q_rad'],q)),
        source_snapshot_fingerprint=snapshot['fingerprint'],identity=snapshot['native_model']['init']['identity'],
        joint_names=snapshot['native_model']['init']['joint_names'],
        motion='NO_MOTION_EXECUTED',dynamics='NOT_EVALUATED'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['snapshot','candidate','failure','output']: p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();prepare(a.snapshot,a.candidate,a.failure,a.output)
