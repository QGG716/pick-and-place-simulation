"""Explicit new simulation start after a retained HOME connection failure."""
import argparse
import hashlib
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_sim.eco65.model import ROOT,load_json,save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.unloading_layout import UnloadingWorld


def propose(snapshot_path,failure_path,output):
    started=perf_counter();out=Path(output).resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    snapshot=verify_snapshot(load_json(snapshot_path));failure=load_json(failure_path)
    identity=snapshot['native_model']['init']['identity']
    calls=failure['native_calls']
    if any(c['request']['identity']!=identity for c in calls):raise ValueError('FAILED_RUN_MODEL_CHANGED')
    goals=[np.array(r['request']['q_goal']) for r in failure['native_records']
           if r['request']['stage']=='pregrasp' and r['result']['status']!='SUCCESS']
    if not goals:raise ValueError('NO_RETAINED_FAILED_PREGRASP_CONNECTION')
    q=goals[0]
    ik=next((c for c in calls if c['request']['op']=='ik' and c.get('result',{}).get('status')=='SUCCESS'
             and c['result'].get('native_ik_calls')==1 and np.array_equal(c['result']['q'],q)),None)
    checked=next((c for c in calls if c['request']['op']=='check_state_offline'
                  and np.array_equal(c['request']['q_start'],q) and c.get('result',{}).get('status')=='SUCCESS'),None)
    if ik is None or checked is None:raise ValueError('CURRENT_NATIVE_IK_AND_STATE_EVIDENCE_REQUIRED')
    world=UnloadingWorld(snapshot['tool'],snapshot['scene']);attempts=[];candidate=None
    try:
        # Two bounded joint-state proposals, not a path or an extra TCP standoff.
        # Keep the original fixed pregrasp target and require a nonzero PTP to it.
        for offset in (.04,-.04):
            proposed=q.copy();proposed[0]+=offset
            valid=world.valid(proposed,'pose')
            attempts.append(dict(q_rad=proposed.tolist(),valid=valid,failure=world.last_failure))
            if valid:
                candidate=make_candidate(proposed,snapshot['tool'],snapshot['scene'],dict(
                    source_failure_sha256=hashlib.sha256(Path(failure_path).read_bytes()).hexdigest(),
                    snapshot_fingerprint=snapshot['fingerprint'],native_ik_request=ik['request'],
                    native_ik_result=ik['result'],proposal_offset_q_rad=[offset,0,0,0,0,0],
                    reason='Previous HOME was state-valid but its pregrasp connection failed within the native budget',
                    previous_HOME_connection='NOT_VERIFIED'))
                break
    finally:world.close()
    save_json(out/'proposal_report.json',dict(attempts=attempts,wall_s=perf_counter()-started,
        native_recheck='REQUIRED_BY_PLAN_ENTRY',previous_HOME_connection='NOT_VERIFIED'))
    if candidate is None:raise ValueError('BOUNDED_HOME_PROPOSALS_REJECTED')
    save_json(out/'home_candidate.json',candidate);print('PROPOSED_NOT_NATIVE_ACCEPTED',candidate['fingerprint'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','failure','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();propose(a.snapshot,a.failure,a.output)
