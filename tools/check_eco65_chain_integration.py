"""Directed real native collision/profile checks on the unchanged frozen scene.

Diagnostic paths are never native planning inputs or exportable plans.
"""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_sim.eco65.model import load_json, save_json
from unloading_sim.eco65.native_task import NativeTask
from unloading_sim.eco65.evaluation import CHAIN
from unloading_sim.moveit2_backend import MoveItUnavailable, ResidentMoveItClient


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','home-candidate','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--worker-command',required=True)
    a=p.parse_args();start=perf_counter();out=a.output.resolve()
    from unloading_sim.eco65.model import ROOT
    out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    report=dict(scope='ACTUAL_NATIVE_FCL_CURRENT_SCENE_DIRECTED_DIAGNOSTIC',evaluation_profile=CHAIN)
    task=None
    try:
        home=load_json(a.home_candidate)
        task=NativeTask(load_json(a.snapshot),a.worker_command,home_candidate=home,evaluation_profile=CHAIN,
            task_seconds=600,stage_seconds=5,rpc_seconds=180,progress_path=out/'native_progress.jsonl')
        good=np.asarray(home['q_rad']);report['home']=task.call(task.request(good,'pregrasp'))
        assert report['home']['status']=='SUCCESS',report['home']
        bad=None;report['collision_probes']=[]
        for q in (np.zeros(6), np.array([.2,-.3,.4,.5,-.6,.7]), np.array([-.4,.2,-.35,-.6,.4,-.8])):
            checked=task.call(task.request(q,'pregrasp'))
            report['collision_probes'].append(dict(q=q.tolist(),result=checked))
            failure=(checked.get('clearance') or {}).get('rejected_examples',[])
            if checked['status']=='STATE_REJECTED' and any(r.get('reason')=='INTERSECTION_OR_CONTACT' for r in failure):
                bad=q;break
        if bad is None:raise ValueError('NO_REAL_COLLISION_IN_BOUNDED_DIAGNOSTIC_SET')
        for label,source,goal in [('colliding_start',bad,good),('colliding_goal',good,bad)]:
            req=task.request(source,'pregrasp',op='plan')
            req.update(q_goal=goal.tolist(),stage_id=label,pipeline_id='pilz_industrial_motion_planner',
                       planner_id='PTP',require_native_motion=True)
            reply=task.call(req);report[label]=reply
            assert reply['status']==('INVALID_START_CLEARANCE' if label=='colliding_start' else 'INVALID_GOAL_CLEARANCE'),reply
        req=task.request(good,'pregrasp',op='check_path_offline')
        req.update(probe_path=[good.tolist(),bad.tolist(),good.tolist()],output_check_timeout_s=120.)
        report['legal_endpoints_colliding_interior']=task.call(req)
        evidence=report['legal_endpoints_colliding_interior']['basic_path_check']
        assert evidence['status']=='BASIC_SAMPLED_REJECTED' and evidence['checked_states']>1,evidence
        assert 0 < evidence['failure']['fraction'] <= 1,evidence
        # A short legal same-scene path exercises completed traversal and the
        # final endpoint=true process call. No native source is claimed.
        near=good.copy();near[0]+=1e-5
        req.update(probe_path=[good.tolist(),near.tolist(),good.tolist()])
        report['basic_completed']=task.call(req)
        evidence=report['basic_completed']['basic_path_check']
        assert evidence['status']=='BASIC_SAMPLED_PASS' and evidence['checked_states']==3,evidence
        assert evidence['endpoint_process_checks']==1,evidence
        for key in ('clearance','process_policy'):
            assert report['basic_completed'][key]['dense_path_calls']==0
        # The transport deliberately closes on a protocol error; one mismatch
        # per client. Unknown initialization is checked on a fresh client below.
        for profile in ('strict',):
            req=task.request(good,'pregrasp');req['evaluation_profile']=profile
            try:
                task.native.request(req,timeout=30)
            except MoveItUnavailable as error:
                assert str(error)=='EVALUATION_PROFILE_MISMATCH',str(error)
                report['mixed_'+profile]=dict(status='REJECTED',reason=str(error))
            else:raise AssertionError('MIXED_PROFILE_ACCEPTED')
        other=ResidentMoveItClient(a.worker_command)
        try:
            req=deepcopy(task.init);req['evaluation_profile']='unknown_profile'
            try:other.request(req,timeout=30)
            except MoveItUnavailable as error:
                assert str(error)=='UNKNOWN_EVALUATION_PROFILE',str(error)
                report['unknown_initial_profile']=dict(status='REJECTED',reason=str(error))
            else:raise AssertionError('UNKNOWN_INITIAL_PROFILE_ACCEPTED')
        finally:other.close()
        report['status']='PASS'
    except Exception as error:
        report.update(status='FAIL',error=str(error));raise
    finally:
        if task:
            report['native_calls_archive']=task.call_archive_info();task.close()
        report['wall_s']=perf_counter()-start;save_json(out/'report.json',report)
    print({'status':report['status'],'wall_s':report['wall_s']})


if __name__=='__main__':main()
