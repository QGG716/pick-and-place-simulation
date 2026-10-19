"""Bounded extraction/transfer investigation from an archived actual contact endpoint.

This is a diagnostic adapter. It never writes a complete plan or bypasses native checks.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
from time import perf_counter
import uuid
import numpy as np
from unloading_sim.eco65.model import load_json,save_json,transform
from unloading_sim.eco65.native_task import NativeTask,extraction_candidates
from unloading_sim.eco65.transfer_policy import clear_stack_extractions,endpoint_fk


class DiagnosticTask(NativeTask):
    def call(self,request):
        if request['op']=='plan':
            request=dict(request,op='diagnostic_connection',search_trace=request['stage']=='transit')
        return super().call(request)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','contact-record','home-candidate','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--stage-seconds',type=float,default=30.)
    p.add_argument('--task-seconds',type=float,default=600.);p.add_argument('--max-lifts',type=int,default=2)
    p.add_argument('--lift-start',type=int,default=0,help='First finite extraction candidate index, for directed comparisons')
    p.add_argument('--waypoints-only',action='store_true',help='Investigate finite staging routes without repeating the direct connection')
    p.add_argument('--transit',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    snapshot=load_json(a.snapshot);source=load_json(a.contact_record);q=np.asarray(source['result']['points'][-1]['q'])
    req=source['request'];process=req['process_policy'];report=dict(scope='DIAGNOSTIC_ONLY_NOT_A_COMPLETE_PLAN',source=str(a.contact_record),attempts=[])
    started=perf_counter();task=None
    try:
        task=DiagnosticTask(snapshot,a.worker_command,task_seconds=a.task_seconds,stage_seconds=a.stage_seconds,
            rpc_seconds=240.,home_candidate=load_json(a.home_candidate),evaluation_profile='chain_integration_v1',
            search_trace=True,progress_path=a.output/'native_progress.jsonl')
        task.task_id='diagnostic-'+uuid.uuid4().hex
        task.face=process['target_face'];task.active_ids=process['active_ids'];task.candidate_id=process['selection_binding']['candidate_id']
        task.final_home=np.asarray(task.home_candidate['q_rad'])
        task.attachment=np.linalg.inv(task.robot.named_link_frames(q)['link_6'])@task.target_pose
        place_box=np.asarray(task.scene['native_task']['receiving']['box_pose_D'])
        place=place_box@np.linalg.inv(task.attachment)@np.asarray(task.identity['flange_from_task_tcp'])
        preplace=place.copy();preplace[2,3]+=task.scene['native_task']['preplace_m']
        report.update(task_id=task.task_id,contact_q=q.tolist(),flange_from_target=task.attachment.tolist(),
                      preplace_tcp=preplace.tolist(),home_q=task.final_home.tolist())
        lifts=clear_stack_extractions(task.scene,task.target_pose,task.target['size_m'],task.robot.fk(q),task.face)
        if not lifts:lifts=extraction_candidates(task.scene,task.target_pose,task.target['size_m'],task.robot.fk(q),task.face)
        for index,(pose,evidence) in enumerate(lifts[a.lift_start:a.lift_start+a.max_lifts],a.lift_start):
            task.parent='';task.stages=[];task.extraction_branch_id=f'diagnostic-extraction-{index}'
            row=dict(geometry=evidence,goal_pose=pose.tolist(),record_begin=len(task.records));report['attempts'].append(row)
            try:
                extracted=task.motion(q,'extraction',pose=pose);row['extracted_q']=extracted.tolist()
                probe=task.request(extracted,'transit');probe.update(check_phase='search');probe['clearance_policy']['search_payload_reserve_m']=.001
                row['transfer_start_check']=task.call(probe)
                if row['transfer_start_check']['status']!='SUCCESS':row['status']='SEARCH_START_REJECTED';continue
                if a.transit:
                    end=task.transfer(extracted,preplace,skip_direct=a.waypoints_only);row.update(transit_q=end.tolist(),preplace_fk=endpoint_fk(task.robot,end,preplace))
                row['status']='PASS_DIAGNOSTIC_PREFIX'
            except ValueError as error:row.update(status='REJECTED',error=str(error))
            finally:
                row['record_end']=len(task.records);row['stage_ids']=[s['stage_id'] for s in task.stages]
                save_json(a.output/'report.json',report)
        report['status']='DIAGNOSTIC_FINISHED'
    except Exception as error:report.update(status='INTERRUPTED',error=str(error))
    finally:
        if task:
            report['motions']=[dict(stage=r['request']['stage'],stage_id=r['request']['stage_id'],q_goal=r['request'].get('q_goal'),
              status=r['result']['status'],failure=r['result'].get('failure'),solver_message=r['result'].get('solver_message'),
              basic=r['result'].get('basic_path_check'),trace=r['result'].get('search_trace_ref')) for r in task.records]
            report['last_stages']=task.stages;task.close()
        report['wall_s']=perf_counter()-started;save_json(a.output/'report.json',report)
    print(json.dumps({k:report[k] for k in ('status','wall_s')}))


if __name__=='__main__':main()
