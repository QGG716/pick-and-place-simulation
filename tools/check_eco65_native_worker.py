"""Equipment-only resident worker diagnostic; never produces a planning snapshot."""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace
import numpy as np
from unloading_sim.eco65.model import ROOT, load_json, save_json, robot_for, transform
from unloading_sim.eco65.native_model import asset_inventory
from unloading_sim.eco65.native_scene import combined_scene, verify_scene
from unloading_sim.eco65.home_candidate import verify_candidate
from unloading_sim.eco65.native_task import NativeTask
from unloading_sim.moveit2_backend import ResidentMoveItClient, digest


def diagnose(model_path, home_path, command, output):
    out=Path(output).resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True,exist_ok=False)
    started=perf_counter(); model=load_json(model_path); init=model['init']; scene=combined_scene()
    verify_scene(scene,require_environment=False)
    audit=asset_inventory()
    if audit['status']!='PASS' or audit['asset_sha256']!=model['asset_audit']['asset_sha256']:
        raise ValueError('DIAGNOSTIC_ASSET_IDENTITY_MISMATCH')
    if init['identity']['scene_fingerprint']!=scene['scene_fingerprint'] or digest(init['parameters'])!=init['identity']['model_tool_fingerprint']:
        raise ValueError('DIAGNOSTIC_MODEL_IDENTITY_MISMATCH')
    candidate=verify_candidate(load_json(home_path),model['tool'],scene)
    calls=[]; result=dict(scope='EQUIPMENT_ONLY_DIAGNOSTIC',environment_status=scene['environment_status'],
        planning='NOT_RUN',dynamics='NOT_EVALUATED',native_calls=calls)
    native=ResidentMoveItClient(command,log_path=out/'worker.log')
    def call(request):
        if request['op'] not in ('init','fk','check_state_offline'): raise ValueError('DIAGNOSTIC_CANNOT_PLAN')
        record=dict(request=deepcopy(request)); calls.append(record); start=perf_counter()
        try: record['result']=native.request(request,timeout=90.); return record['result']
        except Exception as error: record['error']=str(error); raise
        finally: record['wall_s']=perf_counter()-start
    try:
        ready=call(init); result['ready']=ready
        if ready.get('status')!='READY' or ready.get('identity')!=init['identity']: raise ValueError('NATIVE_INIT_REJECTED')
        context=SimpleNamespace(scene=scene,snapshot={'tool':model['tool']},init=init,identity=init['identity'],
            robot=robot_for(model['tool'],scene),target=scene['box'],target_pose=transform(scene['box']['pose']),
            attachment=None,face='top',task_id='eco65-equipment-diagnostic',parent='',counter=0,call=call)
        result['fk']=NativeTask.check_fk(context,np.asarray(candidate['q_rad']))
        result['states']=[]
        for name,q in [('candidate_HOME',candidate['q_rad']),('historical_seed',scene['source_equipment_config']['task']['initial_seed_q_rad']),
                       ('nonzero_wrist',[.2,-.3,.4,.5,-.6,.7])]:
            request=NativeTask.request(context,q,'pregrasp')
            request['record_contact_distances']=True
            reply=call(request); result['states'].append(dict(name=name,q_rad=q,result=reply))
        result['status']='DIAGNOSTIC_COMPLETED'
    except Exception as error:
        result.update(status='FAILED',error=str(error)); raise
    finally:
        native.close(); result['wall_s']=perf_counter()-started; save_json(out/'report.json',result)
    print({k:result[k] for k in ('status','scope','wall_s')})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True); p.add_argument('--home-candidate',type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();diagnose(a.model,a.home_candidate,a.worker_command,a.output)
