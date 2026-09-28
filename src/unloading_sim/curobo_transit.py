"""CPU-side optional TRANSIT adapter; an isolated synchronous GPU subprocess."""
from __future__ import annotations
import json
import os
from pathlib import Path
import subprocess
from threading import Lock
from time import perf_counter
import numpy as np
from .stage_backend import fingerprint,run_stage
from .stage_export import export_request,box_record,worker_context_key


def connected_delivery_path(start, native_q, goal):
    """Retain every native point; exact requested endpoints are additional edges."""
    path=[np.asarray(q,float).copy() for q in native_q]
    if not path:
        raise ValueError('empty native candidate')
    if not np.array_equal(start,path[0]): path.insert(0,np.asarray(start,float).copy())
    if not np.array_equal(goal,path[-1]): path.append(np.asarray(goal,float).copy())
    return path


class CuroboTransitAdapter:
    def __init__(self,scene,python,output_directory,*,cancelled=lambda:False,geometry_fit_seed=716):
        self.geometry_fit_seed=geometry_fit_seed
        self.request_count=0
        self.scene=scene
        self.python=str(python)
        self.output=Path(output_directory);self.output.mkdir(parents=True,exist_ok=True)
        self.cancelled=cancelled
        self.lock=Lock()
        self.process=None;self.context_key=None;self.generation=0
        self.last_result=None

    def close(self):
        # Explicit drain-and-reap. No claim of immediate CUDA interruption.
        if self.process is not None:
            self.process.stdin.close();self.process.wait();self.log.close()
            self.process=None

    def _candidate(self,request,bundle,attempt):
        key=worker_context_key(bundle)
        if self.process is None or self.context_key!=key:
            reason='initialization' if self.context_key is None else 'model_scene_attachment_or_policy_changed'
            self.close();self.generation+=1
            folder=self.output/f'generation_{self.generation:03d}';folder.mkdir()
            path=folder/'bundle.json';path.write_text(json.dumps(bundle,allow_nan=False))
            self.log=(folder/'worker.log').open('w')
            worker_env=os.environ.copy()
            source_root=str(Path(__file__).resolve().parents[1])
            worker_env['PYTHONPATH']=source_root+os.pathsep+worker_env.get('PYTHONPATH','')
            self.process=subprocess.Popen([self.python,'-m','unloading_sim.curobo_v2_backend',
                str(path.resolve()),str((self.output/'sphere_cache').resolve())],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,env=worker_env)
            self.context_key=key
            self.rebuild_reason=reason
        t=perf_counter()
        self.process.stdin.write(json.dumps(dict(op='solve',attempt=attempt,request=request.to_dict()))+'\n')
        self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:
            result=dict(status='DEPENDENCY_UNAVAILABLE',trajectory=None,error='worker exited; inspect worker.log',backend={'name':'curobo_v2'})
        else:
            result=json.loads(line)
        result['ipc_inclusive_s']=perf_counter()-t
        result.update(worker_generation=self.generation,rebuild_reason=self.rebuild_reason)
        (self.request_directory/f'native_attempt_{attempt:03d}.json').write_text(
            json.dumps(result,indent=2,allow_nan=False))
        return result

    def __call__(self,connector,start,goal,obstacles,attachment,*,seed):
        entered=perf_counter()
        with self.lock:
            request,bundle=export_request(self.scene,connector,attachment,start,goal,
                 request_id=f'{attachment.rigid.name}-transit-{seed}',seed=seed,geometry_fit_seed=self.geometry_fit_seed)
            self.request_count+=1
            self.request_directory=self.output/f'request_{self.request_count:04d}'
            self.request_directory.mkdir()
            for name,value in (('request',request.to_dict()),('bundle',bundle)):
                (self.request_directory/(name+'.json')).write_text(json.dumps(value,indent=2,allow_nan=False))
            # The caller's exact scene must equal the frozen export. Refuse stale
            # obstacle lists instead of silently solving a visually similar scene.
            actual=[box_record(x,np.linalg.inv(connector.robot.base_transform)) for x in obstacles]
            expected=sorted(bundle['obstacles'],key=lambda x:x['name'])
            if fingerprint(sorted(actual,key=lambda x:x['name']))!=fingerprint(expected):
                return [],{'reason':'SCENE_STALE','stage':'transit'},{}
            revision=lambda:(self.scene.snapshot.get('actual_state_context',{}).get('revision',self.scene.snapshot['scene_fingerprint']),
                             self.scene.snapshot['scene_fingerprint'])
            state=lambda q:connector._state_failure(np.asarray(q,float),obstacles,attachment=attachment,stage='transit')
            authority=lambda t:connector._path_failure(connected_delivery_path(start,t['q'],goal),
                obstacles,attachment=attachment,stage='transit')
            result=run_stage(request,lambda r,a:self._candidate(r,bundle,a),authority,state,revision,self.cancelled)
            result['timings']['request_entry_to_result_s']=perf_counter()-entered
            result['handoff_semantics']='geometry_to_existing_final_task_and_replay_validation'
            if result['authority_accepted']:
                delivery=connected_delivery_path(start,result['trajectory']['q'],goal)
                result['validated_delivery_path']=dict(q=[q.tolist() for q in delivery],
                    interpolation='linear_joint_samples', geometry_sha256=fingerprint([q.tolist() for q in delivery]),
                    native_points_preserved=True, added_endpoint_edges=len(delivery)-len(result['trajectory']['q']),
                    timed_execution_claimed=False)
            self.last_result=result
            (self.request_directory/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
            (self.output/'last_result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
            if not result['authority_accepted']:
                return [],{'reason':result['status'],'stage':'transit','first_failure':result['first_failure']},result
            # The enclosing connector checks stage joins and the task contract;
            # replay export retains its separate retiming and execution gates.
            # Native times stay in evidence and are not claimed as executed times.
            result['handoff_semantics']='geometry_to_existing_final_task_and_replay_validation'
            return delivery,None,result


def install_curobo_transit(scene,connector,python,output_directory,**kwargs):
    adapter=CuroboTransitAdapter(scene,python,output_directory,**kwargs)
    connector.transit_backend=adapter
    return adapter
