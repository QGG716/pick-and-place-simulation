"""CPU-side optional TRANSIT adapter; an isolated synchronous GPU subprocess."""
from __future__ import annotations
import json
from pathlib import Path
import subprocess
from threading import Lock
from time import perf_counter
import numpy as np
from .stage_backend import fingerprint,run_stage
from .stage_export import export_request,box_record


class CuroboTransitAdapter:
    def __init__(self,scene,python,output_directory,*,cancelled=lambda:False):
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
        key=fingerprint([request.data[x] for x in ('robot_model_fingerprint','tool_fingerprint',
            'payload_fingerprint','collision_policy_fingerprint','scene_fingerprint','seed','resources')])
        if self.process is None or self.context_key!=key:
            reason='initialization' if self.context_key is None else 'model_scene_attachment_or_policy_changed'
            self.close();self.generation+=1
            folder=self.output/f'generation_{self.generation:03d}';folder.mkdir()
            path=folder/'bundle.json';path.write_text(json.dumps(bundle,allow_nan=False))
            self.log=(folder/'worker.log').open('w')
            self.process=subprocess.Popen([self.python,'-m','unloading_sim.curobo_v2_backend',
                str(path.resolve()),str((self.output/'sphere_cache').resolve())],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
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
        return result

    def __call__(self,connector,start,goal,obstacles,attachment,*,seed):
        entered=perf_counter()
        with self.lock:
            request,bundle=export_request(self.scene,connector,attachment,start,goal,
                 request_id=f'{attachment.rigid.name}-transit-{seed}',seed=seed)
            # The caller's exact scene must equal the frozen export. Refuse stale
            # obstacle lists instead of silently solving a visually similar scene.
            actual=[box_record(x,np.linalg.inv(connector.robot.base_transform)) for x in obstacles]
            expected=sorted(bundle['obstacles'],key=lambda x:x['name'])
            if fingerprint(sorted(actual,key=lambda x:x['name']))!=fingerprint(expected):
                return [],{'reason':'SCENE_STALE','stage':'transit'},{}
            revision=lambda:(self.scene.snapshot.get('actual_state_context',{}).get('revision',self.scene.snapshot['scene_fingerprint']),
                             self.scene.snapshot['scene_fingerprint'])
            state=lambda q:connector._state_failure(np.asarray(q,float),obstacles,attachment=attachment,stage='transit')
            authority=lambda t:connector._path_failure(t['q'],obstacles,attachment=attachment,stage='transit')
            result=run_stage(request,lambda r,a:self._candidate(r,bundle,a),authority,state,revision,self.cancelled)
            result['timings']['request_entry_to_result_s']=perf_counter()-entered
            self.last_result=result
            (self.output/'last_result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
            if not result['authority_accepted']:
                return [],{'reason':result['status'],'stage':'transit','first_failure':result['first_failure']},result
            # The enclosing connector rechecks the complete assembled task; the
            # existing replay exporter must revalidate its own retimed trajectory.
            # Native times stay in evidence and are not claimed as executed times.
            result['handoff_semantics']='geometry_to_existing_final_task_and_replay_validation'
            return [np.asarray(q,float) for q in result['trajectory']['q']],None,result


def install_curobo_transit(scene,connector,python,output_directory,**kwargs):
    adapter=CuroboTransitAdapter(scene,python,output_directory,**kwargs)
    connector.transit_backend=adapter
    return adapter
