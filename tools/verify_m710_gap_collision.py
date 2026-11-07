"""Read-only CPU/native collision counterexamples; never a planning completion."""
import argparse,json,sys,tarfile,hashlib
from copy import deepcopy
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from plan_m710_mount_comparison import make_context
from unloading_sim.coupled_batch import BatchState,batch_scene
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.moveit2_backend import ResidentMoveItClient

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--worker-command',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--draws',type=int,default=256);a=p.parse_args()
    db=json.loads((a.run/'checkpoint.json').read_text());trial=db['trials'][0];record=db['layouts'][trial['layout']]
    archive=a.run/trial['evidence']['archive'];assert hashlib.sha256(archive.read_bytes()).hexdigest()==trial['evidence']['sha256']
    with tarfile.open(archive) as tar:
        name=archive.name.removesuffix('.tar.gz')+'/search-trace.json';raw=tar.extractfile(name).read()
        assert hashlib.sha256(raw).hexdigest()==trial['evidence']['file_sha256']['search-trace.json'];trace=json.loads(raw)
    layout_path=ROOT/'configs/workcells'/record['layout_path']
    original,built,_=make_context('gap-counterexample',layout_path=layout_path,initial_q=record['initial']['q_rad'])
    layout=original.policy.layout_validation.layout;state=BatchState(FrozenCartonPopulation.capture(original.cartons),tuple(original.cartons),np.asarray(record['initial']['q_rad']),.6)
    scene=batch_scene(original,state,built.robot,layout);template=trace['attempts'][0]['request']
    assert template['world']==[__import__('unloading_sim.moveit2_backend',fromlist=['box_message']).box_message(b) for b in scene.all_obstacles]
    rng=np.random.default_rng(71108);proposals=[np.asarray(record['initial']['q_rad']),np.zeros(6),
        *rng.uniform(built.robot.joint_limits[:,0]+.05,built.robot.joint_limits[:,1]-.05,size=(a.draws,6))]
    output=dict(schema='gap_cpu_native_counterexamples_v1',seed=71108,maximum_random_draws=a.draws,checks=[],
        source_identity=db['identity'],helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='SAME_FULL_FRESH40_WORLD_STATE_CHECKS_ONLY_NO_MOTION_NO_COMPLETION',physical_execution='NOT_RUN')
    client=ResidentMoveItClient(a.worker_command,log_path=str(a.output.with_suffix('.worker.log')))
    try:
        init=client.request(trace['model']);assert init['status']=='READY'
        found=set()
        for index,q in enumerate(proposals):
            cpu=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
            body=json.dumps(cpu)
            if index and not any(x in body for x in ('chassis','mount_plinth')):continue
            req=deepcopy(template);req.update(op='check_start',q_start=q.tolist(),q_goal=q.tolist(),parent_stage_id='',parent_state_receipt=None,ownership_transition=None)
            native=client.request(req)
            check=dict(index=index,q_rad=q.tolist(),cpu=cpu,native=native);output['checks'].append(check)
            if index==0:assert cpu is None and native['status']=='SUCCESS'
            else:
                pair=native.get('failure',{}).get('pair',[])
                if native['status']=='INVALID_START_CLEARANCE' and any(x in str(pair) for x in ('chassis','mount_plinth')):
                    found.add('tool' if 'tool_' in str(pair) else 'robot')
            if found=={'robot','tool'}:break
        output.update(counterexample_kinds=sorted(found),status='PASS_BOTH_CPU_NATIVE_COUNTEREXAMPLES' if found=={'robot','tool'} else 'PARTIAL_FINITE_COUNTEREXAMPLE_SEARCH')
    finally:client.close();a.output.write_text(json.dumps(output,indent=2))
    print(json.dumps(dict(status=output['status'],checks=len(output['checks']),kinds=output['counterexample_kinds'])))
if __name__=='__main__':main()
