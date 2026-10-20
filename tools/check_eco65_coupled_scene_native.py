"""Three stopped-height native checks using the repository worker launcher."""
import argparse
import numpy as np
from unloading_sim.eco65.model import load_json, save_json, ROOT
from unloading_sim.eco65.native_task import NativeTask
from unloading_sim.eco65.native_scene import world_objects
from unloading_sim.eco65.coupled_scene import with_coupled_state, receiving_height_candidates
from unloading_sim.eco65.evaluation import CHAIN
from unloading_sim.moveit2_backend import digest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','home','output','worker-command'):
        parser.add_argument('--'+name,required=True)
    args=parser.parse_args()
    folder=(ROOT/args.output).resolve()
    folder.relative_to(ROOT/'outputs')
    folder.mkdir(parents=True,exist_ok=False)
    snapshot=load_json(args.snapshot)
    home=load_json(args.home)
    task=NativeTask(snapshot,args.worker_command,task_seconds=240,stage_seconds=8,rpc_seconds=60,
                    evaluation_profile=CHAIN,search_trace=False,
                    progress_path=folder/'native_progress.jsonl')
    try:
        scene=snapshot['scene']
        spec=scene['coupled_conveyor_spec']
        heights=[spec['low_height_m']]+[x['height_m'] for x in
            receiving_height_candidates(spec,scene['box']) if x['in_hardware_range']]
        q=np.array(home['q_rad'])
        rows=[]
        carton_ids={b['id'] for b in scene['boxes']}
        for height in heights:
            req=task.request(q,'pregrasp')
            world=world_objects(with_coupled_state(scene,height))
            world += [x for x in req['world'] if x['id'] in carton_ids]
            req.update(world=world,scene_fingerprint=digest(world))
            rows.append(dict(height_m=height,result=task.call(req)))
        report=dict(status='PASS' if all(r['result']['status']=='SUCCESS' for r in rows) else 'FAIL',
            rows=rows,ready=task.ready,physical_execution='NOT_RUN',execution_ready=False,
            launcher='REPOSITORY_WORKER_SH_TRACE_DISABLED',
            collision_policy=snapshot['native_model']['init']['collision_policy'])
        save_json(folder/'report.json',report)
        print([(r['height_m'],r['result']['status'],r['result'].get('process',{}).get('last_failure'),
                r['result'].get('clearance',{}).get('rejected_examples')) for r in rows])
        return int(report['status']!='PASS')
    finally:
        task.close()


if __name__=='__main__':
    raise SystemExit(main())
