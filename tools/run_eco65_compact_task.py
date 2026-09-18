"""Single-carton task from an explicit frozen layout run; private artifacts remain local."""
import argparse,copy,hashlib,json,time,subprocess,importlib.util
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import ROOT,load_json,save_json,canonical_fingerprint
from unloading_sim.eco65.compact_task import *
from unloading_sim.eco65.unloading_layout import CONFIG

def load_run(config,run_id):
    if not run_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in run_id):raise ValueError('Invalid run-id')
    out=ROOT/'outputs/eco65_desktop_layout'/run_id;snapshot=load_json(out/'scene_snapshot.json');claimed=snapshot.pop('fingerprint')
    if claimed!=canonical_fingerprint(snapshot):raise ValueError('Snapshot fingerprint')
    snapshot['fingerprint']=claimed
    if hashlib.sha256(config.read_bytes()).hexdigest()!=snapshot['config_sha256']:raise ValueError('Config differs from frozen run')
    for name,sha in snapshot['asset_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha:raise ValueError('Changed asset '+name)
    home=load_json(out/'reports/poses.json')['home']['q']
    return out,snapshot,home

def render_replay(snapshot,home,artifact,replay,outfeed,out):
    import imageio.v2 as imageio
    from unloading_sim.eco65.layout_figures import _label
    w=prepare_world(snapshot,artifact['selection'],home)
    frames=replay['frames']+outfeed.get('frames',[]);duration=frames[-1]['t'];times=np.array([f['t'] for f in frames]);folder=out/'replay';folder.mkdir(exist_ok=True)
    try:
        with imageio.get_writer(folder/'single_box.mp4',fps=5,codec='libx264',quality=7,macro_block_size=1) as video:
            for t in np.arange(0,duration+.2,.2):
                i=min(len(frames)-1,max(0,np.searchsorted(times,t+1e-9,side='right')-1));f=frames[i]
                # Recorded actual simulator states, same quintic executor. Frame rate is independent of checking.
                w.attached=False;w.released_pose=np.array(f['box_pose']);w.active_pose=np.array(f['box_pose'])
                rgb=w.render(f['q'],f['phase'],640,360,azimuth=320,elevation=-27,distance=2.25,lookat=(-.28,0,.30))
                im=_label(rgb,'ECO65-B | GEOMETRIC_REPLAY_ONLY',f"t={t:.1f}s | {f['phase']} | dynamics NOT_EVALUATED")
                video.append_data(np.array(im))
                if i==len(frames)-1:im.save(folder/'final.png')
        save_json(folder/'video_manifest.json',dict(file='single_box.mp4',width=640,height=360,fps=5,playback_speed=1,simulation_duration_s=duration,rendered_duration_s=(int(np.ceil(duration*5))+1)/5,trajectory_fingerprint=artifact['trajectory_fingerprint'],scene_fingerprint=snapshot['fingerprint'],mode='GEOMETRIC_REPLAY_ONLY'))
    finally:w.close()
    print('REPLAY',folder/'single_box.mp4',replay['status'],outfeed['status'],flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('stage',choices=['plan','validate','execute','replay']);parser.add_argument('--config',type=Path,default=CONFIG);parser.add_argument('--run-id',required=True);parser.add_argument('--candidate-index',type=int);parser.add_argument('--cartesian-guide',action='store_true');args=parser.parse_args()
    out,snapshot,home=load_run(args.config.resolve(),args.run_id);folder=out/'task';folder.mkdir(exist_ok=True)
    if args.stage=='plan':
        if (folder/'plan.json').exists():raise FileExistsError('Plan already exists; use a new run-id')
        if (folder/'candidate_manifest.json').exists():raise FileExistsError('Search evidence exists; use a new run-id')
        failures=[];started=time.perf_counter();settings=snapshot['scene']['source_config']['task'];deadline=started+settings['planning_budget_s']
        choices=candidates(snapshot['scene'])
        if args.candidate_index is not None:choices=[choices[args.candidate_index]]
        if args.cartesian_guide:
            for choice in choices:choice['guide']=True
        save_json(folder/'candidate_manifest.json',dict(candidates=choices,seed=6518,budget=settings))
        for i,selection in enumerate(choices):
            if time.perf_counter()>deadline or (out/'CANCEL').exists():break
            w=prepare_world(snapshot,selection,home);print('CANDIDATE',i,selection,flush=True)
            try:
                if not w.valid(home,'initial'):raise CandidateFailure('INITIAL_CLEARANCE_FAILED','initial',w.last_failure)
                artifact=plan_candidate(w,home,selection,6518+i,min(deadline,time.perf_counter()+settings['candidate_budget_s']),lambda:(out/'CANCEL').exists())
                save_json(folder/f'candidate_{i:02d}_plan.json',artifact)
            except CandidateFailure as exc:
                record=dict(index=i,selection=selection,status=exc.code,stage=exc.phase,evidence=exc.evidence,elapsed_s=time.perf_counter()-started)
                failures.append(record);save_json(folder/'failures.json',failures);print('FAILED',i,exc.code,exc.phase,flush=True);continue
            finally:w.close()
            validation_world=prepare_world(snapshot,selection,home)
            try:
                env=envelope_for(validation_world,home,artifact,args.run_id);validator=ECO65PlanValidator(validation_world,folder/f'candidate_{i:02d}_validation.json')
                result=validator.validate(env,env.planned_snapshot,env.expected_start_boundary)
                if not result.valid:
                    failures.append(dict(index=i,selection=selection,status='FINAL_VALIDATION_FAILED',evidence=validator.report));save_json(folder/'failures.json',failures);continue
                save_json(folder/'plan.json',artifact);save_json(folder/'validation.json',validator.report)
                save_json(folder/'planning_summary.json',dict(status='VALIDATED_PLAN',selected=selection,candidates_attempted=i+1,elapsed_s=time.perf_counter()-started,scene_fingerprint=snapshot['fingerprint']))
                print('FIRST COMPLETE VALIDATED PATH',selection,flush=True);return
            finally:validation_world.close()
        save_json(folder/'planning_summary.json',dict(status='CANCELLED' if (out/'CANCEL').exists() else ('BUDGET_EXHAUSTED' if time.perf_counter()>deadline else 'CANDIDATES_EXHAUSTED'),candidates_attempted=len(failures),candidate_count=len(choices),elapsed_s=time.perf_counter()-started,geometric_tasks_completed=0,outfed_assumed=0,dynamics='NOT_EVALUATED'))
        print('NO COMPLETE VALIDATED PATH',flush=True);return
    artifact=load_json(folder/'plan.json');selection=artifact['selection']
    if args.stage=='replay':render_replay(snapshot,home,artifact,load_json(folder/'execution.json'),load_json(folder/'outfeed.json'),out);return
    w=prepare_world(snapshot,selection,home)
    try:
        env=envelope_for(w,home,artifact,args.run_id)
        if args.stage=='validate':
            validator=ECO65PlanValidator(w,folder/'validation.json');v=validator.validate(env,env.planned_snapshot,env.expected_start_boundary);print('VALIDATION',v.status.value);return
        validation=load_json(folder/'validation.json')
        if validation['status']!='VALID' or validation['trajectory_fingerprint']!=artifact['trajectory_fingerprint'] or validation['world_fingerprint']!=env.planned_snapshot.fingerprint:raise ValueError('Validation evidence mismatch')
        v=P.PlanValidationResult(P.ValidationStatus.VALID,env.planned_snapshot,env.expected_start_boundary,'independent stored full geometry validation','eco65-complete-geometry','2')
        env=env.revalidated(v,timestamp_seconds=time.monotonic(),generation=0)
        executor=GeometricExecutionBackend(w);executor.start(env)
        try:replay=executor.run()
        except Exception as exc:
            save_json(folder/'execution_failure.json',dict(error=str(exc),events=executor.events,frames=executor.frames,state=executor.state.value));raise
        replay['retained_cartons_actual']={b['id']:w.data.xpos[w.model.body(b['id']).id].tolist() for b in w.scene['boxes'] if b['id']!=w.box_id}
        save_json(folder/'execution.json',replay);outfeed=ideal_outfeed(w,replay,selection);save_json(folder/'outfeed.json',outfeed)
        print('EXECUTION',replay['status'],'OUTFEED',outfeed['status'],flush=True)
    finally:w.close()

if __name__=='__main__':main()
