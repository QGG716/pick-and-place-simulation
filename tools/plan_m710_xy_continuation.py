"""Continue a verified prefix with explicit collision-checked unloaded XY moves."""
import argparse,hashlib,json,os,sys,time,traceback
from copy import deepcopy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import yaml
from plan_m710_joint_search import prepare_layout,restore_state,offered,run_trial,archive_task,save,emit
from verify_m710_joint_search import verify_search
from unloading_sim.joint_search_layout import LayoutCandidate,BASELINE
from unloading_sim.moveit2_backend import digest
from unloading_sim.xy_base_continuation import XY_SCHEMA,xy_candidate,relocation_receipt
from unloading_sim.xy_base_native import check_xy_relocation

def candidate(e):
    return LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])

def locations(source,baseline):
    # Finite, declared business grid. Admission order tries the useful forward
    # positions early; both axes and both signs remain available and visible.
    # Declared fine lateral positions stay inside the same full-support domain.
    # A 200 mm negative-Y move can hit the wall at the retained empty tool pose;
    # 150 mm is separately checked, with no relaxation of the 5 mm pair gap.
    xy=[(source.x,source.y),(.8,-.35),(.8,source.y),(.6,-.35),(.4,-.35),(.4,source.y),(.6,source.y),(.2,source.y),(.4,0),(.4,-.4),(.6,.2),(.6,-.4)]
    xy += [(x,y) for x in (.4,.6,.2,.8,0.,-.2,-.4,-.6) for y in (0.,-.2,.2,-.4,.4)]
    result={}
    for point in xy:
        c=xy_candidate(source,point,baseline);result.setdefault(c.key,c.evidence())
    return list(result.values())

def anchored_state(node,context):return restore_state(node,context[2])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',action='store_true');p.add_argument('--source-revision-from',type=Path)
    p.add_argument('--seed',type=int,default=71070)
    p.add_argument('--initial-draws',type=int,default=64);p.add_argument('--trial-seconds',type=float,default=120.)
    p.add_argument('--prefix-seconds',type=float,default=45.);p.add_argument('--stage-seconds',type=float,default=60.)
    p.add_argument('--max-trials',type=int,default=600);p.add_argument('--worker-command',nargs='+',required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest=ROOT/'joint-search-manifest.json';source_manifest=json.loads(manifest.read_text())
    for name,expected in source_manifest['sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,('SOURCE_CHANGED',name)
    baseline=yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text())
    checkpoint=args.output/'checkpoint.json';began=time.monotonic();contexts={};db=None
    def persist():
        db['current_session_elapsed_s']=time.monotonic()-began
        temporary=checkpoint.with_suffix('.json.tmp');save(temporary,db);os.replace(temporary,checkpoint)
    def context_for(c):
        if c.key not in contexts:
            context,proof=prepare_layout(args,c);contexts[c.key]=context
            save(args.output/('layout-'+c.key+'.json'),proof)
            if context:db['layout_contexts'][c.key]=dict(candidate=c.evidence(),initial_q=context[3].q.tolist(),layout_fingerprint=context[0].policy.layout_validation.layout.layout_fingerprint)
            else:db['layout_contexts'][c.key]=dict(candidate=c.evidence(),failure=proof)
        return contexts[c.key]
    try:
        if args.resume:
            db=json.loads(checkpoint.read_text());assert db['schema']==XY_SCHEMA
            current_manifest=hashlib.sha256(manifest.read_bytes()).hexdigest()
            if db['source_manifest_sha256']!=current_manifest:
                assert args.source_revision_from and hashlib.sha256(args.source_revision_from.read_bytes()).hexdigest()==db['source_manifest_sha256']
                prior=json.loads(args.source_revision_from.read_text())['sha256'];current=source_manifest['sha256']
                changed={n for n in set(prior)|set(current) if prior.get(n)!=current.get(n)}
                assert changed<={'tools/plan_m710_xy_continuation.py','tools/verify_m710_xy_continuation.py',
                    'tests/test_xy_search_order.py','tests/test_xy_saved_evidence.py',
                    'src/unloading_sim/joint_search_recipes.py','tests/test_joint_search_process.py'},changed
                db.setdefault('source_revisions',[]).append(dict(previous=db['source_manifest_sha256'],current=current_manifest,
                    changed_files=sorted(changed),accepted_root_sha256=digest(db['accepted_nodes'][-1]['root']),trials=len(db['trials']),epoch=time.time()))
                db['source_manifest_sha256']=current_manifest;db['source_snapshot']=str(ROOT)
            assert db['prefix']['checkpoint_sha256']==hashlib.sha256((args.prefix/'checkpoint.json').read_bytes()).hexdigest()
            if db.get('pending'):
                item=db['pending'];folder=args.output/item['folder']
                item.update(complete=False,failure=dict(reason='INTERRUPTED_WITHOUT_ACCEPTANCE'),wall_s=item.get('interrupted_wall_s'))
                if folder.exists():
                    if (folder/'result.json').exists():
                        result=json.loads((folder/'result.json').read_text())
                        item.update(failure=result.get('failure'),wall_s=result.get('wall_s'),contact_complete=result.get('contact_complete'),
                            prior_result_complete=result.get('complete'),acceptance='NOT_COMMITTED_BEFORE_INTERRUPTION')
                    if not folder.name.startswith('task-'):
                        renamed=args.output/('task-recovered-%04d'%item['id']);assert not renamed.exists()
                        folder.rename(renamed);folder=renamed;item['folder']=folder.name
                    item['evidence']=archive_task(folder,args.output)
                db['trials'].append(item);db['pending']=None
            db['previous_session_elapsed_s']+=db.get('current_session_elapsed_s',0.)
        else:
            assert not checkpoint.exists()
            old=json.loads((args.prefix/'checkpoint.json').read_text());record=old['layouts'][old['best']]
            proof=verify_search(args.prefix,old['best']);assert proof['completed']==31 and proof['remaining']==9
            save(args.output/'accepted-prefix-checks.json',proof)
            anchor=deepcopy(record['nodes'][record['best_node']]);start=candidate(record['candidate'])
            node=dict(id=digest(dict(root=anchor['root'],layout=start.evidence())),root=anchor['root'],ledger=anchor['ledger'],layout=start.evidence(),parent=None,incoming=None)
            db=dict(schema=XY_SCHEMA,status='RUNNING',options={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                source_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),source_snapshot=str(ROOT),
                source_baseline_commit=source_manifest['xy_baseline_commit'],started_epoch=time.time(),previous_session_elapsed_s=0.,
                prefix=dict(path=str(args.prefix),checkpoint_sha256=hashlib.sha256((args.prefix/'checkpoint.json').read_bytes()).hexdigest(),
                    anchor_root_sha256=digest(anchor['root']),accepted_count=31,prior_search_cost_s=old['elapsed_search_s']),
                accepted_nodes=[node],trials=[],relocations=[],pending=None,layout_contexts={},
                search={},physical_execution='NOT_RUN',base_actuation='KINEMATIC_ASSUMPTION',continuous_swept_proof=False)
            persist()
        if db.get('error'):db.setdefault('historical_errors',[]).append(db.pop('error'))
        db['status']='RUNNING';persist()
        while len(db['accepted_nodes'][-1]['root']['completed'])<40 and len(db['trials'])<args.max_trials:
            if (args.output/'stop-requested.json').exists():db['status']='STOPPED_AT_CANDIDATE_BOUNDARY';persist();return
            node=db['accepted_nodes'][-1];source=candidate(node['layout']);source_context=context_for(source)
            assert source_context,'ACCEPTED_LAYOUT_CONTEXT_UNAVAILABLE'
            choices=locations(source,baseline);search=db['search'].setdefault(node['id'],dict(round=0,visits={},moves={},closed=[]))
            search['location_candidates']=choices
            active=choices[:min(len(choices),4+2*search['round'])];accepted=False;attempts_before=len(db['trials'])
            for e in active:
                destination=candidate(e);key=destination.key
                if key in search['closed']:continue
                emit(event='XY_LAYOUT_VISIT',completed=len(node['root']['completed']),destination=e)
                move=None;work=anchored_state(node,source_context)
                if key!=source.key:
                    if key not in search['moves']:
                        results=[]
                        for order in ((0,1),(1,0)):
                            if order==(1,0) and (source.x==destination.x or source.y==destination.y):break
                            folder=args.output/('base-%04d'%len(db['relocations']))
                            row=check_xy_relocation(args,source_context,work,source,destination,baseline,folder,order=order)
                            index=len(db['relocations']);db['relocations'].append(dict(folder=folder.name,parent=node['id'],to_layout=e,
                                passed=row['passed'],failure=row['failure'],wall_s=row['wall_s'],motion_sha256=hashlib.sha256((folder/'xy-motion.json').read_bytes()).hexdigest()))
                            results.append(index);persist()
                            if row['passed']:break
                        search['moves'][key]=results;persist()
                    good=[i for i in search['moves'][key] if db['relocations'][i]['passed']]
                    if not good:search['closed'].append(key);persist();continue
                    move=good[-1];receipt=relocation_receipt(node['root'],source,destination,db['relocations'][move]['motion_sha256'])
                    work.cycles.append(dict(final_state_receipt=receipt))
                context=context_for(destination)
                if context is None:search['closed'].append(key);persist();continue
                original,built,frozen,initial=context
                built.joint_search_short_drop_departure_first=True
                _,visits,screens=offered(original,built,work,frozen)
                search.setdefault('candidate_counts',{})[key]=len(visits)
                latest={t['candidate_key']:t for t in db['trials'] if t['parent']==node['id'] and candidate(t['layout']).key==key}
                retry={k for k,t in latest.items() if (t.get('failure') or {}).get('detail')=='SEARCH_RELEASE_HEIGHT_CHANGED'
                    and t.get('source_manifest_sha256')!=db['source_manifest_sha256']}
                attempted=set(latest)-retry
                remaining=[item for item in visits if item[1]['candidate_key'] not in attempted]
                remaining.sort(key=lambda item:item[1]['candidate_key'] not in retry)
                if not remaining:search['closed'].append(key);persist();continue
                cursor=0;prior_visits=search['visits'].get(key,0)
                for _ in range(8):
                    if (args.output/'stop-requested.json').exists():db['status']='STOPPED_AT_CANDIDATE_BOUNDARY';persist();return
                    if cursor>=len(remaining) or len(db['trials'])>=args.max_trials:break
                    target,option=remaining[cursor];cursor+=1;search['visits'][key]=prior_visits+cursor
                    index=len(db['trials']);folder=args.output/('task-%04d'%index)
                    trial=dict(id=index,parent=node['id'],layout=e,move=move,folder=folder.name,target_id=target.target_id,
                        candidate_key=option['candidate_key'],seed=args.seed+index*1009,started_epoch=time.time(),
                        source_manifest_sha256=db['source_manifest_sha256'],retry_of=latest[option['candidate_key']]['id'] if option['candidate_key'] in retry else None)
                    db['pending']=trial;persist();emit(event='XY_TRIAL_BEGIN',**trial)
                    plan,result=run_trial(args,original,built,work,target,option,folder,args.trial_seconds,trial['seed'])
                    trial.update(complete=result['complete'],failure=result['failure'],wall_s=result['wall_s'],contact_complete=result['contact_complete'])
                    trial['evidence']=archive_task(folder,args.output);db['trials'].append(trial);db['pending']=None
                    if plan:
                        work.commit(plan);root=work.root()
                        new=dict(id=digest(dict(root=root,layout=e)),root=root,ledger=deepcopy(work.ledger),layout=e,parent=node['id'],incoming=index)
                        db['accepted_nodes'].append(new);accepted=True
                        emit(event='XY_ACCEPTED',target=target.target_id,completed=len(root['completed']),remaining=len(root['world_cartons']),layout=e)
                    persist()
                    if result['implementation_failure']:raise RuntimeError('IMPLEMENTATION_FAILURE_RETAINED:'+str(result['failure']))
                    if accepted:break
                if accepted:break
            if accepted:continue
            search['round']+=1;persist()
            if len(search['closed'])==len(choices):db['status']='FINITE_CANDIDATES_EXHAUSTED';break
        if len(db['accepted_nodes'][-1]['root']['completed'])==40:db['status']='COMPLETE_PENDING_INDEPENDENT_VERIFICATION'
        elif db['status']=='RUNNING':db['status']='FINITE_TRIAL_RESOURCE_LIMIT'
    except BaseException as exc:
        if db is not None:db['status']='INTERRUPTED' if isinstance(exc,KeyboardInterrupt) else 'IMPLEMENTATION_FAILURE';db['error']=dict(type=type(exc).__name__,detail=str(exc))
        (args.output/'exception.txt').write_text(traceback.format_exc());raise
    finally:
        if db is not None:persist()
    emit(event='XY_SEARCH_CLOSED',status=db['status'],completed=len(db['accepted_nodes'][-1]['root']['completed']),trials=len(db['trials']))

if __name__=='__main__':main()
