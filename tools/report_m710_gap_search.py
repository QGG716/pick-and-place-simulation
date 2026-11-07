"""Summarize a frozen gap-search checkpoint without exporting raw evidence."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import tarfile


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def native_failure_summary(run,row):
    """Keep the failed native attempt inspectable without copying its world."""
    archive=run/row['evidence']['archive']
    assert sha256(archive)==row['evidence']['sha256']
    with tarfile.open(archive) as t:
        name=archive.name.removesuffix('.tar.gz')+'/requests.json'
        raw=t.extractfile(name).read()
    assert hashlib.sha256(raw).hexdigest()==row['evidence']['file_sha256']['requests.json']
    calls=json.loads(raw);counts=Counter()
    for call in calls:
        response=call.get('response') or {}
        counts[(call.get('op'),call.get('stage'),call.get('status'),response.get('solver_message'))]+=1
    return dict(trial_number=row['trial_number'],target_id=row['target_id'],
        contact_complete=row.get('contact_complete'),archive=row['evidence']['archive'],
        archive_sha256=row['evidence']['sha256'],request_count=len(calls),
        calls=[dict(op=op,stage=stage,status=status,solver_message=message,count=count)
            for (op,stage,status,message),count in counts.items()],
        scope='ONE_RETAINED_FAILED_ATTEMPT_NOT_A_GLOBAL_REACHABILITY_PROOF')


def receipt_summary(receipt):
    canonical=json.dumps(receipt,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return dict(receipt_id=receipt['receipt_id'],task_id=receipt['task_id'],stage_id=receipt['stage_id'],
        q_rad=receipt['q_rad'],canonical_json_sha256=hashlib.sha256(canonical).hexdigest(),
        full_context_location='plan.json/final_state_receipt in the named verified server archive')


def summarize(run):
    path=run/'checkpoint.json';raw=path.read_bytes();db=json.loads(raw)
    assert db['contract']=='fixed40_gap_search_20261107_v1'
    layouts=[]
    for key,r in db['layouts'].items():
        trials=[t for t in db['trials'] if t['layout']==key]
        nodes=r.get('nodes',{})
        layouts.append(dict(key=key,gap_m=r['gap_m'],status=r['status'],best_continuous_count=r.get('best_count',0),
            trials=len(trials),trial_wall_s=sum(t['wall_s'] for t in trials),admission_wall_s=r['admission_wall_s'],
            failures=dict(Counter(t['category'] for t in trials)),
            nodes=[dict(id=n['id'],completed=len(n['root']['completed']),closed=n['closed'],
                candidate_count=n.get('candidate_count'),cursor=n['cursor'],
                unvisited_candidates=max(0,n.get('candidate_count',0)-n['cursor']) if 'candidate_count' in n else None,
                unvisited_round_visits=max(0,n.get('candidate_count',0)*db['options']['max_rounds']-n['cursor']) if 'candidate_count' in n else None)
                for n in nodes.values()],
            floor_support=[dict(height_m=c['height_m'],status=c['floor_support']['status'],
                unknown_parts=[f['id'] for f in c['floor_support']['footprints'] if f['status']!='VERIFIED_ON_MODELED_FLOOR']) for c in r['checks']]))
    best=db['layouts'].get(db['best']);chain=[];cycles=[]
    if best:
        node=best['nodes'][best['best_node']]
        remaining=[b['id'] for b in node['root']['world_cartons']]
        while node['incoming'] is not None:
            chain.append(db['trials'][node['incoming']]);node=best['nodes'][node['parent']]
        chain.reverse()
        for row in chain:
            archive=run/row['evidence']['archive']
            assert sha256(archive)==row['evidence']['sha256']
            with tarfile.open(archive) as t:
                plan=json.load(t.extractfile(archive.name.removesuffix('.tar.gz')+'/plan.json'))
            motion=sum(s['points'][-1]['t'] for s in plan['stages'] if s['generation_source']=='NATIVE_GENERATED')
            drop=plan['nominal_release'];outfeed=plan['wait_outfeed']['path']
            cycles.append(dict(target_id=row['target_id'],trial_number=row['trial_number'],
                face=plan['option']['face']['geometric_face_id'],mode=plan['option']['template'],
                face_role=plan['target']['face_role'],
                receive_height_m=plan['height_m'],release_bottom_m=plan['height_m']+drop['drop_m'],
                actual_nominal_drop_m=drop['drop_m'],planning_wall_s=row['wall_s'],
                robot_trajectory_s=motion,nominal_fall_s=drop['samples'][-1]['t'],
                ideal_outfeed_s=outfeed['samples'][-1]['t'],hardware_complete_cycle_s=None,
                native_stages=len(plan['stages']),target_still_attached=plan['target_still_attached'],
                archive=row['evidence']['archive'],archive_sha256=row['evidence']['sha256'],
                finite_outfeed_support=outfeed.get('finite_support'),
                final_state_receipt_summary=receipt_summary(plan['final_state_receipt'])))
    else:remaining=[]
    # Inspect the most recent failure for each carton still offered at the
    # deepest accepted node. Older worlds remain in trials and raw archives.
    blocking=[]
    if best:
        node=best['nodes'][best['best_node']]
        for name in node.get('screens',{}):
            matches=[r for r in db['trials'] if r['layout']==db['best'] and r['node']==node['id']
                and r['target_id']==name and not r['complete']]
            if matches:blocking.append(native_failure_summary(run,matches[-1]))
    trials=[]
    for row in db['trials']:
        f=row.get('failure') or {};last=f.get('last_failure') or {}
        trials.append(dict(number=row['trial_number'],gap_m=db['layouts'][row['layout']]['gap_m'],
            target_id=row['target_id'],parent_node=row['node'],complete=row['complete'],category=row['category'],
            wall_s=row['wall_s'],contact_complete=row.get('contact_complete'),
            reason=f.get('reason'),termination=f.get('termination'),last_stage=last.get('stage'),
            option=row['option'],seed=row['seed'],archive=row['evidence']['archive'],archive_sha256=row['evidence']['sha256']))
    return dict(schema='fixed_gap_search_delivery_summary_v1',status=db['status'],
        checkpoint=dict(server_path=str(path),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw)),
        identity=db['identity'],base_commit=db['source_commit'],source_revisions=db.get('source_revisions',[]),
        invocations=db.get('invocations',[]),pending=db.get('pending'),
        elapsed_search_s=db['elapsed_search_s'],admission_wall_s=sum(x['admission_wall_s'] for x in layouts),
        trial_count=len(trials),interrupted=db['interrupted'],layouts=sorted(layouts,key=lambda x:x['gap_m']),
        best_gap_m=best['gap_m'] if best else None,best_count=best.get('best_count',0) if best else 0,
        best_layout=best['candidate'] if best else None,best_cycles=cycles,remaining_target_ids=remaining,
        deepest_branch_failure_diagnostics=blocking,
        trials=trials,requested_gaps_m=db['requested_gaps_m'],
        queued_unadmitted_gaps_m=[g for g in db['requested_gaps_m'] if g not in {r['gap_m'] for r in layouts}],
        refinement_queue=db['refinement_queue'],physical_execution='NOT_RUN',global_infeasibility_proven=False,
        search_scope='Finite candidate visits and numerical budgets; unvisited nodes and gaps remain open',
        assumptions=['ideal_independent_cups','nominal_gravity_fixed_orientation','RECEPTION_ASSUMED',
            'finite_belt_contact_with_ideal_overhang_no_dynamic_stability_proof','ideal_outfeed',
            'sampled_collision_checks_not_continuous_swept_proof'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--additional-run',type=Path,action='append',default=[])
    p.add_argument('--allow-active',action='store_true');a=p.parse_args()
    runs=[summarize(path) for path in [a.run,*a.additional_run]]
    if not a.allow_active and any(r['pending'] for r in runs):raise ValueError('FREEZE_AT_TRIAL_BOUNDARY_BEFORE_FINAL_REPORT')
    result=runs[0]
    if a.additional_run:
        winner=max(runs,key=lambda r:r['best_count'])
        result=dict(schema='independent_gap_search_runs_summary_v1',runs=runs,
            best_count=winner['best_count'],best_gap_m=winner['best_gap_m'],best_run=winner['checkpoint']['server_path'],
            completion_count_definition='MAXIMUM_OF_ONE_CONTINUOUS_BRANCH_NEVER_SUM_ACROSS_WORLDS',
            status='COUNTS_REQUIRE_SEPARATE_BEST_BRANCH_VERIFICATION',
            trial_count=sum(r['trial_count'] for r in runs),elapsed_search_s=sum(r['elapsed_search_s'] for r in runs),
            physical_execution='NOT_RUN',global_infeasibility_proven=False)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','trial_count','best_gap_m','best_count','elapsed_search_s')}))


if __name__=='__main__':main()
