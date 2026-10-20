"""Verify saved native lineage, committed continuity and nominal event counts."""
import argparse,json
from pathlib import Path
import numpy as np

def verify(run):
    protocol=json.loads((run/'protocol.json').read_text());reports=[]
    original={b['id']:b for b in protocol['original_cartons']};all_targets=set(protocol['frozen']['target_ids'])
    targets=set(protocol.get('scoped_target_ids',all_targets))
    assert len(original)==40 and len(all_targets)==5
    for seed in protocol['seeds']:
        folder=run/f'seed-{seed}';r=json.loads((folder/'result.json').read_text())
        q=protocol['configured_initial_q'];height=r['initial']['height_m']
        historical=list(r['initial']['completed']);removed=historical.copy()
        previous=None;stages=0;lift_samples=0;drop_samples=0
        successful=[a for a in r['attempts'] if a.get('complete')]
        assert len(successful)==len(r['cycles'])==r['completed_count']
        for attempt,cycle in zip(successful,r['cycles']):
            root=cycle['batch_root'];assert root['q_rad']==q and root['height_m']==height and root['completed']==removed
            assert root['predecessor_final_state_receipt']==previous
            assert {b['id']:b for b in root['world_cartons']}=={n:b for n,b in original.items() if n not in removed}
            trace=json.loads((folder/attempt['folder']/'search-trace.json').read_text())
            request={t['request']['stage_id']:t['request'] for t in trace['attempts']}
            calls=json.loads((folder/attempt['folder']/'requests.json').read_text())
            responses={c['stage_id']:c['response'] for c in calls if c.get('op')=='plan' and c['status']=='SUCCESS'}
            parent=''
            extraction_started=False
            for s in cycle['stages']:
                req=request[s['stage_id']];resp=responses[s['stage_id']]
                assert req['q_start']==q and req['parent_stage_id']==parent
                assert req['pair_profile_enabled'] is False and req['planning_mode']=='cold_from_scratch'
                assert req['start_velocity']==[0.]*6
                assert resp['generated_during_task'] and resp['mtc_generation']
                if s['generation_source']=='NATIVE_EXTERNAL_AXIS':
                    if protocol.get('third_sync_disabled'):
                        assert not extraction_started and s['stage'] in ('pregrasp','contact')
                    axis=resp['external_axis'];assert axis['from_height_m']==height
                    assert all(c['passed'] for c in axis['checks']) and axis['maximum_height_step_m']<=.010000001
                    assert not axis['continuous_swept_proof'];lift_samples+=len(axis['checks']);height=axis['to_height_m']
                    assert resp['terminal_q']==q
                elif s['generation_source']=='SEMANTIC_EVENT':
                    drop=resp['nominal_release'];assert drop['same_pose_detachment'] and drop['target_in_world_once'] and drop['first_contact']
                    assert all(c['passed'] for c in drop['checks']);drop_samples+=len(drop['checks'])
                    assert resp['terminal_q']==q
                    assert req['nominal_release']['samples'][0]['box']==cycle['nominal_release']['samples'][0]['box']
                else:
                    if s['stage']=='extraction':
                        extraction_started=True
                        if protocol.get('scope')=='ONE_NEW_JUNCTION_C00':
                            belt=next(b for b in req['world'] if b['id']=='conveyor_transverse')
                            assert abs(belt['pose'][2][3]+belt['size'][2]/2-1.75)<1e-8
                            assert cycle['actual_extraction_height_m']==height
                    assert s['points'][0]['q']==q and s['points'][-1]['t']>0
                    assert max(abs(v) for v in s['points'][-1]['v'])<=1e-8
                    q=s['points'][-1]['q']
                parent=s['stage_id'];stages+=1
            assert q==cycle['end_q_rad'] and height==cycle['height_m']
            selection=cycle['attachment']['selection']
            assert all(len(selection[k])==72 for k in ('actual_contact_mask','commanded_active_mask','geometrically_eligible_mask'))
            assert selection['actual_contact_mask']==selection['commanded_active_mask'] and any(selection['actual_contact_mask'])
            assert cycle['separation']['passed'] and cycle['separation']['response']['status']=='SUCCESS'
            types=[e['event_type'] for e in cycle['events']]
            required=['RELEASE','NOMINAL_FIRST_CONTACT','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN']
            assert all(types.count(x)==1 for x in required)
            assert [types.index(x) for x in required]==sorted(types.index(x) for x in required)
            assert cycle['target_id'] in targets and cycle['target_id'] not in removed
            removed.append(cycle['target_id']);previous=cycle['final_state_receipt']
        assert r['terminal']['q_rad']==q and r['terminal']['height_m']==height
        assert {b['id']:b for b in r['terminal']['world_cartons']}=={n:b for n,b in original.items() if n not in removed}
        assert all(r[k]==len(removed)-len(historical) for k in ('release_count','nominal_first_contact_count','separation_count','ideal_drain_count'))
        assert r['lower_layer_preserved'] and r['physical_execution']=='NOT_RUN' and not r['execution_ready']
        assert r['qualification_status']=='NOT_EVALUATED' and not r['continuous_swept_proof']
        if protocol.get('scope')=='ONE_NEW_JUNCTION_C00':
            assert len(historical)==3 and len(r['initial']['world_cartons'])==37
            assert len(r['terminal']['world_cartons'])==36 and r['completed_count']==1
            assert r['cycles'][0]['receive_policy_check']['reception']['junction']
            assert abs(r['cycles'][0]['nominal_release']['drop_m']-.35)<1e-5
            assert 'carton_l07_c04' in {b['id'] for b in r['terminal']['world_cartons']}
        reports.append(dict(seed=seed,status='PASS_SAVED_LINEAGE_AND_CONTINUITY',completed=len(removed)-len(historical),targets=len(targets),
            historical_completed=len(historical),initial_cartons=len(r['initial']['world_cartons']),
            remaining_cartons=len(r['terminal']['world_cartons']),
            native_stages=stages,coupled_native_samples=lift_samples,nominal_drop_samples=drop_samples,lower_cartons_unchanged=35,
            failed_attempts=sum(not a.get('complete') for a in r['attempts']),planning_wall_s=r['planning_wall_s']))
    return dict(source_commit=protocol['source_commit'],runs=reports,physical_execution='NOT_RUN',execution_ready=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=verify(a.run)
    (a.run/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
