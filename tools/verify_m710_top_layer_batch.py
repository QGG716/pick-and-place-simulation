"""Verify saved native lineage, committed continuity and nominal event counts."""
import argparse,json
from pathlib import Path
import numpy as np

def verify(run):
    protocol=json.loads((run/'protocol.json').read_text());reports=[]
    original={b['id']:b for b in protocol['original_cartons']};all_targets=set(protocol['frozen']['target_ids'])
    targets=set(protocol.get('scoped_target_ids',all_targets))
    mount_retry='authorized_mount_change' in protocol
    if mount_retry:
        from unloading_sim.y_shift_retry import verify_mount_protocol
        verify_mount_protocol(protocol,Path(__file__).resolve().parents[1])
    ordered_l04=protocol.get('scope')=='ORDERED_L04_C02_THEN_C03_AFTER_ACCEPTED_L05'
    paired_mount=protocol.get('scope')=='PAIRED_MOUNT_INDEPENDENT_SINGLE_CARTON' or ordered_l04
    l04_trial=protocol.get('scope')=='L04_C03_INDEPENDENT_FACE_TRIAL_AFTER_ASSUMED_L05_REMOVAL'
    face_trial=l04_trial or protocol.get('scope')=='L05_C03_INDEPENDENT_FACE_TRIAL'
    assert len(original)==40 and (len(all_targets) in (5,40) if paired_mount or face_trial else len(all_targets)==5)
    if face_trial:
        from unloading_sim.l05_face_trial import TARGET,permission,option_for
        if l04_trial:
            from unloading_sim.l04_face_trial import TARGET,permission,option_for
        from unloading_sim.carton_population import FrozenCartonPopulation
        from unloading_sim.coupled_batch import message_box
        from unloading_sim.conveyor_lift import coupled_conveyor_state
        from unloading_sim.current_population_recipe import LAYOUT
        from unloading_sim.workcell_layout import load_workcell_layout
        frozen=FrozenCartonPopulation.capture(tuple(message_box(b) for b in original.values()))
        marker='l04_face_permission' if l04_trial else 'l05_face_permission'
        grant=permission(frozen,protocol[marker]['requested_face'],protocol['initial_root']['height_m'],
            *([] if l04_trial else [protocol.get('receive_mode','Hmax')]))
        assert grant==protocol[marker] and targets=={TARGET}
        if not mount_retry:assert protocol['active_layout_file']=='configs/workcells/'+LAYOUT
        assert protocol['all_joint_rate_limits_retained'] and protocol['third_sync_disabled']
        assert protocol['task_budget_s']==300. and protocol['stage_budget_s']==90.
    if paired_mount:assert len(targets)==1 and protocol.get('third_sync_disabled')
    for seed in protocol['seeds']:
        folder=run/f'seed-{seed}';r=json.loads((folder/'result.json').read_text())
        q=protocol['configured_initial_q'];height=r['initial']['height_m']
        historical=list(r['initial']['completed']);removed=historical.copy()
        assumed_ids=set()
        if ordered_l04:
            order=['carton_l04_c02','carton_l04_c03'];index=protocol['sequence_index']
            assert mount_retry and protocol['ordered_target_ids']==order and index in (0,1)
            assert targets=={order[index]} and r['initial']==protocol['initial_root']
            assert len(historical)==25+index and len(r['initial']['world_cartons'])==15-index
            assert historical[-1]==('carton_l05_c03' if index==0 else order[0])
            assert not r['initial'].get('assumed_removed') and r['initial']['predecessor_final_state_receipt']
            assert r['historical_completed_count']==len(historical)
            if index==0:assert r['initial']==protocol['accepted_predecessor']['terminal']
        elif mount_retry:
            assert r['initial']==protocol['initial_root'] and len(historical)==24
            assert r['initial']['predecessor_final_state_receipt'] is None
            assumed_ids=set(protocol.get('assumed_removed_target_ids',[]))
            assert len(r['initial']['world_cartons'])==16-len(assumed_ids)
            if assumed_ids:
                from unloading_sim.assumed_removal import validate_assumption
                assert assumed_ids=={'carton_l05_c03'} and targets=={'carton_l04_c03'}
                validate_assumption(protocol['assumed_removal'],original,r['initial'],r['historical_ledger'])
            else:assert targets=={'carton_l05_c03'}
        if face_trial:
            terminal_trial=protocol.get('root_stage')=='population_terminal'
            assert r['initial']==protocol['initial_root'] and len(historical)==(24 if terminal_trial or l04_trial else 11)
            assert r['initial']['predecessor_final_state_receipt'] is None and len(r['initial']['world_cartons'])==(15 if l04_trial else 16 if terminal_trial else 29)
            if terminal_trial:assert grant['requested_face']!='top'
            if l04_trial:
                from unloading_sim.assumed_removal import validate_assumption
                validate_assumption(protocol['assumed_removal'],original,r['initial'],r['historical_ledger'])
                assert protocol['assumed_removed_target_ids']==['carton_l05_c03']
                assumed_ids={'carton_l05_c03'}
            layout=load_workcell_layout(Path(__file__).resolve().parents[1]/protocol['active_layout_file'])
            world={b.name:b for b in coupled_conveyor_state(layout,grant['height_m'],
                tuple(message_box(b) for b in r['initial']['world_cartons'])).boxes}
            expected,_=option_for(message_box(original[TARGET]),world['conveyor_transverse'],world['conveyor_longitudinal'],
                frozen,r['initial']['height_m'],grant)
            for attempt in r['attempts']:
                if 'option' in attempt:assert attempt['option']==expected
                if 'folder' not in attempt:continue
                calls=json.loads((folder/attempt['folder']/'requests.json').read_text())
                trace=json.loads((folder/attempt['folder']/'search-trace.json').read_text())
                for entry in trace['attempts']:
                    req=entry['request']
                    if 'clearance_policy' in req:
                        assert req['clearance_policy']['source_policy']==protocol['collision_policy']
                        assert req['planning_mode']=='cold_from_scratch' and req['planning_only']
                assert not any('prior' in str(c.get('op')) or c.get('pipeline_id')=='static_prior' for c in calls)
                if attempt.get('failure')=='BUSINESS_FACE_NOT_CURRENTLY_EXPOSED':
                    assert all(c['op'] in ('init','fk') for c in calls) and not trace['attempts']
                    assert any(s['geometric_face_id']==grant['geometric_face_id'] and not s['available'] for s in attempt['screening'])
        previous=r['initial']['predecessor_final_state_receipt'] if ordered_l04 else None
        stages=0;lift_samples=0;drop_samples=0
        successful=[a for a in r['attempts'] if a.get('complete')]
        assert len(successful)==len(r['cycles'])==r['completed_count']
        for attempt,cycle in zip(successful,r['cycles']):
            root=cycle['batch_root'];assert root['q_rad']==q and root['height_m']==height and root['completed']==removed
            assert root['predecessor_final_state_receipt']==previous
            assert {b['id']:b for b in root['world_cartons']}=={n:b for n,b in original.items() if n not in removed and n not in assumed_ids}
            if assumed_ids:assert root['assumed_removed']==['carton_l05_c03']
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
            if paired_mount or face_trial:
                assert cycle['attachment']['pose_continuity_max_abs']<1e-8
                assert cycle['third_sync_disabled'] and cycle['nominal_release']['passed']
                assert abs(cycle['actual_extraction_height_m']-cycle['height_m'])<1e-8
                if (cycle['option'].get('reception') or {}).get('junction'):
                    assert cycle['receive_policy_check']['reception']['passed']
                if not protocol['right_front_eccentric']:
                    assert cycle['target'].get('world_y_offset_m',0.)==0
                    assert cycle['target'].get('eccentric_authorization') is None
            if face_trial and grant['requested_face']=='top':
                lift=cycle['stationary_low_receiver']['top_lift']
                assert lift['sample_check']['passed'] and lift['terminal_check']['status']=='SUCCESS'
                assert lift['terminal_check']['free_space_rules_restored']
                assert cycle['stationary_low_receiver']['throughout_contact_lift_transfer_release']
            assert cycle['separation']['passed'] and cycle['separation']['response']['status']=='SUCCESS'
            types=[e['event_type'] for e in cycle['events']]
            required=['RELEASE','NOMINAL_FIRST_CONTACT','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN']
            assert all(types.count(x)==1 for x in required)
            assert [types.index(x) for x in required]==sorted(types.index(x) for x in required)
            assert cycle['target_id'] in targets and cycle['target_id'] not in removed
            removed.append(cycle['target_id']);previous=cycle['final_state_receipt']
        assert r['terminal']['q_rad']==q and r['terminal']['height_m']==height
        assert {b['id']:b for b in r['terminal']['world_cartons']}=={n:b for n,b in original.items() if n not in removed and n not in assumed_ids}
        if assumed_ids:assert r['terminal']['assumed_removed']==['carton_l05_c03']
        assert all(r[k]==len(removed)-len(historical) for k in ('release_count','nominal_first_contact_count','separation_count','ideal_drain_count'))
        assert r['lower_layer_preserved'] and r['physical_execution']=='NOT_RUN' and not r['execution_ready']
        assert r['qualification_status']=='NOT_EVALUATED' and not r['continuous_swept_proof']
        if (face_trial or mount_retry) and not r['completed_count']:
            assert r['terminal']==r['initial'] and not (folder/'checkpoint.json').exists()
        if protocol.get('scope')=='ONE_NEW_JUNCTION_C00':
            assert len(historical)==3 and len(r['initial']['world_cartons'])==37
            assert len(r['terminal']['world_cartons'])==37-r['completed_count']
            if r['completed_count']:
                assert r['completed_count']==1
                assert r['cycles'][0]['receive_policy_check']['reception']['junction']
                assert abs(r['cycles'][0]['nominal_release']['drop_m']-.35)<1e-5
            else:
                assert r['terminal']==r['initial'] and r['status'] in ('INCOMPLETE_RETAINED','REQUEST_STOPPED')
                assert not (folder/'checkpoint.json').exists()
            assert 'carton_l07_c04' in {b['id'] for b in r['terminal']['world_cartons']}
        reports.append(dict(seed=seed,status='PASS_SAVED_LINEAGE_AND_CONTINUITY' if r['completed_count'] else 'PASS_FAILURE_RETENTION_UNCHANGED_COMMITTED_ROOT',
            completed=len(removed)-len(historical),targets=len(targets),
            historical_completed=len(historical),initial_cartons=len(r['initial']['world_cartons']),
            remaining_cartons=len(r['terminal']['world_cartons']),
            native_stages=stages,coupled_native_samples=lift_samples,nominal_drop_samples=drop_samples,
            lower_cartons_unchanged=35 if len(all_targets)==5 else None,
            unscoped_cartons_unchanged=len(original)-len(targets),
            failed_attempts=sum(not a.get('complete') for a in r['attempts']),planning_wall_s=r['planning_wall_s']))
    return dict(source_commit=protocol['source_commit'],runs=reports,physical_execution='NOT_RUN',execution_ready=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=verify(a.run)
    (a.run/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
