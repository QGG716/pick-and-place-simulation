"""Independent saved-trace and single-branch fixed40 acceptance verifier."""
import argparse,hashlib,json,tarfile
from pathlib import Path
import numpy as np
from unloading_sim.coupled_batch import BatchState,message_box
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.joint_search_layout import SCHEMA,qualified_mount_pairs
from unloading_sim.joint_search_recipes import validate_receive_contract
from unloading_sim.joint_search_outfeed import outfeed_path,check_full_tool_wait,check_full_tool_park,WAIT_FIRST,WITHDRAW_FIRST,PARK_FIRST,withdrawal_extra
from unloading_sim.moveit2_backend import box_message,digest
from plan_m710_mount_comparison import make_context

def selected_branch(db,key):
    record=db['layouts'][key];initial=record['initial'];original=initial['world_cartons']
    assert len(original)==len({b['id'] for b in original})==40
    assert not initial['completed'] and not initial.get('assumed_removed') and initial['predecessor_final_state_receipt'] is None
    node=record['nodes'][record['best_node']];chain=[];seen=set()
    while node['incoming'] is not None:
        assert node['id'] not in seen;seen.add(node['id'])
        trial=db['trials'][node['incoming']];parent=record['nodes'][node['parent']]
        assert trial['layout']==key and not trial['diagnostic'] and trial['complete'] and trial['node']==parent['id']
        before=parent['root'];after=node['root'];target=trial['target_id']
        assert not after.get('assumed_removed') and target not in before['completed']
        assert after['completed']==before['completed']+[target]
        assert after['world_cartons']==[b for b in before['world_cartons'] if b['id']!=target]
        assert len(after['world_cartons'])==len(before['world_cartons'])-1
        chain.append(node);node=parent
    assert node['root']==initial;chain.reverse()
    return chain

def verify_cycle(plan,folder,root,layout,built,*,read=None):
    read=read or (lambda name:json.loads((folder/name).read_text()))
    assert plan['batch_root']==root and plan['status']=='COUPLED_COMPLETE_CYCLE_PLANNED_NOT_EXECUTABLE'
    contract=plan['option']['joint_search_contract'];target=next(message_box(b) for b in root['world_cartons'] if b['id']==plan['target_id'])
    validate_receive_contract(contract,target,plan['height_m'])
    limits=layout.data['conveyors']['coupling']
    assert contract['minimum_m']==limits['minimum_surface_height_world_m'] and contract['maximum_m']==limits['maximum_surface_height_world_m']
    assert plan['target_id'] not in root['completed'] and not plan['target_still_attached']
    calls=read('requests.json');trace=read('search-trace.json')
    requests={t['request']['stage_id']:t['request'] for t in trace['attempts']}
    responses={c['stage_id']:c['response'] for c in calls if c.get('op')=='plan' and c['status']=='SUCCESS'}
    assert not any('prior' in str(c.get('op','')) or c.get('pipeline_id')=='static_prior' for c in calls)
    import xml.etree.ElementTree as ET
    model=ET.fromstring(trace['model']['parameters']['robot_description'])
    mount=next(j for j in model.findall('joint') if j.find('parent').get('link')=='world' and j.find('child').get('link')=='base_link')
    np.testing.assert_allclose(list(map(float,mount.find('origin').get('xyz').split())),layout.robot_base_transform()[:3,3],atol=1e-12,rtol=0)
    timing=plan['option'].get('outfeed_timing',WAIT_FIRST)
    extra=withdrawal_extra(plan['option'])
    assert timing in (WAIT_FIRST,WITHDRAW_FIRST,PARK_FIRST)
    q=root['q_rad'];height=root['height_m'];parent='';outfeed_stage=None;drop_stage=None;withdraw_stage=None;park_stage=None;wait_q=None
    for stage in plan['stages']:
        req=requests[stage['stage_id']];resp=responses[stage['stage_id']]
        assert req['q_start']==q and req['parent_stage_id']==parent and req['planning_mode']=='cold_from_scratch'
        assert req['planning_only'] and not req['pair_profile_enabled'] and req['start_velocity']==[0.]*6
        assert resp['generated_during_task'] and resp['mtc_generation']
        assert req['clearance_policy']['source_policy']==json.loads(json.dumps(built.collision_policy.to_mapping()))
        world=[message_box(b) for b in req['world']]
        expected=built.collision_policy.robot_mount_pairs()|built.collision_policy.tool_chassis_pairs(plan['target_id'],req['clearance_policy']['tool_links'])|qualified_mount_pairs(built.robot,world)
        assert {tuple(p) for p in req['allowed_pairs'] if 'chassis' in p or 'mount_plinth' in p}==expected
        if stage['generation_source']=='NATIVE_EXTERNAL_AXIS':
            axis=resp['external_axis'];assert axis['from_height_m']==height and all(c['passed'] for c in axis['checks'])
            assert axis['maximum_height_step_m']<=.010000001 and not axis['continuous_swept_proof']
            height=axis['to_height_m'];assert resp['terminal_q']==q
        elif stage['generation_source']=='SEMANTIC_EVENT':
            assert resp['terminal_q']==q
            if stage['planner_id']=='NOMINAL_DROP_RELEASE':
                d=resp['nominal_release'];assert d['same_pose_detachment'] and d['target_in_world_once'] and d['first_contact']
                assert all(c['passed'] for c in d['checks']);drop_stage=stage['stage_id']
            elif stage['planner_id']=='WAIT_IDEAL_OUTFEED':
                assert drop_stage and req['parent_stage_id']==(park_stage if timing==PARK_FIRST else withdraw_stage if timing==WITHDRAW_FIRST else drop_stage)
                d=resp['outfeed'];assert d['passed'] and d['tool_q_constant'] and d['full_robot_tool_checked']
                assert len(d['checks'])==len(req['outfeed']['samples']) and all(c['passed'] for c in d['checks'])
                assert req['outfeed']==plan['wait_outfeed']['path'];outfeed_stage=stage['stage_id'];wait_q=q
            else:raise AssertionError('UNKNOWN_SEMANTIC_EVENT')
        else:
            points=stage['points'];assert points[0]['q']==q and points[-1]['t']>0
            assert all(built.robot.within_limits(np.asarray(p['q'])) for p in points)
            assert max(abs(v) for v in points[-1]['v'])<=1e-8
            assert all(points[i]['t']<=points[i+1]['t'] for i in range(len(points)-1))
            q=points[-1]['q']
            if stage['stage']=='withdrawal':
                assert req['parent_stage_id']==(outfeed_stage if timing==WAIT_FIRST else drop_stage)
                assert req['planner_id']=='LIN' and req['attachment'] is None
                actual_target=next(b for b in req['world'] if b['id']==plan['target_id'])
                assert actual_target==(plan['wait_outfeed']['path']['samples'][-1]['box'] if timing==WAIT_FIRST else plan['landed_box'])
                if timing!=WAIT_FIRST:
                    direction=np.asarray(plan['target']['outward_normal'])
                    stationary=message_box(plan['landed_box'])
                    tool=built.tool_collision_obbs_provider(np.asarray(req['q_start']))
                    gap=min(float(np.min(t.corners()@direction)-np.max(stationary.corners()@direction)) for t in tool)
                    distance=max(.005-gap,0.)+2*float(built.ik['position_tolerance_m'])+extra
                    goal=built.physical_from_virtual(built.robot.fk(np.asarray(req['q_start']))).copy()
                    goal[:3,3]+=direction*distance
                    np.testing.assert_allclose(req['goal_pose'],built.virtual_from_physical(goal),atol=1e-9,rtol=0)
                withdraw_stage=stage['stage_id']
            if stage['stage']=='residence':
                assert timing==PARK_FIRST and req['parent_stage_id']==withdraw_stage
                assert req['purpose']=='FREE_UNLOADED_REPOSITION' and req['attachment'] is None
                assert req['process_policy']['initial_proximity'] is None and req['process_policy']['support_names']==[]
                assert (req['pipeline_id'],req['planner_id']) in {('pilz_industrial_motion_planner','PTP'),('ompl','RRTConnectkConfigDefault')}
                assert next(b for b in req['world'] if b['id']==plan['target_id'])==plan['landed_box']
                park=plan['outfeed_park'];assert park['goal_source']=='FRESH40_FROZEN_INITIAL_LEGAL_STATE'
                assert req['q_goal']==park['goal_q_rad']==plan['option']['park_q_rad']
                assert park['goal_q_rad']==built.joint_search_initial_q
                assert park['path']==[p['q'] for p in points]
                proof=check_full_tool_park(built,park['path'],message_box(plan['landed_box']))
                assert proof['passed'] and proof==park['full_tool'];park_stage=stage['stage_id']
        parent=stage['stage_id']
    assert q==plan['end_q_rad'] and height==plan['height_m'] and outfeed_stage and withdraw_stage
    masks=plan['attachment']['selection']
    assert all(len(masks[k])==72 for k in ('actual_contact_mask','commanded_active_mask','geometrically_eligible_mask'))
    assert any(masks['actual_contact_mask']) and masks['actual_contact_mask']==masks['commanded_active_mask']
    assert all(not a or b for a,b in zip(masks['commanded_active_mask'],masks['geometrically_eligible_mask']))
    assert plan['attachment']['pose_continuity_max_abs']<1e-8
    assert plan['separation']['passed'] and plan['nominal_release']['passed']
    events=plan['events'];types=[e['event_type'] for e in events]
    needed=['RELEASE','NOMINAL_FIRST_CONTACT','WAIT_OUTFEED_COMPLETE','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN']
    if timing!=WAIT_FIRST:needed[2],needed[3]=needed[3],needed[2]
    if timing==PARK_FIRST:
        assert park_stage and plan['outfeed_park']['stage_id']==park_stage
        needed.insert(3,'OUTFEED_PARK_COMPLETE')
    assert all(types.count(t)==1 for t in needed) and [types.index(t) for t in needed]==sorted(types.index(t) for t in needed)
    release=next(e for e in events if e['event_type']=='RELEASE');waiting=plan['wait_outfeed']
    world=coupled_conveyor_state(layout,height,tuple(message_box(b) for b in root['world_cartons'])).boxes
    expected_path=outfeed_path(message_box(plan['landed_box']),world,tool_timing=timing)
    assert waiting['path']==expected_path and waiting['passed']
    assert wait_q==(release['q_rad'] if timing==WAIT_FIRST else plan['end_q_rad'])
    assert next(e for e in events if e['event_type']=='WAIT_OUTFEED_COMPLETE')['q_rad']==wait_q
    full=check_full_tool_wait(built,wait_q,expected_path)
    assert full['passed'] and full==waiting['full_tool']
    final=message_box(expected_path['samples'][-1]['box'])
    assert max(final.corners()[:,0])< -3.2
    return dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED',native_stages=len(plan['stages']),outfeed_samples=len(expected_path['samples']))

def verify_search(run,key,require_complete=False):
    run=Path(run);db=json.loads((run/'checkpoint.json').read_text());record=db['layouts'][key]
    assert db['contract']==SCHEMA
    chain=selected_branch(db,key)
    path=Path(__file__).resolve().parents[1]/'configs/workcells'/('joint-search-'+key+'.yaml')
    original,built,_=make_context(key,layout_path=path,robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True,
        initial_q=record['initial']['q_rad'])
    layout=original.policy.layout_validation.layout;frozen=FrozenCartonPopulation.capture(original.cartons)
    initial=record['initial'];assert len(initial['world_cartons'])==40 and not initial['completed'] and not initial.get('assumed_removed')
    built.joint_search_initial_q=initial['q_rad']
    assert initial['predecessor_final_state_receipt'] is None
    assert initial['world_cartons']==[box_message(b) for b in original.cartons]
    state=BatchState(frozen,tuple(original.cartons),np.asarray(initial['q_rad']),initial['height_m'])
    from unloading_sim.coupled_batch import batch_scene
    scene=batch_scene(original,state,built.robot,layout)
    assert built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded') is None
    assert record['candidate']==layout.data['joint_search']
    reports=[]
    for node in chain:
        trial=db['trials'][node['incoming']];assert trial['layout']==key and not trial['diagnostic'] and trial['complete']
        assert record['nodes'][trial['node']]['root']==state.root()
        archive=run/trial['evidence']['archive'];assert hashlib.sha256(archive.read_bytes()).hexdigest()==trial['evidence']['sha256']
        with tarfile.open(archive) as tar:
            prefix=archive.name.removesuffix('.tar.gz')+'/'
            def read(name):
                raw=tar.extractfile(prefix+name).read()
                assert hashlib.sha256(raw).hexdigest()==trial['evidence']['file_sha256'][name]
                return json.loads(raw)
            plan=read('plan.json');assert read('dispatch.json')['layout_fingerprint']==layout.layout_fingerprint
            if plan['option'].get('outfeed_timing')==PARK_FIRST:assert plan['option']['park_q_rad']==initial['q_rad']
            reports.append(verify_cycle(plan,None,state.root(),layout,built,read=read))
        state.commit(plan);assert state.root()==node['root'] and state.ledger==node['ledger']
    assert len(state.ledger)==record['best_count']==len({x['target_id'] for x in state.ledger})
    assert len(state.cartons)+len(state.ledger)==40 and not state.assumed_removed
    if require_complete:assert len(state.ledger)==40 and not state.cartons
    return dict(status='PASS_FIXED40_COMPLETE_UNDER_LISTED_MODEL_ASSUMPTIONS' if not state.cartons else 'PASS_PARTIAL_SINGLE_BRANCH_ONLY',
        layout=record['candidate'],layout_fingerprint=layout.layout_fingerprint,completed=len(state.ledger),remaining=len(state.cartons),
        historical_completed_count=0,assumed_removed=[],cycles=reports,physical_execution='NOT_RUN',continuous_swept_proof=False,
        collision_policy=built.collision_policy.to_mapping(),search_cost_s=db['elapsed_search_s'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--layout');p.add_argument('--require-complete',action='store_true');a=p.parse_args()
    db=json.loads((a.run/'checkpoint.json').read_text());print(json.dumps(verify_search(a.run,a.layout or db['best'],a.require_complete),ensure_ascii=False))
