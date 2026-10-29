"""Independent accepted-chain audit for external XY moves and native cycles."""
import argparse,hashlib,json,tarfile,xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from plan_m710_joint_search import ROOT,restore_state,prepare_layout
from plan_m710_mount_comparison import make_context
from verify_m710_joint_search import verify_search,verify_cycle
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.joint_search_layout import LayoutCandidate,qualified_mount_pairs
from unloading_sim.moveit2_backend import box_message,digest,model_request
from unloading_sim.xy_base_continuation import XY_SCHEMA,xy_path,relocation_world,check_plinth_world,relocation_receipt

def candidate(e):return LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_native_model(model,built,scene):
    expected,tools,compliant=model_request(built,scene)
    actual_parameters=deepcopy(model['parameters']);expected_parameters=deepcopy(expected['parameters'])
    actual_xml=ET.fromstring(actual_parameters['robot_description']);expected_xml=ET.fromstring(expected_parameters['robot_description'])
    am,em=list(actual_xml.iter('mesh')),list(expected_xml.iter('mesh'));assert len(am)==len(em)
    for a,e in zip(am,em):
        assert a.get('filename').startswith('file://') and e.get('filename').startswith('file://')
        assert sha(a.get('filename').removeprefix('file://'))==sha(e.get('filename').removeprefix('file://'))
        a.set('filename',e.get('filename'))
    actual_parameters['robot_description']=ET.tostring(actual_xml,encoding='unicode')
    expected_parameters['robot_description']=ET.tostring(expected_xml,encoding='unicode')
    assert actual_parameters==expected_parameters,'XY_OFFICIAL_MODEL_OR_TOOL_CHANGED'
    for key in ('tool_links','compliant_tool_links','tool_geometry_binding','process_geometry','expected_collision_shapes','collision_policy'):
        assert model[key]==json.loads(json.dumps(expected[key])),('XY_MODEL_CHANGED',key)
    assert model['identity']['model_tool_fingerprint']==digest(model['parameters'])
    for key in ('scene_fingerprint','policy_fingerprint','validator_identity','reviewed_sources','task_tcp_link','flange_from_task_tcp','task_tcp_fingerprint'):
        assert model['identity'][key]==json.loads(json.dumps(expected['identity'][key])),('XY_NATIVE_IDENTITY_CHANGED',key)
    assert model['planning_only'] and model['planning_mode']=='cold_from_scratch'
    return tools,compliant

def verify_move(folder,state,original,built,source,destination):
    folder=Path(folder);row=json.loads((folder/'xy-motion.json').read_text());raw=json.loads((folder/'native-checks.json').read_text())
    assert row['schema']==XY_SCHEMA and row['passed'] and row['failure'] is None
    assert row['batch_root']==state.root() and row['from_layout']==source.evidence() and row['to_layout']==destination.evidence()
    assert source.z==destination.z and source.minimum==destination.minimum and source.maximum==destination.maximum and source.profile==destination.profile
    assert row['native_checks_sha256']==sha(folder/'native-checks.json')
    assert row['robot_q_constant']==state.q.tolist() and row['receiver_height_constant_m']==state.height
    assert row['maximum_step_m']==.005 and row['continuous_swept_proof'] is False and row['physical_execution']=='NOT_RUN'
    path=xy_path([source.x,source.y],[destination.x,destination.y],order=row['axis_order']);assert path.tolist()==row['path_xy_a_m']
    assert len(path)==len(raw)==len(row['samples'])
    layout=original.policy.layout_validation.layout;scene=batch_scene(original,state,built.robot,layout)
    model=json.loads((folder/'native-model.json').read_text());tools,compliant=verify_native_model(model,built,scene)
    calls=json.loads((folder/'native-records.json').read_text());assert calls[0]['op']=='init' and calls[0]['status'] in ('READY','SUCCESS')
    assert all(c['status']=='SUCCESS' for c in calls if c['op']=='fk') and any(c['op']=='fk' for c in calls)
    validator=built.robot_state_validator;old={k:getattr(validator,k,None) for k in ('contact_target_name','commanded_cup_mask','stack_carton_names')}
    reference=min(state.cartons,key=lambda b:(b.center[2],b.name)).name
    assert row['reference_object_id']==reference and row['reference_is_not_grasp_target'] and row['commanded_active_mask']==[False]*72
    assert reference not in built.collision_policy.tool_chassis_contact_target_ids
    validator.contact_target_name=reference;validator.commanded_cup_mask=(False,)*72;validator.stack_carton_names={b.name for b in state.cartons}
    try:
        for index,(xy,sample,item) in enumerate(zip(path,row['samples'],raw)):
            delta=np.r_[xy-np.array([source.x,source.y]),0.]
            canonical=relocation_world(scene.all_obstacles,delta);evaluation=relocation_world(scene.all_obstacles,delta,evaluation_frame=True)
            assert sample['index']==item['sample']==index and sample['base_xy_a_m']==xy.tolist() and sample['delta_world_m']==delta.tolist()
            assert check_plinth_world(canonical)==sample['plinth'] and sample['plinth']['passed']
            assert sample['canonical_world_sha256']==digest([box_message(b) for b in canonical])
            assert sample['cpu_robot_failure'] is None and built.validate_unloaded_state(state.q,evaluation,stage='unloaded') is None
            req=item['request'];resp=item['response'];world=[box_message(b) for b in evaluation]
            assert req['world']==world and req['scene_fingerprint']==digest(world)==sample['evaluation_world_sha256']
            assert req['q_start']==req['q_goal']==state.q.tolist() and req['start_velocity']==[0.]*6
            assert req['op']=='check_start' and req['stage']=='pregrasp' and req['attachment'] is None
            assert req['planning_only'] and req['require_native_motion'] and req['planning_mode']=='cold_from_scratch' and not req['history_enabled']
            assert req['identity']==model['identity'] and req['path_constraints']=={} and not req['pair_profile_enabled']
            pairs=built.collision_policy.robot_mount_pairs()|qualified_mount_pairs(built.robot,evaluation)
            pairs|={(cup,b.name) for cup in compliant for b in state.cartons if b.name!=reference}
            assert {tuple(x) for x in req['allowed_pairs']}==pairs and len(req['allowed_pairs'])==len(pairs)
            policy=json.loads(json.dumps(built.collision_policy.to_mapping()))
            clear=req['clearance_policy'];assert clear['allowed_pairs']==req['allowed_pairs'] and clear['source_policy']==policy
            assert clear['tool_links']==sorted(tools) and clear['compliant_tool_links']==sorted(compliant)
            assert clear.get('concept_solids',[])==[b for b in world if 'solid' in b]
            process=req['process_policy'];assert process['target_id']==reference and process['target']==next(b for b in world if b['id']==reference)
            assert not process['attached'] and process['initial_proximity'] is None and process['support_names']==[] and not process['require_terminal_seal']
            assert process['collision_policy']==policy and process['cups']==model['process_geometry']['cups']
            for mask in ('commanded_active_mask','geometrically_eligible_mask','actual_contact_mask'):assert process[mask]==[False]*72
            assert sample['native_request_sha256']==digest(req) and sample['native_response_sha256']==digest(resp)
            assert resp['status']==sample['native_status']=='SUCCESS' and resp['checked_states']==1 and resp['failure'] is None
            assert resp['request_check_evidence']['completion']=='COMPLETED_REQUEST' and resp['request_check_evidence']['clearance']=='AVAILABLE_EXECUTED'
            assert resp['process_policy']['state_checks']==1 and not resp['process_policy']['attached'] and not resp['process_policy']['relaxed_stack']
            assert resp['clearance']['counts']['endpoint']['rejected']==0 and not resp['clearance']['continuous_swept_proof']
    finally:
        for k,v in old.items():setattr(validator,k,v)
    return dict(status='PASS_UNLOADED_XY_NATIVE_DISCRETE_CHECKS',samples=len(path),motion_sha256=sha(folder/'xy-motion.json'),
        from_base_world_m=source.evidence()['base_xyz_world_m'],to_base_world_m=destination.evidence()['base_xyz_world_m'],
        physical_execution='NOT_RUN',base_actuation='KINEMATIC_ASSUMPTION',continuous_swept_proof=False)

def verify(run,*,require_complete=False):
    run=Path(run);db=json.loads((run/'checkpoint.json').read_text());assert db['schema']==XY_SCHEMA
    prefix=Path(db['prefix']['path']);assert sha(prefix/'checkpoint.json')==db['prefix']['checkpoint_sha256']
    old=json.loads((prefix/'checkpoint.json').read_text());proof=verify_search(prefix,old['best']);assert proof['completed']==31
    anchor=old['layouts'][old['best']]['nodes'][old['layouts'][old['best']]['best_node']]
    nodes=db['accepted_nodes'];assert nodes[0]['root']==anchor['root'] and nodes[0]['ledger']==anchor['ledger']
    assert digest(anchor['root'])==db['prefix']['anchor_root_sha256'];contexts={}
    def context(e):
        c=candidate(e)
        if c.key not in contexts:
            record=db['layout_contexts'][c.key];assert record['candidate']==e
            prepared,_=prepare_layout(SimpleNamespace(seed=db['options']['seed'],initial_draws=db['options']['initial_draws']),c)
            assert prepared is not None
            original,built,frozen,initial=prepared
            assert initial.q.tolist()==record['initial_q'] and built.joint_search_initial_q==record['initial_q']
            assert original.policy.layout_validation.layout.layout_fingerprint==record['layout_fingerprint']
            contexts[c.key]=(original,built,frozen)
        return contexts[c.key]
    moves=[];cycles=[]
    for previous,node in zip(nodes,nodes[1:]):
        assert node['parent']==previous['id'] and previous['id']==digest(dict(root=previous['root'],layout=previous['layout']))
        trial=db['trials'][node['incoming']];assert trial['complete'] and trial['parent']==previous['id'] and trial['layout']==node['layout']
        original,built,frozen=context(previous['layout']);state=restore_state(previous,frozen)
        if trial['move'] is not None:
            move=db['relocations'][trial['move']];assert move['parent']==previous['id'] and move['passed'] and move['to_layout']==node['layout']
            assert sha(run/move['folder']/'xy-motion.json')==move['motion_sha256']
            source,destination=candidate(previous['layout']),candidate(node['layout'])
            moves.append(verify_move(run/move['folder'],state,original,built,source,destination))
            state.cycles.append(dict(final_state_receipt=relocation_receipt(state.root(),source,destination,move['motion_sha256'])))
        else:assert previous['layout']==node['layout']
        original,built,frozen=context(node['layout']);evidence=trial['evidence'];archive=run/evidence['archive'];assert sha(archive)==evidence['sha256']
        with tarfile.open(archive) as tar:
            stem=archive.name.removesuffix('.tar.gz')+'/'
            def read(name):
                raw=tar.extractfile(stem+name).read();assert hashlib.sha256(raw).hexdigest()==evidence['file_sha256'][name];return json.loads(raw)
            plan=read('plan.json');scene=batch_scene(original,state,built.robot,original.policy.layout_validation.layout)
            verify_native_model(read('search-trace.json')['model'],built,scene)
            cycles.append(verify_cycle(plan,None,state.root(),original.policy.layout_validation.layout,built,read=read))
        state.commit(plan);assert state.root()==node['root'] and state.ledger==node['ledger']
        assert node['id']==digest(dict(root=node['root'],layout=node['layout']))
    last=nodes[-1];count=len(last['root']['completed']);remaining=last['root']['world_cartons']
    assert count+len(remaining)==40 and len(set(last['root']['completed']))==count and not last['root'].get('assumed_removed')
    if require_complete:assert count==40 and not remaining
    return dict(status='PASS_COMPLETE_WITH_CHECKED_BASE_MOVEMENT' if count==40 else 'PASS_PARTIAL_CHECKED_MOVING_BASE_BRANCH',
        completed=count,remaining=len(remaining),additional_completed=count-31,prefix=proof,moves=moves,cycles=cycles,
        physical_execution='NOT_RUN',base_actuation='KINEMATIC_ASSUMPTION',continuous_swept_proof=False,
        fixed_installation_40_complete=False,remaining_ids=[b['id'] for b in remaining])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--require-complete',action='store_true');a=p.parse_args()
    result=verify(a.run,require_complete=a.require_complete);(a.run/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
