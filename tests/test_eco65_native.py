"""Synthetic contract tests are explicitly separate from real asset acceptance."""
import copy
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT, URDF, transform, robot_for
from unloading_sim.eco65.native_contact import validate_geometry, check_contours
from unloading_sim.eco65.native_scene import combined_scene, verify_scene
from unloading_sim.eco65.native_model import load_robot_config, asset_inventory, POLICY
from unloading_sim.eco65.native_task import NativeTask, supported
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.moveit2_backend import digest, MoveItUnavailable, ResidentMoveItClient


def synthetic_geometry():
    rows = []
    for i in range(12):
        x, y = (i%4-1.5)*.04, (i//4-1)*.045
        points = [[x+a, y+b, 0] for a,b in [(-.014,-.011),(.012,-.012),(.015,.006),(.004,.014),(-.013,.01)]]
        rows.append(dict(id=f'synthetic_{i}', points_m=points, normal=[0,0,-1]))
    return dict(geometry_type='cad_seal_contours_v1', contours=rows,
                expected_contour_ids=[r['id'] for r in rows], flange_from_contact_frame=np.eye(4).tolist())


@pytest.mark.parametrize('height,expected',[(.08,True),(.0804,True),(.08040001,False),(.0796,True),(.07959999,False)])
def test_synthetic_gap_boundary(height, expected):
    result=check_contours(synthetic_geometry(), transform([0,0,height]), np.eye(4), [.22,.22,.16],gap=.0004,penetration=.0004)
    assert (result['status']=='PASS') == expected
    assert result['checked_count']==12


@pytest.mark.parametrize('case',['missing','duplicate','wrong_id','nan','nonplanar','zero_normal','wrong_type'])
def test_reject_malformed_contours(case):
    g=synthetic_geometry()
    if case=='missing': g['contours'].pop()
    if case=='duplicate': g['contours'][1]['id']=g['contours'][0]['id']
    if case=='wrong_id': g['expected_contour_ids'][0]='wrong'
    if case=='nan': g['contours'][0]['points_m'][0][0]=float('nan')
    if case=='nonplanar': g['contours'][0]['points_m'][0][2]=.001
    if case=='zero_normal': g['contours'][0]['normal']=[0,0,0]
    if case=='wrong_type': g['geometry_type']='circular_72_v1'
    with pytest.raises(ValueError):validate_geometry(g)


def test_edge_normal_and_nonidentity_cad_frame():
    g=synthetic_geometry(); flange=transform([0,0,.08])
    g['contours'][0]['points_m'][0][0]=.107
    assert check_contours(g,flange,np.eye(4),[.22,.22,.16],gap=.0004,penetration=.0004)['status']=='PASS'
    g['contours'][0]['points_m'][0][0]=.107001
    assert check_contours(g,flange,np.eye(4),[.22,.22,.16],gap=.0004,penetration=.0004)['status']=='FAIL'
    g=synthetic_geometry();g['contours'][3]['normal']=[0,0,1]
    assert check_contours(g,flange,np.eye(4),[.22,.22,.16],gap=.0004,penetration=.0004)['status']=='FAIL'
    g=synthetic_geometry(); C=transform([.1,-.2,.3],Rotation.from_euler('x',.7).as_matrix())
    for row in g['contours']:
        row['points_m']=(np.array(row['points_m'])@C[:3,:3].T+C[:3,3]).tolist()
        row['normal']=(C[:3,:3]@row['normal']).tolist()
    g['flange_from_contact_frame']=np.linalg.inv(C).tolist()
    assert check_contours(g,flange,np.eye(4),[.22,.22,.16],gap=.0004,penetration=.0004)['status']=='PASS'


def environment_fixture():
    e=dict(schema='eco65_warehouse_bounds_v1',T_W_D=transform([.32,-.1,.8]).tolist(),complete_traversal=True,
           objects=[dict(id='synthetic_wall',bounds_W_m=[[2,2,0],[2.1,3,2]])],asset_sha256={})
    e['fingerprint']=digest(e);return e


def test_combined_scene_numeric_and_dw_once():
    s=combined_scene(environment=environment_fixture());verify_scene(s)
    assert len(s['boxes'])==4 and len(s['obstacles'])==31
    assert s['target_box_id']=='carton_r00_l01_c00'
    assert s['base_position_m']==[-.36,.1,.18]
    assert np.allclose(np.array(s['base_position_m'])+np.array(s['T_W_D'])[:3,3],[-.04,0,.98])
    obs={o['id']:o for o in s['obstacles']}
    assert obs['trailer_floor']['size_m']==[2.,2.,.03]
    assert len([o for o in obs if o.startswith('Table/')])==12
    assert np.allclose(obs['warehouse/synthetic_wall']['position_m'],[1.73,2.6,.2])
    assert s['source_equipment_config']['table']['bounds_xy_m']==[-1.,.4,-.45,.45]
    assert supported(s,transform([-.5,-.22,.26]))
    assert not supported(s,transform([-.5,-.22,.2595]))
    assert not supported(s,transform([-.5,-.5,.26]))


def test_missing_environment_and_scene_tamper_rejected():
    with pytest.raises(ValueError,match='ENVIRONMENT_NOT'):verify_scene(combined_scene())
    s=combined_scene(environment=environment_fixture());s['boxes'].pop()
    with pytest.raises(ValueError,match='FINGERPRINT'):verify_scene(s)


def test_robot_identity_and_fixed_limits(tmp_path):
    import yaml
    c=load_robot_config();assert c['joint_names']==[f'joint_{i}' for i in range(1,7)]
    for key,value in [('robot_variant','ECO65'),('joint_names',[f'J{i}' for i in range(1,7)]),('geometry_scale',.5)]:
        wrong=copy.deepcopy(c);wrong[key]=value;p=tmp_path/'robot.yaml';p.write_text(yaml.safe_dump(wrong))
        with pytest.raises(ValueError):load_robot_config(p)
    assert POLICY['required_pair_clearance_m']==.0024 and POLICY['contact_tolerance_m']==.0004


def test_no_worker_no_fallback():
    with pytest.raises((MoveItUnavailable,FileNotFoundError)):
        ResidentMoveItClient(['this-worker-does-not-exist-eco65'])


def test_snapshot_tamper_fails_before_worker():
    with pytest.raises(ValueError,match='SNAPSHOT_IDENTITY'):
        NativeTask(dict(schema='eco65_native_snapshot_v1',fingerprint='wrong'), ['not-a-worker'])


@pytest.mark.real_eco65_asset
def test_pinned_official_fk_against_independent_xml_chain():
    if not URDF.is_file():pytest.skip('Pinned real ECO65-B URDF missing; not asset acceptance')
    audit=asset_inventory()
    assert not [e for e in audit['failures'] if 'realman_eco65' in e['path']]
    scene=combined_scene();tool={'T_flange_tcp':transform([.017,-.009,.12],Rotation.from_euler('xyz',[.1,.2,-.3]).as_matrix()).tolist()}
    robot=robot_for(tool,scene);root=ET.parse(URDF).getroot()
    assert robot.active_joint_names==[f'joint_{i}' for i in range(1,7)]
    for q in [np.zeros(6), [.2,-.3,.4,.5,-.6,.7],[-.4,.2,-.35,-.6,.4,-.8]]:
        frames={'base_link':transform([-.36,.1,.18])}
        for joint,angle in zip(root.findall('joint'),q):
            o=joint.find('origin');T=transform(np.fromstring(o.get('xyz'),sep=' '),Rotation.from_euler('xyz',np.fromstring(o.get('rpy'),sep=' ')).as_matrix())
            axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            frames[joint.find('child').get('link')]=frames[joint.find('parent').get('link')]@T@transform(rotation=Rotation.from_rotvec(axis*angle).as_matrix())
        actual=robot.named_link_frames(q)
        for name,T in frames.items():assert np.allclose(actual[name],T,atol=1e-12,rtol=0)
        assert np.allclose(robot.fk(q),frames['link_6']@np.array(tool['T_flange_tcp']),atol=1e-12,rtol=0)


def test_synthetic_python_cpp_parity():
    exe=os.environ.get('ECO65_CONTACT_PROBE')
    if not exe:pytest.skip('Set ECO65_CONTACT_PROBE to compiled seal_contours_test; no native pass inferred')
    rows=[];rng=np.random.default_rng(65)
    for i in range(60):
        g=synthetic_geometry(); T=transform([rng.uniform(-.045,.045),0,.08+rng.uniform(-.0006,.0006)],Rotation.from_euler('xyz',rng.uniform(-.012,.012,3)).as_matrix())
        p=dict(g,target_face='top',max_attachment_gap_m=.0004,maximum_penetration_m=.0004,suction_edge_margin_m=.003,max_normal_misalignment_rad=.01)
        rows.append(dict(policy=p,flange=T.tolist(),target=dict(id='synthetic_target',pose=np.eye(4).tolist(),size=[.22,.22,.16])))
    result=subprocess.run([exe],input=''.join(json.dumps(r)+'\n' for r in rows),capture_output=True,text=True,check=True)
    replies=[json.loads(line) for line in result.stdout.splitlines()];assert len(replies)==len(rows)
    for row,cpp in zip(rows,replies):
        py=check_contours(row['policy'],row['flange'],np.eye(4),[.22,.22,.16],gap=.0004,penetration=.0004)
        assert cpp['status']==py['status']
        for a,b in zip(cpp['contours'],py['contours']):
            assert a['valid']==b['valid'] and a['id']==b['id']
            for key in ['min_gap_m','max_gap_m','edge_margin_m']:assert abs(a[key]-b[key])<1e-12


# Deliberately fabricated protocol fixture: tests the rejection contract only.
# It must never be written as a planning or replay success artifact.
def synthetic_plan_fixture():
    from types import SimpleNamespace
    from unloading_sim.eco65.native_task import PHASES, STATUS
    scene=combined_scene(environment=environment_fixture());identity={'test_only':'synthetic'}
    init=dict(identity=identity,joint_names=[f'joint_{i}' for i in range(1,7)],
              process_geometry=synthetic_geometry(),compliant_tool_links=[f'band_{i}' for i in range(12)],tool_links=['rigid'])
    identity['flange_from_task_tcp']=np.eye(4).tolist()
    model=dict(init=init,simulation_limits=load_robot_config()['simulation_limits'],
               official_limits={j:dict(lower=-3,upper=3,velocity=1) for j in init['joint_names']})
    snapshot=dict(scene=scene,native_model=model,fingerprint='synthetic_snapshot_only',tool={'contact':{'normal_tolerance_rad':.01}})
    task=object.__new__(NativeTask);task.scene=scene;task.snapshot=snapshot;task.init=init;task.identity=identity
    task.target=scene['box'];task.target_pose=transform(task.target['pose']);task.parent='';task.counter=0
    task.task_id='synthetic_contract_only';task.robot=SimpleNamespace(named_link_frames=lambda q:{'link_6':np.eye(4)})
    release=transform([-.5,-.22,.26]).tolist();stages=[];previous=np.zeros(6)
    for i,phase in enumerate(PHASES):
        task.attachment=np.eye(4) if phase in {'extraction','transit','place'} else None
        if phase in {'withdrawal','residence'}:task.target_pose=np.array(release)
        q=[previous.tolist(),([.01*(i+1)]*6 if phase!='residence' else [0.]*6)]
        req=task.request(q[0],phase,op='plan');req.update(stage_id=phase,pipeline_id='pilz_industrial_motion_planner',planner_id='LIN',goal_pose=np.eye(4).tolist())
        raw=dict(status='SUCCESS',joint_names=init['joint_names'],points=[dict(q=v,t=t,v=[0.]*6,a=[0.]*6) for v,t in zip(q,[0.,2.])],
                 native_solver_calls={'synthetic':1},generation_source='NATIVE_GENERATED',mtc_generation=True,generated_during_task=True,stage_id=phase,
                 parent_stage_id=task.parent,task_id=task.task_id,planning_only_status=STATUS,native_output_status='PASS')
        stages.append(dict(stage=phase,stage_id=phase,parent_stage_id=task.parent,q=q,times=[0.,2.],native_result=raw,native_request=req,
                           attachment=None if task.attachment is None else task.attachment.tolist()))
        task.parent=phase;previous=np.array(q[-1])
    events=[dict(event='ATTACH_PREDICTED',target_id=task.target['id'],after_stage='contact',q=stages[1]['q'][-1],flange_from_target=np.eye(4).tolist()),
            dict(event='SUPPORTED_RELEASE_PREDICTED',target_id=task.target['id'],after_stage='place',q=stages[4]['q'][-1],pose=release)]
    plan=dict(status=STATUS,hardware_enabled=False,identity=identity,snapshot_fingerprint=snapshot['fingerprint'],stages=stages,
              events=events,home_q=[0.]*6,task_id=task.task_id,simulated_duration_s=14.)
    plan['fingerprint']=digest(plan);return snapshot,plan


def test_synthetic_complete_chain_contract():
    from unloading_sim.eco65.native_task import validate_plan_contract
    snapshot,plan=synthetic_plan_fixture();assert validate_plan_contract(plan,snapshot)


@pytest.mark.parametrize('case',['empty','nan','short_times','parent','join','event_id','attachment','lost_other_box','tool_identity','free_contact','source','robot','scene','duration'])
def test_synthetic_chain_rejections(case):
    from unloading_sim.eco65.native_task import validate_plan_contract
    snapshot,plan=synthetic_plan_fixture();s=plan['stages'][2]
    if case=='empty':s['q']=[]
    if case=='nan':s['q'][1][0]=float('nan')
    if case=='short_times':s['times']=[0.,.0001]
    if case=='parent':s['parent_stage_id']='wrong'
    if case=='join':s['q'][0][0]+=.1
    if case=='event_id':plan['events'][0]['target_id']='carton_r00_l00_c01'
    if case=='attachment':s['attachment']=None
    if case=='lost_other_box':s['native_request']['world'].pop()
    if case=='tool_identity':s['native_request']['process_policy']['contours']=[]
    if case=='free_contact':s['native_request']['planner_id']='PTP'
    if case=='source':s['native_result']['generation_source']='STATIC_PRIOR_REUSE'
    if case=='robot':plan['identity']=dict(robot_id='FANUC')
    if case=='scene':plan['snapshot_fingerprint']='changed'
    if case=='duration':plan['simulated_duration_s']=13.
    del plan['fingerprint']
    if case=='nan':
        with pytest.raises(ValueError):digest(plan)
        return
    plan['fingerprint']=digest(plan)
    with pytest.raises(ValueError):validate_plan_contract(plan,snapshot)


def test_same_quintic_replay_keeps_all_cartons():
    from types import SimpleNamespace
    from unloading_sim.eco65.native_replay import state_at
    snapshot,plan=synthetic_plan_fixture();robot=SimpleNamespace(named_link_frames=lambda q:{'link_6':transform([float(q[0]),0,0])})
    before=state_at(snapshot,plan,robot,1.);loaded=state_at(snapshot,plan,robot,5.);after=state_at(snapshot,plan,robot,11.)
    assert abs(before['q'][0]-.005)<1e-12
    assert loaded['attached'] and not after['attached']
    target=snapshot['scene']['target_box_id']
    for name in before['cartons']:
        if name!=target:
            assert np.array_equal(before['cartons'][name],loaded['cartons'][name])
            assert np.array_equal(before['cartons'][name],after['cartons'][name])
    assert len(after['cartons'])==4 and np.array_equal(after['cartons'][target],plan['events'][1]['pose'])
    assert after['dynamics']=='NOT_EVALUATED' and after['planning_status']=='PLANNING_ONLY_NOT_EXECUTABLE'
