"""Small portability/alignment/entry regressions, not live worker acceptance."""
from copy import deepcopy
from pathlib import PurePosixPath
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.eco65.asset_paths import asset_relative, resolve_asset, resource_uri
from unloading_sim.eco65.mount_alignment import relative_roll, check_alignment
from unloading_sim.eco65.home_candidate import make_candidate, verify_candidate
from unloading_sim.eco65.native_task import NativeTask


def test_windows_manifest_to_linux_root_and_uri(tmp_path):
    relative=asset_relative(r'assets\tools\part.obj',migrate_windows=True)
    assert relative=='assets/tools/part.obj'
    assert PurePosixPath('/work/repo')/relative==PurePosixPath('/work/repo/assets/tools/part.obj')
    assert resolve_asset(tmp_path,relative)==tmp_path/'assets/tools/part.obj'
    assert resource_uri(tmp_path,relative,'/work/repo')=='file:///work/repo/assets/tools/part.obj'
    for bad in ('../part.obj',r'assets\..\part.obj',r'C:\part.obj','/part.obj','//host/share','assets//part.obj'):
        with pytest.raises(ValueError): asset_relative(bad,migrate_windows=True)
    with pytest.raises(ValueError): resolve_asset(tmp_path,r'assets\part.obj')
    with pytest.raises(ValueError): resource_uri(tmp_path,relative,r'C:\rootfs')


def pattern(angle):
    a=np.arange(6)*np.pi/3+angle
    return .0245*np.column_stack([np.cos(a),np.sin(a)])


def mount(angle):
    c,s=np.cos(angle),np.sin(angle)
    T=np.eye(4); T[:3,:3]=np.array([[c,-s,0],[s,c,0],[0,0,1]])@np.array([[1,0,0],[0,0,1],[0,-1,0]])
    T[:3,3]=[0,0,.086]
    return T


def test_nonzero_tool_hole_phase_requires_relative_roll():
    robot,tool=pattern(.08),pattern(.23)
    angle=relative_roll(robot,tool)
    assert angle==pytest.approx(-.15)
    good=check_alignment(robot,tool,.086,mount(angle))
    assert good['status']=='PASS' and good['max_hole_residual_m']<1e-15
    assert len(set(good['tool_to_robot_indices']))==6
    assert check_alignment(robot,tool,.086,mount(.08))['status']=='FAIL'
    bad=mount(angle); bad[2,3]+=.001
    assert check_alignment(robot,tool,.086,bad)['status']=='FAIL'


def inputs():
    scene=dict(config_sha256={'config':'hash'},base_position_m=[0,0,0],T_W_D=np.eye(4).tolist(),
        boxes=[],obstacles=[],scene_fingerprint='synthetic',
        source_equipment_config={'task':{'initial_seed_q_rad':[0.]*6}})
    tool=dict(mounting_evidence=dict(official_bolt_centers_xy_m=pattern(0).tolist(),
        bolt_centers_cad_xz_m=pattern(0).tolist(),mount_point_cad_m=[0,.086,0]),T_flange_tool_cad=mount(0).tolist())
    return scene,tool


def test_home_identity_rejects_changed_tool_or_equipment():
    scene,tool=inputs(); candidate=make_candidate([.1]*6,tool,scene,{'source':'synthetic_test'})
    assert verify_candidate(candidate,tool,scene)==candidate
    other=deepcopy(tool); other['T_flange_tool_cad'][0][3]=.001
    with pytest.raises(ValueError,match='TOOL_OR_EQUIPMENT'): verify_candidate(candidate,other,scene)
    other=deepcopy(scene); other['base_position_m'][0]=.1
    with pytest.raises(ValueError,match='TOOL_OR_EQUIPMENT'): verify_candidate(candidate,tool,other)


def test_entry_consumes_candidate_and_preserves_rejection_before_bounded_search():
    scene,tool=inputs(); task=NativeTask.__new__(NativeTask)
    task.scene=scene; task.snapshot={'tool':tool}; task.home_checks=[]
    task.robot=SimpleNamespace(joint_limits=np.array([[-2.,2.]]*6))
    task.home_candidate=make_candidate([.1]*6,tool,scene,{'source':'synthetic_test'})
    task.request=lambda q,stage: {'q':q.tolist()}
    seen=[]
    def reply(request): seen.append(request['q']); return {'status':'SUCCESS'}
    task.call=reply; task.check_fk=lambda q: [{'q':q.tolist(),'synthetic':True}]
    q,_=task.select_home(np.eye(4))
    assert np.array_equal(q,[.1]*6) and seen==[[.1]*6]
    task.call=lambda request: {'status':'FAIL','reason':'synthetic_collision'}
    budget=[]
    def search(pose,seed,stage,count): budget.append(count); yield np.array([.2]*6)
    task.ik=search
    q,_=task.select_home(np.eye(4))
    assert budget==[64] and np.array_equal(q,[.2]*6)
    assert task.home_checks[-2]['status']=='REJECTED'
    assert task.home_checks[-2]['native_state']['reason']=='synthetic_collision'
    assert scene['source_equipment_config']['task']['initial_seed_q_rad']==[0.]*6
