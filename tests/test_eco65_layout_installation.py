from copy import deepcopy
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import transform, robot_for, URDF
from unloading_sim.eco65.layout_installation import CONFIG, remount_transforms
from unloading_sim.eco65.home_candidate import equipment_identity
from test_eco65_camera_and_edge_alignment import installation


def source_scene():
    camera, mast, _ = installation()
    return dict(mast=mast,permanent_camera=camera,base_position_m=[-.34,0.,.24],
                enable_hardware=False,config_sha256={},T_W_D=np.eye(4).tolist(),boxes=[],obstacles=[])


@pytest.mark.parametrize('q1',[-2.,0.,1.7])
def test_remount_at_same_raw_q_preserves_camera_and_turns_mast_world_ccw90(q1):
    scene=source_scene();cfg=yaml.safe_load(CONFIG.with_name('eco65_base_mast_remount_v1.yaml').read_text())
    t=remount_transforms(scene,cfg);J=Rotation.from_euler('z',q1).as_matrix()
    assert t['new_base']@np.array([1.,0.,0.])==pytest.approx([-1.,0.,0.],abs=1e-12)
    old=t['old_base']@J@t['old_mast'][:3,:3]
    new=t['new_base']@J@t['new_mast'][:3,:3]
    assert new@old.T==pytest.approx(np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]),abs=1e-12)
    C=np.asarray(scene['permanent_camera']['T_mast_optical'])
    assert new@(t['camera_counterrotation']@C)[:3,:3]==pytest.approx(old@C[:3,:3],abs=1e-12)
    assert t['new_mast'][:3,3]==pytest.approx([0.,0.,.036])
    p=np.r_[t['axis']['point_mast_m'],1.]
    assert t['camera_counterrotation']@p==pytest.approx(p)
    assert np.linalg.inv(t['camera_counterrotation']@scene['permanent_camera']['T_mast_CAD'])@(t['camera_counterrotation']@C)==pytest.approx(np.linalg.inv(scene['permanent_camera']['T_mast_CAD'])@C)


@pytest.mark.parametrize('q1', [-2., 0., 1.7])
def test_current_mast_base_adds_180_to_previous_layout_without_turning_camera(q1):
    scene = source_scene()
    old = remount_transforms(scene, yaml.safe_load(CONFIG.with_name('eco65_base_mast_remount_v1.yaml').read_text()))
    new = remount_transforms(scene, yaml.safe_load(CONFIG.read_text()))
    J = Rotation.from_euler('z', q1).as_matrix()
    a = old['new_base'] @ J @ old['new_mast'][:3, :3]
    b = new['new_base'] @ J @ new['new_mast'][:3, :3]
    assert b @ a.T == pytest.approx(np.diag([-1., -1., 1.]), abs=1e-12)
    C = np.asarray(scene['permanent_camera']['T_mast_optical'])
    assert b @ (new['camera_counterrotation'] @ C)[:3, :3] == pytest.approx(
        a @ (old['camera_counterrotation'] @ C)[:3, :3], abs=1e-12)
    assert new['new_mast'][:3, 3] == pytest.approx(old['new_mast'][:3, 3])
    assert new['new_base'] == pytest.approx(old['new_base'])


def test_fixed_base_yaw_changes_fk_and_initial_state_identity_without_rewriting_q():
    if not URDF.exists(): pytest.skip('official private asset cache unavailable')
    scene=source_scene();changed=deepcopy(scene);changed['base_rpy_rad']=[0.,0.,np.pi]
    tool=dict(T_flange_tcp=np.eye(4).tolist());q=np.array([.4,.2,.6,-.5,.3,.8])
    old=robot_for(tool,scene);new=robot_for(tool,changed)
    expected=transform(scene['base_position_m'])@transform(rotation=np.diag([-1.,-1.,1.]))@transform(-np.array(scene['base_position_m']))
    assert new.fk(q)==pytest.approx(expected@old.fk(q),abs=1e-12)
    assert equipment_identity(scene)!=equipment_identity(changed)


def test_failed_lift_replay_uses_checked_world_instead_of_request_start_height():
    from export_eco65_coupled_replay import transition_failures
    q=[.1,.2,.3,.4,.5,.6]
    def request(h):
        return dict(q_start=q,world=[dict(id='belt',pose=transform([0,0,h]).tolist())],
                    attachment=dict(id='carton'),process_policy=dict(target_id='carton'),
                    mechanism_state=dict(height_m=h))
    before,after,sample=request(.18),request(.43),request(.28)
    before['transition']=dict(after_request=after,sample_requests=[sample])
    failure=dict(status='STATE_REJECTED',failure=dict(pair=['base_link','belt']))
    rows=transition_failures(before,dict(after_check=failure))
    assert len(rows)==1 and rows[0]['state']['height_m']==.43
    assert rows[0]['state']['world']==after['world'] and rows[0]['q']==q
    rows=transition_failures(before,dict(after_check=dict(status='SUCCESS'),
                                        sample_checks=[dict(index=0,check=failure)]))
    assert len(rows)==1 and rows[0]['state']['height_m']==.28
    assert transition_failures(before,dict(after_check=dict(status='SUCCESS')))==[]


def test_native_candidate_deadline_does_not_abort_remaining_task_budget(monkeypatch):
    from time import perf_counter
    from unloading_sim.eco65.coupled_task import CoupledTask
    from unloading_sim.eco65.native_task import NativeTask
    from unloading_sim.moveit2_backend import MoveItUnavailable
    task=object.__new__(CoupledTask)
    task.deadline=perf_counter()+30.;task.connection_deadline=perf_counter()+5.
    task.rpc_seconds=100.
    task.business_interval='OUTBOUND';task.reverse_used={'OUTBOUND':0.}
    def expired(*args):raise MoveItUnavailable('NATIVE_ABSOLUTE_DEADLINE_EXPIRED')
    monkeypatch.setattr(NativeTask,'call',expired)
    original_deadline=task.deadline
    with pytest.raises(ValueError,match='NATIVE_CONNECTION_BUDGET_EXHAUSTED'):
        task.call(dict(op='check_state_offline'))
    assert task.deadline==original_deadline
    with pytest.raises(MoveItUnavailable,match='NATIVE_ABSOLUTE_DEADLINE_EXPIRED'):
        task.call(dict(op='plan'))
    task.connection_deadline=None
    with pytest.raises(MoveItUnavailable,match='NATIVE_ABSOLUTE_DEADLINE_EXPIRED'):
        task.call(dict(op='check_state_offline'))
