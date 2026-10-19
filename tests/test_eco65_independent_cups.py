import numpy as np
import pytest
from unloading_sim.eco65.native_contact import check_contours
from unloading_sim.eco65.native_cups import grasp_candidates


def geometry():
    rows=[]
    for i in range(12):
        x=(i%4-1.5)*.04;y=(i//4-1)*.04
        rows.append(dict(id=f'cup_{i}',normal=[0,0,1],points_m=[[x+a,y+b,0] for a,b in [(-.012,-.012),(.012,-.012),(.012,.012),(-.012,.012)]]))
    return dict(geometry_type='cad_seal_contours_v1',expected_contour_ids=[r['id'] for r in rows],contours=rows,flange_from_contact_frame=np.eye(4).tolist())


@pytest.mark.parametrize('face',['top','front'])
def test_real_polygon_subset_placements_use_face_normal(face):
    g=geometry();size=[.15,.10,.10]
    result=grasp_candidates(g,np.eye(4),size,np.eye(4),[-.3,0,0],target_id='target',faces=[face])
    assert result['candidates']
    c=result['candidates'][0]
    assert 0<len(c['active_ids'])<12
    report=check_contours(g,c['tcp_pose_D'],np.eye(4),size,gap=.0004,penetration=.0004,face=face,active_ids=c['active_ids'])
    assert report['status']=='PASS' and report['checked_count']==12
    assert check_contours(g,c['tcp_pose_D'],np.eye(4),size,gap=.0004,penetration=.0004,face=face)['status']=='FAIL'
    moved=np.array(c['tcp_pose_D']);moved[2 if face=='top' else 0,3]+=.03
    assert check_contours(g,moved,np.eye(4),size,gap=.0004,penetration=.0004,face=face,active_ids=c['active_ids'])['status']=='FAIL'


@pytest.mark.parametrize('active',[[],['unknown'],['cup_0','cup_0']])
def test_invalid_enabled_set_rejected(active):
    with pytest.raises(ValueError,match='CONTACT_ACTIVE_SELECTION_INVALID'):
        check_contours(geometry(),np.eye(4),np.eye(4),[.15,.1,.1],gap=.0004,penetration=.0004,active_ids=active)


@pytest.mark.parametrize('face,count',[('front',13),('top',4)])
def test_extraction_uses_bounded_normal_offsets_and_neighbor_height(monkeypatch,face,count):
    from unloading_sim.eco65 import native_task
    from unloading_sim.eco65.model import transform
    monkeypatch.setattr(native_task,'world_objects',lambda scene:[])
    scene=dict(target_box_id='target',support_surface_ids=[],boxes=[
        dict(id='target',pose=[.205,0,.53],size_m=[.15,.1,.1]),
        dict(id='neighbor',pose=[.205,.11,.53],size_m=[.15,.1,.1])],
        native_task=dict(minimum_lift_m=.01,cup_selection_mode=native_task.INDEPENDENT,
                         receiving=dict(box_pose_D=np.eye(4).tolist())))
    target=transform([.205,0,.53]);tcp=transform([.13,0,.58])
    goals=native_task.extraction_candidates(scene,target,[.15,.1,.1],tcp,face)
    assert len(goals)==count
    assert {round(e['lift_m'],5) for _,e in goals}=={.01,.06,.08,.11}
    assert {round(e['outward_m'],5) for _,e in goals}==({.01,.0475,.085,.16} if face=='front' else {0.})
    for pose,e in goals:
        assert np.allclose(pose[:3,:3],tcp[:3,:3])
        assert (pose[0,3]<tcp[0,3] if face=='front' else pose[0,3]==tcp[0,3])
        assert pose[2,3]>tcp[2,3] and pose[1,3]==tcp[1,3]
        assert e['fully_before_original_front']==(e['outward_m']>=.16-1e-9)


def test_shorter_approach_restarts_from_home_without_retaining_failed_path(monkeypatch):
    from types import SimpleNamespace
    from unloading_sim.eco65 import native_task
    task=object.__new__(native_task.NativeTask)
    task.scene=dict(native_task=dict(preapproach_m=.03,preplace_m=.03,
        cup_selection_mode=native_task.INDEPENDENT,receiving=dict(box_pose_D=np.eye(4).tolist())))
    task.target_pose=np.eye(4);task.target=dict(id='target',size_m=[.15,.1,.1])
    task.robot=SimpleNamespace(fk=lambda q:np.eye(4),named_link_frames=lambda q:{'link_6':np.eye(4)})
    task.init=dict(process_geometry={});task.snapshot=dict(tool=dict(contact=dict(normal_tolerance_rad=.01)))
    task.identity=dict(flange_from_task_tcp=np.eye(4));task.face='front';task.active_ids=['cup_0']
    task.parent='';task.stages=[];task.events=[];task.records=[];task.approach_attempts=[]
    task.ik=lambda *args:iter([np.ones(6)]);task.select_grasp=lambda grasp:None
    task.selection_binding=lambda:{}
    monkeypatch.setattr(native_task,'check_contours',lambda *args,**kwargs:dict(status='PASS',actual_sealed_ids=['cup_0']))
    def stop_after_contact(*args):raise RuntimeError('CONTACT_TEST_COMPLETE')
    monkeypatch.setattr(native_task,'extraction_candidates',stop_after_contact)
    home=np.zeros(6);top=np.eye(4);top[:3,3]=[.2,0,.3];calls=[]
    def motion(q,stage,*,pose,**kwargs):
        offset=round(.2-pose[0,3],6) if stage=='pregrasp' else calls[-1][1]
        calls.append((stage,offset,np.array(q),task.parent))
        if stage=='pregrasp':
            assert np.array_equal(q,home) and task.parent=='' and task.stages==[]
        if (stage,offset) in [('pregrasp',.03),('contact',.025)]:
            raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:'+stage)
        task.parent=f'{stage}-{offset}';task.stages.append(task.parent)
        return np.full(6,offset)
    task.motion=motion
    with pytest.raises(RuntimeError,match='CONTACT_TEST_COMPLETE'):
        task.run_grasp(dict(tcp_pose_D=top),home)
    assert [a['status'] for a in task.approach_attempts]==['REJECTED','REJECTED','HOME_APPROACH_AND_CONTACT_PASS']
    assert task.stages==['pregrasp-0.01','contact-0.01']
    assert len(task.events)==1 and task.events[0]['after_stage']=='contact-0.01'
