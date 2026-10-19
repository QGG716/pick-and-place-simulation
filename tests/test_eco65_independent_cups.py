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


def test_front_extraction_uses_bounded_normal_offsets_and_neighbor_height(monkeypatch):
    from unloading_sim.eco65 import native_task
    from unloading_sim.eco65.model import transform
    monkeypatch.setattr(native_task,'world_objects',lambda scene:[])
    scene=dict(target_box_id='target',support_surface_ids=[],boxes=[
        dict(id='target',pose=[.205,0,.53],size_m=[.15,.1,.1]),
        dict(id='neighbor',pose=[.205,.11,.53],size_m=[.15,.1,.1])],
        native_task=dict(minimum_lift_m=.01,receiving=dict(box_pose_D=np.eye(4).tolist())))
    target=transform([.205,0,.53]);tcp=transform([.13,0,.58])
    goals=native_task.extraction_candidates(scene,target,[.15,.1,.1],tcp,'front')
    assert len(goals)==10
    assert {round(e['lift_m'],5) for _,e in goals}=={.01,.06,.11}
    assert {round(e['outward_m'],5) for _,e in goals}=={.01,.0475,.085,.16}
    for pose,e in goals:
        assert np.allclose(pose[:3,:3],tcp[:3,:3])
        assert pose[0,3]<tcp[0,3] and pose[2,3]>tcp[2,3] and pose[1,3]==tcp[1,3]
        assert e['fully_before_original_front']==(e['outward_m']>=.16-1e-9)
