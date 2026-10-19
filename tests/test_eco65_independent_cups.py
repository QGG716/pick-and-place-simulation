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
