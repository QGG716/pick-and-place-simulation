from copy import deepcopy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import transform
from unloading_sim.eco65.camera_installation import rotate_camera,camera_world_transform
from unloading_sim.eco65.native_contact import check_contours, EDGE_ALIGNMENT
from eco65_camera_mount import mount_check
from eco65_mast_attachment import evaluate


def installation():
    optical=transform([.141,.029,.468],[[1,0,0],[0,-.5,np.sqrt(3)/2],[0,-np.sqrt(3)/2,-.5]])
    cad=optical@transform([.02,.03,.04])
    camera=dict(T_mast_CAD=cad.tolist(),T_mast_optical=optical.tolist(),
        mount=dict(downward_pitch_deg=30.,pivot_mast_m=[.113,-.00405,.46],
                   axis_mast=[1,0,0],connections=[]),
        pitch_adapter=[dict(name='pitch_riser_'+side,size_m=[.002,.024,.011],
            T_mast=transform([x,-.00405,.4555]).tolist())
            for side,x in [('left',.0705),('right',.1555)]])
    mast=dict(T_link1_mast=transform([0,0,.036]).tolist(),parts=[dict(id='mast_part_019',
        visual_sha256='synthetic',planar_faces=[dict(point_m=[.1045,-.00405,.008],
            normal=[0,0,-1],area_m2=np.pi*.0125**2)])],
        attachment='link_1',arm_forward_link1=[-1,0,0],relative_heading_deg=-90,
        T_mast_camera=optical.tolist())
    config=dict(schema='eco65_camera_axis_yaw_v1',yaw_deg=180.,
        downward_pitch_deg=30.,
        expected_axis_point_mast_m=[.1045,-.00405,.008],
        heading_from_link1_minus_X_deg=90.,calibration_status='NOMINAL_NOT_CALIBRATED',
        observation_source='FROZEN_SCENE')
    return camera,mast,config


def test_pole_axis_rotation_keeps_pitch_optical_relation_and_actual_diagnostic():
    old,mast,config=installation()
    new=rotate_camera(old,mast,config)
    A=np.array(new['axis_rotation']['T_new_from_old_mast'])
    for z in (.008,.20,.45):
        p=[.1045,-.00405,z,1.]
        assert A@p==pytest.approx(p)
    assert np.linalg.inv(new['T_mast_CAD'])@new['T_mast_optical']==pytest.approx(
        np.linalg.inv(old['T_mast_CAD'])@old['T_mast_optical'])
    for q1 in (-1.,0.,1.):
        L=transform([-.34,0,.24],Rotation.from_euler('z',q1).as_matrix())
        C=L@new['T_link1_optical']
        check=mount_check(C,L,new)
        assert check['accepted'] and check['downward_pitch_deg']==pytest.approx(30.)
        assert check['signed_heading_from_link1_minus_X_deg']==pytest.approx(90.)
        assert evaluate(L,mast,new)['T_D_camera']==pytest.approx(C)
    assert [r['size_m'][0] for r in new['pitch_adapter']]==pytest.approx([.019,.019])
    assert new['calibration']['prior_calibration_valid'] is False
    scene=dict(permanent_camera=new,camera_extrinsics=new['calibration'])
    assert camera_world_transform(scene,np.eye(4))==pytest.approx(np.array(new['T_link1_optical']))
    old_extrinsics=deepcopy(new['calibration']);old_extrinsics['T_parent_child']=old['T_mast_optical']
    with pytest.raises(ValueError,match='CURRENT_INSTALLATION'):
        camera_world_transform(scene,np.eye(4),old_extrinsics)
    altered=deepcopy(new);altered['mount']['heading_from_link1_minus_X_deg']=-90
    assert not mount_check(new['T_link1_optical'],np.eye(4),altered)['accepted']


@pytest.mark.parametrize('box_yaw',[0.,.73])
@pytest.mark.parametrize('roll,expected',[(0.,'PASS'),(np.pi/2,'PASS'),(np.pi/4,'FAIL')])
@pytest.mark.parametrize('face',['top','front'])
def test_actual_contact_uses_rigid_plate_edges_in_rotated_box_frame(box_yaw,roll,expected,face):
    rows=[]
    for i in range(12):
        x,z=(i%4-1.5)*.005,(i//4-1)*.005
        rows.append(dict(id=str(i),normal=[0,-1,0],
            points_m=[[x+a,0,z+b] for a,b in [(-.001,-.001),(.001,-.001),(.001,.001),(-.001,.001)]]))
    F=transform([.01,0,.08],Rotation.from_euler('z',.013).as_matrix())
    g=dict(geometry_type='cad_seal_contours_v1',contours=rows,expected_contour_ids=[r['id'] for r in rows],
           flange_from_contact_frame=F.tolist(),edge_alignment=deepcopy(EDGE_ALIGNMENT))
    target=transform([.2,.1,.53],Rotation.from_euler('z',box_yaw).as_matrix())
    base=np.array([[1,0,0],[0,0,-1],[0,1,0]]) if face=='top' else np.array([[0,-1,0],[1,0,0],[0,0,1]])
    R=Rotation.from_euler('z' if face=='top' else 'x',roll).as_matrix()@base
    actual_flange=target@transform([0,0,.05] if face=='top' else [-.075,0,0],R)@np.linalg.inv(F)
    result=check_contours(g,actual_flange,target,[.15,.1,.1],gap=.0004,penetration=.0004,face=face)
    assert len(result['actual_sealed_ids'])==12  # 45 degrees also seals, but is rejected.
    assert result['status']==expected
    assert result['edge_alignment']['maximum_angle_error_rad']==pytest.approx(0 if expected=='PASS' else np.pi/4,abs=2e-8)
