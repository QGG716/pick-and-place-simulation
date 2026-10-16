import json
import numpy as np
import unittest
import tempfile
from pathlib import Path
import importlib.util
from scipy.spatial.transform import Rotation
from unloading_sim.touch_calibration import rigid,inv,points,register_points,fuse,intrinsics,OPTICAL_FROM_USD,camera_from_board,solve


def T(p,rot):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rot).as_matrix();t[:3,3]=p;return t


def check_planar_registration_and_absolute_extrinsics_recovery():
    local=np.array([[0,0,0],[.2,0,0],[0,.15,0],[.2,.15,0],[.1,0,0],[.1,.15,0]])
    P=T([.3,-.1,.8],[.2,-.4,.7]);X=T([.1,.02,.45],[.6,-.2,.3])
    recovered,report=register_points(local,points(P,local));assert report['max_m']<1e-14
    estimates=[]
    for j1 in [-.4,.1,.7]:
        L=T([.02,-.3,.1],[0,0,j1]);C=inv(L@X)@P
        estimates.append(inv(L)@recovered@inv(C))
    fitted,errors=fuse(estimates)
    assert np.max(abs(fitted-X))<1e-14
    assert abs(np.linalg.det(fitted[:3,:3])-1)<1e-14


def check_near_collinear_rejected_but_planar_valid():
    p=np.array([[0,0,0],[.1,1e-7,0],[.2,0,0],[.3,1e-7,0]])
    with unittest.TestCase().assertRaisesRegex(ValueError,'COLLINEAR'):register_points(p,p)


def check_invalid_correspondence_and_reflection():
    with unittest.TestCase().assertRaisesRegex(ValueError,'CORRESPONDENCES'):register_points([[0,0,0]],[[0,0,0]])
    with unittest.TestCase().assertRaisesRegex(ValueError,'RIGID'):rigid(np.diag([-1,1,1,1]))


def check_pixel_centres_fov_and_optical_usd_direction():
    K=intrinsics();assert K[0,0]!=K[1,1];assert np.array_equal(K[:2,2],[319.5,239.5])
    rays=np.linalg.inv(K)@np.array([[-.5,639.5,319.5,319.5],[239.5,239.5,-.5,479.5],[1,1,1,1]])
    assert abs(np.degrees(np.arctan(rays[0,1])-np.arctan(rays[0,0]))-64)<1e-12
    assert abs(np.degrees(np.arctan(rays[1,3])-np.arctan(rays[1,2]))-50)<1e-12
    assert np.array_equal(points(OPTICAL_FROM_USD,[0,0,-1]),[0,0,1])
    assert np.array_equal(points(OPTICAL_FROM_USD,[0,1,-1]),[0,-1,1])


def check_wrong_capture_joint_and_optical_flip_have_detectable_error():
    X=T([.1,.02,.45],[.6,-.2,.3]);P=T([.3,-.1,.8],[.2,-.4,.7]);L=T([0,0,.2],[0,0,.2]);C=inv(L@X)@P
    wrong=inv(T([0,0,.2],[0,0,.5]))@P@inv(C)
    assert Rotation.from_matrix(X[:3,:3].T@wrong[:3,:3]).magnitude()>.2
    flipped=inv(L)@P@inv(OPTICAL_FROM_USD@C)
    assert Rotation.from_matrix(X[:3,:3].T@flipped[:3,:3]).magnitude()>3


def check_observation_schema_rejects_truth_injection(tmp_path):
    require_cv2()
    (tmp_path/'observations.json').write_text(json.dumps({'schema':'known_tip_touch_observations_v1','truth':{'T_L_C':np.eye(4).tolist()}}))
    with unittest.TestCase().assertRaisesRegex(ValueError,'OBSERVATIONS_ONLY'):solve(tmp_path,tmp_path/'result')


def check_insufficient_image_features_rejected():
    cv2=require_cv2();board=cv2.aruco.CharucoBoard((6,5),.025,.018,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    config=dict(dictionary='DICT_4X4_50',squares=[6,5],square_m=.025,marker_m=.018,charuco_points_P_m=board.getChessboardCorners().tolist())
    with unittest.TestCase().assertRaisesRegex(ValueError,'INSUFFICIENT_VISUAL'):camera_from_board(np.zeros((480,640,3),np.uint8),config,intrinsics(),{'minimum_charuco_corners':8})


def check_board_placement_mismatch_rejected(tmp_path):
    require_cv2()
    obs=dict(schema='known_tip_touch_observations_v1',kinematics={},camera={},board={'touch_points':[]},probe={'p_F_tip_m':[0,0,0]},photos=[{'board_placement_id':'wrong','split':'train'}],touches=[],thresholds={},source='SIMULATION',identifiers={})
    (tmp_path/'observations.json').write_text(json.dumps(obs))
    with unittest.TestCase().assertRaisesRegex(ValueError,'BOARD_PLACEMENT'):solve(tmp_path,tmp_path/'result')


def require_cv2():
    if importlib.util.find_spec('cv2') is None:raise unittest.SkipTest('OpenCV unavailable in this environment')
    import cv2
    return cv2


class TouchCalibrationTests(unittest.TestCase):
    pass


def bind(fn):
    def run(self):
        if fn.__code__.co_argcount:
            with tempfile.TemporaryDirectory() as folder:fn(Path(folder))
        else:fn()
    return run


for name,fn in list(globals().items()):
    if name.startswith('check_'):setattr(TouchCalibrationTests,name.replace('check_','test_',1),bind(fn))
