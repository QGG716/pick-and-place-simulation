import numpy as np
from prepare_eco65_current_tcp_calibration import board_capture_projection


def test_whole_board_must_fit_even_when_center_is_visible():
    camera=np.eye(4);board=np.eye(4);board[:3,3]=[-.075,.2,.5]
    K=[[500.,0.,320.],[0.,500.,240.],[0.,0.,1.]]
    # Centre is at v=502.5 and lower edge v=565: a cropped board is rejected.
    cropped=board_capture_projection(camera,K,[640,480],board,[.15,.125])
    assert not cropped['accepted']
    assert np.allclose(cropped['bounds_px'],[[245.,440.],[395.,565.]])
    board[1,3]=.12  # Centre v=422.5 is visible, but bottom v=485 is still outside.
    assert not board_capture_projection(camera,K,[640,480],board,[.15,.125])['accepted']
    board[1,3]=-.0625
    visible=board_capture_projection(camera,K,[640,480],board,[.15,.125])
    assert visible['accepted']
    assert np.allclose(visible['bounds_px'],[[245.,177.5],[395.,302.5]])


def test_behind_camera_board_is_rejected():
    board=np.eye(4);board[2,3]=-.5
    result=board_capture_projection(np.eye(4),np.eye(3),[640,480],board,[.15,.125])
    assert result==dict(accepted=False,reason='BOARD_BEHIND_CAMERA')
