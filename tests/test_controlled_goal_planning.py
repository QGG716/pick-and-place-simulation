"""Target/degenerate native-output regressions. No SDK, ROS process or hardware."""
import json
import time

import numpy as np
import pytest

from unloading_sim.workbench.controlled.planner import EmptyPlanner,validate_live_goal
from unloading_sim.workbench.controlled.trajectory import Authority


def sample(q=None):
    return dict(source='LIVE',received_s=10.,q=[0.]*6 if q is None else q)


@pytest.mark.parametrize('delta_deg',[0.,.005,-.008,.05,-.05])
def test_current_target_and_encoder_noise_are_not_motion(delta_deg):
    target=[0.]*5+[np.deg2rad(delta_deg)]
    with pytest.raises(ValueError,match='目标与当前实测姿态基本相同'):
        validate_live_goal(sample(),target,now=10.001)


def test_pilz_minimum_and_nearby_goal_are_in_radians():
    with pytest.raises(ValueError,match='最小运动量'):
        validate_live_goal(sample(),[.001]*6,now=10.)
    q,target=validate_live_goal(sample(),[0.]*5+[np.deg2rad(.5)],now=10.001)
    assert q==[0.]*6
    assert target[-1]==pytest.approx(.008726646259971648)
    with pytest.raises(ValueError,match='MAX_10_DEGREES'):
        validate_live_goal(sample(),[np.deg2rad(10.01)]*6,now=10.)


@pytest.mark.parametrize('change,code',[
    ({'source':'REPLAY'},'FRESH_LIVE_START_REQUIRED'),
    ({'received_s':9.8},'FRESH_LIVE_START_REQUIRED'),
    ({'received_s':float('nan')},'FRESH_LIVE_START_REQUIRED'),
    ({'q':[0.]*5},'INVALID_LIVE_START'),
    ({'q':[float('nan')]*6},'INVALID_LIVE_START'),
])
def test_no_motion_diagnostic_does_not_hide_invalid_feedback(change,code):
    with pytest.raises(ValueError,match=code):
        validate_live_goal(dict(sample(),**change),[0.]*6,now=10.)


@pytest.mark.parametrize('points',[
    [],[dict(t=.1,q=[0.]*6,v=[0.]*6,a=[0.]*6)],
])
def test_native_empty_or_single_point_never_mints_execution_permission(tmp_path,monkeypatch,points):
    from unloading_sim import moveit2_backend
    calls=[]
    class FakeNativeClient:
        def __init__(self,*args,**kwargs):pass
        def request(self,request,**kwargs):
            calls.append(request['op'])
            if request['op']=='init':return {'status':'SUCCESS'}
            assert request['op']=='plan', 'No fabricated trajectory may be collision-checked'
            return dict(status='SUCCESS',native_output_status='PASS',points=points)
        def close(self):calls.append('close')
    monkeypatch.setattr(moveit2_backend,'ResidentMoveItClient',FakeNativeClient)
    planner=EmptyPlanner(dict(record_root=str(tmp_path)),None,Authority())
    monkeypatch.setattr(planner,'build',lambda:None)
    monkeypatch.setattr(planner,'binding',lambda:{'test_only':True})
    monkeypatch.setattr(planner,'request',lambda q,**kw:kw)
    planner.init={'op':'init'}
    with pytest.raises(ValueError,match='原生规划器未生成可运动的时间轨迹'):
        planner.plan(dict(sample(),received_s=time.monotonic()),[0.]*5+[np.deg2rad(.5)])
    assert calls==['init','plan','close'] and planner.client is None
    assert not planner.authority._plans
    assert not list(tmp_path.glob('*/preview-NOT-IMPORTABLE.json'))
    recorded=json.loads(next(tmp_path.glob('*/native-plan.json')).read_text())
    assert recorded['points']==points  # Preserve native evidence, never pad/re-time it.
