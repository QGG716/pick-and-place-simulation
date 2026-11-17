"""Python reference for the same seven limits cases in native_review_contracts.cpp."""
import pytest
from unloading_sim.workbench.controlled.trajectory import Limits,prepare

@pytest.mark.parametrize('case',range(7))
def test_invalid_stationary_limits_match_native_rejections(case):
    lower=[-1.]*6;upper=[1.]*6;velocity=[.1]*6;acceleration=[.2]*6
    if case==0:lower[0]=upper[0]=0.
    if case==1:lower[0]=float('-inf')
    if case==2:upper[0]=float('nan')
    if case==3:velocity[0]=0.
    if case==4:acceleration[0]=-1.
    if case==5:lower.pop()
    if case==6:velocity[0]=float('inf')
    points=[dict(t=t,q=[0.]*6,v=[0.]*6,a=[0.]*6) for t in (0.,1.)]
    with pytest.raises(ValueError,match='INVALID_LIMITS'):
        prepare(points,Limits(lower,upper,velocity,acceleration),1.,.005,1.)
