import copy
import pytest
from unloading_sim.workbench.single_carton.native_flow import ReplayClient,NativeFlowDiverged

def test_changed_middle_path_cannot_reuse_pass():
    request=dict(op='validate',probe_path=[[0.]*6,[.1]*6,[.2]*6])
    replay=ReplayClient(None,None)
    replay.add(dict(calls=[dict(request=request,result=dict(status='SUCCESS',native_valid=True))]))
    changed=copy.deepcopy(request);changed['probe_path'][1][2]+=1
    with pytest.raises(NativeFlowDiverged):replay.request(changed)
    assert replay.request(request)['native_valid']
