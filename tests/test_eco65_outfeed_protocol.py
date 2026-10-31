"""Compact stopped-outfeed diagnostics retain the actual failed sample world."""
from copy import deepcopy
from pathlib import Path
import numpy as np


def test_failed_compact_outfeed_reconstructs_only_named_carton_pose(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    from tools.export_eco65_coupled_replay import transition_failures
    original=np.eye(4).tolist();moved=np.eye(4);moved[0,3]=-.25;moved=moved.tolist()
    request=dict(q_start=[0,1,2,3,4,5],attachment=None,
        world=[dict(id='box',pose=original,size=[.15,.1,.1]),dict(id='wall',pose=original)],
        process_policy=dict(target_id='box',target=dict(pose=original),active_ids=['cup_1']),
        transition=dict(sample_contract='HELD_WORLD_TARGET_POSES_V2',sample_requests=[],
            outfeed=dict(samples=[dict(pose=original),dict(pose=moved)])))
    before=deepcopy(request)
    result=dict(sample_checks=[dict(index=0,check=dict(status='SUCCESS')),
        dict(index=1,check=dict(status='STATE_REJECTED',failure=dict(a='box',b='wall')))])
    failure,=transition_failures(request,result)
    assert failure['state']['world'][0]['pose']==moved
    assert failure['state']['world'][1]==request['world'][1]
    assert failure['state']['q']==request['q_start']
    assert failure['state']['attachment'] is None
    assert failure['source']=='SAME_NATIVE_TRANSITION_SAMPLE_1'
    assert request==before
