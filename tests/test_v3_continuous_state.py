import numpy as np

from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_scenes import box
from tools import run_m710id70_v3 as runner


def test_scheduler_tries_third_candidate_and_retains_released_box(tmp_path,monkeypatch):
    cfg=load_validation_config()
    cartons=[box(name,[1.3,y,.15],[.6,.4,.3]) for name,y in [('a',-.7),('b',0),('c',.7)]]
    attempted=[]
    class FakeRun:
        output=tmp_path
        def __init__(self):self.cfg=cfg
        def batch(self,jobs):
            results=[]
            for job in jobs:
                name=job[2].name;attempted.append(name);success=name=='c'
                selected=None if not success else {'cycle_s':1.,'placed_box_pose':np.eye(4).tolist(),
                    'final_q':cfg.data['robot']['home_joints'],'state':[0,.2]}
                results.append({'box':name,'seed':job[6],'mode':'dynamic','grasp_reachable':success,
                    'extraction_feasible':success,'geometric_feasible':success,'payload_qualified':False,
                    'dynamics_verified':False,'load_status':'FAIL','failure_stage':'complete' if success else 'approach',
                    'failure_reason':'OK' if success else 'DISTINCT_FAILED_ATTEMPT','attempts':[{'reason':'OK' if success else name}],
                    'selected':selected})
            return results
    monkeypatch.setattr(runner,'clear_receiving_area',lambda *args:(None,{'reason':'L_CORNER_SUPPORT_GAP'}))
    summary,rows=runner.continuous(FakeRun(),'scheduler_test',cartons,0)
    assert attempted==['a','b','c']
    assert summary['geometric_cycles']==1
    assert summary['received_boxes_retained']==1
    assert summary['remaining_boxes']==2
    assert summary['events'][-1]['retained_box']=='received_c'
    failures=[row for row in rows if row.get('failure_reason')=='DISTINCT_FAILED_ATTEMPT']
    assert len(failures)==2
    assert {next(iter(row['candidate_failure_counts'])) for row in failures}=={'a','b'}
