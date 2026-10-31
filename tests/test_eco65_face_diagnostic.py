"""The diagnostic uses native grasp target_face and never restricts the return."""
from copy import deepcopy
from tools.check_eco65_prototype_face_cycle import FaceDiagnostic
from unloading_sim.eco65.coupled_task import CoupledTask


def test_first_face_filter_preserves_next_target_and_original_candidates(monkeypatch):
    source=dict(candidates=[dict(target_face=f) for f in ('top','front','left')],rejected=[])
    monkeypatch.setattr(CoupledTask,'geometry_candidates',lambda self:deepcopy(source))
    task=object.__new__(FaceDiagnostic)
    task.target=dict(id='first');task.first_target_id='first';task.completed_ids=[]
    task.diagnostic_face='front'
    result=task.geometry_candidates()
    assert result['candidates']==[dict(target_face='front')]
    assert result['diagnostic_face_filter']['excluded_candidate_count']==2
    assert len(source['candidates'])==3
    task.target=dict(id='next');task.completed_ids=['first']
    assert task.geometry_candidates()==source
