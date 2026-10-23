"""Literal separation cases; no synthetic trajectories or native success claims."""
import numpy as np
import pytest
from unloading_sim.eco65.withdrawal import retreat_candidates


def test_received_box_can_separate_up_when_normal_folds_wrist():
    # 150 mm nominal drop has already placed the box below the lowest tool
    # point. The normal projection still overlaps and requires a longer move.
    tool=[[-.160,-.02,.356],[-.060,.02,.460]]
    box=[[-.140,-.05,.231],[-.065,.05,.331]]
    rows=retreat_candidates(tool,box,[-1.,0.,0.])
    assert [r['strategy'] for r in rows]==['FACE_NORMAL']*2+['EMPTY_TOOL_UP']*2
    assert rows[0]['offset_m']==pytest.approx([-.092,0.,0.])
    assert rows[2]['offset_m']==pytest.approx([0.,0.,.010])
    assert rows[2]['endpoint_projected_gap_m']==pytest.approx(.035)
    for row in rows:
        axis=np.array(row['separating_axis'])
        moved=np.array(tool)+row['offset_m']
        assert (moved@axis).min()-(np.array(box)@axis).max()>=.012-1e-12


def test_upward_retreat_clears_full_tool_not_tcp_and_no_duplicate_top():
    rows=retreat_candidates([[0,0,.20],[.1,.1,.40]],[[0,0,.18],[.15,.1,.28]],[-1.,0.,0.])
    assert rows[2]['offset_m']==pytest.approx([0,0,.092])
    assert len(retreat_candidates([[0,0,.20]],[[0,0,.28]],[0.,0.,1.]))==2
    with pytest.raises(ValueError,match='WITHDRAWAL_GEOMETRY_INVALID'):
        retreat_candidates([],[[0,0,.28]],[0.,0.,1.])
