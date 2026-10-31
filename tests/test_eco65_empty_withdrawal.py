"""Literal separation cases; no synthetic trajectories or native success claims."""
import numpy as np
import pytest
from unloading_sim.eco65.withdrawal import retreat_candidates


def test_rising_longitudinal_outfeed_changes_top_retreat_goal():
    # Actual failure: receiver 100 mm, 100 mm carton, far belt 150 mm.
    # A gap above the landed top at 200 mm does not cover the 250 mm route top.
    tool=[[0,0,.205],[.15,.1,.350]]
    landed=[[0,0,.1],[.15,.1,.2]]
    future=[[-.9,-.25,.15],[-.2,-.15,.25]]
    rows=retreat_candidates(tool,landed,[0.,0.,1.],outfeed_points=future)
    assert rows[0]['strategy']=='IDEAL_OUTFEED_ENVELOPE_UP'
    assert rows[0]['offset_m']==pytest.approx([0,0,.057])
    assert rows[0]['endpoint_projected_gap_m']==pytest.approx(.012)
    assert rows[0]['box_envelope_scope']=='COMPLETE_IDEAL_OUTFEED_ROUTE'
    # Legacy received-carton candidates remain available for native alternatives.
    assert rows[2]['strategy']=='FACE_NORMAL'
    assert rows[2]['offset_m']==pytest.approx([0,0,.007])


def test_outfeed_retreat_does_not_pull_front_grip_one_metre_along_route():
    rows=retreat_candidates([[0,0,.205]],[[0,0,.1],[.15,.1,.2]],[-1.,0.,0.],
                           outfeed_points=[[-.9,-.2,.25]])
    assert rows[0]['offset_m']==pytest.approx([0,0,.057])
    assert any(r['strategy']=='FACE_NORMAL' for r in rows)
    with pytest.raises(ValueError,match='WITHDRAWAL_OUTFEED_GEOMETRY_INVALID'):
        retreat_candidates([[0,0,.2]],[[0,0,.1]],[0.,0.,1.],outfeed_points=[])


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
