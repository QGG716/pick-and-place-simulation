import yaml
from pathlib import Path
from plan_m710_xy_continuation import locations
from unloading_sim.joint_search_layout import LayoutCandidate,BASELINE

def test_fine_lateral_positions_keep_all_coarse_positions_and_frozen_height():
    baseline=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/workcells'/BASELINE).read_text())
    source=LayoutCandidate(.6,-.2,.9,.2,profile='low_profile_200_outfeed20_v3')
    rows=locations(source,baseline);points=[tuple(r['base_xyz_a_m'][:2]) for r in rows]
    assert points[:5]==[(.6,-.2),(.8,-.35),(.8,-.2),(.6,-.35),(.4,-.35)]
    coarse={(x,y) for x in (-.6,-.4,-.2,0.,.2,.4,.6,.8) for y in (-.4,-.2,0.,.2,.4)}
    assert set(points)==coarse|{(.8,-.35),(.6,-.35),(.4,-.35)}
    assert len(points)==len(set(points))==43
    assert all(r['base_xyz_a_m'][2]==.9 and r['minimum_surface_height_m']==.2 for r in rows)
