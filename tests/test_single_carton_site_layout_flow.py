"""Layout start gates use native validation, never a motion submission."""
from pathlib import Path
import numpy as np
import pytest
from test_single_carton_planner import case
from unloading_sim.workbench.single_carton.site_layout import check_layout_start


@pytest.mark.parametrize('valid',[True,False])
def test_q_start_layout_collision_is_checked_before_any_motion(case,valid):
    p,obs,box,folder,calls,options=case
    layout=dict(id='site_layout_fixture',category='site_layout_cad',pose=np.eye(4).tolist(),size=[.1,.1,.1])
    world=[layout,dict(id=box['id'],pose=box['pose'],size=box['size_m'],category='carton')]
    class Native:
        def request(self,r,**kwargs):
            calls.append(r)
            return dict(native_valid=valid,legacy_intersection_valid=valid,
                failure=None if valid else dict(pair=['link_3',layout['id']],reason='PAIR_CLEARANCE'))
    p.client=Native()
    if valid:assert check_layout_start(p,obs.q_start,world,[box],folder)['native_valid']
    else:
        with pytest.raises(ValueError,match='START_POSE_COLLIDES_WITH_LAYOUT'):
            check_layout_start(p,obs.q_start,world,[box],folder)
    assert calls and all(r['op']=='validate' and r['attachment'] is None for r in calls)
    assert all(r['allowed_pairs']==[['base_link','robot_mount_plate']] for r in calls)
    assert not p.authority._plans


def test_unregistered_cad_cannot_enter_planning(case):
    p,obs,box,folder,calls,_=case
    obs.frame.observation_id='layout-unregistered'
    scene=dict(observation_id=obs.frame.observation_id,world=[dict(id='cad',category='site_layout_cad',pose=np.eye(4).tolist(),size=[.1]*3)])
    with pytest.raises(ValueError,match='LAYOUT_REGISTRATION_MISMATCH'):p.plan_cycle(obs,[box],folder,scene=scene)
    assert not calls and not p.authority._plans
