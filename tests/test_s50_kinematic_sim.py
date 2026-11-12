"""Focused checks for the explicit simulation entry, not model certification."""
from pathlib import Path
import numpy as np
import pytest

from unloading_sim.s50_kinematic_sim import make_context
from unloading_sim.s50_cad_model import poe_fk
from unloading_sim.robot_replacement_layout import freeze_non_robot_scene, frozen_baseline_layout, compare_non_robot_scenes

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def context(tmp_path_factory):
    return make_context(ROOT,tmp_path_factory.mktemp('s50-kinematic-sim'))


def test_explicit_simulation_limits_mount_and_single_tool_extension(context):
    c=context
    np.testing.assert_allclose(np.rad2deg(c.robot.joint_limits),
        [[-360,360],[-100,100],[-160,160],[-360,360],[-360,360],[-360,360]])
    np.testing.assert_allclose(c.world[:3,3],[-1.85,.15,.9],atol=1e-12)
    np.testing.assert_allclose(c.world[:3,:3]@np.array([1.,0,0]),[0,1,0],atol=1e-12)
    np.testing.assert_allclose(c.frames['nominal_compressed_contact'][:3,3],[0,0,.4175],atol=1e-12)
    np.testing.assert_allclose(c.frames['virtual_task_tcp'][:3,3],[0,0,.45],atol=1e-12)
    assert c.intrinsic['connector_extension_m']==.2
    assert c.intrinsic['tool_mass_kg']==20
    assert c.intrinsic['carton_mass_kg']==42.5
    assert not any(c.settings['qualification'].values())
    original=freeze_non_robot_scene(frozen_baseline_layout(ROOT))
    assert compare_non_robot_scenes(original,c.frozen)['passed']
    assert len(c.frozen['initial_carton_ids'])==40


def test_core_fk_matches_independent_cad_axis_product(context):
    c=context
    for q in [np.zeros(6),[.1,.3,-.4,.2,.5,-.2],[.9,.45,-1.7,-1.8,-.6,3.14],[-1.,-.5,2.,.2,-2.,-3.]]:
        independent=poe_fk(c.definition,q,c.world)
        core=c.robot.named_link_frames(q)
        for name in independent:
            np.testing.assert_allclose(core[name],independent[name],atol=2e-12)
        np.testing.assert_allclose(c.robot.fk(q),independent['s50_flange']@c.frames['nominal_compressed_contact'],atol=2e-12)


def test_real_mesh_negative_contact_is_rejected(context):
    pytest.importorskip('pinocchio');pytest.importorskip('coal')
    from unloading_sim.s50_sim_collision import SimCollision
    checker=SimCollision(context)
    # Retained failed release: raising the real transverse belt intersects link 2.
    q=np.array([.063180399745377,.936457110686790,.393259557142224,-1.32968684960525,-1.63397235885763,.00000172470245])
    assert not checker.check(q,height=1.5,phase='regression_negative_coal_contact')
    assert checker.last['pair']==['s50_link_2','conveyor_transverse']
    assert any(d<0 for d in checker.last['raw_contact_depths_m'])
    assert checker.check(q,height=.6,phase='regression_clear_lower_belt')
