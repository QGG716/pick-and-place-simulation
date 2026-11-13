"""Focused checks for the explicit simulation entry, not model certification."""
from pathlib import Path
import numpy as np
import pytest

from unloading_sim.s50_kinematic_sim import make_context,cad_coordinates,simulation_coordinates,simulation_layout
from unloading_sim.s50_cad_model import poe_fk
from unloading_sim.robot_replacement_layout import freeze_non_robot_scene, frozen_baseline_layout, compare_non_robot_scenes

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def context(tmp_path_factory):
    # Keep literal checks on the retained 105 mm baseline; the current default
    # with its independently checked 80 mm flange spacer is exercised below.
    return make_context(ROOT,tmp_path_factory.mktemp('s50-kinematic-sim'),ROOT/'configs/robots/huayan_s50_short105_kinematic_sim.yaml')


def test_explicit_simulation_limits_mount_and_single_tool_extension(context):
    c=context
    assert c.definition['source_step_path']=='assets/robots/huayan/S50_1.STEP'
    assert c.definition['source_step_sha256']=='2f3d1d987c146ba9ba59377377f2590882c664cdd5117912b81eea47e242973b'
    assert all('/huayan/s50_1_kinematic/' in link['visual_mesh']['path'] for link in c.definition['links'])
    np.testing.assert_allclose(np.rad2deg(c.robot.joint_limits),
        [[-360,360],[-190,10],[-160,160],[-360,360],[-360,360],[-360,360]])
    np.testing.assert_allclose(c.world[:3,3],[-1.25,.15,.9],atol=1e-12)
    np.testing.assert_allclose(c.world[:3,:3]@np.array([1.,0,0]),[0,1,0],atol=1e-12)
    np.testing.assert_allclose(c.frames['nominal_compressed_contact'][:3,3],[0,0,.1725],atol=1e-12)
    np.testing.assert_allclose(c.frames['virtual_task_tcp'][:3,3],[0,0,.205],atol=1e-12)
    assert c.intrinsic['connector_extension_m']==0
    assert c.intrinsic['tool_mass_kg']==20
    # Every S50 config uses the S50 carton mass; FANUC keeps 42.5 kg.
    assert c.settings['carton_payload_profile']=='S50_ROBOT_CONFIG'
    assert c.intrinsic['carton_mass_kg']==30.0
    assert c.intrinsic['inherited_carton_mass_kg']==42.5
    assert not any(c.settings['qualification'].values())
    original=freeze_non_robot_scene(simulation_layout(ROOT,c.settings))
    assert original['height_samples']==c.frozen['height_samples']
    assert original['initial_cartons']==c.frozen['initial_cartons']
    assert len(c.frozen['initial_carton_ids'])==40


def test_core_fk_matches_independent_cad_axis_product(context):
    c=context
    for q in [np.zeros(6),[.1,.3,-.4,.2,.5,-.2],[.9,.45,-1.7,-1.8,-.6,3.14],[-1.,-.5,2.,.2,-2.,-3.]]:
        independent=poe_fk(c.definition,cad_coordinates(c.settings,q),c.world)
        core=c.robot.named_link_frames(q)
        for name in independent:
            np.testing.assert_allclose(core[name],independent[name],atol=2e-12)
        np.testing.assert_allclose(c.robot.fk(q),independent['s50_flange']@c.frames['nominal_compressed_contact'],atol=2e-12)


def test_researched_family_coordinates_preserve_the_actual_cad_assembly(context):
    reference=np.deg2rad([0,-90,0,-90,0,0])
    np.testing.assert_allclose(cad_coordinates(context.settings,reference),np.zeros(6),atol=1e-14)
    np.testing.assert_allclose(simulation_coordinates(context.settings,np.zeros(6)),reference,atol=1e-14)
    independent=poe_fk(context.definition,np.zeros(6),context.world)
    actual=context.robot.named_link_frames(reference)
    for name in independent:np.testing.assert_allclose(actual[name],independent[name],atol=2e-12)
    # Published shoulder limits map to the two opposite 100-degree CAD sweeps.
    for published,cad in [(-190,100),(10,-100)]:
        q=reference.copy();q[1]=np.deg2rad(published)
        assert np.isclose(np.rad2deg(cad_coordinates(context.settings,q)[1]),cad)


def test_real_mesh_negative_contact_is_rejected(context):
    pytest.importorskip('pinocchio');pytest.importorskip('coal')
    from unloading_sim.s50_sim_collision import SimCollision
    legacy_settings={k:v for k,v in context.settings.items() if k not in ('tool_mount_profile','layout_gap_m')}
    legacy=make_context(ROOT,context.output/'legacy_connector',legacy_settings)
    checker=SimCollision(legacy)
    # Retained failed release: raising the real transverse belt intersects link 2.
    q=simulation_coordinates(context.settings,np.array([.063180399745377,.936457110686790,.393259557142224,-1.32968684960525,-1.63397235885763,.00000172470245]))
    assert not checker.check(q,height=1.5,phase='regression_negative_coal_contact')
    assert checker.last['pair']==['s50_link_2','conveyor_transverse']
    assert any(d<0 for d in checker.last['raw_contact_depths_m'])
    assert checker.check(q,height=.6,phase='regression_clear_lower_belt')


def test_s50_direct_mount_keeps_fanuc_connector_and_stable_body_ids(context):
    original=freeze_non_robot_scene(frozen_baseline_layout(ROOT))['tool_intrinsic']
    direct=context.intrinsic
    assert original['connector_extension_m']==.2
    assert len(original['rigid_local_boxes_virtual_tcp_m'])==130
    assert len(direct['rigid_local_boxes_virtual_tcp_m'])==130
    assert direct['removed_body_ids']==['tool_rigid_11','tool_rigid_12','tool_rigid_13']
    keep=[i for i in range(130) if i not in (11,12,13)]
    assert direct['rigid_body_ids'][:127]==[f'tool_rigid_{i}' for i in keep]
    assert direct['rigid_body_ids'][127:]==['tool_s50_mount_side_1','tool_s50_mount_side_2','tool_s50_mount_flange']
    np.testing.assert_array_equal(direct['rigid_local_boxes_virtual_tcp_m'][:127],np.array(original['rigid_local_boxes_virtual_tcp_m'])[keep])
    np.testing.assert_array_equal(direct['compliant_local_boxes_virtual_tcp_m'],original['compliant_local_boxes_virtual_tcp_m'])
    assert len(direct['cup_body_ids'])==72
    # The retained backplate rear is 105 mm ahead of the flange. The new
    # mount rear meets the flange; the 200 mm extension is absent.
    row=np.array(direct['rigid_local_boxes_virtual_tcp_m'][0]);T=context.frames['virtual_task_tcp']
    rear=T[:3,:3]@(row[:3]-[0,0,row[5]/2])+T[:3,3]
    np.testing.assert_allclose(rear,[0,0,.105],atol=1e-8)
    mount=np.array(direct['rigid_local_boxes_virtual_tcp_m'][-1])
    np.testing.assert_allclose(T[:3,:3]@(mount[:3]-[0,0,mount[5]/2])+T[:3,3],[0,0,0],atol=1e-8)
    assert np.isclose(direct['axial_shortening_from_inherited_mount_m'],.245)


def test_face_run_stops_at_the_centre_target_without_trying_neighbours(context):
    from types import SimpleNamespace
    from unloading_sim.s50_face_kinematic import FaceRun

    # Exercise task ordering only; the actual geometry is covered by the retained
    # centre-carton run. A failed target must not alter the 40-carton scene.
    run=FaceRun.__new__(FaceRun)
    run.active={b.name:b for b in context.layout.cartons()}
    run.completed=[];run.report={};run.q=np.zeros(6);run.target=None
    run.c=SimpleNamespace(settings=dict(context.settings),layout=context.layout);run.checker=SimpleNamespace(last=None)
    attempted=[]
    def fail_current_target():
        attempted.append(run.target.name)
        raise RuntimeError('TEST_EXACT_TARGET_FAILED')
    run.plan_carton=fail_current_target
    run.change_height=lambda height: None
    run.event=lambda *args,**kwargs: None
    run.checkpoint=lambda: None
    result=run.run()
    assert attempted==['carton_l07_c02']
    assert result['status']=='KINEMATIC_FACE_ATTEMPT_FAILED'
    assert result['failure']['reason']=='TEST_EXACT_TARGET_FAILED'
    assert result['failure']['target']=='carton_l07_c02'
    assert len(run.active)==40
    assert run.completed==[]


def test_flange_spacer_moves_wide_plate_and_counts_axial_length_once(context):
    c=make_context(ROOT,context.output/'spacer80')
    np.testing.assert_allclose(c.frames['nominal_compressed_contact'][:3,3],[0,0,.2525],atol=1e-12)
    np.testing.assert_allclose(c.frames['virtual_task_tcp'][:3,3],[0,0,.285],atol=1e-12)
    assert c.intrinsic['connector_extension_m']==0
    assert len(c.intrinsic['rigid_body_ids'])==131
    for name,low_z,high_z in [('tool_s50_mount_flange',.08,.092),('tool_s50_mount_spacer',0,.08)]:
        row=np.asarray(c.intrinsic['rigid_local_boxes_virtual_tcp_m'][c.intrinsic['rigid_body_ids'].index(name)])
        np.testing.assert_allclose(row[2]+.285+np.array([-row[5]/2,row[5]/2]),[low_z,high_z],atol=1e-8)
    np.testing.assert_array_equal(c.intrinsic['rigid_local_boxes_virtual_tcp_m'][:130],context.intrinsic['rigid_local_boxes_virtual_tcp_m'])


def test_default_s50_scene_uses_30kg_carton_without_changing_fanuc_record(context):
    c=make_context(ROOT,context.output/'payload30')
    assert c.settings['carton_payload_profile']=='S50_ROBOT_CONFIG'
    assert c.intrinsic['tool_mass_kg']==20
    assert c.intrinsic['carton_mass_kg']==30.0
    assert c.intrinsic['inherited_carton_mass_kg']==42.5
    assert c.intrinsic['carton_mass_source']=='configs/robots/huayan_s50_2000.yaml#carton_payload'
    assert c.frozen['initial_cartons']==context.frozen['initial_cartons']
    assert freeze_non_robot_scene(frozen_baseline_layout(ROOT))['tool_intrinsic']['carton_mass_kg']==42.5
    assert c.frozen['source_environment_fingerprint_before_s50_payload_change']!=c.frozen['environment_fingerprint']


def test_completed_outfeed_body_does_not_reappear_during_belt_lowering(context):
    pytest.importorskip('pinocchio');pytest.importorskip('coal')
    from types import SimpleNamespace
    from unloading_sim.s50_sim_collision import SimCollision
    c=SimpleNamespace(**vars(context));c.settings=dict(context.settings)
    q=[.13321719581922287,-1.4429302127844457,-1.3376641785418004,2.7805971237330325,-1.4375789563789023,-3.141593027554974]
    box=np.array([[.9999999999996636,8.182178684636784e-7,5.1528070631758325e-8,-3.5010001713726835],
                  [-8.182178687700529e-7,.9999999999996652,5.0279907123495e-9,-.8],
                  [-5.152806652542882e-8,-5.028032880543928e-9,.9999999999999986,.8350011457180528],[0,0,0,1]])
    checker=SimCollision(c)
    assert not checker.check(q,height=1.475,payload=box,phase='outfeed_before_handoff')
    assert checker.last['pair']==['carton_l07_c02','conveyor_longitudinal']
    c.active_cartons=tuple(b for b in c.layout.cartons() if b.name!='carton_l07_c02')
    c.outfed_carton_ids={'carton_l07_c02'}
    assert checker.check(q,height=1.475,payload=box,phase='belt_lower_after_handoff')
    assert len(c.active_cartons)==39
