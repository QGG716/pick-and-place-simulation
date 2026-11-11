"""Protocol/wiring regressions; synthetic fixtures are not S50 task evidence."""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from test_robot_model_binding import fixture_config, change_config
from unloading_sim.robot import URDFRobot
from unloading_sim.robot_model_binding import RobotModelBinding, ModelNotReady
from unloading_sim.robot_native_binding import native_fk_model_request, configured_native_timing
from unloading_sim import configured_layout_trajectory as entry

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))


def scene_context():
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout,freeze_non_robot_scene
    layout=frozen_baseline_layout(ROOT);frozen=freeze_non_robot_scene(layout)
    return dict(environment_fingerprint=frozen['environment_fingerprint'],
        world_from_physical_mount=layout.robot_base_transform().tolist(),
        world_from_model_base=layout.robot_base_transform().tolist(),
        initial_carton_ids=frozen['initial_carton_ids'])


def test_bare_native_protocol_declares_fk_only_and_unknown_source_effort(fixture_config,tmp_path):
    change_config(fixture_config,lambda d:d.update(model_usage='cad_preview_fk_only',
        preview_only={'urdf_effort_is_placeholder':True}))
    xml=ET.parse(fixture_config.with_name('fixture.urdf'))
    for limit in xml.getroot().iter('limit'):limit.set('effort','0')
    xml.write(fixture_config.with_name('fixture.urdf'))
    binding=RobotModelBinding.from_config(fixture_config)
    before=binding.urdf_path.read_bytes()
    init=native_fk_model_request(binding,project_root=tmp_path)
    assert init['model_usage']=='cad_preview_fk_only'
    assert init['identity']['motion_generation_allowed'] is False
    assert init['tool_links']==[] and init['process_geometry'] is None
    roles=init['identity']['native_frame_roles']['urdf_parser_compatibility']
    assert roles['effort_placeholder_joint_names']==[]
    assert roles['physical_effort_unknown_joint_names']==list(binding.active_joint_names)
    assert binding.urdf_path.read_bytes()==before
    assert not any('planning_plugin' in key for key in init['parameters'])


def test_bound_pin_loader_applies_world_once_and_disables_unmeasured_capsules(fixture_config,tmp_path):
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    binding=RobotModelBinding.from_config(fixture_config)
    class LoaderFixture(PinocchioHppFclBackend):
        def __init__(self,urdf_path,**kwargs):
            self.active_joint_names=binding.active_joint_names
            self.joint_limits=binding.joint_limits.copy()
            self.base_transform=kwargs['base_transform']
            self.loaded=ET.parse(urdf_path).getroot()
    mount=np.eye(4);mount[:3,3]=[-1.85,.15,.9]
    robot=LoaderFixture.from_binding(binding,project_root=tmp_path,output_dir=tmp_path/'converted',world_from_base=mount)
    origin=robot.loaded.find("joint[@name='configured_world_mount']/origin")
    assert list(map(float,origin.get('xyz').split()))==[0.,0.,0.]
    np.testing.assert_allclose(robot.base_transform,mount)
    assert robot.model_binding.identity()==binding.identity()
    with pytest.raises(ValueError,match='CAPSULE_PROXY_UNAVAILABLE'):
        robot.link_capsules(np.zeros(6))


def test_configured_timing_requires_and_consumes_explicit_values(fixture_config):
    with pytest.raises(ModelNotReady,match='joint_acceleration'):
        configured_native_timing(RobotModelBinding.from_config(fixture_config))
    change_config(fixture_config,lambda d:d.update(time_parameterization_assumption=dict(
        kind='ENGINEERING_ASSUMPTION',manufacturer_confirmed=False,source_or_rationale='synthetic test only',
        velocity_rad_s=[.2,.3,.4,.5,.6,.7],acceleration_rad_s2=[.1,.2,.3,.4,.5,.6])))
    timing=configured_native_timing(RobotModelBinding.from_config(fixture_config))
    assert timing['velocity_rad_s']==[.2,.3,.4,.5,.6,.7]
    assert timing['acceleration_rad_s2']==[.1,.2,.3,.4,.5,.6]
    assert timing['audit']['manufacturer_confirmed'] is False


def test_no_fanuc_support_exception_for_configured_robot(fixture_config):
    from unloading_sim.joint_search_layout import qualified_mount_pairs
    robot=URDFRobot.from_config(fixture_config)
    robot.joint_search_mount={'mount_design':'SOLID_PLINTH'}
    assert qualified_mount_pairs(robot,[])==set()


def test_tool_adapter_required_and_composition_uses_physical_frame(fixture_config):
    intrinsic={'intrinsic_frames':{}}
    for name,z in [('virtual_task_tcp',.25),('uncompressed_cup_plane',.2275),('nominal_compressed_contact',.2225)]:
        transform=np.eye(4);transform[2,3]=z
        intrinsic['intrinsic_frames']['T_tool_mount_reference_'+name]=transform.tolist()
    with pytest.raises(ModelNotReady,match='flange_from_project_tool_mount'):
        entry.configured_tool_frames(RobotModelBinding.from_config(fixture_config),intrinsic)
    adapter=np.eye(4);adapter[0,3]=.012
    change_config(fixture_config,lambda d:d.update(tool_interface=dict(
        flange_from_project_tool_mount=adapter.tolist(),physical_connection=dict(verified=True,source='synthetic test'))))
    frames=entry.configured_tool_frames(RobotModelBinding.from_config(fixture_config),intrinsic)
    np.testing.assert_allclose(frames['virtual_task_tcp'][:3,3],[.012,0.,.25])
    assert frames['nominal_cup_compression_m']==pytest.approx(.005)


def test_preview_cannot_construct_motion_connector(fixture_config,tmp_path):
    change_config(fixture_config,lambda d:d.update(model_usage='cad_preview_fk_only'))
    result=entry.build_configured_layout_trajectory_connector(lightweight_robot=URDFRobot.from_config(fixture_config),
        tool_intrinsic={},project_root=tmp_path,output_dir=tmp_path,ik_policy={},collision_policy={},
        collision_margin_m=.005,contact_tolerance_m=.0002,joint_margin_rad=.01,
        maximum_jacobian_condition=1e4,floor_z_m=0.,right_wall_y_m=-1.1,left_wall_y_m=1.1)
    assert result.connector is None and result.failure_reason=='CAD_PREVIEW_MODEL_FORBIDS_MOTION_CONNECTOR'


def test_formal_not_ready_never_constructs_context_or_worker(fixture_config,tmp_path,monkeypatch):
    monkeypatch.setattr(entry,'make_configured_context',lambda *a,**k:pytest.fail('must not build context'))
    admission=dict(config_sha256=hashlib.sha256(fixture_config.read_bytes()).hexdigest(),
        geometric_planning=dict(ready=False,missing=['installation.mount_clocking.NOT_VERIFIED']),initial_carton_ids=['a']*40)
    report=entry.plan_single_carton_from_admission(admission,config=fixture_config,output=tmp_path/'out',root=ROOT,
        worker_command='must-not-launch')
    assert report['status']=='NOT_RUN_MODEL_NOT_READY'
    assert report['native_worker_launched'] is False and report['planning_attempts']==0
    assert report['single_carton_run_trial_connected'] is True
    assert report['missing']==['installation.mount_clocking.NOT_VERIFIED']


def test_frozen_resources_have_literal_current_values():
    resources=entry.frozen_native_resources(ROOT)
    assert [resources[k] for k in ('task_seconds','stage_seconds','prefix_seconds','connection_seconds','ompl_seconds')]==[600.,360.,360.,30.,20.]
    assert 'q_rad' not in resources and resources['candidate_task_budget_shared_without_reset']


def test_current_native_export_matches_viewer_contract(fixture_config,tmp_path):
    from export_s50_cad_motion_viewer import trajectory_data
    definition=tmp_path/'definition.json';definition.write_text('{"fixture":true}')
    change_config(fixture_config,lambda d:d.update(generated_definition='definition.json'))
    binding=RobotModelBinding.from_config(fixture_config)
    plan={'stages':[
        dict(stage='pregrasp',stage_id='current-native-1',generation_source='NATIVE_GENERATED',
            points=[dict(q=[0.]*6,t=0.),dict(q=[.1]*6,t=1.)]),
        dict(stage='release',stage_id='current-event-2',generation_source='SEMANTIC_EVENT',
            points=[dict(q=[.1]*6,t=0.),dict(q=[.1]*6,t=.1)])]}
    verified=dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED',native_stages=2)
    exported=entry.export_native_trajectory(plan,binding,verified,scene_context=scene_context())
    path=tmp_path/'trajectory.json';path.write_text(json.dumps(exported))
    loaded=trajectory_data(path,dict(definition_sha256=hashlib.sha256(definition.read_bytes()).hexdigest(),
        joints=[dict(name=name) for name in binding.active_joint_names]),expected_scene_context=scene_context())
    assert loaded['native_provenance']['current_run'] and loaded['status']=='NATIVE_PLANNED_NOT_PHYSICALLY_EXECUTED'
    assert loaded['points'][-1]['time_s']==1.1
    assert loaded['native_provenance']['moving_native_stage_ids']==['current-native-1']
    plan['stages'][1]['points'][-1]['q']=[.2]*6
    with pytest.raises(ValueError,match='NON_NATIVE_MOVING_STAGE'):
        entry.export_native_trajectory(plan,binding,verified,scene_context=scene_context())


def test_formal_wiring_reaches_run_trial_without_copying_fanuc_home(fixture_config,tmp_path,monkeypatch):
    """Injected planner/context prove wiring only, no geometry success claim."""
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout
    from unloading_sim.layout_single_carton import load_layout_motion_policy
    from unloading_sim.support import SupportRelationGraph
    from unloading_sim import coupled_batch,contact_target,gap_search_recipes,l01_selected_routes
    import plan_m710_joint_search
    layout=frozen_baseline_layout(ROOT);cartons=tuple(layout.cartons())
    definition=tmp_path/'definition.json';definition.write_text('{}')
    change_config(fixture_config,lambda d:d.update(generated_definition='definition.json'))
    binding=RobotModelBinding.from_config(fixture_config)
    q=[.123]*6
    connector=SimpleNamespace(robot=SimpleNamespace(),budget=SimpleNamespace(pregrasp_standoff_m=.1))
    frozen_context=scene_context()
    context=SimpleNamespace(layout=SimpleNamespace(layout_fingerprint=layout.layout_fingerprint,
        physical_mount_transform=layout.robot_base_transform,robot_base_transform=layout.robot_base_transform),
        binding=binding,connector=connector,frozen=frozen_context)
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    scene=SimpleNamespace(cartons=cartons,fixed_components=tuple(layout.fixed_components()),policy=policy,
        support_graph=SupportRelationGraph.build(cartons),receiver=next(b for b in layout.fixed_components() if b.name=='conveyor_transverse'),snapshot={})
    monkeypatch.setattr(entry,'make_configured_context',lambda *a,**k:context)
    monkeypatch.setattr(entry,'solve_configured_initial',lambda *a,**k:dict(status='PASS',q_rad=q,attempts=[]))
    monkeypatch.setattr(entry,'configured_scene',lambda *a,**k:scene)
    monkeypatch.setattr(coupled_batch,'batch_scene',lambda original,*a:original)
    monkeypatch.setattr(contact_target,'simulation_contact_targets',lambda *a,**k:([SimpleNamespace(target_id=cartons[0].name)],[]))
    monkeypatch.setattr(gap_search_recipes,'candidate_options',lambda *a,**k:iter([(SimpleNamespace(),{})]))
    monkeypatch.setattr(l01_selected_routes,'finite_side_exposure',lambda *a,**k:{'depth_m':.4})
    calls=[]
    def trial(args,original,built,state,face,option,folder,budget,seed):
        calls.append(dict(q=state.q.tolist(),budget=budget,posture=args.front_arm_posture,
            stage=args.stage_seconds,prefix=args.prefix_seconds,connection=args.connection_seconds,ompl=args.ompl_seconds))
        folder.mkdir()
        (folder/'cycle-verification.json').write_text(json.dumps(dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED')))
        return dict(stages=[dict(stage='transit',stage_id='native-fixture',generation_source='NATIVE_GENERATED',
            points=[dict(q=q,t=0.),dict(q=[.124]*6,t=1.)])]),dict(native_worker_process_started=True)
    monkeypatch.setattr(plan_m710_joint_search,'run_trial',trial)
    admission=dict(config_sha256=hashlib.sha256(fixture_config.read_bytes()).hexdigest(),geometric_planning=dict(ready=True),
        initial_carton_ids=[b.name for b in cartons],native_single_carton=dict(target=cartons[0].name),
        time_parameterization=dict(ready=True))
    report=entry.plan_single_carton_from_admission(admission,config=fixture_config,root=ROOT,output=tmp_path/'run',worker_command='fixture')
    assert report['complete'] is True,report
    assert calls[0]['q']==q and calls[0]['posture']=='none'
    assert 590.<calls[0]['budget']<=600.
    assert [calls[0][k] for k in ('stage','prefix','connection','ompl')]==[360.,360.,30.,20.]
    assert report['initial_root']['completed']==[] and len(report['initial_root']['world_cartons'])==40
    assert report['native_worker_launched'] and report['planning_attempts']==1


def test_saved_native_points_cannot_be_replaced_between_endpoints():
    from copy import deepcopy
    from verify_m710_joint_search import verify_saved_native_stage_payloads
    stage=dict(stage='transit',stage_id='fresh:1',parent_stage_id='',generation_source='NATIVE_GENERATED',
        terminal_q=[1.]*6,pipeline_id='ompl',planner_id='RRTConnect',
        points=[dict(q=[0.]*6,t=0.),dict(q=[.4]*6,t=.5),dict(q=[1.]*6,t=1.)])
    request=dict(identity={'model':'synthetic'},task_id='fresh',stage_id='fresh:1',parent_stage_id='',stage='transit')
    response=dict(**{key:value for key,value in stage.items() if key not in ('points','stage')},
        task_id='fresh',request_id='17',status='SUCCESS')
    trace=dict(model=dict(identity=request['identity']),attempts=[dict(request=request,
        response_identity={key:response[key] for key in ('task_id','stage_id','parent_stage_id','request_id')},
        final_timed_path=deepcopy(stage['points']))])
    calls=[dict(op='plan',stage_id=stage['stage_id'],status='SUCCESS',response=response)]
    plan=dict(stages=[deepcopy(stage)])
    assert verify_saved_native_stage_payloads(plan,calls,trace)['stage_count']==1
    plan['stages'][0]['points'][1]['q']=[.8]*6
    with pytest.raises(AssertionError,match='NATIVE_POINTS_CHANGED'):
        verify_saved_native_stage_payloads(plan,calls,trace)


def test_new_initial_state_keeps_exact_collision_failures(fixture_config):
    from unloading_sim.layout_single_carton import load_layout_motion_policy
    core=URDFRobot.from_config(fixture_config)
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    connector=SimpleNamespace(robot=core,ik=policy.data['ik'],joint_margin_rad=.01,
        virtual_from_physical=lambda x:x,
        validate_unloaded_state=lambda *a,**k:{'reason':'FIXTURE_EXACT_COLLISION','pair':['segment_2','test_box']})
    context=SimpleNamespace(connector=connector,layout=SimpleNamespace(fixed_components=lambda:[],cartons=lambda:[]),
        frozen={'initial_physical_contact_target':core.fk(np.zeros(6)).tolist()})
    report=entry.solve_configured_initial(context,seed=123,draws=0)
    assert report['status']=='FINITE_INITIAL_IK_EXHAUSTED' and report['q_rad'] is None
    assert report['attempts'][0]['collision_or_state_failure']['reason']=='FIXTURE_EXACT_COLLISION'
    assert report['global_unreachable'] is False and not report['reused_fanuc_joint_values']


def test_no_binding_native_model_keeps_fanuc_defaults(monkeypatch):
    from unloading_sim import moveit2_backend,independent_cups
    binding=RobotModelBinding.from_config(ROOT/'configs/robots/fanuc_m710id_70.yaml')
    robot=URDFRobot.fanuc_m710id_70(binding.urdf_path,base_position=[0.,0.,0.],tool_length=0.)
    connector=SimpleNamespace(robot=robot,collision_policy=SimpleNamespace(wrist_tool_pairs=lambda _:set(),to_mapping=lambda:{}),
        robot_state_validator=SimpleNamespace(_compliant_boxes=lambda _:[]),validator_identity='fanuc_fixture',
        flange_from_virtual_task_tcp=np.eye(4),flange_from_physical_contact=np.eye(4))
    scene=SimpleNamespace(policy=SimpleNamespace(project_root=ROOT,policy_fingerprint='fixture',data={'suction':{}}),snapshot={})
    monkeypatch.setattr(moveit2_backend,'native_tool_collision_boxes',lambda *_:([],{}))
    monkeypatch.setattr(independent_cups,'m710_cup_array_from_mapping',lambda _:SimpleNamespace(cups=[]))
    request,_,_=moveit2_backend.model_request(connector,scene)
    assert request['joint_names']==['J1','J2','J3','J4','J5','J6']
    assert 'robot_model_binding' not in request['identity']
    assert request['parameters']['robot_description_planning.joint_limits.J2.max_acceleration']==.5


def test_actual_fanuc_pin_configured_and_original_loaders_agree(tmp_path):
    """Server optional dependency check; three FK/bounds probes, no search."""
    pytest.importorskip('pinocchio')
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    binding=RobotModelBinding.from_config(ROOT/'configs/robots/fanuc_m710id_70.yaml')
    mount=np.eye(4);mount[:3,3]=[-1.85,.15,.9]
    configured=PinocchioHppFclBackend.from_binding(binding,project_root=ROOT,output_dir=tmp_path/'fanuc-pin',world_from_base=mount)
    original=PinocchioHppFclBackend(binding.urdf_path,tip_frame=binding.tip_link,srdf_path=binding.srdf_path,
        base_transform=mount,package_dirs=[ROOT/'assets/robots/fanuc_m710id_70/official'])
    core=URDFRobot.from_config(binding.config_path,mounting_world_transform=mount)
    assert configured.active_joint_names==original.active_joint_names==binding.active_joint_names
    assert len(configured.geometry_model.geometryObjects)==len(original.geometry_model.geometryObjects)==7
    for q in binding.fk_probe_configurations():
        np.testing.assert_allclose(configured.fk(q),original.fk(q),atol=1e-10,rtol=0)
        np.testing.assert_allclose(configured.fk(q),core.fk(q),atol=1e-10,rtol=0)
        np.testing.assert_allclose(configured.geometric_jacobian(q),original.geometric_jacobian(q),atol=1e-10,rtol=0)
        expected=original.collision_world_axis_extrema(q)
        actual=configured.collision_world_axis_extrema(q)
        assert actual.keys()==expected.keys()
        for name in expected:
            assert actual[name]['exact_mesh'] and expected[name]['exact_mesh']
            for key in ('lower_m','upper_m'):
                np.testing.assert_allclose(actual[name][key],expected[name][key],atol=1e-10,rtol=0)
