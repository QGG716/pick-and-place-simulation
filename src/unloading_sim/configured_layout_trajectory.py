"""Exact-model connector construction with explicit installed tool semantics.

This module never supplies missing robot dimensions, controller mappings, home
angles or tool adapters. The existing FANUC factory remains unchanged. A
configured connector is a geometric checker, not a claim of dynamic execution.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from copy import deepcopy
from dataclasses import replace
from time import perf_counter
from types import SimpleNamespace
import shlex

import numpy as np

from .collision_policy import SimulationCollisionPolicy
from .robot_model_binding import ModelNotReady, rigid_transform


def configured_tool_frames(binding, tool_intrinsic):
    """Compose a verified physical flange adapter with the unchanged tool."""
    interface = binding.data.get('tool_interface') or {}
    adapter = interface.get('flange_from_project_tool_mount')
    if adapter is None:
        raise ModelNotReady(binding.model_id, ['tool_interface.flange_from_project_tool_mount'])
    connection = interface.get('physical_connection') or {}
    if connection.get('verified') is not True or not connection.get('source'):
        raise ModelNotReady(binding.model_id, ['tool_interface.physical_connection.NOT_VERIFIED'])
    adapter = rigid_transform(adapter, 'flange_from_project_tool_mount')
    frames = tool_intrinsic['intrinsic_frames']
    names = ('virtual_task_tcp', 'uncompressed_cup_plane', 'nominal_compressed_contact')
    result = {name: adapter @ rigid_transform(frames['T_tool_mount_reference_'+name], name)
              for name in names}
    delta = np.linalg.inv(result['uncompressed_cup_plane']) @ result['nominal_compressed_contact']
    if (not np.allclose(delta[:3, :3], np.eye(3), atol=1e-10, rtol=0)
            or not np.allclose(delta[:2, 3], 0, atol=1e-10, rtol=0)
            or not -.015-1e-12 <= delta[2, 3] <= 1e-12):
        raise ValueError('CONFIGURED_TOOL_COMPRESSION_NOT_ALONG_PHYSICAL_CUP_AXIS')
    result['nominal_cup_compression_m'] = max(0., -float(delta[2, 3]))
    return result


def build_configured_layout_trajectory_connector(*, lightweight_robot, tool_intrinsic,
        project_root, output_dir, ik_policy, collision_policy, collision_margin_m,
        contact_tolerance_m, joint_margin_rad, maximum_jacobian_condition,
        floor_z_m, right_wall_y_m, left_wall_y_m, budget=None,
        surface_directions_world=None, post_landing_transport=None):
    """Build the existing exact validator from one explicitly qualified binding.

    No radial-reach approximation or historical FANUC posture constraint is
    installed. A robot-specific mounting-contact exemption is deliberately not
    inferred: unreviewed base/support contacts remain collisions.
    """
    from .layout_trajectory import (ExactM710LayoutStateValidator,
        LayoutTrajectoryConnector, LayoutTrajectoryConnectorBuildResult)
    from .pinocchio_backend import PinocchioHppFclBackend
    from .robot_native_binding import physical_flange_frame
    from .validation_physics import urdf_collision_shapes, world_link_boxes

    evidence = dict(schema='configured_exact_layout_trajectory_backend_v1',
        status='UNAVAILABLE', dynamics_qualified=False, machine_certified=False,
        kinematic_consistency_probes=[], mounting_contact_exemption='NONE_UNTIL_ROBOT_SPECIFIC_PLANE_PROOF')

    def unavailable(reason, detail):
        evidence.update(failure_reason=reason, detail=str(detail))
        return LayoutTrajectoryConnectorBuildResult(None, 'UNAVAILABLE', reason, evidence)

    binding = getattr(lightweight_robot, 'model_binding', None)
    if binding is None:
        return unavailable('EXPLICIT_MODEL_BINDING_REQUIRED', 'Legacy model fallback is disabled')
    if binding.data.get('model_usage') == 'cad_preview_fk_only':
        return unavailable('CAD_PREVIEW_MODEL_FORBIDS_MOTION_CONNECTOR', 'Display ranges are not mechanical limits')
    try:
        binding.verify_source_unchanged()
        frames = configured_tool_frames(binding, tool_intrinsic)
        policy = SimulationCollisionPolicy.for_robot_binding(binding, collision_policy)
        if policy.robot_mount_contact_exemption:
            raise ValueError('CONFIGURED_ROBOT_FORBIDS_LEGACY_CHASSIS_EXEMPTION')
        mesh = PinocchioHppFclBackend.from_binding(binding, output_dir=output_dir,
            project_root=project_root, world_from_base=lightweight_robot.base_transform,
            tip_from_tcp=lightweight_robot.tip_from_tcp)
        for index, q in enumerate(binding.fk_probe_configurations()):
            errors = dict(
                flange=float(np.max(np.abs(physical_flange_frame(mesh, q)-physical_flange_frame(lightweight_robot, q)))),
                tcp=float(np.max(np.abs(mesh.fk(q)-lightweight_robot.fk(q)))),
                jacobian=float(np.max(np.abs(mesh.geometric_jacobian(q)-lightweight_robot.geometric_jacobian(q)))),
                tool_contract=float(np.max(np.abs(lightweight_robot.fk(q)-
                    physical_flange_frame(lightweight_robot, q) @ frames['virtual_task_tcp']))))
            if not np.isfinite(list(errors.values())).all():
                raise ValueError('CONFIGURED_CORE_MESH_TOOL_FRAME_NONFINITE')
            evidence['kinematic_consistency_probes'].append(dict(index=index, q_rad=q.tolist(), errors=errors))
            if max(errors.values()) > 1e-8:
                raise ValueError('CONFIGURED_CORE_MESH_TOOL_FRAME_MISMATCH')
        shapes = urdf_collision_shapes(lightweight_robot)
        validator = ExactM710LayoutStateValidator(mesh, lightweight_robot,
            collision_margin_m=collision_margin_m, floor_z_m=floor_z_m,
            right_wall_y_m=right_wall_y_m, left_wall_y_m=left_wall_y_m,
            robot_world_boxes=lambda q: world_link_boxes(lightweight_robot, np.asarray(q, float), shapes),
            base_support_obstacle_name='mount_plinth', tool_mount_link_name=binding.chain_child_links[-1],
            collision_policy=policy.to_mapping(), nominal_cup_compression_m=frames['nominal_cup_compression_m'])
        probe = binding.fk_probe_configurations()[0]
        tool_boxes = lightweight_robot.tool_all_physical_obbs(probe)
        if not tool_boxes:
            raise ValueError('CONFIGURED_TOOL_COMPOUND_REQUIRED')
        identity = dict(robot_model_binding=binding.identity(),
            base_transform=np.asarray(lightweight_robot.base_transform).tolist(),
            tool_frames={key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in frames.items()},
            tool_intrinsic=tool_intrinsic, collision_policy=policy.to_mapping(),
            collision_geometry_count=len(mesh.geometry_model.geometryObjects),
            collision_link_names=list(mesh.collision_link_names),
            robot_mount_exempt_pairs=[], dynamics_qualified=False)
        digest = hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()
        connector = LayoutTrajectoryConnector(mesh, validator,
            flange_from_virtual_task_tcp=frames['virtual_task_tcp'],
            flange_from_physical_contact=frames['nominal_compressed_contact'],
            ik_policy=ik_policy, collision_margin_m=collision_margin_m,
            contact_tolerance_m=contact_tolerance_m, joint_margin_rad=joint_margin_rad,
            maximum_jacobian_condition=maximum_jacobian_condition,
            validator_identity='configured_exact_mesh_tool:'+digest,
            execution_qualified=True, budget=budget, official_radial_reach_m=None,
            collision_policy=policy.to_mapping(), surface_directions_world=surface_directions_world,
            tool_collision_obbs_provider=lightweight_robot.tool_all_physical_obbs,
            post_landing_transport=post_landing_transport)
        from .validation_kernel import ValidationKernel
        connector.validation_kernel = ValidationKernel(connector, lightweight_robot, mesh)
        evidence.update(status='AVAILABLE', failure_reason=None, source_identity=identity,
            validator_identity=connector.validator_identity,
            qualification_scope='EXACT_GEOMETRIC_VALIDATOR_ONLY_NOT_INITIAL_OR_TASK_VALIDATION')
        return LayoutTrajectoryConnectorBuildResult(connector, 'AVAILABLE', None, evidence)
    except (OSError, RuntimeError, ValueError, KeyError, AttributeError) as exc:
        return unavailable(type(exc).__name__, str(exc))


def _save(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def _budget(policy):
    from .layout_trajectory import LayoutTrajectoryBudget
    strategy = policy.data.get('search_strategy', {})
    # The frozen POC process still uses the same distances and finite solvers.
    return LayoutTrajectoryBudget(
        stage_connection_iterations=int(strategy.get('stage_connection_iterations', 600)),
        approach_mode=strategy.get('approach_mode', 'auto'),
        maximum_drop_m=float(strategy.get('maximum_drop_m', .05)),
        ideal_release_min_height_m=float(strategy.get('ideal_release_min_height_m', .020)),
        ideal_release_max_height_m=float(strategy.get('ideal_release_max_height_m', .050)),
        ideal_release_height_reserve_m=float(strategy.get('ideal_release_height_reserve_m', .005)),
        **{name: float(strategy.get(name, 0.)) for name in ('approach_runtime_clearance_reserve_m',
            'receiver_runtime_clearance_reserve_m', 'departure_runtime_clearance_reserve_m',
            'extraction_runtime_clearance_reserve_m')},
        receiver_edge_reserve_m=float(strategy.get('receiver_edge_reserve_m', .01)),
        planning_wall_time_s=None, candidate_wall_time_s=None, stage_wall_time_s=None,
        postprocess_wall_time_s=None, proof_of_concept=True,
        local_transit_outward_step_m=float(strategy.get('local_transit_outward_step_m', .03)),
        local_transit_outward_attempts=int(strategy.get('local_transit_outward_attempts', 3)),
        local_transit_cartesian_sample_budget=int(strategy.get('local_transit_cartesian_sample_budget', 240)))


def make_configured_context(config, *, root, output, admission):
    """Construct the frozen layout, unchanged tool and actual exact connector.

    This function does not select a home, copy a FANUC q, or launch native.
    The source model is rehashed after the admission check and before loading.
    """
    from .robot_model_binding import RobotModelBinding
    from .pinocchio_backend import PinocchioHppFclBackend
    from .robot_replacement_layout import (build_replacement_layout, frozen_baseline_layout,
        freeze_non_robot_scene, compare_non_robot_scenes)
    from .layout_single_carton import load_layout_motion_policy, ToolFrameContract

    binding = RobotModelBinding.from_config(config)
    if binding.identity() != admission.get('model_binding'):
        raise ValueError('S50_ADMISSION_MODEL_BINDING_CHANGED')
    if binding.data.get('model_usage') == 'cad_preview_fk_only':
        raise ValueError('CAD_PREVIEW_FORBIDS_INSTALLED_CONTEXT')
    # Derive new body bounds from the same actual Coal collision mesh, never
    # retain the FANUC base envelope or move the physical mounting axis.
    bare = PinocchioHppFclBackend.from_binding(binding, output_dir=output/'bare-model',
        project_root=root, world_from_base=np.eye(4))
    bounds = bare.collision_world_axis_extrema(binding.fk_probe_configurations()[0]).get(binding.base_link)
    if not bounds or not bounds.get('exact_mesh'):
        raise ValueError('CONFIGURED_BASE_ENVELOPE_REQUIRES_EXACT_COLLISION_MESH')
    definition = dict(model_id=binding.model_id, dof=6, model_config=str(config), binding=binding.identity(),
        model_asset_identity=binding.identity(), mounting_from_urdf_base=binding.mounting_from_base.tolist(),
        base_support_bounds=[np.asarray(bounds['lower_m']).tolist(), np.asarray(bounds['upper_m']).tolist()], planning_ready=True)
    layout = build_replacement_layout(definition, root=root)
    intrinsic = freeze_non_robot_scene(layout)
    difference = compare_non_robot_scenes(freeze_non_robot_scene(frozen_baseline_layout(root)), intrinsic)
    if not difference['passed'] or intrinsic['environment_fingerprint'] != admission['environment_fingerprint']:
        raise ValueError('S50_ADMISSION_FROZEN_ENVIRONMENT_CHANGED')
    robot = layout.robot()
    policy = load_layout_motion_policy(root/'configs/validation/m710id70_proof_of_concept.yaml')
    frame = configured_tool_frames(binding, intrinsic['tool_intrinsic'])
    tool = ToolFrameContract(frame['virtual_task_tcp'], frame['uncompressed_cup_plane'],
        frame['nominal_compressed_contact'], 'CONFIGURED_PHYSICAL_CONNECTION_VERIFIED', True)
    validation_data = deepcopy(policy.layout_validation.data)
    collision = dict(validation_data.get('collision_policy') or {})
    collision.update(robot_mount_contact_exemption=False, tool_chassis_contact_target_ids=[],
                     fixed_mount_contact_mode='qualified_planes_only')
    scoped = SimulationCollisionPolicy.for_robot_binding(binding, collision)
    validation_data['collision_policy'] = scoped.to_mapping()
    validation_data['initial_state']['q_rad'] = None
    data = deepcopy(policy.data)
    for name, value in tool.evidence().items():
        if name.startswith('T_flange_'): data['tool_frame_contract'][name] = value
    # Historical default execution_manifest is not a S50 acceptance authority.
    data['execution_collision']['qualification_manifest'] = None
    policy = replace(policy, data=data, tool_frames=tool,
        layout_validation=replace(policy.layout_validation, data=validation_data, layout=layout))
    validity = policy.data['state_validity']
    strategy = policy.data.get('search_strategy', {})
    built = build_configured_layout_trajectory_connector(lightweight_robot=robot,
        tool_intrinsic=intrinsic['tool_intrinsic'], project_root=root, output_dir=output/'installed-model',
        ik_policy=policy.data['ik'], collision_policy=scoped.to_mapping(),
        **{key: float(validity[key]) for key in ('collision_margin_m', 'contact_tolerance_m',
            'joint_margin_rad', 'maximum_jacobian_condition')},
        floor_z_m=float(layout.data['world']['floor_z_m']),
        right_wall_y_m=float(layout.data['trailer']['right_wall_y_m']),
        left_wall_y_m=float(layout.data['trailer']['left_wall_y_m']), budget=_budget(policy),
        surface_directions_world=strategy.get('surface_directions_world', {}),
        post_landing_transport=strategy.get('post_landing_transport'))
    _save(output/'connector.json', built.evidence)
    if built.connector is None:
        raise ValueError('CONFIGURED_CONNECTOR_UNAVAILABLE:'+str(built.failure_reason)+':'+str(built.evidence.get('detail')))
    return SimpleNamespace(layout=layout, policy=policy, connector=built.connector,
        robot=robot, binding=binding, frozen=intrinsic, layout_difference=difference)


def solve_configured_initial(context, *, seed, draws=64):
    """Independent finite seeded IK; no accepted FANUC joint vector is read."""
    from .ik import solve_ik
    if isinstance(draws,bool) or not isinstance(draws,int) or draws<0:
        raise ValueError('INITIAL_RANDOM_DRAW_COUNT_MUST_BE_NONNEGATIVE_INTEGER')
    connector = context.connector; robot = connector.robot
    target = rigid_transform(context.frozen['initial_physical_contact_target'], 'front_working_contact_pose')
    virtual_target = connector.virtual_from_physical(target)
    margin = connector.joint_margin_rad
    low, high = robot.joint_limits[:, 0]+margin, robot.joint_limits[:, 1]-margin
    if np.any(low >= high): raise ValueError('INITIAL_SEARCH_JOINT_MARGIN_EXCEEDS_RANGE')
    rng = np.random.default_rng(seed)
    seeds = [np.clip(np.zeros(6), low, high), (low+high)/2]
    # Facing is a starting-state preference, not a goal/endpoint hard gate.
    # In the configured simulation convention q1 near zero is tried first;
    # unbiased draws retain legal alternatives across the whole known range.
    seeds.extend(rng.uniform(low, high, size=(draws, 6)))
    seeds.sort(key=lambda q: abs(float(q[0])))
    fixed = tuple(context.layout.fixed_components()); cartons = tuple(context.layout.cartons())
    obstacles = (*fixed, *cartons)
    report = dict(status='FINITE_INITIAL_IK_EXHAUSTED', seed=int(seed), random_draws=int(draws),
        initial_seed_count=len(seeds), per_seed_iteration_limit=int(connector.ik['max_iterations']),
        requested_contact_pose_world=target.tolist(), joint_ranges_rad=robot.joint_limits.tolist(),
        reused_fanuc_joint_values=False, global_unreachable=False,
        scope='INITIAL_STATE_ONLY_NO_ADDED_GRASP_POSTURE_GATE', attempts=[], q_rad=None)
    for index, guess in enumerate(seeds):
        result = solve_ik(robot, virtual_target, guess,
            max_iterations=int(connector.ik['max_iterations']), damping=float(connector.ik['damping']),
            max_step=float(connector.ik['max_step_rad']), orientation_weight=float(connector.ik['orientation_weight']),
            position_tolerance=float(connector.ik['position_tolerance_m']),
            orientation_tolerance=float(connector.ik['orientation_tolerance_rad']))
        failure = connector.validate_unloaded_state(result.q, obstacles, stage='unloaded') if result.success else None
        row = dict(index=index, seed_q_rad=guess.tolist(), q_rad=result.q.tolist(), converged=result.success,
            iterations=result.iterations, reason=result.message, position_error_m=result.position_error,
            orientation_error_rad=result.orientation_error, collision_or_state_failure=failure)
        report['attempts'].append(row)
        if result.success and failure is None:
            report.update(status='PASS', q_rad=result.q.tolist(), selected_index=index,
                physical_contact_pose_world=connector.physical_from_virtual(robot.fk(result.q)).tolist(),
                joint_limits_checked=True, exact_self_environment_tool_collision_checked=True)
            break
    report['unsearched_seed_count'] = len(seeds)-len(report['attempts'])
    return report


def configured_scene(context, initial):
    """New-model snapshot after exact home acceptance, with all 40 originals."""
    from .layout_single_carton import FrozenLayoutMotionInput
    from .support import SupportRelationGraph
    from .workcell_layout import _obb_record, canonical_digest
    from .robot_native_binding import physical_flange_frame
    from .tool_geometry import audit_tool_geometry
    if initial.get('status') != 'PASS': raise ValueError('REFUSING_SNAPSHOT_OF_UNCHECKED_INITIAL_STATE')
    layout = context.layout; robot = context.connector.robot; q = np.asarray(initial['q_rad'])
    validation_data = deepcopy(context.policy.layout_validation.data)
    validation_data['initial_state']['q_rad'] = q.tolist()
    policy = replace(context.policy, layout_validation=replace(context.policy.layout_validation, data=validation_data))
    fixed, cartons = tuple(layout.fixed_components()), tuple(layout.cartons())
    if len(cartons) != 40 or sorted(b.name for b in cartons) != context.frozen['initial_carton_ids']:
        raise ValueError('S50_FRESH40_POPULATION_CHANGED')
    coverage = audit_tool_geometry(policy.project_root)
    if not coverage['coverage_verified']: raise ValueError('UNCHANGED_TOOL_COVERAGE_NOT_VERIFIED')
    record = lambda path: dict(repository_path=str(path.relative_to(policy.project_root)).replace('\\', '/'),
                              sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    snapshot = dict(schema='configured_robot_frozen_single_carton_scene_v1',
        layout_id=layout.data['layout_id'], layout_fingerprint=layout.layout_fingerprint,
        environment_fingerprint=context.frozen['environment_fingerprint'], units=layout.data['units'],
        world=deepcopy(layout.data['world']), trailer=deepcopy(layout.data['trailer']),
        assembly=dict(pose_world=layout.world_from_assembly.tolist(),
            fixed_components=[_obb_record(b) for b in fixed], designed_contacts=[
                ['chassis', 'conveyor_transverse'], ['chassis', 'conveyor_longitudinal'],
                ['conveyor_transverse', 'conveyor_longitudinal'], ['chassis', 'floor'], ['mount_plinth', 'chassis']]),
        cartons=[_obb_record(b) for b in cartons], robot=dict(model=context.binding.model_id,
            model_binding=context.binding.identity(), joint_names=list(context.binding.active_joint_names),
            q_rad=q.tolist(), world_from_mount=layout.physical_mount_transform().tolist(),
            world_from_urdf_base=layout.robot_base_transform().tolist(),
            urdf=record(context.binding.urdf_path), model_config=record(context.binding.config_path),
            flange_pose_world=physical_flange_frame(robot,q).tolist(), tcp_pose_world=robot.fk(q).tolist(),
            mounting_reference=dict(base_support_bounds=layout.definition['base_support_bounds'],
                status='CONFIGURED_PHYSICAL_ALIGNMENT_WITH_NO_MOUNT_COLLISION_EXEMPTION')),
        tool=dict(name=context.frozen['tool_intrinsic']['load'].get('name','wantai130'),
            mass_kg=context.frozen['tool_intrinsic']['tool_mass_kg'], task_tcp_pose_world=robot.fk(q).tolist(),
            intrinsic=context.frozen['tool_intrinsic'], geometry_coverage=coverage,
            geometry_status='CONSERVATIVE_RIGID_SOLID_COVERAGE_PROVEN'),
        receiver=dict(state='EMPTY', transport_capability='NOMINAL_RECEPTION_AND_IDEAL_OUTFEED'),
        attachments=[], initial_state_audit=initial, gap_search=deepcopy(layout.data['gap_search']))
    snapshot['scene_fingerprint'] = canonical_digest(snapshot)
    population = policy.data['task_population']
    graph = SupportRelationGraph.build(cartons,
        contact_tolerance_m=float(population['support_contact_tolerance_m']),
        minimum_overlap_ratio=float(population['support_minimum_overlap_ratio']))
    removable = tuple(graph.removable_cartons(face_modes=tuple(population['face_modes'])))
    verification = dict(status='PASS', scope='CONFIGURED_MODEL_AND_FROZEN_ENVIRONMENT_HASHES',
        fanuc_snapshot_verifier_used=False, model_binding=context.binding.identity())
    return FrozenLayoutMotionInput(policy, snapshot, verification, context.layout_difference,
        fixed, cartons, next(b for b in fixed if b.name=='conveyor_transverse'), graph, removable)


def export_native_trajectory(plan, binding, verification, *, scene_context):
    """Export only a verified current-run native plan, never IK probe motion."""
    if verification.get('status') != 'PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED':
        raise ValueError('NATIVE_TRAJECTORY_REQUIRES_CYCLE_VERIFICATION')
    mount=rigid_transform(scene_context.get('world_from_physical_mount'),'trajectory.world_from_physical_mount')
    base=rigid_transform(scene_context.get('world_from_model_base'),'trajectory.world_from_model_base')
    if not np.allclose(base,mount @ binding.mounting_from_base,atol=1e-10,rtol=0):
        raise ValueError('TRAJECTORY_MODEL_MOUNT_ALIGNMENT_MISMATCH')
    ids=scene_context.get('initial_carton_ids') or []
    if len(ids)!=40 or len(set(ids))!=40 or not scene_context.get('environment_fingerprint'):
        raise ValueError('TRAJECTORY_REQUIRES_FROZEN_FRESH40_CONTEXT')
    declared = binding.data.get('generated_definition')
    if not declared: raise ValueError('TRAJECTORY_MODEL_DEFINITION_PATH_MISSING')
    definition = (binding.config_path.parent/declared).resolve()
    points=[]; elapsed=0.; native_stages=[]
    for stage in plan['stages']:
        data=stage.get('points', [])
        if not data: continue
        moving = any(not np.allclose(point['q'],data[0]['q'],atol=1e-12,rtol=0) for point in data[1:])
        if moving and stage.get('generation_source')!='NATIVE_GENERATED':
            raise ValueError('TRAJECTORY_NON_NATIVE_MOVING_STAGE')
        if moving:native_stages.append(stage['stage_id'])
        previous=-1.
        for point in data:
            q=np.asarray(point['q'],float);t=float(point['t'])
            if (q.shape!=(6,) or not np.isfinite(q).all() or not np.isfinite(t) or t<0 or t<previous
                    or np.any(q<binding.joint_limits[:,0]) or np.any(q>binding.joint_limits[:,1])):
                raise ValueError('INVALID_NATIVE_TRAJECTORY_POINT')
            previous=t
            points.append(dict(q_rad=q.tolist(), time_s=elapsed+t, phase=stage['stage'],
                native_stage_id=stage['stage_id']))
        elapsed+=float(data[-1]['t'])
    if not points: raise ValueError('NATIVE_TRAJECTORY_HAS_NO_MOTION')
    return dict(schema='s50_native_joint_trajectory_v1',status='NATIVE_PLANNED_NOT_PHYSICALLY_EXECUTED',
        model_definition_sha256=hashlib.sha256(definition.read_bytes()).hexdigest(),
        model_binding=binding.identity(), joint_names=list(binding.active_joint_names),
        source='native_worker_current_run', points=points,scene_context=scene_context,
        playback_scope='robot_joint_only',environment_playback='initial_scene_static_reference',
        native_provenance=dict(current_run=True,verification=verification,
            verification_sha256=hashlib.sha256(json.dumps(verification,sort_keys=True).encode()).hexdigest(),
            plan_sha256=hashlib.sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest(),
            moving_native_stage_ids=native_stages, historical_motion_loaded=False),
        time_scope='SUM_OF_NATIVE_MOTION_TIMESTAMPS_EXCLUDES_SEMANTIC_WAIT',
        physical_execution='NOT_RUN', dynamics_qualified=False)


def frozen_native_resources(root):
    """Read only numerical resource flags from the actual frozen launch.

    No initial joint vector, historical checkpoint, trial result or trajectory
    is imported. These are existing resource ceilings, not a new task SLA.
    """
    path=Path(root)/'docs/experiments/m710_gap700_front_working_20261110/front-working-search-launch.json'
    launch=json.loads(path.read_text(encoding='utf-8'))
    tokens=shlex.split(launch['command'][-1])
    result={}
    for field,flag in (('task_seconds','--trial-seconds'),('stage_seconds','--stage-seconds'),
            ('prefix_seconds','--prefix-seconds'),('connection_seconds','--connection-seconds'),
            ('ompl_seconds','--ompl-seconds')):
        if tokens.count(flag)!=1:raise ValueError('FROZEN_NATIVE_RESOURCE_FLAG_MISSING_OR_DUPLICATED:'+flag)
        value=float(tokens[tokens.index(flag)+1])
        if not np.isfinite(value) or value<=0:raise ValueError('INVALID_FROZEN_NATIVE_RESOURCE:'+flag)
        result[field]=value
    result.update(source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        scope='REUSED_FROZEN_RESOURCE_CEILINGS_NOT_NEW_BUSINESS_DEADLINE',
        candidate_task_budget_shared_without_reset=True)
    return result


def plan_single_carton_from_admission(admission, *, config, output,
        worker_command=None, seed=50, root=None, initial_draws=64, task_seconds=None):
    """One fresh-40 representative carton through the existing native recipe.

    Task data and geometry admission precede any worker. Initial-state search
    is independent of FANUC, while the contact/extraction/reception/outfeed
    algorithm remains the existing native run_trial, with posture gates off.
    """
    config=Path(config).resolve();output=Path(output);root=Path(root).resolve()
    if hashlib.sha256(config.read_bytes()).hexdigest()!=admission.get('config_sha256'):
        raise ValueError('S50_ADMISSION_CONFIG_CHANGED')
    gate=admission.get('geometric_planning') or {};missing=list(gate.get('missing', []))
    if gate.get('ready') is not True and not missing:missing.append('geometric_planning.NOT_READY')
    report=dict(schema='s50_native_single_carton_attempt_v1', status='NOT_RUN_MODEL_NOT_READY',
        complete=False, missing=sorted(set(missing)), implementation_gaps=[],
        single_carton_run_trial_connected=True, model_binding=admission.get('model_binding'),
        target=(admission.get('native_single_carton') or {}).get('target'),
        start_carton_count=len(admission.get('initial_carton_ids', [])), imported_completed_count=0,
        assumed_removed=[], seed=int(seed), native_worker_launched=False,
        initial_ik=dict(status='NOT_RUN', attempts=[]), collision_check=dict(status='NOT_RUN'),
        planning_attempts=0, global_unreachable=False, attempts=[],
        native_trajectory=dict(status='NOT_RUN', points=[]), physical_execution='NOT_RUN',
        dynamics_qualified=False, payload=admission.get('payload'),
        environment_fingerprint=admission.get('environment_fingerprint'))
    if gate.get('ready') is not True: return report
    output.mkdir(parents=True, exist_ok=True)
    try:
        context=make_configured_context(config,root=root,output=output,admission=admission)
        report['full_scene_fingerprint']=context.layout.layout_fingerprint
        initial=solve_configured_initial(context,seed=seed,draws=initial_draws)
        report['initial_ik']=initial;_save(output/'initial-state.json',initial)
        if initial['status']!='PASS':
            report['status']='INITIAL_STATE_NOT_FOUND_IN_FINITE_SEARCH'
            candidates=[row for row in initial['attempts'] if row.get('collision_or_state_failure')]
            if candidates:
                failed=candidates[-1]
                report['failure_state']=dict(q_rad=failed['q_rad'],stage='initial',
                    collision_objects=failed['collision_or_state_failure'].get('pair'),
                    failure=failed['collision_or_state_failure'])
            return report
        report['collision_check']=dict(status='PASS_INITIAL_EXACT_GEOMETRY')
        scene=configured_scene(context,initial);_save(output/'scene.json',scene.snapshot)
        if not worker_command:
            report.update(status='NOT_RUN_NATIVE_WORKER_COMMAND_REQUIRED',missing=['runtime.worker_command'])
            return report
        timing=admission.get('time_parameterization') or {}
        if timing.get('ready') is not True:
            report.update(status='NOT_RUN_NATIVE_TIMING_NOT_READY',missing=timing.get('missing', ['timing.NOT_READY']))
            return report
        import sys
        sys.path.insert(0,str(root/'tools'))
        from plan_m710_joint_search import run_trial
        from .carton_population import FrozenCartonPopulation
        from .coupled_batch import BatchState, batch_scene
        from .contact_target import simulation_contact_targets
        from .gap_search_recipes import candidate_options
        from .l01_selected_routes import finite_side_exposure
        frozen=FrozenCartonPopulation.capture(scene.cartons)
        state=BatchState(frozen,tuple(scene.cartons),np.asarray(initial['q_rad']),.6)
        built=context.connector;built.joint_search_initial_q=initial['q_rad']
        scene=batch_scene(scene,state,built.robot,context.layout)
        target=next(b for b in scene.cartons if b.name==report['target'])
        exposure=finite_side_exposure(built,state.q)
        faces,screen=simulation_contact_targets(scene,target,context.layout.physical_mount_transform()[:3,3],
            built.budget.pregrasp_standoff_m,side_exposure_depth_m=exposure['depth_m'])
        options=list(candidate_options(faces,scene,frozen,state.height,context.layout,initial_q=state.q))
        report.update(face_screening=screen,side_exposure=exposure,candidate_count=len(options),unsearched_candidate_count=len(options),
            initial_root=state.root())
        resources=frozen_native_resources(root)
        if task_seconds is not None:
            if not np.isfinite(task_seconds) or task_seconds<=0:raise ValueError('INVALID_EXPLICIT_TASK_RESOURCE_LIMIT')
            resources['task_seconds']=float(task_seconds);resources['explicit_resource_override']=True
        task_seconds=resources['task_seconds'];report['native_resources']=resources
        report['task_resource_limit_s']=task_seconds
        args=SimpleNamespace(worker_command=worker_command,front_arm_posture='none',
            **{key:resources[key] for key in ('stage_seconds','prefix_seconds','connection_seconds','ompl_seconds')})
        started=perf_counter()
        for index,(face,option) in enumerate(options):
            remaining=float(task_seconds)-(perf_counter()-started)
            if remaining<=0:break
            folder=output/('trial-%03d'%index)
            plan,result=run_trial(args,scene,built,state,face,option,folder,remaining,seed+index)
            report['native_worker_launched'] |= bool(result.get('native_worker_process_started'))
            report['attempts'].append(result);report['planning_attempts']+=1
            report['unsearched_candidate_count']=len(options)-index-1
            if plan is not None:
                verification=json.loads((folder/'cycle-verification.json').read_text())
                report.update(status='NATIVE_SINGLE_CARTON_PLANNED_NOT_EXECUTED',planning_complete=True,
                    cycle_verification=verification,plan_path=str(folder/'plan.json'))
                try:
                    trajectory=export_native_trajectory(plan,context.binding,verification,scene_context=dict(
                        environment_fingerprint=context.frozen['environment_fingerprint'],
                        world_from_physical_mount=context.layout.physical_mount_transform().tolist(),
                        world_from_model_base=context.layout.robot_base_transform().tolist(),
                        initial_carton_ids=context.frozen['initial_carton_ids']))
                    _save(output/'native-joint-trajectory.json',trajectory)
                    report['native_trajectory']=dict(status='EXPORTED_CURRENT_NATIVE_RUN',
                        path=str(output/'native-joint-trajectory.json'),points=len(trajectory['points']))
                    report['complete']=True
                except ValueError as exc:
                    report['status']='NATIVE_PLAN_COMPLETE_TRAJECTORY_EXPORT_NOT_READY'
                    report['native_trajectory']=dict(status='EXPORT_NOT_READY',reason=str(exc),points=[])
                break
            if result.get('implementation_failure'):
                report['status']='NATIVE_ADAPTER_FAILURE_RETAINED';break
        if not report['complete'] and report['status']=='NOT_RUN_MODEL_NOT_READY':
            report['status']='FINITE_SINGLE_CARTON_SEARCH_FAILED_NOT_UNREACHABILITY'
        return report
    except Exception as exc:
        import traceback
        (output/'integration-exception.txt').write_text(traceback.format_exc(),encoding='utf-8')
        report.update(status='INTEGRATION_FAILURE_RETAINED',complete=False,
            failure=dict(reason=type(exc).__name__,detail=str(exc)))
        return report
