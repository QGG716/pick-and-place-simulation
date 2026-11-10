"""Select a stack-forward initial state; never generate or credit robot motion."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_sim.approach_recovery_policy import equivalent_bounded_configurations
from unloading_sim.geometry import make_transform, make_tool_rotation
from unloading_sim.ik import solve_ik
from unloading_sim.initial_front_working import front_working_initial_evidence
from unloading_sim.layout_single_carton import load_layout_motion_policy
from unloading_sim.tool_connector_extension import bind_connector_frames
from unloading_sim.workcell_layout import _audit_initial_state_prepared, urdf_collision_shapes
from unloading_sim.gap_search_layout import initial_stack_facing_evidence
from plan_m710_gap_search import frozen_layout
from plan_m710_mount_comparison import make_context


def prepare_front_working_home(args):
    if args.front_arm_posture != 'none':
        raise ValueError('FRONT_WORKING_INITIAL_REQUIRES_ENDPOINT_POSTURE_NONE')
    began=perf_counter();layout=frozen_layout(args.gap,args.mount_height);robot=layout.robot()
    root=Path(__file__).resolve().parents[1]
    policy=load_layout_motion_policy(root/'configs/validation/m710id70_proof_of_concept.yaml')
    data=deepcopy(policy.layout_validation.data)
    data['collision_policy'].update(robot_mount_contact_exemption=False,
        tool_chassis_contact_target_ids=[],fixed_mount_contact_mode='qualified_planes_only')
    policy=bind_connector_frames(replace(policy,layout_validation=replace(policy.layout_validation,layout=layout,data=data)))
    # Explicit single home pose, independent of every target's business pose.
    xyz=args.tcp_xyz or [-.2,0.,1.5]
    physical=make_transform(make_tool_rotation(np.array([1.,0.,0.]),np.array([0.,-1.,0.])),xyz)
    virtual=physical@np.linalg.inv(policy.tool_frames.flange_from_physical_contact)@policy.tool_frames.flange_from_virtual_task_tcp
    shapes=urdf_collision_shapes(robot);fixed=list(layout.fixed_components());cartons=layout.cartons()
    limits=robot.joint_limits;rng=np.random.default_rng(args.seed);rows=[];qualified=[];seen=[]
    for i,seed in enumerate(rng.uniform(limits[:,0]+.05,limits[:,1]-.05,(args.draws,6))):
        result=solve_ik(robot,virtual,seed,max_iterations=300,position_tolerance=.0001,orientation_tolerance=.0001)
        row=dict(index=i,seed_q_rad=seed.tolist(),converged=result.success,iterations=result.iterations,
            reason=result.message,position_error_m=result.position_error,orientation_error_rad=result.orientation_error)
        rows.append(row)
        if not result.success:continue
        variants=equivalent_bounded_configurations(robot,result.q)
        q=min(variants,key=lambda v:front_working_initial_evidence(robot,v)['selection_cost'])
        row.update(q_rad=q.tolist(),q_deg=np.degrees(q).tolist(),bounded_equivalent_count=len(variants),
            initial_direction=front_working_initial_evidence(robot,q))
        if not row['initial_direction']['passed']:
            row['failure']='INITIAL_ARM_NOT_STACK_FORWARD';continue
        if any(np.max(np.abs(q-v))<1e-3 for v in seen):
            row['failure']='DUPLICATE_NUMERICAL_CONFIGURATION';continue
        seen.append(q.copy())
        actual=robot.fk(q)@np.linalg.inv(policy.tool_frames.flange_from_virtual_task_tcp)@policy.tool_frames.flange_from_physical_contact
        row['stack_facing']=initial_stack_facing_evidence(actual,cartons)
        audit=_audit_initial_state_prepared(policy.layout_validation,robot,shapes,fixed,cartons,q)
        row['initial_state_audit']=audit
        singular=np.linalg.svd(robot.geometric_jacobian(q),compute_uv=False)
        condition=float(singular[0]/singular[-1]) if singular[-1]>0 else None
        row['jacobian_condition']=condition
        maximum=float(policy.data['state_validity']['maximum_jacobian_condition'])
        row['maximum_jacobian_condition']=maximum
        passed=(audit['status']=='PASS' and row['stack_facing']['passed']
            and condition is not None and np.isfinite(condition) and condition<=maximum)
        row['failure']=None if passed else 'INITIAL_GEOMETRY_OR_ORIENTATION_OR_CONDITION_REJECTED'
        if passed:qualified.append(row)
    qualified.sort(key=lambda r:r['initial_direction']['selection_cost'])
    selected=None;backend=dict(status='NOT_RUN',reason='NO_GEOMETRIC_CANDIDATE')
    for row in qualified:
        q=row['q_rad']
        try:
            original,built,_=make_context('gap',layout_path=layout.config_path,initial_q=q)
        except ValueError as exc:
            if str(exc)!='PINOCCHIO_COAL_DEPENDENCY_UNAVAILABLE':raise
            backend=dict(status='UNAVAILABLE',reason=str(exc));selected=row
            break
        from unloading_sim.coupled_batch import BatchState,batch_scene
        from unloading_sim.carton_population import FrozenCartonPopulation
        state=BatchState(FrozenCartonPopulation.capture(original.cartons),tuple(original.cartons),np.asarray(q),.6)
        scene=batch_scene(original,state,built.robot,layout)
        failure=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
        row['exact_endpoint_failure']=failure
        if failure is None:
            selected=row;backend=dict(status='PASS',backend=type(built.robot).__name__);break
    accepted=selected is not None and backend['status']=='PASS'
    report=dict(schema='m710_front_working_initial_v1',status='PASS' if accepted else 'PREVIEW_ONLY_NOT_EXECUTION_ACCEPTED',
        home_definition='front-working',initial_forearm_direction_rule='STACK_FORWARD_INITIAL_ONLY',
        selected=selected,accepted_initial_state=accepted,exact_endpoint_validation=backend,
        gap_m=args.gap,mount_height_m=args.mount_height,layout_fingerprint=layout.layout_fingerprint,
        requested_physical_pose_world=physical.tolist(),seed=args.seed,maximum_draws=args.draws,
        attempts=rows,unsearched_draws=args.draws-len(rows),geometrically_qualified_configurations=len(qualified),
        front_arm_posture='none',front_endpoint_posture_gate=False,
        j1_fixed=False,initial_carton_count=40,historical_completed_count=0,
        historical_solution_loaded=False,native_motion_credit=False,physical_execution='NOT_RUN',
        wall_s=perf_counter()-began,scope='FRESH40_STATIC_INITIALIZATION_ONLY')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(output=str(args.output),status=report['status'],selected_q_deg=None if selected is None else selected['q_deg'],
        geometry_candidates=len(qualified),exact_validation=backend,wall_s=report['wall_s'])),flush=True)
    if not accepted and not (args.preview_only and selected is not None):raise SystemExit(2)
    return report
