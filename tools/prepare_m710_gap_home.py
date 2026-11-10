"""Find a fresh, stack-facing unloaded initial state at one explicit home pose.

This is static initialization, not a trajectory or completed pick. No native
motion, previous solution, target grasp pose or historical state is loaded.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from plan_m710_gap_search import frozen_layout
from plan_m710_mount_comparison import make_context
from unloading_sim.coupled_batch import BatchState,batch_scene
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.gap_search_layout import initial_stack_facing_evidence
from unloading_sim.geometry import make_transform,make_tool_rotation
from unloading_sim.ik import solve_ik


def physical_home_pose(xyz):
    # Same physical cup-grid convention as a FRONT grasp: six short rows
    # point up (+Z), twelve long columns run horizontally (-Y), cups face +X.
    return make_transform(make_tool_rotation(np.array([1.,0.,0.]),np.array([0.,-1.,0.])),xyz)


def upright_stack_home_q():
    # Explicit official-model joint definition, not an imported previous solution.
    # The actual link directions, cup orientation and collisions are checked below.
    # A small wrist bend avoids the straight-wrist singularity without changing
    # J4-to-J5 direction. This is a candidate, not an exact-axis acceptance gate.
    # Cups retain the existing one-degree facing tolerance.
    return np.array([np.pi, 0., np.pi, 0., np.deg2rad(.5), 0.])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gap',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tcp-xyz',nargs=3,type=float)
    p.add_argument('--home-definition',choices=('front-working','pose','stack-facing-upright'),default='front-working')
    p.add_argument('--preview-only',action='store_true',help='Allow a clearly unaccepted static preview if the exact backend is unavailable')
    p.add_argument('--seed',type=int,default=911109)
    p.add_argument('--draws',type=int,default=64)
    p.add_argument('--front-arm-posture',choices=('none','legacy-soft','upright'),default='none')
    p.add_argument('--mount-height',type=float,choices=(.9,1.),default=1.)
    args=p.parse_args();started=time.monotonic()
    if args.output.exists():raise ValueError('INITIALIZATION_EVIDENCE_ALREADY_EXISTS')
    if args.draws<=0:raise ValueError('NONPOSITIVE_INITIAL_DRAWS')
    if args.home_definition=='front-working':
        from prepare_m710_front_working_home import prepare_front_working_home
        prepare_front_working_home(args)
        return
    if args.preview_only:raise ValueError('PREVIEW_ONLY_IS_FOR_FRONT_WORKING_INITIALIZATION')
    if args.home_definition=='stack-facing-upright' and args.tcp_xyz is not None:
        raise ValueError('UPRIGHT_HOME_DERIVES_TCP_DO_NOT_SPECIFY_POSE')
    layout=frozen_layout(args.gap,args.mount_height)
    explicit_q=upright_stack_home_q() if args.home_definition=='stack-facing-upright' else None
    original,built,audit=make_context('gap',layout_path=layout.config_path,
        initial_q=None if explicit_q is None else explicit_q.tolist(),
        initial_seed=args.seed if explicit_q is None else None,initial_draws=64)
    state=BatchState(FrozenCartonPopulation.capture(original.cartons),tuple(original.cartons),
        np.asarray(original.snapshot['robot']['q_rad']),.6)
    scene=batch_scene(original,state,built.robot,layout)
    physical=(physical_home_pose(args.tcp_xyz or [-.6,0.,2.25]) if explicit_q is None else
        built.physical_from_virtual(built.robot.fk(explicit_q)))
    virtual=physical@np.linalg.inv(built.flange_from_physical_contact)@built.flange_from_virtual_task_tcp
    rng=np.random.default_rng(args.seed);limits=built.robot.joint_limits
    proposals=(rng.uniform(limits[:,0]+.05,limits[:,1]-.05,size=(args.draws,6)) if explicit_q is None else [explicit_q])
    rows=[];selected=None
    for i,seed in enumerate(proposals):
        if explicit_q is None:
            result=solve_ik(built.robot,virtual,seed,max_iterations=300,position_tolerance=.0001,orientation_tolerance=.0001)
            q=result.q;success=result.success
            row=dict(index=i,seed_q_rad=seed.tolist(),q_rad=q.tolist(),converged=success,
                iterations=result.iterations,reason=result.message,position_error_m=result.position_error,
                orientation_error_rad=result.orientation_error,method='FRESH_NUMERICAL_IK')
        else:
            q=explicit_q;success=True
            row=dict(index=i,q_rad=q.tolist(),method='EXPLICIT_OFFICIAL_JOINT_DEFINITION_WITH_FK_CHECKS',
                inverse_kinematics_run=False)
        if success:
            row['failure']=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
            actual=built.physical_from_virtual(built.robot.fk(q))
            row['stack_facing']=initial_stack_facing_evidence(actual,original.cartons)
            from unloading_sim.front_arm_posture import front_arm_posture,arm_posture_policy
            row['arm_posture']=front_arm_posture(built.robot,q,initial=True)
            row['arm_posture_policy']=arm_posture_policy(args.front_arm_posture)
            reference=physical_home_pose([0,0,0])[:3,:3]
            angle=float(np.arccos(np.clip((np.trace(reference.T@actual[:3,:3])-1)/2,-1.,1.)))
            tolerance=float(np.deg2rad(1.)) if explicit_q is not None else .0002
            row['cup_grid_orientation_error_rad']=angle
            row['maximum_cup_grid_orientation_error_rad']=tolerance
            row['cup_grid_orientation_passed']=bool(angle<=tolerance+1e-12)
            if (row['failure'] is None and row['stack_facing']['passed']
                    and row['cup_grid_orientation_passed']
                    and (not row['arm_posture_policy']['initial_hard_gate'] or row['arm_posture']['passed'])):selected=row
        rows.append(row)
        if selected is not None:break
    maximum_draws=args.draws if explicit_q is None else 1
    report=dict(schema='gap_stack_facing_home_v1',gap_m=args.gap,layout_fingerprint=layout.layout_fingerprint,
        requested_physical_pose_world=physical.tolist(),seed=args.seed,maximum_draws=maximum_draws,
        home_definition=args.home_definition,initial_forearm_direction_rule=(
            'UPRIGHT_FORWARD_NOT_EXACT_AXIS' if args.front_arm_posture=='upright' else
            'UNCONSTRAINED_ARM_POSTURE' if args.front_arm_posture=='none' else 'LEGACY_SOFT'),
        tool_grid_orientation='SHORT_6_ROW_AXIS_WORLD_PLUS_Z_LONG_12_COLUMN_AXIS_WORLD_MINUS_Y',
        front_arm_posture=args.front_arm_posture,
        mount_height_m=args.mount_height,
        selected=selected,attempts=rows,unsearched_draws=maximum_draws-len(rows),
        wall_s=time.monotonic()-started,historical_solution_loaded=False,native_motion_credit=False,
        scope='FRESH40_STATIC_INITIALIZATION_ONLY',physical_execution='NOT_RUN')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ('selected','wall_s','unsearched_draws')}),flush=True)
    if selected is None:raise SystemExit(2)


if __name__=='__main__':main()
