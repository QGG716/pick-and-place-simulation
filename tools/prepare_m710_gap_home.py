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


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gap',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tcp-xyz',nargs=3,type=float,default=[-.6,0.,2.25])
    p.add_argument('--seed',type=int,default=911109)
    p.add_argument('--draws',type=int,default=64)
    p.add_argument('--front-arm-posture',choices=('legacy-soft','upright'),default='legacy-soft')
    p.add_argument('--mount-height',type=float,choices=(.9,1.),default=1.)
    args=p.parse_args();started=time.monotonic()
    if args.output.exists():raise ValueError('INITIALIZATION_EVIDENCE_ALREADY_EXISTS')
    if args.draws<=0:raise ValueError('NONPOSITIVE_INITIAL_DRAWS')
    layout=frozen_layout(args.gap,args.mount_height)
    original,built,audit=make_context('gap',layout_path=layout.config_path,initial_seed=args.seed,initial_draws=64)
    state=BatchState(FrozenCartonPopulation.capture(original.cartons),tuple(original.cartons),
        np.asarray(original.snapshot['robot']['q_rad']),.6)
    scene=batch_scene(original,state,built.robot,layout)
    physical=physical_home_pose(args.tcp_xyz)
    virtual=physical@np.linalg.inv(built.flange_from_physical_contact)@built.flange_from_virtual_task_tcp
    rng=np.random.default_rng(args.seed);limits=built.robot.joint_limits
    proposals=rng.uniform(limits[:,0]+.05,limits[:,1]-.05,size=(args.draws,6))
    rows=[];selected=None
    for i,seed in enumerate(proposals):
        result=solve_ik(built.robot,virtual,seed,max_iterations=300,position_tolerance=.0001,orientation_tolerance=.0001)
        row=dict(index=i,seed_q_rad=seed.tolist(),q_rad=result.q.tolist(),converged=result.success,
            iterations=result.iterations,reason=result.message,position_error_m=result.position_error,
            orientation_error_rad=result.orientation_error)
        if result.success:
            row['failure']=built.validate_unloaded_state(result.q,scene.all_obstacles,stage='unloaded')
            row['stack_facing']=initial_stack_facing_evidence(built.physical_from_virtual(built.robot.fk(result.q)),original.cartons)
            from unloading_sim.front_arm_posture import front_arm_posture
            row['arm_posture']=front_arm_posture(built.robot,result.q,initial=True)
            if (row['failure'] is None and row['stack_facing']['passed']
                    and (args.front_arm_posture=='legacy-soft' or row['arm_posture']['passed'])):selected=row
        rows.append(row)
        if selected is not None:break
    report=dict(schema='gap_stack_facing_home_v1',gap_m=args.gap,layout_fingerprint=layout.layout_fingerprint,
        requested_physical_pose_world=physical.tolist(),seed=args.seed,maximum_draws=args.draws,
        tool_grid_orientation='SHORT_6_ROW_AXIS_WORLD_PLUS_Z_LONG_12_COLUMN_AXIS_WORLD_MINUS_Y',
        front_arm_posture=args.front_arm_posture,
        mount_height_m=args.mount_height,
        selected=selected,attempts=rows,unsearched_draws=args.draws-len(rows),
        wall_s=time.monotonic()-started,historical_solution_loaded=False,native_motion_credit=False,
        scope='FRESH40_STATIC_INITIALIZATION_ONLY',physical_execution='NOT_RUN')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ('selected','wall_s','unsearched_draws')}),flush=True)
    if selected is None:raise SystemExit(2)


if __name__=='__main__':main()
