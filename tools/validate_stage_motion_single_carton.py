"""One fresh connector plan from an archived actual state and one grasp seed.

No historical path is supplied, no budget is restarted, and no Isaac is launched.
The full audit/export is intentionally conditional on a complete new result.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_sim.layout_single_carton import (load_layout_motion_policy, build_verified_motion_input,
    _build_automatic_trajectory_connector, motion_implementation_identity)
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState
from unloading_sim.planning_profile import DEFAULT_MOTION


def main(args):
    motion_path=args.history/'planning/motion.json';state_path=args.history/'inputs/actual_remaining_state.json'
    prior=json.loads(motion_path.read_text(encoding='utf-8'))['selected_trajectory_segment']
    policy=load_layout_motion_policy(DEFAULT_MOTION);scene=build_verified_motion_input(policy)
    rows=RowUnloadingState();rows.rank(scene.cartons,support_graph=scene.support_graph)
    scene=apply_actual_motion_state(scene,json.loads(state_path.read_text(encoding='utf-8')),row_state=rows)
    build=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
    c=build.connector
    if c is None:raise RuntimeError(build.evidence)
    target=next(b for b in scene.cartons if b.name==prior['target'])
    grasp=np.asarray(prior['path'][prior['grasp_index']])
    c.start_planning_request();started=perf_counter()
    result=c.plan(target=target,face=prior['face'],
        requested_virtual_contact=np.asarray(prior['contact']['requested_virtual_task_tcp_pose_world']),
        grasp_candidates=[dict(q_rad=grasp.tolist(),candidate_id='archived_contact_seed_only')],
        home_q=scene.policy.layout_validation.initial_q,all_obstacles=scene.all_obstacles,
        receiver=scene.receiver,support_names=tuple(scene.support_graph.supported_by[target.name]),
        suction=scene.policy.data['suction'],seed=71070)
    document=dict(scope='NEW_SINGLE_CANDIDATE_COMPLETE_CONNECTOR_PLAN',
        historical_path_revalidation=False,historical_path_supplied=False,
        elapsed_seconds=perf_counter()-started,source=motion_implementation_identity(Path.cwd()),
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (motion_path,state_path)},
        target=target.name,result=asdict(result),isaac_executed=False,
        execution_ready=False,independent_preflight_run=False)
    args.output.write_text(json.dumps(document,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else str(x)),encoding='utf-8')
    print(json.dumps(dict(success=result.success,failure=result.failure,elapsed_seconds=document['elapsed_seconds']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--history',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
