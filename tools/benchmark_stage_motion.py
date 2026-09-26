"""Bounded same-input dispatch probes; no full historical search or Isaac.

The loaded probe calls the pre-change local-first sequence or the new free
connector on the same recorded, already extracted endpoints. It is a new
connection between those endpoints, not a new pick/place task. Analytic blocked
geometry isolates real RRT expansion; official contact uses the production
approach entry. Profiles are separate from three unprofiled repetitions.
"""
import argparse
import cProfile
import hashlib
import inspect
import json
from pathlib import Path
import pstats
import statistics
import sys
from time import perf_counter
import numpy as np

from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import LayoutTrajectoryConnector, PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState
from unloading_sim.planner import RRTConnectPlanner

Q = np.array([-.37183334454120454,-.245641902048464,-.17740205572363327,
              1.7438672860074798,.37775574769199,-4.898300480640961])
CONTACT = np.array([-.35901958261242695,-.21005953163247132,-.1488362496014934,
                    1.7325125925312939,.36397880205562794,-4.885213472402706])


def main(args):
    modern = 'purpose' in inspect.signature(LayoutTrajectoryConnector._connect_pose).parameters
    policy = load_layout_motion_policy('configs/validation/m710id70_layout_v1_single_carton.yaml')
    scene = build_verified_motion_input(policy)
    c = _build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot()).connector
    history = json.loads((args.history/'planning/motion.json').read_text(encoding='utf-8'))
    actual = json.loads((args.history/'inputs/actual_remaining_state.json').read_text(encoding='utf-8'))
    loaded_policy = load_layout_motion_policy('configs/validation/m710id70_handoff_continuation.yaml')
    initial = build_verified_motion_input(loaded_policy)
    rows = RowUnloadingState();rows.rank(initial.cartons,support_graph=initial.support_graph)
    actual_scene = apply_actual_motion_state(initial,actual,row_state=rows)
    loaded = _build_automatic_trajectory_connector(actual_scene,loaded_policy.layout_validation.layout.robot()).connector
    segment=history['selected_trajectory_segment'];nodes=np.asarray(segment['path'])
    target=next(b for b in actual_scene.cartons if b.name==segment['target'])
    rigid=RigidAttachment(np.asarray(segment['contact']['physical_contact_from_box']),target.half_extents.copy(),target.name)
    attachment=PhysicalContactAttachment(loaded.robot,rigid,loaded.flange_from_virtual_task_tcp,loaded.flange_from_physical_contact)
    obstacles=[b for b in actual_scene.all_obstacles if b.name!=target.name]
    # Same short historical endpoints as the preceding kernel benchmark.
    a,b=nodes[120:122];b=a+min(1.,.004/np.abs(b-a).sum())*(b-a)
    recorded=[]
    real_plan=RRTConnectPlanner.plan
    def observe_rrt(self,*a,**kw):
        result=real_plan(self,*a,**kw)
        recorded.append(dict(result.search_evidence))
        return result
    RRTConnectPlanner.plan=observe_rrt

    def connect(c,start,end,obs,attachment=None):
        purpose={'purpose':'FREE_LOADED_TRANSFER' if attachment else 'FREE_APPROACH'} if modern else {}
        _,path,failure,trace=c._connect_pose(c.robot.fk(end),[start,end],start,obs,
            attachment=attachment,stage='transit' if attachment else 'pregrasp',
            ik_seed=3,connection_seed=44,**purpose)
        return path,failure,trace
    def loaded_connection():
        if not modern:
            path,failure,trace=loaded._local_cartesian_transit(a,loaded.robot.fk(b),obstacles,attachment,seed=55)
            if failure is None:return path,failure,dict(trace,selected_method='LOCAL_CARTESIAN_CANDIDATE')
        return connect(loaded,a,b,obstacles,attachment)
    def contact():
        target=next(b for b in scene.cartons if b.name=='carton_l07_c02')
        c._contact_selection(CONTACT,target,'front',scene.policy.data['suction'])
        prefix,terminal,failure,trace=c._approach(Q,CONTACT,c.robot.fk(CONTACT),scene.all_obstacles,target,seed=7)
        return prefix+terminal,failure,trace

    # Actual existing RRT and scalar collision geometry, no successful-path stub.
    sys.path.insert(0,str(Path.cwd()/'tests'))
    from test_extraction_boundary_guard import fixture
    analytic,_,_,_,_=fixture()
    analytic.ik.update(random_restarts=0,candidate_dedup_tolerance_rad=.01,candidate_dedup_tolerance_m=.001)
    from unloading_sim.geometry import OBB
    aa=np.array([0.,0.,1.,0.,0.,0.]);bb=aa+np.array([.4,0,0,0,0,0])
    wall=[OBB([.2,0,1],[.025,.06,.06],np.eye(3),'wall','wall')]
    def blocked():
        return analytic._transit(aa,bb,wall,stage='pregrasp',seed=44,iteration_budget=200,
            **({'purpose':'FREE_APPROACH'} if modern else {}))
    cases=[('official_empty_direct',c,lambda:connect(c,Q,Q+np.array([.01,0,0,0,0,0]),[])),
           ('official_loaded_historical_endpoints_new_connection',loaded,loaded_connection),
           ('analytic_obstructed_real_rrt',analytic,blocked),('official_contact_process',c,contact)]
    results=[]
    for name,owner,run in cases:
        observations=[]
        def reset():
            owner.start_planning_request()
            owner.stack_carton_names={b.name for b in (actual_scene.cartons if owner is loaded else scene.cartons)}
            owner._local_transit_remaining=owner.budget.local_transit_cartesian_sample_budget
            if owner is loaded:
                owner.robot_state_validator.commanded_cup_mask=tuple(segment['contact']['cup_selection']['commanded_active_mask'])
                owner.robot_state_validator.contact_target_name=target.name
        def measure(temperature):
            recorded.clear();before=dict(owner._statistics);contexts=dict(getattr(owner,"context_statistics",{}))
            started=perf_counter();path,failure,trace=run();seconds=perf_counter()-started
            delta={key:owner._statistics[key]-before[key] for key in before if isinstance(before[key],(int,float))}
            return dict(temperature=temperature,seconds=seconds,path_nodes=len(path),failure=failure,
                selected_method=trace.get('selected_method'),trace=trace,work=delta,
                context_delta={k:v-contexts.get(k,0) for k,v in getattr(owner,"context_statistics",{}).items()},
                rrt_interface_calls=len(recorded),rrt_extensions=sum(r.get('extension_attempts',0) for r in recorded),
                rrt_iterations=sum(r.get('planning_iterations_consumed',0) for r in recorded))
        for repeat in range(3):
            reset()
            observations.extend([measure('cold'),measure('warm')])
        reset();measure('prime');prof=cProfile.Profile();prof.enable();warm=measure('profile_warm');prof.disable()
        stats=pstats.Stats(prof).stats
        profile={label:sum(v[0] for key,v in stats.items() if key[2] in names) for label,names in {
            'full_context_builds':{'_validation_context'},'context_hashes':{'_identity'},
            'json_calls':{'dumps'}}.items()}
        result=dict(case=name,runs=observations,median_seconds={t:statistics.median(r['seconds'] for r in observations if r['temperature']==t)
            for t in ('cold','warm')},warm_profile=profile)
        results.append(result)
        print(name,result['median_seconds'],observations[0]['failure'],flush=True)
    # Isolate unchanged lightweight guard work from endpoint IK and optional
    # simplification, which legitimately prepare additional stage contracts.
    c.start_planning_request();probe_obstacles=scene.all_obstacles
    v=c._motion_validator(probe_obstacles,stage='pregrasp')
    before=dict(c.context_statistics);prof=cProfile.Profile();prof.enable()
    for _ in range(100):
        assert c._motion_validator(probe_obstacles,stage='pregrasp') is v
        assert v.context_current()==v.context.context_id
    prof.disable();stats=pstats.Stats(prof).stats
    guards=dict(repeats=100,context_delta={k:value-before.get(k,0) for k,value in c.context_statistics.items()},
        full_context_builds=sum(v[0] for k,v in stats.items() if k[2]=='_validation_context'),
        json_calls=sum(v[0] for k,v in stats.items() if k[2]=='dumps'),
        sha_calls=sum(v[0] for k,v in stats.items() if 'openssl_sha256' in k[2]))
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(inspect.getfile(LayoutTrajectoryConnector)).parent.glob('*.py')}
    args.output.write_text(json.dumps(dict(modern_dispatch=modern,source_sha256=hashes,
        scope='short new connections and process probe; no full task/Isaac',results=results,
        hot_binding_guards=guards),indent=2),encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--history',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);main(parser.parse_args())
