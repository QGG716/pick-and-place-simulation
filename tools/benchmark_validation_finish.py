"""Same-input short CPU comparisons against 720fafc; never starts Isaac/search."""
import argparse
import cProfile
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import pstats
import resource
import statistics
import sys
from time import perf_counter
import tracemalloc
import numpy as np

from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowSequencePolicy,RowUnloadingState
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.motion_validation import MotionValidator,ValidationContext

Q=np.array([-.37183334454120454,-.245641902048464,-.17740205572363327,1.7438672860074798,.37775574769199,-4.898300480640961])


def binding_probe(c, scene):
    """Preparation-only costs, separated from edge validation and copying inputs."""
    rows=[]
    for mode in ('deepcopy', 'new_container'):
        for repeat in range(3):
            c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
            a=list(deepcopy(scene.all_obstacles));va=c._motion_validator(a,stage='pregrasp')
            assert va.check_motion(Q,Q+.000001).valid
            b=deepcopy(a) if mode=='deepcopy' else list(a)
            before=dict(c.context_statistics);start=perf_counter()
            vb=c._motion_validator(b,stage='pregrasp');seconds=perf_counter()-start
            delta={k:v-before.get(k,0) for k,v in c.context_statistics.items()}
            first=vb.check_motion(Q,Q+.000001)
            start=perf_counter();again=c._motion_validator(b,stage='pregrasp');hot_seconds=perf_counter()-start
            hot=again.check_motion(Q,Q+.000001)
            assert again is vb and hot.cache_hit
            if mode=='deepcopy':b[-1].center[0]+=.001
            else:b.pop()
            result=vb.check_motion(Q,Q+.000001)
            rows.append(dict(mode=mode,repeat=repeat,prepare_seconds=seconds,hot_prepare_seconds=hot_seconds,
                context_delta=delta,same_validator=va is vb,same_semantic_id=va.context.context_id==vb.context.context_id,
                first_status=first.status.value,hot_edge_cache_hit=hot.cache_hit,
                after_mutation=result.evidence(),old_binding_status=va.check_motion(Q,Q+.000001).status.value))
    # Isolate the actual hot lookup/guard from report serialization.
    c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
    obstacles=list(scene.all_obstacles);v=c._motion_validator(obstacles,stage='pregrasp')
    assert v.check_motion(Q,Q+.000001).valid
    before=dict(c.context_statistics);prof=cProfile.Profile();prof.enable()
    for _ in range(100):
        assert c._motion_validator(obstacles,stage='pregrasp') is v
        assert v.context_current()==v.context.context_id
    prof.disable();stats=pstats.Stats(prof).stats
    counts={label:sum(value[0] for key,value in stats.items() if key[2] in names)
        for label,names in {'context_builds':{'_validation_context'},'json_serializations':{'dumps'},
            'sha256_calls':{'<built-in method _hashlib.openssl_sha256>'}}.items()}
    return dict(runs=rows,hot_lookup_guard=dict(repeats=100,profile_counts=counts,
        context_delta={k:v-before.get(k,0) for k,v in c.context_statistics.items()}))


def run(args):
    history=Path(args.history_dir);motion_path=history/'planning/motion.json';actual_path=history/'inputs/actual_remaining_state.json'
    motion=json.loads(motion_path.read_text());actual=json.loads(actual_path.read_text())
    segment=motion['selected_trajectory_segment'];path=np.asarray(segment['path'])
    begin=perf_counter()
    policy=load_layout_motion_policy('configs/validation/m710id70_handoff_continuation.yaml')
    initial=build_verified_motion_input(policy);strategy=policy.data.get('search_strategy',{})
    rows=RowUnloadingState(RowSequencePolicy(row_height_fraction=float(strategy.get('row_height_fraction',.05))))
    rows.rank(initial.cartons,support_graph=initial.support_graph)
    scene=apply_actual_motion_state(initial,actual,row_state=rows)
    build=_build_automatic_trajectory_connector(scene,scene.policy.layout_validation.layout.robot())
    assert build.connector is not None,build.evidence
    c=build.connector;rv=c.robot_state_validator;k=c.validation_kernel
    setup_seconds=perf_counter()-begin
    target=next(b for b in scene.cartons if b.name==segment['target'])
    rigid=RigidAttachment(np.asarray(segment['contact']['physical_contact_from_box']),target.half_extents.copy(),target.name)
    attachment=PhysicalContactAttachment(c.robot,rigid,c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
    obstacles=[b for b in scene.all_obstacles if b.name!=target.name]
    mask=segment['contact']['cup_selection']['commanded_active_mask']
    cases=[('open_direct',Q,Q+np.array([.01,0,0,0,0,0]),[],dict(stage='pregrasp')),
           ('very_short_valid',Q,Q+np.array([.00001,0,0,0,0,0]),scene.all_obstacles,dict(stage='pregrasp')),
           ('loaded_early_invalid',path[segment['grasp_index']],path[segment['grasp_index']]+np.array([.001,0,0,0,0,0]),obstacles,dict(stage='transit',attachment=attachment))]
    fragments=[]
    for name,stage,index in [('history_extract_start','extraction',38),('history_extract_middle','extraction',58),('history_loaded_transit','transit',120)]:
        a,b=path[index:index+2];fraction=min(1.,.004/max(float(np.abs(b-a).sum()),1e-15));end=a+fraction*(b-a)
        cases.append((name,a,end,obstacles,dict(stage=stage,attachment=attachment)))
        fragments.append(dict(name=name,original_edge=[index,index+1],fraction=[0.,fraction],start=a.tolist(),end=end.tolist(),stage=stage))
    if args.cases:
        unknown=set(args.cases)-{case[0] for case in cases}
        if unknown:raise ValueError(f'unknown cases: {sorted(unknown)}')
        cases=[case for case in cases if case[0] in args.cases]
        fragments=[item for item in fragments if item['name'] in args.cases]

    def cold(options):
        c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
        c._state_cache.clear();rv._geometry_cache.clear();rv._static_cache.clear();k.cache.clear()
        if hasattr(k,'_tool_template_key'):k._tool_template_key=None
        for name in ('_obstacle_cache','_clearance_motion_cache'):
            getattr(c.robot,name,{}).clear()
        if hasattr(c,'_prepared_contexts'):c._prepared_contexts.clear()
        c.stack_carton_names={b.name for b in scene.cartons};rv.stack_carton_names=set(c.stack_carton_names)
        rv.commanded_cup_mask=tuple(mask);rv.contact_target_name=target.name
        if options['stage']=='extraction':
            tracker,error=c._initial_proximity(attachment.box_at(path[segment['grasp_index']]),obstacles,())
            assert error is None,error
            return dict(options,initial_proximity=tracker)
        return dict(options)

    def observe(a,b,obs,options):
        before_k=dict(k.statistics);before_c=dict(c._statistics)
        before_context=dict(getattr(c,'context_statistics',{}))
        v=c._motion_validator(obs,**options);before_v=dict(v.statistics)
        result=v.check_motion(a,b,c._validation_request())
        return dict(status=result.status.value,failure=result.failure,guarantee=result.guarantee,
            strict_grid_samples=v.context.samples(a,b)+1,
            fk_prepared=k.statistics['fk_states']-before_k['fk_states'],
            expensive_states=c._statistics['state_validations']-before_c['state_validations'],
            state_dispatches=v.statistics['state_samples']-before_v['state_samples'],
            state_cache_hits=c._statistics['state_cache_hits']-before_c['state_cache_hits'],
            edge_cache_hit=result.cache_hit,validation=result.evidence(),
            kernel_delta={n:k.statistics[n]-before_k.get(n,0) for n in k.statistics},
            context_delta={n:value-before_context.get(n,0) for n,value in getattr(c,'context_statistics',{}).items()})

    exact_counts={}
    for function_name in ('collide','distance'):
        original=getattr(c.robot.coal,function_name)
        def wrapper(*a,_name=function_name,_original=original,**kw):
            exact_counts[_name]=exact_counts.get(_name,0)+1
            return _original(*a,**kw)
        setattr(c.robot.coal,function_name,wrapper)
    report=[]
    for name,a,b,obs,base_options in cases:
        times={'cold':[],'warm':[]};last={}
        for repeat in range(3):
            options=cold(base_options)
            for temperature in ('cold','warm'):
                # Stateful history is replayed from the same captured checkpoint.
                # No validity cache is permitted for its ordered observations.
                if temperature=='warm' and 'initial_proximity' in options:
                    tracker,error=c._initial_proximity(attachment.box_at(path[segment['grasp_index']]),obstacles,())
                    assert error is None;options=dict(options,initial_proximity=tracker)
                start=perf_counter();last[temperature]=observe(a,b,obs,options);times[temperature].append(perf_counter()-start)
        # Profiles/counts measured separately so profiler overhead is not mixed
        # into the unprofiled three-repeat timing median.
        options=cold(base_options);exact_before=dict(exact_counts);prof=cProfile.Profile();prof.enable();detail=observe(a,b,obs,options);prof.disable()
        ps=pstats.Stats(prof);counts={};work_seconds={}
        for label,names in {'context_builds':{'_validation_context'},'json_serializations':{'dumps'},
            'sha256_calls':{'<built-in method _hashlib.openssl_sha256>'},'obb_constructions':{'__post_init__'},
            'transform_constructions':{'make_transform'},'exact_collision':{'<built-in method collide>'},
            'exact_distance':{'<built-in method distance>'}}.items():
            counts[label]=sum(v[0] for key,v in ps.stats.items() if key[2] in names and
                (label!='obb_constructions' or key[0].endswith('/geometry.py')))
            work_seconds[label]=sum(v[3] for key,v in ps.stats.items() if key[2] in names and
                (label!='obb_constructions' or key[0].endswith('/geometry.py')))
        counts['exact_collision']=exact_counts.get('collide',0)-exact_before.get('collide',0)
        counts['exact_distance']=exact_counts.get('distance',0)-exact_before.get('distance',0)
        if 'initial_proximity' in options:
            tracker,error=c._initial_proximity(attachment.box_at(path[segment['grasp_index']]),obstacles,())
            assert error is None;options=dict(options,initial_proximity=tracker)
        warm_prof=cProfile.Profile();warm_prof.enable();warm_detail=observe(a,b,obs,options);warm_prof.disable()
        warm_stats=pstats.Stats(warm_prof).stats
        warm_counts={label:sum(value[0] for key,value in warm_stats.items() if key[2] in names)
            for label,names in {'context_builds':{'_validation_context'},'json_serializations':{'dumps'},
                'sha256_calls':{'<built-in method _hashlib.openssl_sha256>'}}.items()}
        hotspots=sorted([dict(file=Path(key[0]).name,line=key[1],function=key[2],calls=value[0],self_seconds=value[2],cumulative_seconds=value[3])
                         for key,value in ps.stats.items()],key=lambda r:r['cumulative_seconds'],reverse=True)[:35]
        profile_path=args.output.parent/(args.output.stem+'_'+name+'.profile.txt')
        with profile_path.open('w') as stream:pstats.Stats(prof,stream=stream).sort_stats('cumtime').print_stats(45)
        report.append(dict(case=name,timing_seconds=times,median_seconds={k:statistics.median(v) for k,v in times.items()},
            observed=last,profile_counts=counts,profile_cumulative_seconds=work_seconds,profile_result=detail,
            warm_profile_counts=warm_counts,warm_profile_result=warm_detail,hotspots=hotspots))
        print(name,{k:round(statistics.median(v),6) for k,v in times.items()},last['cold']['status'],flush=True)

    # Deterministic middle-obstacle geometry, separate from official mesh cases.
    from unloading_sim.geometry import OBB
    obstacle=OBB([.25,0,0],[.01,.1,.1],np.eye(3),'thin_middle')
    seen=[]
    def scalar(q):
        seen.append(float(q[0]))
        return dict(reason='POINT_OBSTACLE',pair=['point',obstacle.name]) if obstacle.contains([q[0],0,0]) else None
    middle=MotionValidator(ValidationContext.create({'obstacle':[.25,.01]},resolution_rad=1/64),scalar)
    middle_times=[];middle_runs=[]
    for repeat in range(0 if args.cases else 3):
        middle.states.clear();middle.edges.clear();seen.clear();start=perf_counter();r=middle.check_motion([0],[1]);middle_times.append(perf_counter()-start)
        first=next(i for i,x in enumerate(seen) if abs(x-.25)<=.01)
        middle_runs.append(dict(result=r.evidence(),actual_checks=len(seen),first_failed_sample=first,extra_checks_after_failure=len(seen)-first-1))
    # Python traced memory is a separate, untimed representative cold check.
    name,a,b,obs,base_options=cases[0];options=cold(base_options)
    tracemalloc.start();observe(a,b,obs,options);retained,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
    data=dict(baseline=args.baseline_commit,python=sys.version,numpy=np.__version__,
        setup_seconds=setup_seconds,source_files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path('src').rglob('*.py'))},
        inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (motion_path,actual_path)},history_fragments=fragments,
        cases=report,
        middle_obstacle=None if not middle_times else dict(times=middle_times,median_seconds=statistics.median(middle_times),runs=middle_runs),
        memory=dict(python_retained_bytes=retained,python_peak_bytes=peak,process_max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            cache_sizes=dict(fk=len(k.cache),connector_states=len(c._state_cache),geometry=len(rv._geometry_cache),static=len(rv._static_cache))),
        note='CPU fragment revalidation only; no new candidate search, no historical fingerprint rewrite, no Isaac.')
    if args.binding_probe:data['binding_probe']=binding_probe(c,scene)
    args.output.write_text(json.dumps(data,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--history-dir',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',help='Run only these existing short cases (also omit synthetic middle case).')
    parser.add_argument('--baseline-commit',default='720fafc6a65765dab849cf75ac873969c2f3aafc')
    parser.add_argument('--binding-probe',action='store_true')
    args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True);run(args)
