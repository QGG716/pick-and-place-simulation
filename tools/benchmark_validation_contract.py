"""Small reproducible CPU comparison; run unchanged in baseline and current trees."""
import argparse,cProfile,json,pstats,sys
from pathlib import Path
from time import perf_counter
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.robot import URDFRobot
import unloading_sim.pinocchio_backend as backend_module

Q=np.array([-.37183334454120454,-.245641902048464,-.17740205572363327,1.7438672860074798,.37775574769199,-4.898300480640961])
CONTACT=np.array([-.35901958261242695,-.21005953163247132,-.1488362496014934,1.7325125925312939,.36397880205562794,-4.885213472402706])


def run(output):
    started=perf_counter(); policy=load_layout_motion_policy('configs/validation/m710id70_handoff_continuation.yaml')
    scene=build_verified_motion_input(policy)
    build=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
    c=build.connector
    assert c is not None,build.evidence
    prepare=perf_counter()-started
    c.start_planning_request();c._deadline_monotonic=None;c._request_deadline_monotonic=None
    if hasattr(c,'validation_budget'):c.validation_budget.deadline=None
    counts={}
    def instrument(owner,name,label):
        original=getattr(owner,name)
        def wrapped(*args,**kwargs):
            counts[label]=counts.get(label,0)+1
            return original(*args,**kwargs)
        setattr(owner,name,wrapped)
    for name in ('forwardKinematics','computeFrameJacobian','updateGeometryPlacements'):
        instrument(c.robot.pin,name,'pin_'+name)
    for name in ('distance','collide'): instrument(c.robot.coal,name,'coal_'+name)
    instrument(URDFRobot,'_frames_and_axes','python_chain_fk')
    instrument(backend_module,'_world_aabb','python_world_aabb')
    target=next(b for b in scene.cartons if b.name=='carton_l07_c02')
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(c.physical_from_virtual(c.robot.fk(CONTACT)),target),
        c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
    rows=[];profile=cProfile.Profile();profile.enable()
    for name,a,b,obstacles,options in [
        ('open_direct',Q,Q+np.array([.01,0,0,0,0,0]),[],dict(stage='pregrasp')),
        ('layout_direct',Q,Q+np.array([.001,0,0,0,0,0]),scene.all_obstacles,dict(stage='pregrasp')),
        ('loaded_near_stack',CONTACT,CONTACT+np.array([.001,0,0,0,0,0]),
         [x for x in scene.all_obstacles if x.name!=target.name],dict(stage='transit',attachment=attachment))]:
        # Independent cold context: retain the identical model and policy.
        c._state_cache.clear();c.robot_state_validator._static_cache.clear();c.robot_state_validator._geometry_cache.clear()
        if hasattr(c,'_motion_validators'):c._motion_validators.clear();c.validation_kernel.cache.clear()
        for cache_name in ('_obstacle_cache','_clearance_motion_cache'):
            getattr(c.robot,cache_name,{}).clear()
        for cache in ('cold','warm'):
            before=dict(counts);stats=dict(c._statistics);now=perf_counter()
            failure=c._path_failure([a,b],obstacles,**options)
            elapsed=perf_counter()-now
            rows.append(dict(case=name,cache=cache,seconds=elapsed,
                failure=failure,counts={k:v-before.get(k,0) for k,v in counts.items()},
                samples=c._statistics['edge_state_samples']-stats['edge_state_samples'],
                motion=getattr(c,'last_motion_validation',None)))
    profile.disable()
    # Standalone batch FK measurement uses the same official states in both modes.
    qs=Q+np.linspace(0,.01,512)[:,None]*np.array([1,0,0,0,0,0])
    urdf=c.robot_state_validator.tool_transform_robot
    now=perf_counter();reference=[urdf.fk(q) for q in qs];scalar_time=perf_counter()-now
    native_time=None
    if hasattr(c,'validation_kernel'):
        from unloading_sim.validation_kernel import batch_kinematics
        now=perf_counter();_,_,tcp,_=batch_kinematics(urdf,qs);native_time=perf_counter()-now
        np.testing.assert_allclose(tcp,reference,atol=2e-14)
    from unloading_sim.planner import RRTConnectPlanner
    predicate=lambda q: not (abs(q[0]-.25)<.022 and abs(q[1])<.18)
    motion=None
    if hasattr(c,'validation_kernel'):
        from unloading_sim.motion_validation import MotionValidator,ValidationContext
        motion=MotionValidator(ValidationContext.create({'scene':'thin_detour'},resolution_rad=.5),
            lambda q:None if predicate(q) else {'reason':'THIN_OBSTACLE'})
    detours=[]
    for cache in ('cold','warm'):
        now=perf_counter()
        planner=RRTConnectPlanner(np.array([0.,-.5]),np.array([1.,.5]),predicate,
            step_size=.15,edge_resolution=.5,max_iterations=600,rng=np.random.default_rng(9),
            **({} if motion is None else {'motion_validator':motion}))
        result=planner.plan(np.array([0.,0.]),np.array([1.,0.]))
        valid=result.success and all(predicate(q) for a,b in zip(result.path[:-1],result.path[1:])
            for q in np.linspace(a,b,2*max(1,int(np.ceil(np.max(np.abs(b-a))/.5)))+1))
        elapsed=perf_counter()-now
        detours.append(dict(cache=cache,search_success=result.success,strict_valid=valid,
            seconds=elapsed,first_strict_valid_seconds=elapsed if valid else None,evidence=result.search_evidence))
    output.parent.mkdir(parents=True,exist_ok=True)
    pstats.Stats(profile,stream=(output.with_suffix('.profile.txt')).open('w')).sort_stats('cumtime').print_stats(35)
    report=dict(python=sys.version,numpy=np.__version__,pinocchio=c.robot.pin.__version__,prepare_seconds=prepare,
        mode=getattr(getattr(c,'validation_kernel',None),'mode','PYTHON_SCALAR_REFERENCE'),cases=rows,
        fk_512=dict(scalar_seconds=scalar_time,native_seconds=native_time),
        kernel=getattr(getattr(c,'validation_kernel',None),'statistics',None),
        mesh_counters=getattr(c.robot,'performance_counters',{}),
        exact_counters=c.robot_state_validator.performance_counters,detour=detours)
    output.write_text(json.dumps(report,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v)))
    print(json.dumps(dict(output=str(output),prepare_seconds=prepare,cases=[{k:r[k] for k in ('case','cache','seconds','counts')} for r in rows],fk_512=report['fk_512']),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
