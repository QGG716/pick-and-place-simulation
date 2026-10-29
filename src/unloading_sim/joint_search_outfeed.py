"""Explicit sampled ideal conveyor motion while the complete robot waits."""
import math
from copy import deepcopy
import numpy as np
from .geometry import OBB
from .lifted_reception import bounds
from .moveit2_backend import box_message
from .pair_clearance import obb_surface_distance

SCHEMA='fixed40_wait_outfeed_v1'
WAIT_FIRST='WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW'
WITHDRAW_FIRST='WITHDRAW_THEN_WAIT_FULL_OUTFEED'
PARK_FIRST='WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED'

def withdrawal_extra(option):
    extra=option.get('withdraw_extra_m',0.)
    timing=option.get('outfeed_timing',WAIT_FIRST)
    allowed=(0.,.10) if timing==PARK_FIRST else (0.,.10,.30,.50) if timing==WITHDRAW_FIRST else (0.,)
    if extra not in allowed:
        raise ValueError('OUTFEED_FINITE_WITHDRAWAL_DISTANCE')
    return float(extra)

def outfeed_path(box,world,*,step=.005,speed=.2,tool_timing=WAIT_FIRST):
    if tool_timing not in (WAIT_FIRST,WITHDRAW_FIRST,PARK_FIRST):raise ValueError('OUTFEED_TIMING_UNSUPPORTED')
    if not 0<step<=.005 or not speed>0:raise ValueError('OUTFEED_RESOLUTION_OR_SPEED')
    parts={b.name:b for b in world};belt=parts['conveyor_transverse'];long=parts['conveyor_longitudinal']
    hx=float((np.abs(box.rotation)@box.half_extents)[0]);hy=float((np.abs(box.rotation)@box.half_extents)[1])
    hz=float((np.abs(box.rotation)@box.half_extents)[2]);seam=belt.center[0]-belt.half_extents[0]
    height=belt.center[2]+belt.half_extents[2];slope=long.rotation[2,0]/long.rotation[0,0]
    y=min(float(box.center[1]),float(long.center[1]));long_lo,long_hi=bounds(long)
    if y-hy<long_lo[1]+.005 or y+hy>long_hi[1]-.005:raise ValueError('OUTFEED_FULL_WIDTH_UNSUPPORTED')
    waypoints=[box.center.copy()]
    if y<box.center[1]-1e-12:waypoints.append(np.array([box.center[0],y,box.center[2]]))
    exit_x=-3.2-hx-.001
    waypoints.append(np.array([exit_x,y,box.center[2]]))
    samples=[dict(t=0.,box=box_message(box))];time=0.;previous=box.center.copy()
    for a,b in zip(waypoints,waypoints[1:]):
        n=max(1,math.ceil(np.linalg.norm(b-a)*math.sqrt(1+slope*slope)/step))
        for alpha in np.linspace(0,1,n+1)[1:]:
            p=a+(b-a)*alpha
            if p[0]-hx<seam:
                long_x=min(p[0]+hx,seam) if slope>=0 else max(p[0]-hx,long_lo[0])
                support=height+(long_x-seam)*slope
                if p[0]+hx>=seam:support=max(support,height)
                p[2]=support+hz
            time+=float(np.linalg.norm(p-previous))/speed
            samples.append(dict(t=time,box=box_message(OBB(p,box.half_extents,box.rotation,box.name,box.category))))
            previous=p.copy()
    if len(samples)>4096:raise ValueError('OUTFEED_SAMPLE_RESOURCE_LIMIT')
    return dict(schema=SCHEMA,samples=samples,maximum_displacement_m=step,speed_m_s=speed,
        output_plane_x_m=-3.2,tool_timing=tool_timing,
        model='IDEAL_KINEMATIC_SUPPORT_ENVELOPE_NOT_DYNAMICS',continuous_swept_proof=False,
        physical_execution='NOT_RUN')

def check_full_tool_wait(connector,q,path):
    # Include uncompressed flexible lips; the native worker additionally checks
    # the official full robot meshes and its complete collision-tool model.
    tools=list(connector.tool_collision_obbs_provider(q))
    if not tools:raise ValueError('OUTFEED_FULL_TOOL_GEOMETRY_MISSING')
    cups=[b for b in tools if b.name.startswith('tool_compliant_bellows_')]
    centers=np.asarray([b.center for b in tools]);extents=np.asarray([np.abs(b.rotation)@b.half_extents for b in tools])
    from .coupled_batch import message_box
    checks=[]
    for i,sample in enumerate(path['samples']):
        box=message_box(sample['box']);extent=np.abs(box.rotation)@box.half_extents
        gaps=np.maximum(np.abs(centers-box.center)-extents-extent,0.)
        for k in np.flatnonzero(np.linalg.norm(gaps,axis=1)<.005000001):
            gap=obb_surface_distance(box,tools[k])
            if gap<.005-1e-9:return dict(passed=False,sample_index=i,pair=[box.name,tools[k].name],surface_gap_m=gap)
        checks.append(i)
    return dict(passed=True,tool_solids=len(tools),uncompressed_cup_solids=len(cups),samples=len(checks),
        required_clearance_m=.005,q_rad=np.asarray(q).tolist(),continuous_swept_proof=False)

def check_full_tool_park(connector,path,stationary,*,step=.01):
    if not 0<step<=.01:raise ValueError('PARK_JOINT_RESOLUTION')
    samples=[np.asarray(path[0])]
    for a,b in zip(path,path[1:]):
        a=np.asarray(a);b=np.asarray(b)
        n=max(1,math.ceil(float(np.max(np.abs(b-a)))/step))
        samples.extend(a+f*(b-a) for f in np.linspace(0,1,n+1)[1:])
        if len(samples)>10000:raise ValueError('PARK_SAMPLE_RESOURCE_LIMIT')
    for i,q in enumerate(samples):
        checked=check_full_tool_wait(connector,q,dict(samples=[dict(t=0.,box=box_message(stationary))]))
        if not checked['passed']:return dict(passed=False,joint_sample_index=i,detail=checked)
    return dict(passed=True,samples=len(samples),maximum_joint_step_rad=step,required_clearance_m=.005,
        stationary_box=box_message(stationary),continuous_swept_proof=False)

def plan_wait_outfeed(connector,state,landed,world,seed,*,tool_timing=WAIT_FIRST):
    c=connector;path=outfeed_path(landed,world,tool_timing=tool_timing)
    full=check_full_tool_wait(c,state,path)
    if not full['passed']:return None,dict(reason='OUTFEED_FULL_TOOL_COLLISION',detail=full)
    req=c._build_native_request(state,state,world,seed=seed,stage='withdrawal',target_contact=landed)
    parent,_,transition=c._native_parent_for_request(state,req)
    if transition is not None:raise ValueError('OUTFEED_PARENT_OWNERSHIP_CHANGED')
    budget,timeout=c._native_limits()
    req.update(pipeline_id='semantic_event',planner_id='WAIT_IDEAL_OUTFEED',outfeed=path,
        parent_stage_id=parent.stage_id,parent_state_receipt=parent.evidence(),ownership_transition=None,
        stage_id=c.native_task_id+':'+str(next(c._native_stage_ids)),allowed_planning_time_s=budget,purpose='DEPARTURE_PROCESS')
    result=c.native.request(req,timeout=timeout,cancelled=c._native_cancelled)
    if result['status']!='SUCCESS':return None,dict(reason='OUTFEED_NATIVE_REJECTED',detail=result)
    if result['terminal_q']!=np.asarray(state).tolist():raise ValueError('OUTFEED_ROBOT_MOVED')
    c.native_generated.append(dict(result,stage='withdrawal',submitted_request=deepcopy(req),authoritative_status='SKIPPED_PLANNING_ONLY'))
    from .coupled_batch import message_box
    out=message_box(path['samples'][-1]['box']);endworld=[out if b.name==out.name else b for b in world]
    terminal=dict(req,world=[box_message(b) for b in endworld],process_policy=dict(req['process_policy'],target=box_message(out)))
    end=c._native_issue_state(state,req['stage_id'],c._native_state_context(terminal))
    return (end,out,endworld,dict(passed=True,path=path,full_tool=full,native=result,stage_id=req['stage_id'])),None
