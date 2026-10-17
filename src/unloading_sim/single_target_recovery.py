"""Budget-bound recovery for a fixed pose, local to one scene/parent connection."""
from copy import deepcopy
from time import perf_counter
import numpy as np
from .moveit2_backend import MoveItUnavailable, digest


def connect_fixed_target(c,pose,start,obstacles,seed,*,stage,purpose,attachment=None,residual_reason):
    stage_s,_=c._native_limits()
    deadline=perf_counter()+stage_s
    previous=getattr(c,'_single_connection_deadline',None)
    c._single_connection_deadline=min(deadline,previous) if previous is not None else deadline
    original_q=np.asarray(start).copy();original_pose=np.asarray(pose).copy()
    # Bind current parent/attachment/world exactly; never share exclusions across calls.
    def current_context():
        request=c._build_native_request(start,None,obstacles,seed=seed,stage=stage,attachment=attachment,goal_pose=pose)
        receipt,_,transition=c._native_parent_for_request(start,request)
        return digest(dict(request=request,parent=receipt.evidence(),transition=transition))
    context_key=current_context();excluded=[];attempt=0;last_failure=None
    def check_context():
        if c._native_cancelled():raise MoveItUnavailable('REQUEST_CANCELLED_OR_DEADLINE')
        if current_context()!=context_key or not np.array_equal(start,original_q) or not np.array_equal(pose,original_pose):
            raise MoveItUnavailable('FIXED_TARGET_CONTEXT_CHANGED')
    def brief(failure):
        if failure is None:return None
        return dict(reason=failure.get('reason'),native_request_id=failure.get('native',{}).get('request_id'),
            requests=[{k:a.get(k) for k in ('request_id','stage_id','status','solver_message','moveit_error_code')}
                for a in failure.get('attempts',[])])
    costs=dict(ik_s=0.,motion_s=0.)
    if not hasattr(c,'connection_attempts'):c.connection_attempts=[]
    invocation=len(c.connection_attempts)
    try:
        while perf_counter()<c._single_connection_deadline:
            check_context()
            attempt+=1;row=dict(connection_id=invocation,attempt=attempt,stage=stage,
                context_sha256=context_key,method='IK_RECOVERY_THEN_RRT_RESTART',tree_retained=False,
                excluded_local_ik_count=len(excluded),remaining_stage_s=c._single_connection_deadline-perf_counter())
            c.connection_attempts.append(row)
            before=perf_counter()
            try:
                q,failure=c.ik_pose_once(pose,start,obstacles,seed,stage=stage,attachment=attachment,
                    purpose=purpose,residual_reason=residual_reason,ik_mode='bounded_target',solver_budget_s=.10,
                    excluded_ik=excluded)
            except MoveItUnavailable as exc:
                row.update(status='IK_INTERRUPTED',detail=str(exc));raise
            finally:
                row['ik_s']=perf_counter()-before;costs['ik_s']+=row['ik_s']
            check_context()
            if failure:
                last_failure=failure;row.update(status='IK_NOT_COMPLETED',failure=brief(failure),motion_s=None)
                raw=failure.get('native',{})
                unaccepted=raw.get('solver_candidate_q',raw.get('q'))
                if unaccepted is not None:
                    u=np.asarray(unaccepted,dtype=float)
                    if u.shape==(6,) and np.isfinite(u).all():excluded.append(u.copy())
                if failure.get('reason') not in {'NATIVE_IK_FAILED',residual_reason}:
                    raise MoveItUnavailable('FIXED_TARGET_IK_REQUEST_FAILURE:'+str(failure.get('reason')))
                # A bounded KDL failure used its current RNG stream; no reseeding.
                # A malformed mock/adapter reporting no bounded work must not spin.
                if not raw.get('bounded_search',False) and unaccepted is None:
                    return [],None,dict(reason='SINGLE_TARGET_NO_NUMERICAL_PROGRESS',last_failure=failure),costs
                continue
            if any(np.max(np.abs(q-old))<=c.ik['candidate_dedup_tolerance_rad'] for old in excluded):
                row.update(status='REPEATED_EXCLUDED_IK',q=q.tolist())
                return [],None,dict(reason='SINGLE_TARGET_NO_NUMERICAL_PROGRESS'),costs
            excluded.append(q.copy());row['q_goal']=q.tolist()
            before=perf_counter()
            try:
                # One native request per loop iteration; the loop, not this IPC
                # transaction size, governs the available solver opportunities.
                path,failure,_=c._native_plan(start,q,obstacles,seed=seed,stage=stage,attachment=attachment,
                    support_names=(),initial_proximity=None,purpose=purpose,allow_ompl=True,max_ompl_attempts=1,
                    try_ptp=False,try_prior=False)
            except MoveItUnavailable as exc:
                row.update(status='MOTION_INTERRUPTED',detail=str(exc));raise
            finally:
                row['motion_s']=perf_counter()-before;costs['motion_s']+=row['motion_s']
            row.update(status='SUCCESS' if failure is None else 'CONNECTION_NOT_COMPLETED',failure=brief(failure))
            check_context()
            if perf_counter()>=c._single_connection_deadline:break
            if failure is None:return path,q,None,costs
            last_failure=failure
            if failure.get('reason') not in {'MOVEIT2_SEARCH_EXHAUSTED','NATIVE_PLANNING_FAILED','NATIVE_PLANNING_TIMEOUT',
                    'SEARCH_EXHAUSTED','INVALID_GOAL_CLEARANCE','INVALID_GOAL_PROCESS','INVALID_GOAL_AUTHORITY'}:
                raise MoveItUnavailable('FIXED_TARGET_CONNECTION_REQUEST_FAILURE:'+str(failure.get('reason')))
        return [],None,dict(reason='SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED',last_failure=last_failure,
            interpretation='INCOMPLETE_NOT_GLOBAL_UNREACHABILITY',attempts=attempt),costs
    except MoveItUnavailable as exc:
        if str(exc) in {'SINGLE_TARGET_STAGE_DEADLINE','REQUEST_CANCELLED_OR_DEADLINE'} and not c._native_cancelled() and perf_counter()>=c._single_connection_deadline:
            return [],None,dict(reason='SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED',last_failure=last_failure,
                interpretation='INCOMPLETE_NOT_GLOBAL_UNREACHABILITY',attempts=attempt),costs
        raise
    finally:
        c._single_connection_deadline=previous
