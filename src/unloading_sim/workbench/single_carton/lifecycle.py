"""Task-local planning state and staged permits; late work cannot publish."""
import copy
from contextlib import contextmanager
import threading
import time
import types
from ..controlled.trajectory import Authority

class StagedAuthority(Authority):
    def __init__(self,parent,guard):
        super().__init__(parent.schema)
        self.session=parent.session;self.digest_method=parent.digest_method;self.guard=guard
    def invalidate(self):
        with self._lock:self._plans.clear()
    def register(self,*args,**kwargs):
        self.guard('REGISTER')
        result=super().register(*args,**kwargs)
        self.guard('REGISTER_COMPLETE')
        return result


def run_task(owner,observation,boxes,folder,progress,selected,cancelled,scene,deadline,finalize):
    from .planner import PlanningBudgetExhausted
    stop=threading.Event();done=threading.Event();settled=threading.Event();returned=threading.Event();outcome={};task=None
    def guard(stage):
        if cancelled() or stop.is_set():raise RuntimeError('CANCELLED')
        if deadline is not None and time.monotonic()>=deadline:
            error=PlanningBudgetExhausted(stage)
            if task is None:
                from .budget import snapshot
                error.evidence=snapshot(types.SimpleNamespace(deadline=deadline,search_counts={}),boxes,[],selected,error)
            raise error
    guard('BEFORE_DISPATCH')
    @contextmanager
    def bounded_lock(lock, stage):
        # A concurrent commit/close may own either lock. Waiting is part of the
        # same absolute caller budget; no blocking filesystem work holds them.
        while True:
            guard(stage)
            wait=.005 if deadline is None else max(0.,min(.005,deadline-time.monotonic()))
            if lock.acquire(timeout=wait):break
        try:
            guard(stage)
            yield
        finally:lock.release()
    with bounded_lock(owner.task_lock,'TASK_LOCK'),bounded_lock(owner.authority._lock,'AUTHORITY_LOCK'):
        owner.task_generation+=1;generation=owner.task_generation
        owner.authority.invalidate();session=owner.authority.session
        task=copy.copy(owner)
        # Transfer pool ownership. No subsequent task can take it until cleanup
        # completed and this task was successfully adopted by the owner.
        owner.resident=None
        previous_evidence=owner.evidence;owner.evidence=None
        task.authority=StagedAuthority(owner.authority,guard)
        task.task_id=folder.name
        task.complete_plan_path=folder/f'complete-plan.pending-{generation}.json'
        for key in ('pool','pool_status','pool_threads'):setattr(task,key,[])
        for key in ('ipc_executors','search_counts','scene_handles'):setattr(task,key,{})
        task.abandoned=set();task.search_thread=None;task.client=None;task.evidence=None;task.cleanup_threads=[];task.cleanup_errors=[]
        task.pool_lock=threading.Lock();task.scene_lock=threading.Lock();task.build_lock=threading.RLock()
        task.files=dict(owner.files);task.hash_cache=dict(owner.hash_cache)
        for key,value in owner.__dict__.items():
            if isinstance(value,types.MethodType) and value.__self__ is owner:setattr(task,key,types.MethodType(value.__func__,task))
        owner.last_task=dict(task_id=folder.name,generation=generation,status='RUNNING',worker_requests_finished=False,cleanup_finished=False)
        status=owner.last_task
        owner.last_task_context=task
    def task_progress(message):
        if not stop.is_set() and owner.task_generation==generation and (deadline is None or time.monotonic()<deadline):
            progress(message)
    def run():
        record_locked=False;record_moved=False
        try:
            guard('BEFORE_DISPATCH')
            result=task._plan_cycle(observation,boxes,folder,task_progress,selected,lambda:stop.is_set() or cancelled(),scene,deadline)
            guard('PLAN_EVIDENCE_COMPLETE')
            if finalize is not None:finalize(result,guard)
            guard('FINALIZE_COMPLETE')
            # Persist off the caller thread. Serialize record replacement and
            # rollback, including repeated use of the same task folder. A late
            # writer cannot overwrite a newer task's committed record.
            owner.record_commit_lock.acquire();record_locked=True
            guard('COMMIT_RECORD_LOCK')
            task.complete_plan_path.replace(folder/'complete-plan.json')
            record_moved=True
            guard('COMMIT_RECORD_RENAME')
            outcome['plan']=result
        except BaseException as e:
            outcome['error']=e;status['planning_error']=f'{type(e).__name__}:{e}'
        finally:
            done.set()
            status['planning_thread_finished_s']=time.monotonic()
            settled.wait()
            returned.wait()
            try:
                if status['status']!='COMMITTED':
                    task.authority.invalidate()
                    if record_moved:
                        try:(folder/'complete-plan.json').replace(task.complete_plan_path)
                        except Exception as error:task.cleanup_errors.append(dict(stage='RECORD_ROLLBACK',error=str(error)))
            finally:
                if record_locked:owner.record_commit_lock.release()
            try:
                if previous_evidence is not None:
                    previous_evidence.close();previous_evidence._thread.join()
                search=task.search_thread
                if search is not None and search.is_alive():search.join()
                for cleanup in task.cleanup_threads:cleanup.join()
                if task.resident is not None:
                    if status['status']!='COMMITTED':
                        with task.resident.lock:task.resident.closed=True
                    for worker in (task.resident.thread,task.resident.background):
                        if worker is not None:worker.join()
                if status['status']!='COMMITTED':
                    if task.resident is not None:
                        clients=task.resident.all()
                        with task.resident.lock:task.resident.closed=True
                        for client in clients:client.close()
                        task.resident=None
                    writer=task.evidence
                    task.close()
                    if writer is not None:writer._thread.join()
                status['worker_requests_finished']=True;status['worker_requests_finished_s']=time.monotonic()
                status['cleanup_errors']=list(task.cleanup_errors)
                status['cleanup_finished']=not task.cleanup_errors
            except Exception as error:status['cleanup_error']=str(error)
            status['cleanup_completed_s']=time.monotonic()
            try:
                from .. import timing
                timing.write_json(folder/'planning-task-lifecycle.json',dict(status),'planning-task-lifecycle.json')
            except Exception as error:status['record_error']=str(error)
    thread=threading.Thread(target=run,daemon=True,name='planning-task-'+folder.name[:8]);thread.start()
    try:
        while not done.is_set():
            guard('TASK_WAIT')
            done.wait(.005 if deadline is None else max(0.,min(.005,deadline-time.monotonic())))
        if 'error' in outcome:raise outcome['error']
        guard('PUBLISH')
        with bounded_lock(owner.task_lock,'PUBLISH_TASK_LOCK'),bounded_lock(owner.authority._lock,'PUBLISH_AUTHORITY_LOCK'):
            guard('COMMIT')
            if owner.task_generation!=generation or owner.authority.session!=session:raise RuntimeError('PLAN_INVALIDATED')
            owner.authority._plans.update(task.authority._plans)
            # Only the successful current task may adopt its model/cache/pool.
            excluded={'authority','task_lock','task_generation','last_task','last_task_context'}
            owner.__dict__.update({k:v for k,v in task.__dict__.items() if k not in excluded})
            owner.deadline=None;owner.deadline_reference=None
            status['status']='COMMITTED';settled.set()
            return outcome['plan']
    except BaseException as error:
        stop.set();status['status']='INVALIDATED';settled.set()
        if isinstance(error,PlanningBudgetExhausted) and error.evidence is None:
            if task.search_thread is not None and task.search_thread.is_alive():
                error.where='SEARCH_STEP_RUNNING_AT_DEADLINE'
                error.args=('PLANNING_BUDGET_EXHAUSTED:'+error.where,)
            from .budget import snapshot
            error.evidence=snapshot(task,boxes,list(getattr(task,'task_failures',[])),selected,error,deadline=deadline)
            from .. import timing
            record=dict(failures=list(getattr(task,'task_failures',[])),planning_budget=error.evidence)
            error.record_thread=threading.Thread(target=timing.write_json,args=(folder/'planning-deadline-return.json',record,'planning-deadline-return.json'),daemon=True)
            error.record_thread.start()
        # Local permits were never visible. Do not globally revoke a newer task.
        raise
    finally:
        status['caller_returned_s']=time.monotonic()
        returned.set()
