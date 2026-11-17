import copy,time,threading,types
import pytest
from test_single_carton_planner import case
from unloading_sim.workbench.single_carton.planner import PlanningBudgetExhausted

@pytest.mark.parametrize('stage',['binding','finalize'])
def test_entire_task_deadline_and_no_late_permits(case,monkeypatch,stage):
    p,obs,box,folder,calls,opts=case
    found=[];original=type(p).search_box
    def capture(self,*a,**k):
        value=original(self,*a,**k);found.append(copy.deepcopy(value));return value
    monkeypatch.setattr(p,'search_box',types.MethodType(capture,p))
    p.plan_cycle(obs,[box],folder)
    monkeypatch.setattr(p,'search_box',lambda *a,**k:copy.deepcopy(found[0]))
    if stage=='binding':
        real=p.binding
        def slow():time.sleep(.25);return real()
        monkeypatch.setattr(p,'binding',slow)
    def finalize(*a):time.sleep(.25)
    started=time.monotonic()
    with pytest.raises(PlanningBudgetExhausted):
        p.plan_cycle(obs,[box],folder,deadline=started+.1,**({'finalize':finalize} if stage=='finalize' else {}))
    assert time.monotonic()-started<.15
    assert not p.authority._plans
    time.sleep(.3)
    assert not p.authority._plans


def test_late_old_task_cannot_revoke_new_task(case,monkeypatch):
    p,obs,box,folder,calls,opts=case
    found=[];original=type(p).search_box
    def capture(self,*a,**k):
        value=original(self,*a,**k);found.append(copy.deepcopy(value));return value
    monkeypatch.setattr(p,'search_box',types.MethodType(capture,p));p.plan_cycle(obs,[box],folder)
    monkeypatch.setattr(p,'search_box',lambda *a,**k:copy.deepcopy(found[0]))
    started=time.monotonic()
    with pytest.raises(PlanningBudgetExhausted):
        p.plan_cycle(obs,[box],folder,deadline=started+.1,finalize=lambda *a:time.sleep(.25))
    new=folder.parent/'new-task';new.mkdir()
    plan=p.plan_cycle(obs,[box],new,deadline=time.monotonic()+5)
    ids=set(p.authority._plans);assert len(ids)==4
    time.sleep(.3)
    assert set(p.authority._plans)==ids
    for s in plan['segments']:p.authority.verify(s['package'],plan['binding'])

@pytest.mark.parametrize('stage',['register','plan_record','flush','cleanup','build','summary'])
@pytest.mark.parametrize('failure',[False,True])
def test_tail_stages_are_transactional_and_bounded(case,monkeypatch,stage,failure):
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.single_carton.evidence import EvidenceWriter
    from unloading_sim.workbench import timing
    p,obs,box,folder,calls,opts=case
    found=[];original=type(p).search_box
    def capture(self,*a,**k):
        value=original(self,*a,**k);found.append(copy.deepcopy(value));return value
    monkeypatch.setattr(p,'search_box',types.MethodType(capture,p));p.plan_cycle(obs,[box],folder)
    monkeypatch.setattr(p,'search_box',lambda *a,**k:copy.deepcopy(found[0]))
    if stage=='register':target,name=Authority,'register'
    elif stage=='plan_record':target,name=timing,'write_json'
    elif stage=='flush':
        p.site['async_evidence']=True;target,name=EvidenceWriter,'flush'
    elif stage=='summary':
        monkeypatch.setattr(p,'search_box',lambda *a,**k:None);target,name=type(p),'budget_summary'
    else:target,name=type(p),('release_workers' if stage=='cleanup' else 'build')
    original=getattr(target,name)
    def injected(*a,**k):
        active=stage!='plan_record' or (len(a)>2 and a[2]=='complete-plan.json')
        if active:
            if failure:raise OSError('TAIL_FAILURE_INJECTED')
            time.sleep(.25)
        return original(*a,**k)
    monkeypatch.setattr(target,name,injected)
    task_folder=folder.parent/('tail-'+stage);task_folder.mkdir()
    started=time.monotonic()
    with pytest.raises((PlanningBudgetExhausted,OSError)):
        p.plan_cycle(obs,[box],task_folder,deadline=started+.1)
    assert time.monotonic()-started<.15
    assert not p.authority._plans and not (task_folder/'complete-plan.json').exists()
    time.sleep(.35)
    assert not p.authority._plans and p.last_task['status']=='INVALIDATED'


def test_prepare_plan_ready_and_journal_are_inside_deadline(case,monkeypatch):
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.runtime import TaskManager,Journal
    from unloading_sim.workbench.single_carton.cycle import Cycle
    p,obs,box,folder,calls,opts=case
    found=[];original=type(p).search_box
    def capture(self,*a,**k):
        value=original(self,*a,**k);found.append(copy.deepcopy(value));return value
    monkeypatch.setattr(p,'search_box',types.MethodType(capture,p));p.plan_cycle(obs,[box],folder)
    monkeypatch.setattr(p,'search_box',lambda *a,**k:copy.deepcopy(found[0]))
    m=TaskManager.__new__(TaskManager)
    m.lock=threading.RLock();m.busy=False;m.phase='TARGET_READY';m.cycle=None;m.plan=None;m.reason=''
    m.vacuum=SimpleNamespace(holding=False);m.service=None;m.planner=p;m.observation=obs;obs.generation=1
    m.backend=SimpleNamespace(observation_generation=1);m.boxes=[box];m.scene=None;m.stop_event=threading.Event()
    m.folder=folder.parent/'manager';m.folder.mkdir();m.task_id='manager';m.evidence=None
    m.journal=Journal(m.folder,m.task_id);m.config=dict(planning_deadline_s=.1,vacuum={});m.notify=lambda *a:None
    m.check_contact=lambda *a:None;m.check_release=lambda *a:None
    ready=Cycle.ready
    monkeypatch.setattr(Cycle,'ready',lambda *a,**k:(time.sleep(.25),ready(*a,**k))[1])
    try:
        started=time.monotonic()
        with pytest.raises(PlanningBudgetExhausted):m.prepare_plan()
        assert time.monotonic()-started<.15
        time.sleep(.3);m.journal.flush()
        assert m.phase=='FAILED' and m.cycle is None and not p.authority._plans
        assert not (m.folder/'complete-plan.json').exists()
        import json
        events=[json.loads(x) for x in (m.folder/'events.jsonl').read_text().splitlines()]
        assert not any(e.get('state')=='PLAN_READY' for e in events)
    finally:m.journal.close()


def test_previous_resident_reset_wait_uses_remaining_budget(tmp_path):
    from unloading_sim.workbench.single_carton.resident import WorkerPool
    pool=WorkerPool(None,{},0,tmp_path)
    release=threading.Event();pool.background=threading.Thread(target=lambda:release.wait(2));pool.background.start()
    started=time.monotonic()
    try:
        with pytest.raises(PlanningBudgetExhausted,match='RESIDENT_PREVIOUS_RESET'):
            pool.begin_task(deadline=started+.1)
        assert time.monotonic()-started<.15
    finally:release.set();pool.background.join()


def test_late_prewarm_cannot_publish_into_new_task(case,monkeypatch):
    from types import SimpleNamespace
    p,*_=case;p.site['resident_workers']=True
    entered=threading.Event();release=threading.Event();closed=[]
    pool=SimpleNamespace(close=lambda:closed.append(True))
    def warm(self):entered.set();release.wait(2);return pool
    monkeypatch.setattr(type(p),'resident_pool',warm)
    thread=p.prewarm();assert entered.wait(1)
    p.task_generation+=1
    release.set();thread.join(2)
    assert p.resident is None and closed==[True]
