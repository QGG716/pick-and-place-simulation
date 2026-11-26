"""Operator-clicked vacuum bench test (user request 2026-11-26 "我要真机实测一下真空系统").

No robot motion. Each valve write follows one operator click and uses the
existing VacuumOwner claim, i.e. only the configured establish/release commands
(reviewed set) and the same fresh-pressure checks. The pass rule is the cycle's
unchanged pressure gate: after the valve write, qualifying fresh heartbeats
spanning ``*_dwell_s`` within ``*_timeout_s``. A grip attempt that did not
confirm keeps ``holding`` (the valve may be in the grip state): only a confirmed
release test, or an explicit operator confirmation that the gripper holds no
load, clears it. Stop/close never writes a valve. The test never claims a vacuum
owned by a cycle; a load held by a stopped cycle stays local handling.
Every pressure heartbeat, valve frame and decision is written to
``<record_root>/vacuum_tests/<test_id>/events.jsonl``; results are bench
evidence, never PHYSICAL_CYCLE_COMPLETE.
"""
import json
import threading
import time
from pathlib import Path
from time import monotonic

PREFIX='vacuum-test-'
GAUGE_NAMES={1:'吸具',2:'储气罐',3:'真空泵',4:'压缩机'}


class VacuumBenchTest:
    def __init__(self,manager,record_root,clock=monotonic,notify=lambda kind,value:None,poll_s=.02):
        self.manager=manager;self.owner=manager.vacuum;self.config=manager.config['vacuum']
        self.record_root=Path(record_root);self.clock=clock;self.notify=notify;self.poll_s=poll_s
        self.test_id=None;self.folder=None;self._file=None;self._file_lock=threading.Lock()
        self.state='IDLE';self.message='未开始';self.fault=None;self.last_result=None
        self.hold=None;self._watch=None;self.history=[]
    # -- ownership -----------------------------------------------------------
    def owns(self):return self.test_id is not None and self.owner.task_id==self.test_id
    def blockers(self):
        """Why a grip test may not start now (empty: allowed)."""
        m=self.manager;b=[]
        if not self.owner.connected:b.append('真空板未连接')
        if m.active:b.append('任务或运动操作进行中')
        if m.cycle is not None and m.cycle.state=='STOP_UNCONFIRMED':b.append('停止未确认')
        if self.owner.task_id is not None and not self.owns():b.append('真空由任务占用（停止后的持箱须现场处置）')
        if self.owner.holding:b.append('吸具处于吸附状态，先做释放测试')
        if self.state in ('GRIPPING','RELEASING'):b.append('测试进行中')
        try:self.owner.pressure()
        except Exception as e:b.append('吸具压力：'+str(e))
        return b
    def _reserve(self):
        m=self.manager
        with m.lock:
            if m.busy or (m.cycle is not None and m.cycle.active):raise ValueError('任务或运动操作进行中，不能做真空测试')
            if m.cycle is not None and m.cycle.state=='STOP_UNCONFIRMED':raise ValueError('停止未确认，不能做真空测试')
            m.busy=True
    def _unreserve(self):self.manager.busy=False
    # -- records -------------------------------------------------------------
    def _open(self):
        self.test_id=PREFIX+time.strftime('%Y%m%d-%H%M%S')+f'-{int(time.time()*1000)%1000:03d}'
        self.folder=self.record_root/'vacuum_tests'/self.test_id;self.folder.mkdir(parents=True)
        self._file=open(self.folder/'events.jsonl','a',buffering=1)
        self.hold=None;self.fault=None
    def record(self,**event):
        event.setdefault('t_monotonic_s',self.clock());event.setdefault('t_wall',time.time())
        with self._file_lock:
            if self._file is not None:self._file.write(json.dumps(event,ensure_ascii=False,default=str)+'\n')
    def _close(self):
        with self._file_lock:
            if self._file is not None:self._file.close();self._file=None
    def _on_fault(self,reason):
        # Called from the serial reader; first fault of a kind is recorded.
        if reason!=self.fault:self.fault=reason;self.record(kind='fault',reason=reason)
    def gauges(self):
        """All four board gauges for display: (name, kPa, age s, integrity) or None."""
        now_ns=time.monotonic_ns();out={}
        for number,sample in self.owner.observer.gauges().items():
            if sample is None:out[number]=None;continue
            out[number]=dict(name=GAUGE_NAMES.get(number,str(number)),kpa=sample.kpa,
                             age_s=(now_ns-sample.stamp.received_monotonic_ns)/1e9,integrity=sample.integrity)
        return out
    def _snapshot_gauges(self):
        try:return self.gauges()
        except Exception as e:return dict(error=str(e))
    # -- the pressure gate (same rule as Cycle) ------------------------------
    def _gate(self,phase,sent_s):
        c=self.config;gripping=phase=='grip';stable=None;last=None;first=None;extreme=None
        while True:
            if self.owner.inhibited:return 'STOPPED_BY_OPERATOR',dict(first_qualifying_s=first,extreme_kpa=extreme)
            now=self.clock()
            if now-sent_s>c[phase+'_timeout_s']:return phase.upper()+'_PRESSURE_TIMEOUT',dict(first_qualifying_s=first,extreme_kpa=extreme)
            p=self.owner.pressure()  # raises on stale / alarm / disconnect
            if p['received_s']>sent_s and p['received_s']!=last:
                last=p['received_s'];kpa=p['kpa']
                extreme=kpa if extreme is None else (min(extreme,kpa) if gripping else max(extreme,kpa))
                met=kpa<=c['grip_below_kpa'] if gripping else kpa>=c['release_above_kpa']
                if not met:stable=None
                else:
                    if stable is None:stable=p['received_s']
                    if first is None:first=p['received_s']
                    if p['received_s']-stable>=c[phase+'_dwell_s']:
                        return phase.upper()+'_CONFIRMED',dict(first_qualifying_s=first,confirmed_s=p['received_s'],
                            first_after_valve_s=first-sent_s,confirmed_after_valve_s=p['received_s']-sent_s,
                            pressure_kpa=kpa,extreme_kpa=extreme)
            time.sleep(self.poll_s)
    # -- operator actions (blocking; the UI runs them in a worker thread) ----
    def grip(self):
        blockers=self.blockers()
        if blockers:raise ValueError('不能开始吸附测试：'+'、'.join(blockers))
        self._reserve()
        try:
            self._open();self.state='GRIPPING';self.message='吸附测试：已发送吸附指令，等待压力'
            self.record(kind='test_start',test_id=self.test_id,phase='grip',thresholds={k:self.config[k] for k in
                ('grip_below_kpa','hold_below_kpa','release_above_kpa','grip_dwell_s','release_dwell_s','grip_timeout_s','release_timeout_s','max_age_s')},
                establish_commands=self.config['establish_commands'],release_commands=self.config['release_commands'],
                gauges=self._snapshot_gauges(),robot_motion='NONE',physical_result='BENCH_TEST_NOT_CYCLE_ACCEPTANCE')
            try:
                self.owner.claim(self.test_id,self.record,self._on_fault)
                self.owner.establish(self.test_id)
            except Exception as e:
                self.record(kind='grip_rejected',reason=str(e),holding=self.owner.holding)
                if not self.owner.holding:
                    self._finish('IDLE','吸附测试未发送：'+str(e));raise
                self.state='HOLD_UNCONFIRMED';self.message='吸附指令部分发送：'+str(e)+'；请做释放测试';raise
            sent=self.clock();self.record(kind='valve_sequence_complete',phase='grip',sent_s=sent)
            self._start_watch(sent)
            try:outcome,detail=self._gate('grip',sent)
            except Exception as e:outcome,detail='GRIP_PRESSURE_ERROR',dict(error=str(e))
            self.record(kind='grip_result',outcome=outcome,**detail)
            if outcome=='GRIP_CONFIRMED':
                self.state='HOLDING';self.message=(f"吸附确认：阀门后 {detail['first_after_valve_s']:.2f} s 首次 ≤ {self.config['grip_below_kpa']:g} kPa，"
                    f"{detail['confirmed_after_valve_s']:.2f} s 稳定确认（{detail['pressure_kpa']:.1f} kPa）。保压观察中，完成后点释放测试。")
            else:
                self.state='HOLD_UNCONFIRMED';self.message=(f'吸附未确认：{outcome}' +(f"（最低 {detail['extreme_kpa']:.1f} kPa）" if detail.get('extreme_kpa') is not None else '')+
                    '。阀门可能处于吸附状态，请做释放测试。')
            self.last_result=dict(phase='grip',outcome=outcome,**detail);self.history.append(self.last_result)
            return self.last_result
        finally:self._unreserve()
    def release(self):
        if not self.owns() or not self.owner.holding:raise ValueError('没有本测试的吸附状态可释放（任务持箱须现场处置）')
        if self.state in ('GRIPPING','RELEASING'):raise ValueError('测试进行中')
        self._reserve()
        try:
            self.state='RELEASING';self.message='释放测试：已发送释放指令，等待压力回升'
            with self.owner.lock:self.owner.inhibited=False  # this operator click re-arms the test's own claim
            self.record(kind='release_start',gauges=self._snapshot_gauges(),hold=self.hold_summary())
            try:self.owner.release(self.test_id)
            except Exception as e:
                self.record(kind='release_rejected',reason=str(e));self.state='HOLD_UNCONFIRMED'
                self.message='释放指令未发送：'+str(e);raise
            sent=self.clock();self.record(kind='valve_sequence_complete',phase='release',sent_s=sent)
            try:outcome,detail=self._gate('release',sent)
            except Exception as e:outcome,detail='RELEASE_PRESSURE_ERROR',dict(error=str(e))
            self.record(kind='release_result',outcome=outcome,**detail)
            self.last_result=dict(phase='release',outcome=outcome,**detail);self.history.append(self.last_result)
            if outcome=='RELEASE_CONFIRMED':
                self._finish('RELEASED',f"释放确认：阀门后 {detail['first_after_valve_s']:.2f} s 首次 ≥ {self.config['release_above_kpa']:g} kPa，"
                             f"{detail['confirmed_after_valve_s']:.2f} s 稳定确认（{detail['pressure_kpa']:.1f} kPa）。测试结束。",released=True)
            else:
                self.state='HOLD_UNCONFIRMED';self.message=(f'释放未确认：{outcome}' +(f"（最高 {detail['extreme_kpa']:.1f} kPa）" if detail.get('extreme_kpa') is not None else '')+
                    '。仍视为可能持箱；可再次释放测试，或现场确认无负载后人工清除。')
            return self.last_result
        finally:self._unreserve()
    def operator_clear(self):
        """Operator confirms on site that the gripper holds no load; no valve write."""
        if not self.owns():raise ValueError('没有本测试的吸附状态')
        if self.state in ('GRIPPING','RELEASING'):raise ValueError('测试进行中')
        self.record(kind='operator_confirmed_no_load',gauges=self._snapshot_gauges(),valve_command='NONE',
                    basis='OPERATOR_CONFIRMED_ON_SITE_NOT_PRESSURE_EVIDENCE')
        self._finish('CLEARED_BY_OPERATOR','操作员已现场确认无负载，测试持箱标志已清除（未发阀门指令；不是压力证据）。',released=True)
    def _finish(self,state,message,released=False):
        if self.owns() and (released or not self.owner.holding):self.owner.unclaim(released=released)
        self.state=state;self.message=message
        self.record(kind='test_end',state=state,hold=self.hold_summary())
        self._close();self.test_id=None
    # -- hold watch ----------------------------------------------------------
    def hold_summary(self):
        h=self.hold
        if not h:return None
        return dict(since_valve_s=h['last_s']-h['start_s'] if h['last_s'] else 0.,min_kpa=h['min'],max_kpa=h['max'],
                    last_kpa=h['last'],losses=h['losses'],below_hold_since_s=h['below_since'])
    def _start_watch(self,sent):
        self.hold=dict(start_s=sent,last_s=None,min=None,max=None,last=None,losses=0,below_since=None,below=False)
        test_id=self.test_id
        def watch():
            seen=None;thr=self.config['hold_below_kpa']
            while self.test_id==test_id and self.owner.holding and self.owns():
                try:p=self.owner.pressure()
                except Exception:time.sleep(.05);continue
                if p['received_s']!=seen and p['received_s']>sent:
                    seen=p['received_s'];h=self.hold;k=p['kpa']
                    h['last']=k;h['last_s']=p['received_s']
                    h['min']=k if h['min'] is None else min(h['min'],k);h['max']=k if h['max'] is None else max(h['max'],k)
                    below=k<=thr
                    if below and not h['below']:h['below_since']=p['received_s']-sent
                    if h['below'] and not below and self.state=='HOLDING':
                        h['losses']+=1;self.record(kind='hold_lost',kpa=k,threshold_kpa=thr,after_valve_s=p['received_s']-sent)
                    h['below']=below
                time.sleep(.05)
        self._watch=threading.Thread(target=watch,daemon=True,name='vacuum-bench-hold-watch');self._watch.start()
    def status_text(self):
        lines=[f'状态：{self.state} · {self.message}']
        h=self.hold_summary()
        if h and self.owner.holding and h['last_kpa'] is not None:
            lines.append(f"保压：阀门后 {h['since_valve_s']:.1f} s · 当前 {h['last_kpa']:.1f} kPa · 最低 {h['min_kpa']:.1f} · 最高 {h['max_kpa']:.1f}"
                         f" · 跌出 {self.config['hold_below_kpa']:g} kPa 次数 {h['losses']}")
        if self.fault:lines.append('真空板故障：'+self.fault)
        if self.folder:lines.append('记录：'+str(self.folder/'events.jsonl'))
        return '\n'.join(lines)
