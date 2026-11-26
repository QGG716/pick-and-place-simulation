"""Vacuum bench test with a fake serial port: no hardware, no robot."""
import json
import struct
import threading
import time
from dataclasses import replace
import pytest
from unloading_sim.workbench.contracts import Stamp
from unloading_sim.workbench.devices import GaugeReading
from unloading_sim.workbench.single_carton.vacuum import VacuumOwner, valve_frame
from unloading_sim.workbench.single_carton.vacuum_test import VacuumBenchTest
from unloading_sim.workbench.reference.vacuum_receive import HEADER, DEVICE

CONFIG=dict(grip_below_kpa=-60.,hold_below_kpa=-30.,release_above_kpa=-10.,grip_dwell_s=.3,release_dwell_s=.3,
            grip_timeout_s=1.,release_timeout_s=1.,max_age_s=3.,establish_commands=[0x12],release_commands=[0x13])


def heartbeat(kpa):
    body=bytes((DEVICE,0x61))+struct.pack('<f',kpa)
    return HEADER+bytes((len(body),))+body+bytes((sum(body)&255,))


class Port:
    def __init__(self):self.writes=[];self.inbox=bytearray();self.lock=threading.Lock()
    def read(self,n):
        time.sleep(.005)
        with self.lock:data=bytes(self.inbox[:n]);del self.inbox[:n]
        return data
    def write(self,data):self.writes.append(bytes(data));return len(data)
    def close(self):pass


class Manager:
    def __init__(self,vacuum):
        self.vacuum=vacuum;self.config=dict(vacuum=CONFIG);self.lock=threading.Lock();self.busy=False;self.cycle=None
    @property
    def active(self):return self.busy


class Board:
    """Feeds gripper heartbeats at 50 Hz; value follows the last valve frame."""
    def __init__(self,owner,port,grip_kpa=-80.,release_kpa=-2.):
        self.owner,self.port,self.kpa=owner,port,-1.;self.grip_kpa,self.release_kpa=grip_kpa,release_kpa
        self.stop=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        while not self.stop.wait(.02):
            if self.port.writes:
                self.kpa=self.grip_kpa if self.port.writes[-1]==valve_frame(0x12) else self.release_kpa
            with self.port.lock:self.port.inbox+=heartbeat(self.kpa)


@pytest.fixture
def bench(tmp_path):
    port=Port();owner=VacuumOwner(CONFIG,open_port=lambda p:port);owner.connect('/fake')
    board=Board(owner,port);time.sleep(.1)
    manager=Manager(owner);test=VacuumBenchTest(manager,tmp_path)
    yield test,owner,port,board,manager
    board.stop.set()
    if owner.holding or owner.task_id:owner.unclaim(released=True)
    owner.disconnect()


def events(test_folder):
    return [json.loads(l) for l in (test_folder/'events.jsonl').read_text().splitlines()]


def test_grip_then_release_confirms_with_pressure_gate_and_records(bench):
    test,owner,port,board,manager=bench
    assert port.writes==[] and test.blockers()==[]
    result=test.grip()
    assert result['outcome']=='GRIP_CONFIRMED' and result['confirmed_after_valve_s']>=CONFIG['grip_dwell_s']
    assert port.writes==[valve_frame(0x12)] and owner.holding and test.owns() and not manager.busy
    assert any('先做释放测试' in b for b in test.blockers())
    # A cycle cannot claim the vacuum while the test holds it.
    with pytest.raises(RuntimeError,match='VACUUM_TASK_ACTIVE'):owner.claim('cycle',lambda **e:None,lambda r:None)
    time.sleep(.2);assert test.hold_summary()['min_kpa']==-80.
    folder=test.folder
    result=test.release()
    assert result['outcome']=='RELEASE_CONFIRMED' and port.writes==[valve_frame(0x12),valve_frame(0x13)]
    assert not owner.holding and owner.task_id is None and test.test_id is None and test.state=='RELEASED'
    kinds=[e['kind'] for e in events(folder)]
    assert kinds[0]=='test_start' and 'valve_command' in kinds and 'pressure' in kinds
    assert 'grip_result' in kinds and 'release_result' in kinds and kinds[-1]=='test_end'


def test_grip_without_seal_times_out_and_keeps_holding_until_release(bench):
    test,owner,port,board,manager=bench
    board.grip_kpa=-20.  # open cups: never reaches -60
    result=test.grip()
    assert result['outcome']=='GRIP_PRESSURE_TIMEOUT' and result['extreme_kpa']==-20.
    assert owner.holding and test.state=='HOLD_UNCONFIRMED'
    with pytest.raises(ValueError,match='不能开始吸附测试'):test.grip()
    assert test.release()['outcome']=='RELEASE_CONFIRMED' and not owner.holding


def test_stop_ends_wait_without_valve_write_and_release_rearms(bench):
    test,owner,port,board,manager=bench
    board.grip_kpa=-20.
    t=threading.Thread(target=test.grip);t.start();time.sleep(.2)
    owner.inhibit()  # TaskManager.stop() calls exactly this
    t.join(2);assert test.last_result['outcome']=='STOPPED_BY_OPERATOR'
    assert port.writes==[valve_frame(0x12)] and owner.holding
    assert test.release()['outcome']=='RELEASE_CONFIRMED'


def test_release_not_confirmed_keeps_flag_and_operator_clear_writes_nothing(bench):
    test,owner,port,board,manager=bench
    board.release_kpa=-40.
    assert test.grip()['outcome']=='GRIP_CONFIRMED'
    assert test.release()['outcome']=='RELEASE_PRESSURE_TIMEOUT' and owner.holding
    writes=list(port.writes);folder=test.folder
    test.operator_clear()
    assert port.writes==writes and not owner.holding and owner.task_id is None and test.state=='CLEARED_BY_OPERATOR'
    assert any(e['kind']=='operator_confirmed_no_load' for e in events(folder))


def test_refuses_cycle_owned_vacuum_active_task_and_stale_pressure(bench):
    test,owner,port,board,manager=bench
    manager.busy=True
    with pytest.raises(ValueError,match='任务或运动操作进行中'):test.grip()
    manager.busy=False
    owner.claim('cycle-task',lambda **e:None,lambda r:None)
    with pytest.raises(ValueError,match='真空由任务占用'):test.grip()
    with pytest.raises(ValueError,match='没有本测试'):test.release()
    with pytest.raises(ValueError,match='没有本测试'):test.operator_clear()
    owner.unclaim()
    board.stop.set();board.thread.join(1);time.sleep(.05)
    stamp=Stamp.receive();owner.observer.binary[1]=GaugeReading(1,-1.,replace(stamp,received_monotonic_ns=stamp.received_monotonic_ns-int(5e9)),'CHECKED_BINARY','')
    with pytest.raises(ValueError,match='STALE'):test.grip()
    assert port.writes==[]


def test_timeout_cap_is_4_s_user_decision_2026_11_26():
    from pathlib import Path
    from unloading_sim.workbench.single_carton.config import load, issues
    c=load(Path(__file__).resolve().parents[1]/'configs/workbench/single_carton.example.json', code_defaults=False)
    assert c['vacuum']['grip_timeout_s']==c['vacuum']['release_timeout_s']==4.
    assert not [e for e in issues(c) if e.startswith('vacuum：')]
    looser=dict(c,vacuum=dict(c['vacuum'],release_timeout_s=4.5))
    assert any(e.startswith('vacuum：') and 'release_timeout_s' in e for e in issues(looser))
