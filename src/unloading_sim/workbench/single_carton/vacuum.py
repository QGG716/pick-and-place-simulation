"""Single owner for the common-valve board; no writes on connect/disconnect.

Only reference suction/release and six valve commands are expressible. Conveyor,
threshold writes and independent cup commands are intentionally absent.
"""
import threading
from time import monotonic
from ..devices import VacuumObserver
from ..reference.vacuum_receive import HEADER,DEVICE,BAUD


def valve_frame(command):
    if type(command)!=int or command not in {0x12,0x13,0x51,0x52,0x53,0x54,0x55,0x56}:raise ValueError('NOT_A_REVIEWED_VALVE_COMMAND')
    body=bytes((DEVICE,command))
    return HEADER+bytes((len(body),))+body+bytes((sum(body)&255,))


def opener(path):
    import serial
    import subprocess
    # A previously launched GUI may predate pyserial exclusive locking. Detect
    # an existing opener as well; never steal its incoming heartbeat bytes.
    used=subprocess.run(['fuser',str(path)],capture_output=True,timeout=2)
    if used.returncode==0 and used.stdout.strip():raise RuntimeError('SERIAL_IN_USE_CLOSE_OTHER_WORKBENCH_FIRST')
    port=serial.Serial(port=None,baudrate=BAUD,timeout=.025,write_timeout=.1,exclusive=True)
    port.dtr=False;port.rts=False;port.port=path;port.open();return port


class VacuumOwner:
    def __init__(self,config,open_port=opener,clock=monotonic):
        self.config=config;self.clock=clock;self.observer=VacuumObserver(open_port)
        self.connected=False;self.task_id=None;self.holding=False;self.inhibited=True
        self.stop_event=threading.Event();self.lock=threading.RLock();self.thread=None
        self.record=lambda **kw:None;self.on_fault=lambda reason:None;self.monitor_holding=False
        self.last_error='';self.sequence=0
    def connect(self,path):
        with self.lock:
            if self.connected:raise RuntimeError('VACUUM_ALREADY_CONNECTED')
            if self.task_id:raise RuntimeError('VACUUM_TASK_OWNS_CONNECTION')
            self.observer.connect(path);self.connected=True;self.inhibited=True;self.stop_event.clear()
            self.thread=threading.Thread(target=self._read,daemon=True,name='vacuum-feedback-watchdog');self.thread.start()
    def _read(self):
        try:
            while not self.stop_event.is_set():
                events=self.observer.read()
                for kind,sample in events:
                    if kind=='pressure':
                        self.sequence+=1
                        self.record(kind='pressure',channel=sample.channel,kpa=sample.kpa,
                            received_s=sample.stamp.received_monotonic_ns/1e9,integrity=sample.integrity,
                            raw_hex=sample.raw_hex,sequence=self.sequence)
                    if kind=='alarm':self.on_fault('VACUUM_BOARD_ALARM:'+sample['name'])
                if self.monitor_holding:
                    try:
                        p=self.pressure()
                        if p['kpa']>self.config['hold_below_kpa']:raise RuntimeError('GRIPPER_PRESSURE_LOST')
                    except Exception as e:self.on_fault(str(e))
        except Exception as e:
            self.last_error=str(e);self.connected=False;self.on_fault('VACUUM_CONNECTION_LOST:'+str(e))
        finally:self.observer.disconnect();self.connected=False
    def pressure(self):
        sample=self.observer.gauges().get(1)
        if not self.connected or sample is None or sample.integrity!='CHECKED_BINARY':raise ValueError('FRESH_CHECKED_GRIPPER_PRESSURE_REQUIRED')
        received=sample.stamp.received_monotonic_ns/1e9
        if not 0<=self.clock()-received<=self.config['max_age_s']:raise ValueError('GRIPPER_PRESSURE_STALE')
        if self.observer.alarms:raise ValueError('VACUUM_BOARD_ALARM')
        return dict(kpa=sample.kpa,received_s=received,source='LIVE',integrity=sample.integrity,channel='gripper')
    def claim(self,task_id,record,on_fault):
        with self.lock:
            if self.task_id is not None:raise RuntimeError('VACUUM_TASK_ACTIVE')
            self.pressure();self.task_id=task_id;self.inhibited=False;self.record=record;self.on_fault=on_fault
    def establish(self,task_id):self._sequence(task_id,'establish_commands')
    def release(self,task_id):self._sequence(task_id,'release_commands')
    def _sequence(self,task_id,key):
        with self.lock:
            if task_id!=self.task_id or self.inhibited:raise RuntimeError('VACUUM_WRITE_NOT_OWNED_OR_STOPPING')
            self.pressure()
            if key=='establish_commands':self.holding=True
            else:self.monitor_holding=False
            for cmd in self.config[key]:
                if self.inhibited:raise RuntimeError('VACUUM_STOPPED')
                data=valve_frame(cmd);start=self.clock()
                count=self.observer._port.write(data)
                self.record(kind='valve_command',command=cmd,frame_hex=data.hex(),sent_s=start,
                            returned_s=self.clock(),bytes_written=count)
                if count!=len(data):raise RuntimeError('VACUUM_PARTIAL_WRITE')
    def inhibit(self):
        # No automatic close/vent writes. Already-held load keeps its valve state.
        self.inhibited=True
    def unclaim(self,released=False):
        with self.lock:
            if released:self.holding=False
            if self.holding:return
            self.inhibited=True;self.monitor_holding=False;self.task_id=None
            self.record=lambda **kw:None;self.on_fault=lambda reason:None
    def disconnect(self):
        if self.task_id or self.holding:raise RuntimeError('VACUUM_TASK_OR_HELD_LOAD_REQUIRES_LOCAL_HANDLING')
        self.inhibit();self.stop_event.set()
        if self.thread:self.thread.join(.3)
        self.observer.disconnect();self.connected=False


class VacuumView:
    """Existing observation UI uses the same owner, never a second serial reader."""
    def __init__(self,owner):self.owner=owner
    @property
    def connected(self):return self.owner.connected
    @property
    def alarms(self):return self.owner.observer.alarms
    @property
    def bad_frames(self):return self.owner.observer.bad_frames
    def connect(self,path):return self.owner.connect(path)
    def disconnect(self):return self.owner.disconnect()
    def read(self):return []
    def gauges(self):return self.owner.observer.gauges()
