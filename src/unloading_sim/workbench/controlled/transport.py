"""Owned TCP command/stop channels plus independent measured UDP feedback.

Only explicit connect performs IO. No enable, tool-setting, movej or valve API.
"""
import fcntl
import importlib.metadata
import json
from pathlib import Path
import socket
import threading
import time
import struct
import os
import hashlib
from collections import deque
import numpy as np
from .trajectory import inverse_mapping,expected_controller_payload

def payload_matches(tool,mass):
    try:return np.isfinite(float(tool['payload'])) and abs(float(tool['payload'])-mass)<=.05
    except (TypeError,ValueError,KeyError):return False

def controller_has_alarm(data):
    # Gen-3 sends err:[0] for no system error. A nonempty zero-valued list is
    # truthy in Python; inspect codes instead. Malformed packets are not evidence.
    system=data['err'];joints=data['joint_status']['joint_err_code']
    if type(system) is int:system=[system]
    if (not isinstance(system,list) or not system or not isinstance(joints,list)
            or len(joints)!=6 or any(type(x) is not int for x in system+joints)):
        raise ValueError('INVALID_CONTROLLER_ERROR_CODES')
    return any(x!=0 for x in system+joints)

class StopChannel:
    """Dedicated controller TCP socket, independent of SDK global locks.

    Gen-3 JSON set_arm_stop is the documented counterpart of rm_set_arm_stop.
    Connect sends nothing. No generic command method is exposed.
    """
    def __init__(self,host,port):
        self.socket=socket.create_connection((host,int(port)),timeout=1.)
        self.socket.settimeout(.25);self.reply=None
    def stop(self):
        self.socket.sendall(b'{"command":"set_arm_stop"}\r\n')
        data=b'';deadline=time.monotonic()+.5
        while time.monotonic()<deadline and len(data)<8192:
            chunk=self.socket.recv(4096)
            if not chunk:raise OSError('STOP_CHANNEL_CLOSED')
            data+=chunk
            while b'\n' in data:
                line,data=data.split(b'\n',1);reply=json.loads(line)
                if reply.get('command')=='set_arm_stop':
                    self.reply=reply
                    return 0 if reply.get('arm_stop') is True else 1
        raise TimeoutError('STOP_REPLY_TIMEOUT')
    def close(self):self.socket.close()

class RealmanTransport:
    def __init__(self, mapping, config, sdk_factory=None):
        if mapping.source_unit!='degree' or mapping.source_names!=tuple(f'J{i}' for i in range(1,7)):
            raise ValueError('REALMAN_SDK_REQUIRES_EXPLICIT_J1_J6_DEGREES')
        self.mapping,self.config,self.factory=mapping,config,sdk_factory
        self.connected=False;self.latest=None;self._stop_result=None;self._halt=threading.Event()
        self.sender=self.stopper=None;self.sock=None;self.lease=None;self.sequence=0
        self.capabilities={};self._stop_lock=threading.Lock()
        self.stop_api='GEN3_JSON_set_arm_stop; 0=arm_stop:true, 1=false, -999=transport failure'
        self._send_disabled=threading.Event();self._send_disabled.set()
        self.intervals=deque(maxlen=400)
    def connect(self,host,port):
        if self.connected:raise RuntimeError('ALREADY_CONNECTED')
        root=Path(self.config['record_root']);root.mkdir(parents=True,exist_ok=True)
        lease_id=hashlib.sha256(f'{socket.gethostbyname(host)}:{int(port)}'.encode()).hexdigest()[:20]
        descriptor=os.open(f'/tmp/eco65-execution-{os.getuid()}-{lease_id}.lock',
                           os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
        self.lease=os.fdopen(descriptor,'a')
        try:fcntl.flock(self.lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:self.lease.close();self.lease=None;raise RuntimeError('ROBOT_ALREADY_OWNED')
        try:
            if self.factory is None:
                from Robotic_Arm.rm_robot_interface import RoboticArm,rm_thread_mode_e,rm_api_version
                binary_version=rm_api_version()
                if isinstance(binary_version,bytes):binary_version=binary_version.decode()
                self.capabilities['binary_sdk_version']=str(binary_version)
                if str(binary_version)!='1.1.7':raise RuntimeError('UNREVIEWED_BINARY_SDK')
                # One process-global SDK init. No SDK UDP receiver; timestamps
                # are captured by our independent socket before GUI work.
                self.sender=RoboticArm() if getattr(self,'initialized',False) else RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
                self.initialized=True
            else:self.sender=self.factory()
        except Exception:
            self.lease.close();self.lease=None;raise
        self._halt.clear();self.latest=None;self.intervals.clear()
        try:
            h=self.sender.rm_create_robot_arm(host,int(port))
            if h is None or h.id<=0:raise RuntimeError('CONNECTION_FAILED')
            self.stopper=StopChannel(host,port)
            def get(name):
                start=time.monotonic();code,value=getattr(self.sender,name)()
                if code:raise RuntimeError(f'{name}:{code}')
                self.capabilities[name]={'value':value,'latency_s':time.monotonic()-start};return value
            info=get('rm_get_robot_info');software=get('rm_get_arm_software_info')
            if info['arm_dof']!=6 or info['arm_model']!='ECO_65' or info['robot_controller_version']!=3:raise RuntimeError('UNREVIEWED_CONTROLLER')
            if not software['ctrl_info']['version'].startswith('V1.7.0'):raise RuntimeError('UNREVIEWED_FIRMWARE')
            if importlib.metadata.version('Robotic_Arm')!='1.1.7':raise RuntimeError('UNREVIEWED_SDK')
            self.capabilities['python_sdk_version']=importlib.metadata.version('Robotic_Arm')
            self.capabilities['send_configuration']=dict(period_s=.005,follow=True,trajectory_mode=0,radio=0,expand=0,units='SDK_DEGREES_INTERNAL_RADIANS')
            tool=get('rm_get_current_tool_frame')
            self.capabilities['payload_agrees']=payload_matches(tool,expected_controller_payload(self.config))
            self.capabilities['software_stop']=self.stop_api+'; NOT hardware emergency stop'
            self.saved_udp=get('rm_get_realtime_push')
            if get('rm_get_arm_run_mode')!=1:raise RuntimeError('REAL_CONTROLLER_MODE_REQUIRED')
            get('rm_get_current_work_frame')
            for name in ('rm_get_joint_max_speed','rm_get_joint_max_acc','rm_get_joint_min_pos','rm_get_joint_max_pos'):get(name)
            route=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);route.connect((host,int(port)))
            local=route.getsockname()[0];route.close()
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET,35,1)  # Linux SO_TIMESTAMPNS
            self.sock.bind((local,self.config.get('udp_port',18089)));self.sock.settimeout(.05)
            self.host=host
            from Robotic_Arm.rm_robot_interface import rm_realtime_push_config_t,rm_udp_custom_config_t
            self.udp_type=rm_realtime_push_config_t
            self.custom_type=rm_udp_custom_config_t
            custom=dict(self.saved_udp['custom_config']);custom.update(joint_speed=1,arm_current_status=1)
            code=self.sender.rm_set_realtime_push(rm_realtime_push_config_t(1,True,self.sock.getsockname()[1],-1,local,rm_udp_custom_config_t(**custom)))
            if code:raise RuntimeError(f'UDP_CONFIGURATION_FAILED:{code}')
            self.udp_changed=True
            self.connected=True
            self.receiver=threading.Thread(target=self._receive,daemon=True,name='realman-udp');self.receiver.start()
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:time.sleep(.01)
            if self.latest is None:raise RuntimeError('NO_LIVE_UDP_FEEDBACK')
            self.capabilities['feedback_cadence']=dict(samples=len(self.intervals),
                p95_s=float(np.percentile(self.intervals,95)) if self.intervals else None,
                max_s=max(self.intervals) if self.intervals else None,source='ACTUAL_UDP_RECEIPTS')
            if len(self.intervals)<100 or max(self.intervals)>.05:raise RuntimeError('UDP_MONITORING_CADENCE_UNSUITABLE')
            return self.capabilities
        except Exception:self.disconnect();raise
    def _receive(self):
        previous=None
        while not self._halt.is_set():
            try:
                raw,anc,flags,peer=self.sock.recvmsg(65535,256);now=time.monotonic()
                timestamp=next((value for level,kind,value in anc if level==socket.SOL_SOCKET and kind==35),None)
                if timestamp is None:continue
                sec,nsec=struct.unpack('qq',timestamp[:16]);age=(time.time_ns()-sec*10**9-nsec)/1e9
                if age<0 or age>.05:continue
                now-=age  # Original kernel receipt, never re-date a queued packet.
                if peer[0]!=self.host:continue
                data=json.loads(raw);j=data['joint_status']
                if data.get('state')!='realtime_arm_joint_state':continue
                degrees=np.asarray(j['joint_position'],float)
                if degrees.shape!=(6,) or not np.isfinite(degrees).all():continue
                q=self.mapping.apply(degrees*.001,self.mapping.source_names)
                velocity=None
                speed=np.asarray(j.get('joint_speed',[]),float)
                if speed.shape==(6,) and np.isfinite(speed).all():
                    velocity=(np.deg2rad(speed*.01)[list(self.mapping.source_indices)]*self.mapping.signs).tolist()
                if previous is not None and .001<=now-previous['received_s']<=.05:
                    if velocity is None:velocity=((q-np.asarray(previous['q']))/(now-previous['received_s'])).tolist()
                if previous is not None:self.intervals.append(now-previous['received_s'])
                if len(j['joint_en_flag'])!=6 or len(j['joint_err_code'])!=6:continue
                self.sequence+=1
                sample=dict(source='LIVE',sequence=self.sequence,received_s=now,q=q.tolist(),velocity=velocity,
                            received_utc_ns=sec*10**9+nsec,
                            controller_sample_time='UNKNOWN',velocity_source='MEASURED_UDP_SPEED' if speed.shape==(6,) else 'FINITE_DIFFERENCE_OF_ACTUAL_UDP_SAMPLES',
                            enabled=all(x==1 for x in j['joint_en_flag']),
                            alarm=controller_has_alarm(data),raw=data)
                self.latest=sample;previous=sample
            except socket.timeout:continue
            except (OSError,ValueError,KeyError,TypeError):
                # A malformed packet never refreshes the last valid timestamp.
                continue
    def arm(self):self._send_disabled.clear()
    def check_limits(self,prepared):
        """Controller degree limits must contain the already checked SI envelope."""
        limits=prepared['limits']
        lo=np.asarray(inverse_mapping(limits['lower'],self.mapping))
        hi=np.asarray(inverse_mapping(limits['upper'],self.mapping))
        low=np.minimum(lo,hi);high=np.maximum(lo,hi)
        indices=list(self.mapping.source_indices)
        v=np.empty(6);a=np.empty(6)
        v[indices]=np.rad2deg(limits['velocity']);a[indices]=np.rad2deg(limits['acceleration'])
        values=[np.asarray(self.capabilities[name]['value'],float) for name in
                ('rm_get_joint_min_pos','rm_get_joint_max_pos','rm_get_joint_max_speed','rm_get_joint_max_acc')]
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in values):raise ValueError('INVALID_CONTROLLER_LIMITS')
        lower,upper,speed,acceleration=values
        if np.any(low<lower-1e-4) or np.any(high>upper+1e-4) or np.any(v>speed) or np.any(a>acceleration):
            raise ValueError('CONTROLLER_LIMITS_NARROWER_THAN_CHECKED_ENVELOPE')
    def send(self,q):
        if self._send_disabled.is_set() or (getattr(self,'external_stop',None) is not None and self.external_stop.is_set()):return -998
        return self.sender.rm_movej_canfd(inverse_mapping(q,self.mapping),True,0,0,0)
    def request_stop(self):
        self._send_disabled.set()
        with self._stop_lock:
            self._stop_result=None
            if getattr(self,'stop_thread',None) and self.stop_thread.is_alive():
                self._repeat_stop=True;return
            self._repeat_stop=False
            def run():
                while True:
                    try:result=self.stopper.stop()
                    except Exception:result=-999
                    with self._stop_lock:
                        if not self._repeat_stop:self._stop_result=result;return
                        self._repeat_stop=False
            self.stop_thread=threading.Thread(target=run,name='realman-stop',daemon=True);self.stop_thread.start()
    def stop_result(self):return self._stop_result
    def disconnect(self):
        self._send_disabled.set()
        self.connected=False;self._halt.set()
        if self.sock:self.sock.close();self.sock=None
        if hasattr(self,'receiver'):self.receiver.join(.2)
        if getattr(self,'udp_changed',False):
            old=self.saved_udp
            try:self.sender.rm_set_realtime_push(self.udp_type(old['cycle'],old['enable'],old['port'],old['force_coordinate'],old['ip'],self.custom_type(**old['custom_config'])))
            except Exception:pass
            self.udp_changed=False
        for sdk in (self.sender,):
            if sdk is not None:
                try:sdk.rm_delete_robot_arm()
                except Exception:pass
        self.sender=None
        if self.stopper:self.stopper.close();self.stopper=None
        if self.lease:self.lease.close();self.lease=None
        self.latest=None
