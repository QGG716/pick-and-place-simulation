"""Fake NetworkManager/SDK IO: tests never change host networking or robot state."""
import json
import subprocess
import threading
from types import SimpleNamespace
import pytest
from unloading_sim.workbench.single_carton.network import SavedRobotNetwork,NetworkedRobotView
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.ui import ROOT

CONFIG=dict(manager='NetworkManager',connection='saved-profile',interface='eno1')


class Commands:
    def __init__(self,active='--',kind='802-3-ethernet',default='yes',route='eno1',fail=None):
        self.active=active;self.kind=kind;self.default=default;self.route=route;self.fail=fail;self.calls=[]
    def __call__(self,args,**kwargs):
        self.calls.append(args)
        assert kwargs['timeout']<=10 and kwargs['env']['LC_ALL']=='C'
        if 'up' in args:
            if self.fail:raise self.fail
            self.active='saved-uuid';out='activated'
        elif args[0]=='ip':out=json.dumps([dict(dev=self.route,prefsrc='192.168.1.100')])
        elif 'device' in args:out=self.active
        else:out=f'robot\nsaved-uuid\n{self.kind}\neno1\n{self.default}\n'
        return SimpleNamespace(returncode=0,stdout=out,stderr='')


def test_construction_is_inert_and_saved_network_precedes_robot_connection():
    order=[];commands=Commands()
    network=SavedRobotNetwork(CONFIG,run=commands)
    service=SimpleNamespace(connect=lambda *a:(order.append(('sdk',len(commands.calls))) or {}))
    view=NetworkedRobotView(service,None,CONFIG,network=network)
    assert not commands.calls and not order
    result=view.connect('192.168.1.19',8080)
    assert order==[('sdk',4)] and result['host_network']['activated_saved_profile']
    assert commands.calls[2]==['nmcli','--wait','8','connection','up','uuid','saved-uuid','ifname','eno1']
    assert network.status.startswith('网络已就绪')
    view.connect('192.168.1.19',8080)
    assert len(commands.calls)==4 and len(order)==1  # repeated click never bounces networking


def test_active_profile_is_verified_without_reactivation():
    commands=Commands(active='saved-uuid');network=SavedRobotNetwork(CONFIG,run=commands)
    result=network.ensure('192.168.1.19')
    assert result['activated_saved_profile'] is False
    assert not any('up' in c for c in commands.calls)


@pytest.mark.parametrize('kwargs,reason',[
    ({'active':'unrelated-uuid'},'INTERFACE_IN_USE'),
    ({'kind':'802-11-wireless'},'INTERFACE_MISMATCH'),
    ({'default':'no'},'DEFAULT_ROUTE'),
    ({'route':'wlp0s20f3'},'ROUTE_MISMATCH'),
    ({'fail':subprocess.TimeoutExpired('nmcli',10)},'NETWORK_TIMEOUT'),
])
def test_network_failure_is_explicit_and_never_calls_sdk(kwargs,reason):
    commands=Commands(**kwargs);network=SavedRobotNetwork(CONFIG,run=commands)
    def forbidden(*a):raise AssertionError('SDK must not run before network readiness')
    view=NetworkedRobotView(SimpleNamespace(connect=forbidden),None,CONFIG,network=network)
    with pytest.raises((RuntimeError,ValueError),match=reason):view.connect('192.168.1.19',8080)
    assert '网络配置失败' in network.status and not view.connected


@pytest.mark.parametrize('failure,reason',[
    (FileNotFoundError(),'TOOL_MISSING'),
    (SimpleNamespace(returncode=10,stdout='',stderr='Unknown connection'),'Unknown connection'),
    (SimpleNamespace(returncode=1,stdout='',stderr='Not authorized'),'Not authorized'),
])
def test_missing_profile_tools_and_permissions_are_visible(failure,reason):
    def run(*a,**kw):
        if isinstance(failure,Exception):raise failure
        return failure
    network=SavedRobotNetwork(CONFIG,run=run)
    with pytest.raises(RuntimeError,match=reason):network.ensure('192.168.1.19')


def test_saved_local_device_configuration_overrides_example(tmp_path):
    config=json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text())
    device=tmp_path/'device.json';device.write_text(json.dumps(dict(robot_network=CONFIG)))
    config['device_config']=str(device);path=tmp_path/'single.json';path.write_text(json.dumps(config))
    c=load(path)
    assert c['robot_network']==CONFIG and str(device) in c['loaded_file_sha256']


def test_slow_network_restore_does_not_block_camera_or_vacuum():
    from test_single_carton_live import Camera,Device,Model,until
    from unloading_sim.workbench.single_carton.live import SingleBackend,LiveVision
    entered=threading.Event();gate=threading.Event();calls=[]
    def ensure(host):entered.set();gate.wait(2);return {'status':'READY'}
    service=SimpleNamespace(process=None,connect=lambda *a:(calls.append('sdk') or {}),snapshot=lambda:None)
    robot=NetworkedRobotView(service,None,CONFIG,network=SimpleNamespace(ensure=ensure))
    class Vacuum(Device):
        def read(self):return []
    camera=Camera();vacuum=Vacuum();backend=SingleBackend(camera,robot,vacuum,service)
    vision=LiveVision({},factory=Model);backend.start_observation(vision)
    try:
        backend.command('robot','connect','192.168.1.19',8080)
        backend.command('camera','connect','saved-camera');backend.command('vacuum','connect','/fake/serial')
        until(lambda:entered.is_set() and vision.completed>0 and vacuum.connected)
        assert calls==[] and backend.workers['robot'].busy
        gate.set();until(lambda:robot.connected)
        assert calls==['sdk']
    finally:gate.set();backend.close()
