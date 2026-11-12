"""Controller settings and real UDP schema, without hardware or SDK loading."""
import json
import socket
import struct
import time

import pytest

from unloading_sim.workbench.controlled.planner import ROOT,load_config
from unloading_sim.workbench.controlled.trajectory import expected_controller_payload
from unloading_sim.workbench.controlled.transport import RealmanTransport,controller_has_alarm,payload_matches
from unloading_sim.workbench.ui import load_mapping


def test_test_setting_does_not_change_physical_tool_mass(tmp_path):
    path=ROOT/'configs/workbench/controlled.example.json';config=load_config(path)
    assert config['tool_mass_kg']==1.5 and expected_controller_payload(config)==3.
    assert payload_matches({'payload':3.},expected_controller_payload(config))
    assert not payload_matches({'payload':1.5},expected_controller_payload(config))
    data=json.loads(path.read_text());data.pop('controller_payload_basis')
    changed=tmp_path/'config.json';changed.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='EXPLICIT_FIELD_SETTING'):load_config(changed)
    data.pop('controller_payload_kg');changed.write_text(json.dumps(data))
    assert expected_controller_payload(load_config(changed))==1.5  # Legacy profile.


@pytest.mark.parametrize('mass',[0.,-1.,5.01,float('nan'),float('inf')])
def test_controller_setting_must_be_finite_and_within_rating(mass):
    with pytest.raises(ValueError,match='INVALID_CONTROLLER_PAYLOAD_SETTING'):
        expected_controller_payload(dict(controller_payload_kg=mass,tool_mass_kg=1.5))


@pytest.mark.parametrize('system,joints,alarm',[
    ([0],[0]*6,False),(0,[0]*6,False),([0,0],[0]*6,False),
    ([0,4096],[0]*6,True),(4096,[0]*6,True),([0],[0,0,1,0,0,0],True),
])
def test_real_udp_receiver_uses_error_codes_not_list_truthiness(system,joints,alarm):
    mapping=load_mapping(ROOT/'configs/workbench/joint_mapping.json')
    transport=RealmanTransport(mapping,{})
    transport.host='127.0.0.1'
    packet=dict(state='realtime_arm_joint_state',arm_current_status='idle',err=system,
        joint_status=dict(joint_err_code=joints,joint_en_flag=[1]*6,joint_position=[0]*6,joint_speed=[0]*6))
    class FakeSocket:
        def recvmsg(self,*args):
            transport._halt.set()
            sec,nsec=divmod(time.time_ns(),10**9)
            return json.dumps(packet).encode(),[(socket.SOL_SOCKET,35,struct.pack('qq',sec,nsec))],0,('127.0.0.1',8089)
    transport.sock=FakeSocket();transport._receive()
    assert transport.latest['alarm'] is alarm and transport.latest['source']=='LIVE'
    assert transport.latest['raw']['err']==system


@pytest.mark.parametrize('system,joints',[
    (None,[0]*6),([],[0]*6),(['0'],[0]*6),([False],[0]*6),
    ({'err':[0]},[0]*6),([0],[0]*5),([0],[None]*6),
])
def test_unknown_error_payload_is_not_accepted_as_no_alarm(system,joints):
    with pytest.raises(ValueError,match='INVALID_CONTROLLER_ERROR_CODES'):
        controller_has_alarm(dict(err=system,joint_status=dict(joint_err_code=joints)))


@pytest.mark.parametrize('speed,source',[([0]*6,'MEASURED_UDP_SPEED'),
    ([float('nan')]*6,'FINITE_DIFFERENCE_OF_ACTUAL_UDP_SAMPLES'),([], 'FINITE_DIFFERENCE_OF_ACTUAL_UDP_SAMPLES')])
def test_invalid_udp_speed_is_never_labeled_measured(speed,source):
    transport=RealmanTransport(load_mapping(ROOT/'configs/workbench/joint_mapping.json'),{})
    transport.host='127.0.0.1'
    packet=dict(state='realtime_arm_joint_state',err=[0],
        joint_status=dict(joint_err_code=[0]*6,joint_en_flag=[1]*6,joint_position=[0]*6,joint_speed=speed))
    class FakeSocket:
        def recvmsg(self,*args):
            transport._halt.set();sec,nsec=divmod(time.time_ns(),10**9)
            return json.dumps(packet).encode(),[(socket.SOL_SOCKET,35,struct.pack('qq',sec,nsec))],0,('127.0.0.1',8089)
    transport.sock=FakeSocket();transport._receive()
    assert transport.latest['velocity_source']==source
    if source!='MEASURED_UDP_SPEED':assert transport.latest['velocity'] is None
