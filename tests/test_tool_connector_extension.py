from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.tool_connector_extension import connector_extension_m,extended_connector_boxes,bind_connector_frames
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_load_tool_frame_contract

ROOT=Path(__file__).resolve().parents[1]
BASE='m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile'


def layouts():
    return [load_workcell_layout(ROOT/'configs/workcells'/(BASE+suffix+'.yaml'))
            for suffix in ('','_tool_extension100')]


def test_literal_connector_geometry_and_fixed_flange():
    old,new=layouts()
    data=deepcopy(new.data);assert data['tool'].pop('connector_extension_m')==.1
    assert data==old.data
    a,b=old.robot(),new.robot()
    for q in (np.zeros(6),np.array([.26,1.25,-.82,-3.44,-2.05,-.14])):
        flange=a.named_link_frames(q)['flange']
        assert np.allclose(flange,b.named_link_frames(q)['flange'])
        x=flange[:3,0]
        assert np.allclose(b.fk(q)[:3,3]-a.fk(q)[:3,3],.1*x)
        before={v.name:v for v in a.tool_all_physical_obbs(q)}
        after={v.name:v for v in b.tool_all_physical_obbs(q)}
        assert len(before)==len(after)==202 and before.keys()==after.keys()
        for name,box in before.items():
            result=after[name]
            if name in ('tool_rigid_11','tool_rigid_12'):
                points=result.transformed(np.linalg.inv(flange)).corners()
                assert points[:,0].min()==pytest.approx(.01,abs=1e-7)
                assert points[:,0].max()==pytest.approx(.25,abs=1e-7)
                assert np.allclose(result.center-box.center,.05*x)
            elif name=='tool_rigid_13':
                assert np.allclose(result.corners(),box.corners())
            else:
                assert np.allclose(result.corners()-box.corners(),.1*x)


def test_frame_contract_bound_once_and_snapshot_includes_extension():
    _,layout=layouts()
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    policy=replace(policy,layout_validation=replace(policy.layout_validation,layout=layout))
    scene=build_verified_motion_input(policy,ROOT)
    frames=scene.policy.tool_frames
    assert frames.flange_from_virtual_task_tcp[0,3]==pytest.approx(.35)
    assert frames.flange_from_uncompressed_cup_plane[0,3]==pytest.approx(.3275)
    assert frames.flange_from_physical_contact[0,3]==pytest.approx(.3175)
    assert frames.virtual_to_physical_contact_offset_m==pytest.approx(.0325)
    assert bind_connector_frames(scene.policy).data==scene.policy.data
    assert scene.snapshot['tool']['tcp_transform']['translation_xyz_m']==[.35,0.,0.]
    assert scene.snapshot['tool']['connector_extension']['new_plate_axial_length_m']==.24
    with pytest.raises(ValueError):_load_tool_frame_contract(scene.policy.data['tool_frame_contract'])


def test_unapproved_extension_or_changed_source_is_rejected():
    for value in (.05,.2,float('nan')):
        with pytest.raises(ValueError):connector_extension_m({'connector_extension_m':value})
    with pytest.raises(ValueError,match='SOURCE_BINDING_CHANGED'):extended_connector_boxes(np.zeros((130,6)),.1)


def test_production_backend_uses_extended_physical_tool():
    pytest.importorskip('pinocchio')
    from plan_m710_mount_comparison import make_context
    from unloading_sim.moveit2_backend import native_tool_collision_boxes
    _,layout=layouts()
    scene,connector,_=make_context('x_minus_500_y_minus_100',layout_path=layout.config_path,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    assert type(connector.robot).__name__=='PinocchioHppFclBackend'
    q=np.asarray(scene.snapshot['robot']['q_rad'])
    flange=connector.robot.named_link_frames(q)['flange']
    physical=np.linalg.inv(flange)@connector.physical_from_virtual(connector.robot.fk(q))
    assert physical[0,3]==pytest.approx(.3175)
    boxes,_=native_tool_collision_boxes(connector,q)
    assert len(boxes)==202
    for name,lo,hi in [('tool_rigid_11',.01,.25),('tool_rigid_12',.01,.25),('tool_rigid_13',0.,.012)]:
        box=next(b for b in boxes if b.name==name).transformed(np.linalg.inv(flange))
        assert box.corners()[:,0].min()==pytest.approx(lo,abs=1e-7)
        assert box.corners()[:,0].max()==pytest.approx(hi,abs=1e-7)
