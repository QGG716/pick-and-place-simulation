from pathlib import Path
from types import SimpleNamespace
import json
import tarfile
import numpy as np
import pytest
from test_l01_selected_routes import setup
from l00_top_continuation import ORDER, RETAINED, bind_scope, SCOPE
from unloading_sim.low_top_receiver import layer_top_option


def test_xplus200_moves_only_robot_and_owned_tool():
    from copy import deepcopy
    from unloading_sim.workcell_layout import load_workcell_layout, audit_layout_constraints
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    root=Path(__file__).resolve().parents[1]/'configs/workcells'
    name='m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile_tool_extension200.yaml'
    old=load_workcell_layout(root/name)
    new=load_workcell_layout(root/name.replace('x_minus_500','x_minus_300'))
    changed=deepcopy(new.data)
    assert changed['robot']['base_origin_xy_a_m']==[.325,-.1]
    assert changed['robot']['base_front_edge_x_world_m']==-1.4
    changed['robot']['base_origin_xy_a_m']=old.data['robot']['base_origin_xy_a_m']
    changed['robot']['base_front_edge_x_world_m']=old.data['robot']['base_front_edge_x_world_m']
    assert changed['evidence']['approval_basis']=='user_20261024_l00_mount_xplus200_from_x500'
    changed['evidence']['approval_basis']=old.data['evidence']['approval_basis']
    assert changed==old.data
    audit=audit_layout_constraints(new)
    assert audit['overall_status']=='PASS'
    assert audit['checks']['robot_base_origin_world']['expected']==[-1.625,.25,.6]
    assert audit['checks']['requested_minimum_transverse_surface']['actual']==.25
    a,b=old.robot(),new.robot();q=np.array([-1.12,1.20,-.31,-2.01,-1.55,-.04])
    for key,frame in a.named_link_frames(q).items():
        other=b.named_link_frames(q)[key]
        assert np.allclose(other[:3,3]-frame[:3,3],[.2,0.,0.],atol=1e-12,rtol=0)
        assert np.allclose(other[:3,:3],frame[:3,:3],atol=1e-12,rtol=0)
    before={v.name:v for v in a.tool_all_physical_obbs(q)}
    after={v.name:v for v in b.tool_all_physical_obbs(q)}
    assert len(after)==len(before)==202
    for key,box in before.items():
        assert np.allclose(after[key].corners()-box.corners(),[.2,0.,0.],atol=1e-12,rtol=0)
    for left,right in zip(coupled_conveyor_state(old,.25).boxes,coupled_conveyor_state(new,.25).boxes):
        assert left.name==right.name and np.array_equal(left.corners(),right.corners())


def predecessor(tmp_path):
    from layer_top_continuation import L01_HMAX_RECORDS
    source=Path(__file__).resolve().parents[1]/'docs/experiments/m710_l01_hmax_20261024'
    prior=tmp_path/'prior';prior.mkdir()
    for name in L01_HMAX_RECORDS:(prior/name).write_bytes((source/'records'/name).read_bytes())
    with tarfile.open(source/'formal-v1-raw.tar.xz') as archive:
        for m in archive:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                p=prior/Path(m.name).relative_to('formal-v1');p.parent.mkdir(parents=True,exist_ok=True)
                p.write_bytes(archive.extractfile(m).read())
    return prior


def test_four_bottom_top_options_preserve_accepted_world_and_fixed_height(tmp_path):
    from layer_top_continuation import restore_l01_hmax_terminal
    from unloading_sim.coupled_batch import BatchState
    boxes,frozen,world,_=setup()
    state=BatchState(frozen,tuple(boxes.values()),np.zeros(6),2.05)
    restore_l01_hmax_terminal(predecessor(tmp_path),state)
    permission=bind_scope(frozen,state)
    assert len(state.ledger)==34 and len(state.cartons)==6 and not state.cycles
    assert state.root()['predecessor_final_state_receipt'] is None
    assert {b.name for b in state.cartons}-set(ORDER)==set(RETAINED)
    for name in ORDER:
        o=layer_top_option(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,state.height,permission)
        assert o['face']['geometric_face_id']=='z+'
        assert o['box_top_world_m']==pytest.approx(.3)
        assert o['height_m']==o['pre_approach_height_m']==pytest.approx(.25)
        assert o['release_bottom_world_m']==pytest.approx(.30)
        assert o['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
    state.cartons=state.cartons[:-1]
    with pytest.raises(ValueError,match='ROOT_CHANGED'):bind_scope(frozen,state)


@pytest.mark.parametrize('mount,x',[('x_minus_500_y_minus_100',-1.825),('x_minus_300_y_minus_100',-1.625)])
def test_formal_l00_dispatch_uses_production_backend_and_no_tool_chassis_grant(tmp_path,monkeypatch,mount,x):
    pytest.importorskip('pinocchio')
    import sys
    import plan_m710_xy_top_population as entry
    output=tmp_path/'run';prior=predecessor(tmp_path)
    def dispatch(args,original,built,frozen,seed,folder,*,initial_state,target_ids):
        assert type(built.robot).__name__=='PinocchioHppFclBackend'
        assert np.allclose(built.robot.base_transform[:3,3],[x,.25,.6],atol=1e-12,rtol=0)
        assert seed==71070 and target_ids==[ORDER[0]]
        assert args.layer_top_only and args.tool_connector_extension_m==.2
        assert args.l01_hmax_permission is None and args.c04_face_permission is None
        assert set(ORDER).isdisjoint(built.collision_policy.tool_chassis_contact_target_ids)
        assert all(not built.collision_policy.tool_chassis_pairs(n,('tool_rigid_11',)) for n in ORDER)
        from unloading_sim.coupled_batch import batch_scene
        from unloading_sim.low_top_receiver import LowTopReceiverConnector
        from unloading_sim.contact_target import simulation_contact_targets
        scene=batch_scene(original,initial_state,built.robot,original.policy.layout_validation.layout)
        target=next(b for b in initial_state.cartons if b.name==ORDER[0])
        option=layer_top_option(target,scene.receiver,next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),frozen,initial_state.height,args.layer_top_permission)
        chosen=next(t for t in simulation_contact_targets(scene,target,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m)[0] if t.geometric_face_id=='z+')
        connector=object.__new__(LowTopReceiverConnector);connector.option=option;connector.batch_root=initial_state.root()
        connector.layer_top_permission=args.layer_top_permission
        connector.validate_template(chosen)
        raise RuntimeError('TEST_PRODUCTION_DISPATCH_REACHED')
    monkeypatch.setattr(entry,'run_seed',dispatch)
    monkeypatch.setattr(sys,'argv',['plan','--output',str(output),'--worker-command','UNUSED_TEST_DISPATCH',
        '--worker-binary',str(Path(entry.__file__)),'--source-commit','INTEGRATION_TEST','--development',
        '--mounting-variant',mount,'--continue-l00-top-from',str(prior),
        '--tool-connector-extension-m','0.2'])
    with pytest.raises(RuntimeError,match='TEST_PRODUCTION_DISPATCH_REACHED'):entry.main()
    check=json.loads((output/'initial-state-check.json').read_text())
    assert check['passed'] and check['authorized_root_target']==ORDER[0]
    p=json.loads((output/'protocol.json').read_text())
    assert p['scope']==SCOPE and p['scoped_target_ids']==list(ORDER)
    from verify_m710_population import expected_population_mount
    assert expected_population_mount(p)==[x,.25,.6]
