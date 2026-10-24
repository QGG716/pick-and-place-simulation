from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import tarfile
import numpy as np
import pytest
from test_l01_selected_routes import setup
from unloading_sim.bottom_rotated import ORDER,RETAINED,SCOPE,rotation_permission,rotated_option,rotated_target,validate_rotation
from unloading_sim.low_top_receiver import selected_layer_top_permission,layer_top_option,LowTopReceiverConnector
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.support import SupportRelationGraph


def predecessor(tmp_path):
    from layer_top_continuation import BOTTOM_CURRENT_RECORDS
    source=Path(__file__).resolve().parents[1]/'docs/experiments/m710_l00_top_xplus200_20261024'
    prior=tmp_path/'prior';prior.mkdir()
    for name in BOTTOM_CURRENT_RECORDS:(prior/name).write_bytes((source/'formal-v1'/name).read_bytes())
    with tarfile.open(source/'formal-v1-raw.tar.xz') as archive:
        for m in archive:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                p=prior/Path(m.name).relative_to('formal-v1');p.parent.mkdir(parents=True,exist_ok=True)
                p.write_bytes(archive.extractfile(m).read())
    return prior


def test_checked_36_root_and_support_blocking(tmp_path):
    from layer_top_continuation import restore_bottom_current_terminal
    from unloading_sim.coupled_batch import BatchState
    boxes,frozen,_,_=setup();state=BatchState(frozen,tuple(boxes.values()),np.zeros(6),2.05)
    restore_bottom_current_terminal(predecessor(tmp_path),state)
    assert len(state.ledger)==36 and {b.name for b in state.cartons}==set(ORDER)|set(RETAINED)
    assert not state.cycles and state.root()['predecessor_final_state_receipt'] is None
    assert state.q==pytest.approx([-.22060567355624747,.35615009540041614,-.6779213292413405,4.52160119252798e-11,-.5367249021530525,-2.9209869801140735])
    graph=SupportRelationGraph.build(state.cartons)
    assert graph.supports['carton_l00_c04']=={'carton_l01_c04'}


@pytest.mark.parametrize('degrees',[90,-90])
def test_quarter_turn_keeps_face_and_pregrasp_and_rejects_other_targets(degrees):
    boxes,frozen,world,height=setup();target=boxes[ORDER[0]]
    scene=SimpleNamespace(support_graph=SupportRelationGraph([boxes[n] for n in (*ORDER,*RETAINED)],[]),
        snapshot={'scene_fingerprint':'test'},policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
    original=next(f for f in simulation_contact_targets(scene,target,[-1.625,.25,.6],.1)[0] if f.face_role=='TOP')
    permission=rotation_permission(frozen);layer=selected_layer_top_permission(frozen,ORDER[0])
    option=layer_top_option(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,layer)
    option=rotated_option(option,target,permission,degrees);chosen=rotated_target(original,option,permission)
    assert np.array_equal(original.face_center,chosen.face_center) and np.array_equal(original.face_boundary,chosen.face_boundary)
    assert np.array_equal(original.outward_normal,chosen.outward_normal)
    assert np.array_equal(chosen.contact_pose[:3,:3],[[0,degrees/90,0],[degrees/90,0,0],[0,0,-1]])
    assert chosen.approach_pose[:3,3]-chosen.contact_pose[:3,3]==pytest.approx([0,0,.1])
    connector=object.__new__(LowTopReceiverConnector);connector.option=option;connector.batch_root={'height_m':height}
    connector.rotation_permission=permission;connector.layer_top_permission=layer
    connector.validate_template(chosen)
    with pytest.raises(ValueError,match='ACTUAL_TARGET_CHANGED'):validate_rotation(original,option,permission)
    with pytest.raises(ValueError,match='PERMISSION_CHANGED'):rotated_option(option,boxes['carton_l00_c04'],permission,degrees)
    with pytest.raises(ValueError,match='PERMISSION_CHANGED'):rotated_option(option,target,permission,45)
    changed=deepcopy(option);changed['face']['contact_pose'][0][3]+=.01
    with pytest.raises(ValueError,match='FRAME_CHANGED'):rotated_target(original,changed,permission)


def test_formal_current_root_reaches_production_connector(tmp_path,monkeypatch):
    pytest.importorskip('pinocchio')
    import sys
    import plan_m710_xy_top_population as entry
    output=tmp_path/'run';prior=predecessor(tmp_path)
    def dispatch(args,original,built,frozen,seed,folder,*,initial_state,target_ids):
        assert type(built.robot).__name__=='PinocchioHppFclBackend'
        assert built.robot.base_transform[:3,3]==pytest.approx([-1.625,.25,.6])
        assert seed==71070 and target_ids==[ORDER[0]] and len(initial_state.ledger)==36
        assert args.current_bottom_rotated and args.rotation_permission==rotation_permission(frozen)
        assert args.l01_hmax_permission is None and args.layer_top_only
        assert not built.collision_policy.tool_chassis_pairs(ORDER[0],('tool_rigid_11',))
        from unloading_sim.coupled_batch import batch_scene
        scene=batch_scene(original,initial_state,built.robot,original.policy.layout_validation.layout)
        target=next(b for b in initial_state.cartons if b.name==ORDER[0])
        option=layer_top_option(target,scene.receiver,next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),frozen,initial_state.height,args.layer_top_permission)
        chosen=next(t for t in simulation_contact_targets(scene,target,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m)[0] if t.geometric_face_id=='z+')
        for degrees in (90,-90):
            o=rotated_option(option,target,args.rotation_permission,degrees)
            c=object.__new__(LowTopReceiverConnector);c.option=o;c.batch_root=initial_state.root()
            c.layer_top_permission=args.layer_top_permission;c.rotation_permission=args.rotation_permission
            c.validate_template(rotated_target(chosen,o,args.rotation_permission))
        raise RuntimeError('TEST_PRODUCTION_DISPATCH_REACHED')
    monkeypatch.setattr(entry,'run_seed',dispatch)
    monkeypatch.setattr(sys,'argv',['plan','--output',str(output),'--worker-command','UNUSED_TEST_DISPATCH',
        '--worker-binary',str(Path(entry.__file__)),'--source-commit','INTEGRATION_TEST','--development',
        '--mounting-variant','x_minus_300_y_minus_100','--continue-bottom-rotated-from',str(prior),
        '--tool-connector-extension-m','0.2'])
    with pytest.raises(RuntimeError,match='TEST_PRODUCTION_DISPATCH_REACHED'):entry.main()
    check=json.loads((output/'initial-state-check.json').read_text());assert check['passed']
    p=json.loads((output/'protocol.json').read_text());assert p['scope']==SCOPE and p['scoped_target_ids']==list(ORDER)
    from verify_m710_population import expected_population_mount
    assert expected_population_mount(p)==[-1.625,.25,.6]
