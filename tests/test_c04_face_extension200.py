from copy import deepcopy
from types import SimpleNamespace
import pytest
from test_l01_selected_routes import setup
from unloading_sim.l01_hmax import layer_permission,c04_face_permission,options,validate_option


@pytest.mark.parametrize('face,face_id,legs',[
    ('top','z+',[[0.,0.,1.],[-1.,0.,0.]]),
    ('side','y-',[[0.,-1.,0.],[-1.,0.,0.]]),
    ('front','x-',[[-1.,0.,0.]])])
def test_three_separate_c04_faces_keep_hmax_and_exact_routes(face,face_id,legs):
    boxes,frozen,world,height=setup();p=layer_permission(frozen);grant=c04_face_permission(frozen,face)
    b=boxes['carton_l01_c04']
    opts=options(b,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_face=grant)
    assert len(opts)==1
    o=opts[0]
    assert o['face']['geometric_face_id']==face_id and o['segment_directions_world']==legs
    assert o['height_m']==o['pre_approach_height_m']==pytest.approx(.25)
    assert grant['tool_connector_extension_m']==.2 and grant['dispatch_order']==['carton_l01_c04']
    t=SimpleNamespace(box=b,target_id=b.name,geometric_face_id=face_id,world_y_offset_m=0,eccentric_authority=None)
    validate_option(t,o,p,height,c04_face=grant)
    with pytest.raises(ValueError):validate_option(t,o,p,height)
    bad=deepcopy(o);bad['segment_directions_world']=[[0.,1.,0.],[-1.,0.,0.]]
    with pytest.raises(ValueError):validate_option(t,bad,p,height,c04_face=grant)
    for name in ('carton_l01_c01','carton_l00_c04'):
        with pytest.raises(ValueError):options(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_face=grant)


def test_wrong_face_or_modified_frozen_identity_does_not_authorize():
    boxes,frozen,world,h=setup();p=layer_permission(frozen)
    with pytest.raises(ValueError):c04_face_permission(frozen,'y+')
    grant=c04_face_permission(frozen,'side');grant['start_box_identity']='changed'
    with pytest.raises(ValueError):options(boxes['carton_l01_c04'],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,h,p,c04_face=grant)


@pytest.mark.parametrize('face',['top','side','front'])
def test_formal_entry_reaches_c04_dispatch_with_production_model(tmp_path,monkeypatch,face):
    pytest.importorskip('pinocchio')
    from pathlib import Path
    import json,sys,tarfile
    import plan_m710_xy_top_population as entry
    from layer_top_continuation import L01_HMAX_RECORDS
    root=Path(__file__).resolve().parents[1]
    saved=root/'docs/experiments/m710_l01_hmax_20261024'
    prior=tmp_path/'prior';prior.mkdir()
    for name in L01_HMAX_RECORDS:(prior/name).write_bytes((saved/'records'/name).read_bytes())
    with tarfile.open(saved/'formal-v1-raw.tar.xz') as archive:
        for m in archive:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                path=prior/Path(m.name).relative_to('formal-v1');path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(archive.extractfile(m).read())
    output=tmp_path/'run'
    def dispatch(args,original,built,frozen,seed,folder,*,initial_state,target_ids):
        assert type(built.robot).__name__=='PinocchioHppFclBackend'
        assert target_ids==['carton_l01_c04'] and seed==71070
        assert len(initial_state.ledger)==34 and len(initial_state.cartons)==6 and not initial_state.cycles
        assert args.c04_face_permission['requested_face']==face
        assert args.tool_connector_extension_m==.2
        from unloading_sim.coupled_batch import batch_scene,CoupledCycleConnector
        from unloading_sim.low_top_receiver import LowTopReceiverConnector
        from unloading_sim.contact_target import simulation_contact_targets
        scene=batch_scene(original,initial_state,built.robot,original.policy.layout_validation.layout)
        target=next(b for b in initial_state.cartons if b.name=='carton_l01_c04')
        option=options(target,scene.receiver,next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),
            frozen,initial_state.height,args.l01_hmax_permission,c04_face=args.c04_face_permission)[0]
        chosen=next(t for t in simulation_contact_targets(scene,target,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m)[0]
            if t.geometric_face_id==option['face']['geometric_face_id'])
        cls=LowTopReceiverConnector if face=='top' else CoupledCycleConnector
        connector=object.__new__(cls);connector.option=option;connector.batch_root=initial_state.root()
        connector.l01_hmax_permission=args.l01_hmax_permission;connector.c04_face_permission=args.c04_face_permission
        connector.validate_template(chosen)
        raise RuntimeError('TEST_DISPATCH_REACHED_BEFORE_NATIVE_WORKER')
    monkeypatch.setattr(entry,'run_seed',dispatch)
    monkeypatch.setattr(sys,'argv',['plan','--output',str(output),'--worker-command','UNUSED_TEST_DISPATCH',
        '--worker-binary',str(Path(entry.__file__)),'--source-commit','INTEGRATION_TEST','--development',
        '--mounting-variant','x_minus_500_y_minus_100','--continue-l01-c04-face-from',str(prior),
        '--c04-face',face,'--tool-connector-extension-m','0.2'])
    with pytest.raises(RuntimeError,match='TEST_DISPATCH_REACHED'):entry.main()
    assert json.loads((output/'initial-state-check.json').read_text())['passed']
    assert json.loads((output/'task-001/dispatch.json').read_text())['selection']['target_id']=='carton_l01_c04'
