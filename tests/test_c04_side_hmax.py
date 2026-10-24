from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import tarfile
import numpy as np
import pytest
from test_l01_selected_routes import setup
from unloading_sim.l01_hmax import layer_permission,c04_side_permission,options,validate_option


def test_only_original_c04_negative_y_face_at_frozen_hmax():
    boxes,frozen,world,height=setup();p=layer_permission(frozen);side=c04_side_permission(frozen)
    target=boxes['carton_l01_c04']
    opts=options(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_side=side)
    assert len(opts)==1
    o=opts[0]
    assert o['original_role']=='LEFT_END' and o['face']['geometric_face_id']=='y-'
    assert o['face']['outward_normal']==[0.,-1.,0.]
    assert o['face']['working_axis_into_box']==[0.,1.,0.]
    assert o['segment_directions_world']==[[0.,-1.,0.],[-1.,0.,0.]]
    assert o['height_m']==o['pre_approach_height_m']==pytest.approx(.25)
    t=SimpleNamespace(box=target,target_id=target.name,geometric_face_id='y-',world_y_offset_m=0,eccentric_authority=None)
    validate_option(t,o,p,height,c04_side=side)
    with pytest.raises(ValueError):validate_option(t,o,p,height)
    for legs in ([[0.,1.,0.],[-1.,0.,0.]],[[-1.,0.,0.]]):
        bad=deepcopy(o);bad['segment_directions_world']=legs
        with pytest.raises(ValueError):validate_option(t,bad,p,height,c04_side=side)
    for name in ('carton_l01_c00','carton_l00_c04'):
        with pytest.raises(ValueError):options(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_side=side)


def test_restore_actual_34_terminal_discards_failed_c04_and_native_parents(tmp_path):
    from layer_top_continuation import restore_l01_hmax_terminal,L01_HMAX_RECORDS
    from unloading_sim.coupled_batch import BatchState
    source=Path(__file__).resolve().parents[1]/'docs/experiments/m710_l01_hmax_20261024'
    for name in L01_HMAX_RECORDS:(tmp_path/name).write_bytes((source/'records'/name).read_bytes())
    with tarfile.open(source/'formal-v1-raw.tar.xz') as t:
        for m in t:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                path=tmp_path/Path(m.name).relative_to('formal-v1');path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(t.extractfile(m).read())
    boxes,frozen,_,_=setup();state=BatchState(frozen,tuple(boxes.values()),np.zeros(6),2.05)
    receipt=restore_l01_hmax_terminal(tmp_path,state)
    assert len(state.cartons)==6 and len(state.ledger)==34 and not state.cycles
    assert state.root()['predecessor_final_state_receipt'] is None
    assert receipt['first_target_id']=='carton_l01_c04' and receipt['tree_retained'] is False
    assert np.array_equal(state.q,[-1.1278317036018644,1.2046092883498862,-.31738242737668854,-2.0133002553942263,-1.549885462084515,-.04410126159327743])
    assert {b.name for b in state.cartons}=={'carton_l01_c04',*(f'carton_l00_c{i:02d}' for i in range(5))}
    (tmp_path/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='RECORD_CHANGED'):restore_l01_hmax_terminal(tmp_path,state)
