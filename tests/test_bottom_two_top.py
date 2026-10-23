"""Bottom-two top suction uses the checked 30-box terminal and frozen geometry."""
from pathlib import Path
import tarfile
import numpy as np
import pytest
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import BatchState
from unloading_sim.low_top_receiver import selected_layer_top_permission,layer_top_option
from unloading_sim.workcell_layout import load_workcell_layout

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def layout():
    return load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile.yaml')

def test_bottom_two_only_at_fixed_250(layout):
    frozen=FrozenCartonPopulation.capture(layout.cartons())
    permission=selected_layer_top_permission(frozen,'carton_l01_c02')
    assert permission['ceiling_world_m']==pytest.approx(.6)
    assert len(permission['allowed_target_ids'])==10
    fixed={b.name:b for b in layout.fixed_components()}
    for box in layout.cartons():
        option=layer_top_option(box,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.25,permission)
        if box.center[2]>.6:
            assert option is None
            continue
        assert option['template']=='TOP_FIXED_CURRENT_RECEIVER'
        assert option['height_m']==option['pre_approach_height_m']==.25
        assert option['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
        assert option['release_bottom_world_m']==pytest.approx(.35 if box.center[2]>.3 else .30)
        assert option['face']['working_axis_into_box']==[0.,0.,-1.]

def test_actual_30_box_terminal_clears_native_parent(layout,tmp_path):
    from layer_top_continuation import restore_lowprofile_top_terminal,LOWPROFILE_RECORDS
    source=ROOT/'docs/experiments/m710_lowprofile250_20261023'
    for name in LOWPROFILE_RECORDS:(tmp_path/name).write_bytes((source/'records'/name).read_bytes())
    with tarfile.open(source/'formal-v1-raw.tar.xz') as archive:
        for m in archive:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                dest=tmp_path/Path(m.name).relative_to('formal-v1');dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_bytes(archive.extractfile(m).read())
    frozen=FrozenCartonPopulation.capture(layout.cartons())
    state=BatchState(frozen,tuple(layout.cartons()),np.zeros(6),2.05)
    receipt=restore_lowprofile_top_terminal(tmp_path,state)
    assert len(state.cartons)==10 and len(state.ledger)==30 and not state.cycles
    assert state.height==.25 and state.root()['predecessor_final_state_receipt'] is None
    np.testing.assert_allclose(state.q,[-1.0556381484185202,.891050376919693,-.4469515308987374,
        -2.074314858511779,-1.4568922990087736,-.20346693997183032],atol=1e-14,rtol=0)
    assert receipt['archived_root']['predecessor_final_state_receipt'] is not None
    assert receipt['first_target_id']=='carton_l01_c02' and not receipt['tree_retained']
    assert {b.name for b in state.cartons}==set(selected_layer_top_permission(frozen,'carton_l01_c02')['allowed_target_ids'])
    (tmp_path/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='RECORD_CHANGED'):restore_lowprofile_top_terminal(tmp_path,state)
