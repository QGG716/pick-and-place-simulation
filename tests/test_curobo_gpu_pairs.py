"""GPU tensor primitive/permission tests; native planner checks live in verify tool."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
import importlib.util
if importlib.util.find_spec("curobo") is None:
    pytest.skip("optional cuRobo GPU environment required",allow_module_level=True)
torch=pytest.importorskip("torch")
pytest.importorskip("curobo")
pytestmark=pytest.mark.skipif(not torch.cuda.is_available(),reason="CUDA required")
from unloading_sim.curobo_collision import box_separation,sphere_box_gap,PairGeometry

ROOT=Path(__file__).resolve().parents[1]
DEVICE='cuda'


def t(value):return torch.tensor(value,dtype=torch.float64,device=DEVICE)


@pytest.mark.parametrize('gap',[.004,.006,.02])
def test_complete_box_face_clearance_has_no_cell_sphere_protrusion(gap):
    # Full 0.6 x 0.4 x 0.3 solid next to a plane-like finite box.
    c=t([[0,0,0]]);r=torch.eye(3,device=DEVICE,dtype=torch.float64)[None]
    result=box_separation(c,r,t([[.3,.2,.15]]),t([[0,0,.2+gap]]),r,t([[1,1,.05]]))
    assert result.item()==pytest.approx(gap,abs=1e-10)
    assert bool((result>=.005).item())==(gap>=.005)


def test_box_corner_and_thin_plate_are_solid_not_only_corner_samples():
    r=torch.eye(3,device=DEVICE,dtype=torch.float64)[None];c=t([[0,0,0]])
    for center in [[.3,.2,.15],[0,0,0],[.1,.07,.145]]:
        assert box_separation(c,r,t([[.3,.2,.15]]),t([center]),r,t([[.004]*3])).item()<0
    assert box_separation(c,r,t([[.3,.2,.001]]),t([[.1,.05,.001]]),r,t([[.004]*3])).item()<0


def test_sphere_obb_surface_gap_and_gradient_are_cuda_native():
    sphere=t([[[.309,0,0,.004]]]).requires_grad_(True)
    gap=sphere_box_gap(sphere,t([[0,0,0]]),torch.eye(3,device=DEVICE,dtype=torch.float64)[None],t([[.3,.2,.15]]))
    assert gap.item()==pytest.approx(.005,abs=1e-10)
    gap.sum().backward();assert sphere.grad[0,0,0].item()==pytest.approx(1.)


def test_pair_policy_revocation_rebuilds_actual_gpu_masks():
    b=json.loads((ROOT/'docs/evidence/curobo_v2_20260922/final_business/bundle.json').read_text())
    b['authority_tool']=b['tool'];b['pair_permissions']=dict(base_mount=[['base_link','chassis']],named_stack_cartons=[x['name'] for x in b['obstacles'] if x['category']=='carton'])
    names=['base_link','J4_link','J5_link','J6_link'];a=PairGeometry(b,names)
    revised=deepcopy(b);revised['request']['collision_policy']['wrist_tool_exempt_links']=[]
    revised['pair_permissions']['base_mount']=[]
    c=PairGeometry(revised,names)
    j=[x['name'] for x in b['obstacles']].index('chassis')
    assert a.robot_world[0,j].item()<0 and c.robot_world[0,j].item()==pytest.approx(.005)
    assert a.robot_world[2,j].item()==pytest.approx(.005)
    assert a.robot_moving[2,0].item()<0 and c.robot_moving[2,0].item()==pytest.approx(.005)
    assert a.robot_moving[2,-1].item()==pytest.approx(.005) # payload is not owned tool
    assert a.tool_payload[0].item()==pytest.approx(.005) # rigid plate remains checked
    assert len(a.moving)==203


def test_unknown_ownership_does_not_gain_permission():
    b=json.loads((ROOT/'docs/evidence/curobo_v2_20260922/final_business/bundle.json').read_text())
    b['authority_tool']=b['tool'];b['pair_permissions']=dict(base_mount=[['J4_link','chassis']],named_stack_cartons=[])
    with pytest.raises(ValueError,match='installation permission'):PairGeometry(b,['J4_link'])
