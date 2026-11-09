"""Verify explicit coordinate chains against the existing official-model FK."""
from pathlib import Path
import json
import numpy as np
import pytest

from unloading_sim.workbench.coordinates import CoordinateModel, Extrinsics, ToolFrame

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/urdf/ECO65-B.urdf'
pytestmark = pytest.mark.real_eco65_asset


def model():
    if not URDF.exists(): pytest.skip('Official ECO65-B assets not installed')
    tools = json.loads((ROOT/'configs/workbench/tool_frames.json').read_text())
    world = np.diag([-1.,-1.,1.,1.]); world[:3,3] = [-.44,0,.24]
    return CoordinateModel(URDF, '08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b',
                           world, ToolFrame.from_dict(tools[1]))


def test_dynamic_link1_camera_and_explicit_world_base():
    m = model(); e = Extrinsics.load(ROOT/'configs/workbench/training_extrinsics.example.json')
    zero = m.camera(np.zeros(6),e)
    np.testing.assert_allclose(zero['T_base_camera'][:3,3], [-.021553275982846603,.14914158671415392,.4534768314852272])
    np.testing.assert_allclose(zero['T_world_camera'][:3,3], [-.4184467240171534,-.14914158671415392,.6934768314852272])
    rotated = m.camera([np.pi/2,0,0,0,0,0],e)
    np.testing.assert_allclose(rotated['T_base_camera'][:3,3], [-.14914158671415392,-.021553275982846603,.4534768314852272])
    # link_1 mounting moves with J1 only, never with the tool flange.
    np.testing.assert_allclose(m.camera([0,.2,-.3,.4,-.5,.6],e)['T_base_camera'], zero['T_base_camera'])


def test_probe_tool_separation_and_no_model_repair():
    m = model(); tool = m.tcp(np.zeros(6))
    data = json.loads((ROOT/'configs/workbench/tool_frames.json').read_text())
    m.tool = ToolFrame.from_dict(data[0]); probe = m.tcp(np.zeros(6))
    assert np.linalg.norm(tool[:3,3]-probe[:3,3]) == pytest.approx(.07)
    with pytest.raises(ValueError,match='limits'): m.frames([4,0,0,0,0,0])
    with pytest.raises(ValueError,match='hash mismatch'):
        CoordinateModel(URDF,'wrong',np.eye(4),m.tool)


def test_display_primitive_conversion_keeps_dimensions_pose_and_source():
    from unloading_sim.workbench.scene import display_scene
    original = {'scene_fingerprint':'original-unchanged','native_world_objects':[
        {'id':'belt','size':[.15,.1,.1],'pose':np.eye(4).tolist()}]}
    converted = display_scene(original)
    vertices = np.array(converted['native_world_objects'][0]['convex_mesh']['vertices'])
    np.testing.assert_allclose(vertices.min(0),[-.075,-.05,-.05])
    np.testing.assert_allclose(vertices.max(0),[.075,.05,.05])
    assert len(vertices) == 8 and 'convex_mesh' not in original['native_world_objects'][0]
    assert converted['native_world_objects'][0]['pose'] == original['native_world_objects'][0]['pose']
    assert converted['workbench_display_derivation']['purpose'] == 'DISPLAY_ONLY_NOT_PLANNING'
