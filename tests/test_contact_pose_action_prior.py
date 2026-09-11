from pathlib import Path

from unloading_sim.layout_single_carton import (
    _exposed_faces, _scheduled_contact_poses, build_verified_motion_input, load_layout_motion_policy,
)


def test_existing_offsets_with_receiving_clearance_are_prioritized_without_removing_faces():
    root = Path(__file__).resolve().parents[1]
    policy = load_layout_motion_policy(root / "configs/validation/m710id70_layout_v1_single_carton.yaml")
    scene = build_verified_motion_input(policy)
    target = next(box for box in scene.cartons if box.name == scene.removable_cartons[0])
    poses = list(_scheduled_contact_poses(scene, target, _exposed_faces(scene, target.name)))
    face, (roll, physical, evidence) = poses[0]
    assert face == "front"
    assert evidence["rigid_tool_bottom_above_box_bottom_m"] >= 0.02
    assert physical[2, 3] > target.center[2]
    assert any(face == "top" for face, _ in poses[:4])
    assert any(item[2]["rigid_tool_bottom_above_box_bottom_m"] < 0.02
               for face, item in poses if face == "front")
