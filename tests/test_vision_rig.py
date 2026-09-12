from __future__ import annotations

from math import pi

import pytest

from unloading_perception.vision_rig import (
    audit_j1_sweep,
    evaluate_vision_rig_pose,
    load_vision_rig_spec,
    wrap_to_pi,
)


T_W_BASE = (
    (1.0, 0.0, 0.0, -1.325),
    (0.0, 1.0, 0.0, 0.350),
    (0.0, 0.0, 1.0, 0.600),
    (0.0, 0.0, 0.0, 1.0),
)


@pytest.fixture(scope="module")
def spec():
    return load_vision_rig_spec("configs/isaac/perception_sensing_pose.yaml")


def test_nominal_mast_is_exactly_400_mm_to_world_left(spec):
    pose = evaluate_vision_rig_pose(T_W_BASE, spec.nominal_q1_rad, spec)
    assert pose.j1_center_world_m[:2] == pytest.approx((-1.325, 0.350), abs=1e-12)
    assert pose.mast_center_world_m[:2] == pytest.approx((-1.325, 0.750), abs=1e-12)
    assert pose.mast_center_world_m[1] > pose.j1_center_world_m[1]


def test_nominal_camera_faces_world_plus_x_and_arm_plane_is_yz(spec):
    pose = evaluate_vision_rig_pose(T_W_BASE, spec.nominal_q1_rad, spec)
    optical_forward = tuple(pose.T_W_camera_optical[row][2] for row in range(3))
    optical_down = tuple(pose.T_W_camera_optical[row][1] for row in range(3))
    assert optical_forward == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)
    assert optical_down == pytest.approx((0.0, 0.0, -1.0), abs=1e-12)
    assert pose.j1_heading_world_rad == pytest.approx(-pi / 2.0)
    assert wrap_to_pi(pose.camera_heading_world_rad - pose.j1_heading_world_rad) == pytest.approx(pi / 2.0)


def test_j1_sweep_preserves_radius_phase_vertical_and_level_camera(spec):
    sweep = audit_j1_sweep(T_W_BASE, tuple(value * pi / 180.0 for value in (-90, -45, 0, 45, 90)), spec)
    assert sweep["status"] == "PASS"
    assert all(item["mast_radius_m"] == pytest.approx(0.4) for item in sweep["samples"])
    assert all(item["camera_j1_yaw_offset_rad"] == pytest.approx(pi / 2.0) for item in sweep["samples"])
    assert all(item["camera_roll_pitch_level"] for item in sweep["samples"])


def test_user_defined_mast_and_module_heights_are_not_optimized(spec):
    pose = evaluate_vision_rig_pose(T_W_BASE, spec.nominal_q1_rad, spec)
    flange_z = pose.mast_center_world_m[2]
    assert spec.mast_height_m == 1.5
    assert spec.module_height_m == 1.3
    assert pose.T_W_mast_top[2][3] - flange_z == pytest.approx(1.5)
    assert pose.camera_center_world_m[2] - flange_z == pytest.approx(1.3)
    assert pose.T_W_mast_top[2][3] - pose.camera_center_world_m[2] == pytest.approx(0.2)


def test_exact_user_fovs_require_explicit_resampling(spec):
    assert (spec.rgb.width_px, spec.rgb.height_px) == (2592, 1944)
    assert spec.rgb.frame_rate_hz == 30.0
    assert spec.rgb.hfov_rad == pytest.approx(pi / 2.0)
    assert spec.rgb.vfov_rad == pytest.approx(65.0 * pi / 180.0)
    assert spec.rgb.square_pixel_compatible is False
    assert spec.rgb.intrinsics_mode == "EXACT_USER_SPEC_RESAMPLED"
    assert spec.rgb.K[0] != pytest.approx(spec.rgb.K[4])
    assert spec.depth.K == pytest.approx(spec.rgb.K)


def test_integrated_rgbd_and_symmetric_fill_light_contract(spec):
    assert spec.depth_semantics == "optical_z_m"
    assert spec.calibration_identity == "m710id70_module0_ideal_registered_rgbd_v1"
    assert spec.light_offsets_module_m == ((0.0, 0.12, 0.0), (0.0, -0.12, 0.0))
    assert spec.independent_mast_yaw is False
    assert spec.geometry_qualification == "VISION_RIG_GEOMETRY_NOT_EXECUTION_QUALIFIED"


def test_old_static_or_right_side_geometry_fails_current_acceptance(spec):
    with pytest.raises(ValueError, match=r"world \+Y left"):
        type(spec)(**{**spec.__dict__, "mast_offset_j1_xy_m": (0.0, -0.4)})
    assert not hasattr(spec, "T_W_C")
