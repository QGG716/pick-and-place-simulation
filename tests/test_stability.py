import numpy as np

from unloading_sim.robot import URDFRobot6
from unloading_sim.stability import (
    PointMass,
    SupportFootprint,
    URDFInertialModel,
    evaluate_braking_stability,
    robot_amr_stability,
    worst_case_braking_audit,
)


def test_braking_moves_zmp_opposite_vehicle_acceleration_and_detects_tip():
    masses = [PointMass("cell", 1000.0, np.array([0.0, 0.0, 1.0]))]
    footprint = SupportFootprint(np.zeros(2), np.array([2.0, 1.0]))
    static = evaluate_braking_stability(masses, footprint, [0.0, 0.0])
    braking = evaluate_braking_stability(masses, footprint, [6.0, 0.0])
    tipped = evaluate_braking_stability(masses, footprint, [12.0, 0.0])

    assert np.allclose(static.zmp_m, [0.0, 0.0])
    assert braking.zmp_m[0] < 0.0
    assert braking.minimum_margin_m < static.minimum_margin_m
    assert not tipped.stable


def test_fanuc_urdf_masses_payload_and_platform_form_combined_com():
    robot = URDFRobot6.fanuc_m20id35()
    inertials = URDFInertialModel.from_urdf(robot.urdf_path)
    q = np.array([-0.4, -0.8, -0.7, 0.2, -1.0, 0.1])
    result = robot_amr_stability(
        robot,
        q,
        inertials,
        platform_mass_kg=850.0,
        platform_com_m=[-0.2, 0.0, 0.25],
        footprint=SupportFootprint([-0.2, 0.0], [1.8, 1.5]),
        acceleration_m_s2=[-2.5, 0.0],
        payload_mass_kg=35.0,
        payload_com_m=robot.fk(q)[:3, 3],
    )
    assert result.total_mass_kg > 1100.0
    assert np.all(np.isfinite(result.center_of_mass_m))


def test_worst_case_braking_audit_selects_smallest_margin():
    audit = worst_case_braking_audit(
        [PointMass("cell", 500.0, [0.2, 0.0, 1.2])],
        SupportFootprint([0.0, 0.0], [2.0, 1.5]),
        {"forward": [3.0, 0.0], "reverse": [-5.0, 0.0], "lateral": [0.0, 2.0]},
    )
    assert audit["worst_case"] == "reverse"
    assert len(audit["cases"]) == 3
