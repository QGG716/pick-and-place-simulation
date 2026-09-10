from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from unloading_sim.m710_dynamics import ACTIVE_JOINT_NAMES, load_m710id70_dynamics
from unloading_sim.m710_official_dynamics import OFFICIAL_COMMIT, OFFICIAL_STATUS
from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
from unloading_sim.workcell_layout import load_workcell_layout


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml"
LAYOUT = ROOT / "configs/workcells/m710id70_unloading_layout_v1.yaml"


def test_official_dynamics_preserves_exact_link_inertials_and_finite_drives():
    config = load_m710id70_dynamics(CONFIG)

    assert config.qualification_status == OFFICIAL_STATUS
    assert config.machine_qualified is False
    assert config.manufacturer_evidence["commit"] == OFFICIAL_COMMIT
    assert config.robot_mass_kg == pytest.approx(580.347, abs=1e-12)
    assert [config.robot_links[name].mass_kg for name in config.robot_links] == pytest.approx(
        [115.0, 189.0, 137.0, 93.9, 34.8, 10.2, 0.447]
    )
    assert config.robot_links["J2_link"].inertia_tensor_com_kg_m2[1][2] == -1.14
    assert [config.joint_drives[name].effort_limit_nm for name in ACTIVE_JOINT_NAMES] == [
        8000.0, 10000.0, 5000.0, 2000.0, 1000.0, 900.0
    ]
    assert config.robot_with_fixed_tool_mass_kg == pytest.approx(600.347)
    assert config.total_configured_mass_kg == pytest.approx(2300.347)


def test_ideal_independent_cups_is_explicit_and_does_not_add_a_capacity_gate():
    vacuum = load_m710id70_dynamics(CONFIG).vacuum_attachment
    assert vacuum.suction_mode == "ideal_independent_cups"
    assert vacuum.physical_cup_count == 72
    assert vacuum.require_nonempty_geometric_contact
    assert vacuum.require_validated_contact_endpoint
    assert vacuum.require_actual_fk_contact_recheck
    assert vacuum.require_target_identity_match
    assert not vacuum.teleport_payload_on_attach
    assert not vacuum.enforce_vacuum_force_capacity
    assert not vacuum.enforce_vacuum_break_force
    assert not vacuum.enforce_vacuum_break_torque


def test_official_mesh_backend_and_lightweight_chain_agree_at_nonzero_poses():
    layout = load_workcell_layout(LAYOUT)
    lightweight = layout.robot()
    model = load_m710id70_dynamics(CONFIG)
    robot_cfg = model.data["sources"]
    urdf = (CONFIG.parent / str(robot_cfg["robot_urdf"])).resolve()
    srdf = ROOT / "assets/robots/fanuc_m710id_70/official/m710id_70_official.srdf"
    package_root = ROOT / "assets/robots/fanuc_m710id_70/official"
    exact = PinocchioHppFclBackend(
        urdf,
        tip_frame="tool0",
        package_dirs=[package_root],
        srdf_path=srdf,
        base_transform=layout.robot_base_transform(),
        tool_length=0.250,
    )
    for q in (
        np.zeros(6),
        np.array([0.31, -0.44, 0.73, -0.28, 0.39, -0.62]),
        np.array([-0.52, 0.21, -0.37, 0.48, -0.33, 0.71]),
    ):
        assert np.allclose(exact.fk(q), lightweight.fk(q), atol=2e-10, rtol=0.0)
