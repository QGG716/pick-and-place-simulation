from pathlib import Path

import numpy as np

from unloading_sim.robot_load import load_tool
from unloading_sim.robot_load.vendor import (
    FanucPayloadMassProperties,
    FanucWristLoadDiagramEvaluator,
    FanucWristEvidence,
    FanucWristReference,
    VendorEvidenceStatus,
    VendorWristLoadStatus,
    load_fanuc_wrist_evidence,
)


ROOT = Path(__file__).resolve().parents[1]


def test_public_fanuc_evidence_fails_closed_without_reference_or_diagram():
    evidence = load_fanuc_wrist_evidence(ROOT / "configs/vendor/fanuc_m20id35_wrist_load.yaml")
    result = FanucWristLoadDiagramEvaluator(evidence).evaluate(
        FanucPayloadMassProperties(23.0, None, None, "UNVERIFIED", {"case": "unit_test"})
    )
    assert result.status is VendorWristLoadStatus.NOT_EVALUATED
    assert "NOT_EVALUATED_VENDOR_REFERENCE" in result.reasons
    assert "MANUFACTURER_MOMENT_DEFINITION_NOT_VERIFIED" in result.reasons
    assert "VENDOR_INERTIA_NOT_EVALUATED" in result.reasons
    assert "NOT_EVALUATED_VENDOR_LOAD_DIAGRAM" in result.reasons


def test_engineering_tool_source_is_explicit_and_auditable():
    tool = load_tool(ROOT / "configs/tools/unloading_gripper_8kg.yaml")
    assert tool.mass_properties_source.value == "ENGINEERING_MODEL"
    assert tool.mass_properties_reference_frame == "flange"
    assert tool.inertia_evidence_status == "TOOL_INERTIA_ENGINEERING_ASSUMPTION"
    assert np.all(np.linalg.eigvalsh(tool.inertia_tensor_com_kg_m2) >= 0.0)


def test_vendor_fail_requires_verified_reference_definitions_and_boundary():
    public = load_fanuc_wrist_evidence(ROOT / "configs/vendor/fanuc_m20id35_wrist_load.yaml")
    payload_unknown = FanucPayloadMassProperties(23.0, None, None, "UNVERIFIED")
    assert (
        FanucWristLoadDiagramEvaluator(public, lambda *_: False).evaluate(payload_unknown).status
        is VendorWristLoadStatus.NOT_EVALUATED
    )

    reference = FanucWristReference(
        np.zeros(3),
        np.eye(3),
        "traceable fixture",
        "https://example.invalid/official-fixture",
        "1",
        "fixture defines origin and axes",
        VendorEvidenceStatus.VERIFIED,
    )
    verified = FanucWristEvidence(
        "test robot",
        reference,
        VendorEvidenceStatus.VERIFIED,
        VendorEvidenceStatus.VERIFIED,
        VendorEvidenceStatus.VERIFIED,
        {},
        ({"fixture": "traceable official boundary"},),
    )
    payload = FanucPayloadMassProperties(23.0, np.zeros(3), np.eye(3), "verified FANUC wrist frame")
    result = FanucWristLoadDiagramEvaluator(verified, lambda *_: False).evaluate(payload)
    assert result.status is VendorWristLoadStatus.FAIL
    assert result.reasons == ("CONFIRMED_OFFICIAL_FANUC_LOAD_DIAGRAM_FAIL",)
