"""Fail-closed FANUC wrist-load evidence chain.

This module intentionally does not import the URDF/FK engineering model.  A
joint-axis projection is not silently treated as a FANUC load-diagram value.
Only a documented manufacturer reference frame and diagram definition may
produce a vendor PASS or FAIL.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import yaml


class VendorEvidenceStatus(str, Enum):
    VERIFIED = "VERIFIED"
    NOT_VERIFIED = "NOT_VERIFIED"


class VendorWristLoadStatus(str, Enum):
    PASS = "VENDOR_LOAD_PASS"
    FAIL = "VENDOR_LOAD_FAIL"
    NOT_EVALUATED = "VENDOR_LOAD_NOT_EVALUATED"


def _optional_vector3(value: Sequence[float] | None, name: str) -> np.ndarray | None:
    if value is None:
        return None
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite SI values")
    return result


@dataclass(frozen=True)
class FanucWristReference:
    """Traceable manufacturer coordinate/reference definition, if known."""

    reference_origin_xyz_m: np.ndarray | None
    reference_axes: np.ndarray | None
    source_document: str
    source_url: str
    source_page: str
    source_definition: str
    evidence_status: VendorEvidenceStatus

    def __post_init__(self) -> None:
        origin = _optional_vector3(self.reference_origin_xyz_m, "reference_origin_xyz_m")
        object.__setattr__(self, "reference_origin_xyz_m", origin)
        axes = self.reference_axes
        if axes is not None:
            axes = np.asarray(axes, dtype=float)
            if axes.shape != (3, 3) or not np.all(np.isfinite(axes)):
                raise ValueError("reference_axes must be a finite 3x3 matrix")
            if not np.allclose(axes.T @ axes, np.eye(3), atol=1e-9):
                raise ValueError("reference_axes must be orthonormal")
            object.__setattr__(self, "reference_axes", axes)
        object.__setattr__(self, "evidence_status", VendorEvidenceStatus(self.evidence_status))
        if self.evidence_status is VendorEvidenceStatus.VERIFIED and (origin is None or axes is None):
            raise ValueError("verified FANUC reference requires both origin and axes")


@dataclass(frozen=True)
class FanucPayloadMassProperties:
    total_mass_kg: float
    com_in_fanuc_wrist_frame_xyz_m: np.ndarray | None
    inertia_tensor_at_com_kg_m2: np.ndarray | None
    reference_frame: str
    source: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        mass = float(self.total_mass_kg)
        if not np.isfinite(mass) or mass <= 0.0:
            raise ValueError("total_mass_kg must be finite and positive")
        object.__setattr__(self, "total_mass_kg", mass)
        object.__setattr__(
            self,
            "com_in_fanuc_wrist_frame_xyz_m",
            _optional_vector3(
                self.com_in_fanuc_wrist_frame_xyz_m,
                "com_in_fanuc_wrist_frame_xyz_m",
            ),
        )
        inertia = self.inertia_tensor_at_com_kg_m2
        if inertia is not None:
            inertia = np.asarray(inertia, dtype=float)
            if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
                raise ValueError("inertia_tensor_at_com_kg_m2 must be a finite 3x3 tensor")
            if not np.allclose(inertia, inertia.T, atol=1e-10):
                raise ValueError("inertia_tensor_at_com_kg_m2 must be symmetric")
            if np.min(np.linalg.eigvalsh(inertia)) < -1e-10:
                raise ValueError("inertia_tensor_at_com_kg_m2 must be positive semidefinite")
            object.__setattr__(self, "inertia_tensor_at_com_kg_m2", inertia)
        if not self.reference_frame.strip():
            raise ValueError("reference_frame must be non-empty")


@dataclass(frozen=True)
class FanucWristLoadResult:
    status: VendorWristLoadStatus
    reasons: tuple[str, ...]
    criteria: Mapping[str, str]
    evidence: Mapping[str, Any]

    def to_dict(self) -> dict[str, Any]:
        def convert(value: Any) -> Any:
            if isinstance(value, Enum):
                return value.value
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, Mapping):
                return {str(k): convert(v) for k, v in value.items()}
            if isinstance(value, (tuple, list)):
                return [convert(v) for v in value]
            return value

        return convert(asdict(self))


@dataclass(frozen=True)
class FanucWristEvidence:
    robot_model: str
    reference: FanucWristReference
    moment_definition_status: VendorEvidenceStatus
    inertia_definition_status: VendorEvidenceStatus
    load_diagram_status: VendorEvidenceStatus
    public_numeric_values: Mapping[str, Any]
    sources: tuple[Mapping[str, Any], ...]


class FanucWristLoadDiagramEvaluator:
    """Evaluate only when the complete manufacturer evidence chain is closed."""

    def __init__(
        self,
        evidence: FanucWristEvidence,
        official_boundary_evaluator: Callable[
            [FanucPayloadMassProperties, FanucWristEvidence], bool
        ]
        | None = None,
    ):
        self.evidence = evidence
        self.official_boundary_evaluator = official_boundary_evaluator

    def evaluate(self, payload: FanucPayloadMassProperties) -> FanucWristLoadResult:
        evidence = self.evidence
        reasons: list[str] = []
        if evidence.reference.evidence_status is not VendorEvidenceStatus.VERIFIED:
            reasons.append("NOT_EVALUATED_VENDOR_REFERENCE")
        if evidence.moment_definition_status is not VendorEvidenceStatus.VERIFIED:
            reasons.append("MANUFACTURER_MOMENT_DEFINITION_NOT_VERIFIED")
        if evidence.inertia_definition_status is not VendorEvidenceStatus.VERIFIED:
            reasons.append("VENDOR_INERTIA_NOT_EVALUATED")
        if evidence.load_diagram_status is not VendorEvidenceStatus.VERIFIED:
            reasons.append("NOT_EVALUATED_VENDOR_LOAD_DIAGRAM")
        if payload.com_in_fanuc_wrist_frame_xyz_m is None:
            reasons.append("PAYLOAD_COM_NOT_EXPRESSED_IN_VERIFIED_FANUC_WRIST_FRAME")
        if payload.inertia_tensor_at_com_kg_m2 is None:
            reasons.append("PAYLOAD_INERTIA_NOT_EXPRESSED_IN_VERIFIED_FANUC_WRIST_FRAME")

        criteria = {
            "vendor_reference": (
                "VERIFIED"
                if evidence.reference.evidence_status is VendorEvidenceStatus.VERIFIED
                else "NOT_EVALUATED_VENDOR_REFERENCE"
            ),
            "manufacturer_moment_definition": (
                "VERIFIED"
                if evidence.moment_definition_status is VendorEvidenceStatus.VERIFIED
                else "MANUFACTURER_MOMENT_DEFINITION_NOT_VERIFIED"
            ),
            "manufacturer_inertia_definition": (
                "VERIFIED"
                if evidence.inertia_definition_status is VendorEvidenceStatus.VERIFIED
                else "VENDOR_INERTIA_NOT_EVALUATED"
            ),
            "vendor_load_diagram": (
                "VERIFIED"
                if evidence.load_diagram_status is VendorEvidenceStatus.VERIFIED
                else "NOT_EVALUATED_VENDOR_LOAD_DIAGRAM"
            ),
        }
        if not reasons and self.official_boundary_evaluator is not None:
            passed = bool(self.official_boundary_evaluator(payload, evidence))
            return FanucWristLoadResult(
                VendorWristLoadStatus.PASS if passed else VendorWristLoadStatus.FAIL,
                ("CONFIRMED_OFFICIAL_FANUC_LOAD_DIAGRAM_PASS" if passed else "CONFIRMED_OFFICIAL_FANUC_LOAD_DIAGRAM_FAIL",),
                criteria,
                {
                    "robot_model": evidence.robot_model,
                    "reference": asdict(evidence.reference),
                    "public_numeric_values": dict(evidence.public_numeric_values),
                    "sources": list(evidence.sources),
                    "payload_reference_frame": payload.reference_frame,
                    "payload_source": dict(payload.source),
                    "boundary_evaluator": "SUPPLIED_TRACEABLE_OFFICIAL_BOUNDARY",
                },
            )
        if not reasons:
            reasons.append("OFFICIAL_DIAGRAM_BOUNDARY_EVALUATOR_NOT_IMPLEMENTED")
        return FanucWristLoadResult(
            VendorWristLoadStatus.NOT_EVALUATED,
            tuple(dict.fromkeys(reasons)),
            criteria,
            {
                "robot_model": evidence.robot_model,
                "reference": asdict(evidence.reference),
                "public_numeric_values": dict(evidence.public_numeric_values),
                "sources": list(evidence.sources),
                "payload_reference_frame": payload.reference_frame,
                "payload_source": dict(payload.source),
            },
        )


def load_fanuc_wrist_evidence(path: str | Path) -> FanucWristEvidence:
    resolved = Path(path).resolve()
    data = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != "fanuc_wrist_evidence_v1":
        raise ValueError("unsupported FANUC wrist evidence schema")
    reference = data["reference"]
    return FanucWristEvidence(
        robot_model=str(data["robot_model"]),
        reference=FanucWristReference(
            reference_origin_xyz_m=reference.get("origin_xyz_m"),
            reference_axes=reference.get("axes"),
            source_document=str(reference["source_document"]),
            source_url=str(reference["source_url"]),
            source_page=str(reference["source_page"]),
            source_definition=str(reference["source_definition"]),
            evidence_status=VendorEvidenceStatus(reference["evidence_status"]),
        ),
        moment_definition_status=VendorEvidenceStatus(data["moment_definition_status"]),
        inertia_definition_status=VendorEvidenceStatus(data["inertia_definition_status"]),
        load_diagram_status=VendorEvidenceStatus(data["load_diagram_status"]),
        public_numeric_values=dict(data["public_numeric_values"]),
        sources=tuple(dict(v) for v in data["sources"]),
    )
