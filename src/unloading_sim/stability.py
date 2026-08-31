"""AMR/robot/payload center-of-mass and braking tip-margin model."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from .robot import URDFRobot


@dataclass(frozen=True)
class PointMass:
    name: str
    mass_kg: float
    position_m: np.ndarray

    def __post_init__(self) -> None:
        position = np.asarray(self.position_m, dtype=float)
        if not np.isfinite(self.mass_kg) or self.mass_kg <= 0.0:
            raise ValueError("point mass must be finite and positive")
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("point mass position must be finite shape (3,)")
        object.__setattr__(self, "position_m", position.copy())


@dataclass(frozen=True)
class SupportFootprint:
    center_m: np.ndarray
    size_m: np.ndarray
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        center = np.asarray(self.center_m, dtype=float)
        size = np.asarray(self.size_m, dtype=float)
        if center.shape not in {(2,), (3,)} or not np.all(np.isfinite(center)):
            raise ValueError("footprint center must be finite shape (2,) or (3,)")
        if size.shape != (2,) or np.any(size <= 0.0) or not np.all(np.isfinite(size)):
            raise ValueError("footprint size must be finite positive shape (2,)")
        object.__setattr__(self, "center_m", center[:2].copy())
        object.__setattr__(self, "size_m", size.copy())


@dataclass(frozen=True)
class StabilityResult:
    total_mass_kg: float
    center_of_mass_m: np.ndarray
    zmp_m: np.ndarray
    edge_margins_m: np.ndarray
    minimum_margin_m: float
    stable: bool
    acceleration_m_s2: np.ndarray

    def audit(self) -> dict:
        return {
            "model": "rigid_point_mass_braking_zmp_v1",
            "total_mass_kg": self.total_mass_kg,
            "center_of_mass_m": self.center_of_mass_m.tolist(),
            "zmp_m": self.zmp_m.tolist(),
            "edge_margins_m": self.edge_margins_m.tolist(),
            "minimum_margin_m": self.minimum_margin_m,
            "stable": self.stable,
            "acceleration_m_s2": self.acceleration_m_s2.tolist(),
        }


@dataclass(frozen=True)
class LinkInertial:
    link: str
    mass_kg: float
    local_com_m: np.ndarray


class URDFInertialModel:
    def __init__(self, inertials: Sequence[LinkInertial], source: str) -> None:
        if not inertials:
            raise ValueError("URDF contains no usable inertial masses")
        self.inertials = list(inertials)
        self.source = source

    @classmethod
    def from_urdf(cls, path: str | Path) -> "URDFInertialModel":
        path = Path(path)
        root = ET.parse(path).getroot()
        inertials = []
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass_element = inertial.find("mass")
            if mass_element is None:
                continue
            mass = float(mass_element.attrib["value"])
            origin = inertial.find("origin")
            xyz = [0.0, 0.0, 0.0] if origin is None else [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            inertials.append(LinkInertial(str(link.attrib["name"]), mass, np.asarray(xyz, dtype=float)))
        return cls(inertials, str(path))

    def world_point_masses(self, robot: URDFRobot, q: np.ndarray) -> list[PointMass]:
        link_frames = robot.named_link_frames(q)
        masses = []
        for inertial in self.inertials:
            if inertial.link not in link_frames:
                continue
            frame = link_frames[inertial.link]
            position = frame[:3, :3] @ inertial.local_com_m + frame[:3, 3]
            masses.append(PointMass(f"robot:{inertial.link}", inertial.mass_kg, position))
        if not masses:
            raise ValueError("no URDF inertial links are present on the configured robot chain")
        return masses


def combined_center_of_mass(point_masses: Iterable[PointMass]) -> tuple[float, np.ndarray]:
    masses = list(point_masses)
    if not masses:
        raise ValueError("at least one point mass is required")
    total = float(sum(item.mass_kg for item in masses))
    center = sum((item.mass_kg * item.position_m for item in masses), start=np.zeros(3)) / total
    return total, center


def evaluate_braking_stability(
    point_masses: Iterable[PointMass],
    footprint: SupportFootprint,
    acceleration_m_s2: Sequence[float],
    *,
    gravity_m_s2: float = 9.81,
    required_margin_m: float = 0.0,
) -> StabilityResult:
    acceleration = np.asarray(acceleration_m_s2, dtype=float)
    if acceleration.shape not in {(2,), (3,)} or not np.all(np.isfinite(acceleration)):
        raise ValueError("acceleration must be finite shape (2,) or (3,)")
    if gravity_m_s2 <= 0.0 or required_margin_m < 0.0:
        raise ValueError("gravity must be positive and required margin non-negative")
    total_mass, center = combined_center_of_mass(point_masses)
    # Vehicle acceleration creates an opposite inertial force at the COM.
    zmp = center[:2] - center[2] * acceleration[:2] / gravity_m_s2
    cosine, sine = np.cos(footprint.yaw_rad), np.sin(footprint.yaw_rad)
    world_from_local = np.array([[cosine, -sine], [sine, cosine]])
    local_zmp = world_from_local.T @ (zmp - footprint.center_m)
    margins = 0.5 * footprint.size_m - np.abs(local_zmp)
    minimum = float(np.min(margins))
    return StabilityResult(
        total_mass,
        center,
        zmp,
        margins,
        minimum,
        bool(minimum >= required_margin_m),
        acceleration.copy(),
    )


def robot_amr_stability(
    robot: URDFRobot,
    q: np.ndarray,
    inertial_model: URDFInertialModel,
    *,
    platform_mass_kg: float,
    platform_com_m: Sequence[float],
    footprint: SupportFootprint,
    acceleration_m_s2: Sequence[float],
    payload_mass_kg: float = 0.0,
    payload_com_m: Sequence[float] | None = None,
    additional_masses: Sequence[PointMass] = (),
    required_margin_m: float = 0.0,
) -> StabilityResult:
    masses = [PointMass("platform", platform_mass_kg, np.asarray(platform_com_m, dtype=float))]
    masses.extend(inertial_model.world_point_masses(robot, q))
    masses.extend(additional_masses)
    if payload_mass_kg > 0.0:
        if payload_com_m is None:
            raise ValueError("payload_com_m is required when payload_mass_kg is positive")
        masses.append(PointMass("payload", payload_mass_kg, np.asarray(payload_com_m, dtype=float)))
    return evaluate_braking_stability(
        masses,
        footprint,
        acceleration_m_s2,
        required_margin_m=required_margin_m,
    )


def worst_case_braking_audit(
    point_masses: Iterable[PointMass],
    footprint: SupportFootprint,
    accelerations_m_s2: Mapping[str, Sequence[float]],
    *,
    required_margin_m: float = 0.0,
) -> dict:
    results = {
        name: evaluate_braking_stability(
            point_masses, footprint, acceleration, required_margin_m=required_margin_m
        )
        for name, acceleration in accelerations_m_s2.items()
    }
    if not results:
        raise ValueError("at least one braking case is required")
    worst_name = min(results, key=lambda name: results[name].minimum_margin_m)
    return {
        "worst_case": worst_name,
        "minimum_margin_m": results[worst_name].minimum_margin_m,
        "stable": all(result.stable for result in results.values()),
        "cases": {name: result.audit() for name, result in results.items()},
    }
