"""Exact selected definitions from packages/unloading_contracts/src/unloading_contracts/models.py @ f7668c37019c31b85cc848243231f7c99a6d171d.
Unused orchestration deliberately omitted; local contract types only.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from dataclasses import dataclass, field, replace

from enum import Enum

import hashlib

import json

from math import isfinite, sqrt

from numbers import Integral, Real

from types import MappingProxyType

from typing import Any, Mapping, Protocol, Sequence, runtime_checkable

class EvidenceKind(str, Enum):
    OBSERVED = "OBSERVED"
    MODEL_ESTIMATED = "MODEL_ESTIMATED"
    CONSTRAINT_COMPLETED = "CONSTRAINT_COMPLETED"
    SYNTHETIC = "SYNTHETIC"

def _finite_tuple(values: Sequence[float], length: int | None = None, name: str = "values") -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if length is not None and len(result) != length:
        raise ValueError(f"{name} must contain {length} values")
    if not result or not all(isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result

def _canonical(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _canonical(item) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if hasattr(value, "__dataclass_fields__"):
        return {
            name: _canonical(getattr(value, name))
            for name, definition in value.__dataclass_fields__.items()
            if definition.init
        }
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        number = float(value)
        if not isfinite(number):
            raise ValueError("fingerprint input must be finite")
        return number
    to_list = getattr(value, "tolist", None)
    if callable(to_list):
        return _canonical(to_list())
    raise TypeError(f"cannot fingerprint {type(value).__name__}")

def canonical_fingerprint(value: Any) -> str:
    payload = json.dumps(_canonical(value), ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()

@dataclass(frozen=True)
class Pose3D:
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    frame_id: str
    axis_convention: str
    evidence: EvidenceKind
    covariance: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        position = _finite_tuple(self.position_m, 3, "position")
        quaternion = _finite_tuple(self.orientation_xyzw, 4, "quaternion")
        norm = sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError("quaternion must be normalized xyzw")
        if not self.frame_id or not self.axis_convention:
            raise ValueError("pose frame and axis convention must be explicit")
        covariance = None if self.covariance is None else _finite_tuple(self.covariance, 36, "pose covariance")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "orientation_xyzw", quaternion)
        object.__setattr__(self, "evidence", EvidenceKind(self.evidence))
        object.__setattr__(self, "covariance", covariance)
