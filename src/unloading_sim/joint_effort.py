"""Typed joint-effort evidence, independent of Isaac imports.

The installed CPU PhysX projected channel measures net active joint effort.
Only the verified, unconstrained single-axis scope below identifies that with
implicit drive output. Unknown applicability is not a measured overload.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
import math
from typing import Any

PROJECTED_API = "isaacsim.core.experimental.prims.Articulation.get_dof_projected_joint_forces"
PROJECTED_KIND = "dof_projected_joint_effort"
AXIS = "signed_positive_DOF_motion_axis_child_joint_frame"
CONDITIONS = (
    "zero_explicit_effort", "zero_joint_friction", "inactive_position_limits",
    "inactive_velocity_limits", "no_contact_or_external_joint_constraints",
    "revolute_tree_axis_mapping_verified",
)


def projected_source(*, isaac_version: str, tensor_version: str, backend: str) -> dict:
    return dict(quantity_kind=PROJECTED_KIND, units="Nm", axis=AXIS,
                source_api=PROJECTED_API, isaac_version=isaac_version,
                tensor_version=tensor_version, backend=backend,
                conversion="identity_no_sign_or_scale_adjustment",
                semantics_evidence="docs/evidence/curobo_v2_20260928_effort/README.md")


def source_reasons(source: dict, applicability: dict) -> list[str]:
    expected = projected_source(isaac_version="6.0.1.0", tensor_version="110.1.13", backend="CPU_PhysX")
    reasons = ["SOURCE_" + key.upper() for key in (
        "quantity_kind", "units", "axis", "source_api", "isaac_version",
        "tensor_version", "backend", "conversion") if source.get(key) != expected[key]]
    reasons.extend("UNPROVEN_" + key.upper() for key in CONDITIONS if applicability.get(key) is not True)
    return reasons


def finite_vector(values, count):
    if values is None:
        return None
    try:
        result = [float(v) for v in values]
    except (TypeError, ValueError):
        return None
    return result if len(result) == count and all(math.isfinite(v) for v in result) else None


@dataclass
class JointEffortMonitor:
    joint_names: list[str]
    limits_nm: list[float]
    limit_source: str
    physics_dt: float
    source: dict
    configured_readback_nm: list[float] | None = None
    samples: int = 0
    valid_samples: int = 0
    comparable_samples: int = 0
    last_step: int = 0
    invalid_reasons: Counter = field(default_factory=Counter)
    first_failure: dict | None = None
    peak_ratio: float | None = None

    def __post_init__(self):
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("unique joint names required")
        limits = finite_vector(self.limits_nm, len(self.joint_names))
        if limits is None or min(limits) <= 0 or not self.limit_source:
            raise ValueError("finite positive sourced limits required")
        if not math.isfinite(self.physics_dt) or self.physics_dt <= 0:
            raise ValueError("positive physics_dt required")
        self.limits_nm = limits

    def observe(self, *, joint_names, physics_step, simulation_time, observation_phase,
                raw_values, applicability, invalid_reason=None) -> dict:
        self.samples += 1
        reasons = []
        values = finite_vector(raw_values, len(self.joint_names))
        if values is None:
            reasons.append("MISSING_OR_NONFINITE_VALUES")
        if list(joint_names) != self.joint_names:
            reasons.append("JOINT_ORDER_MISMATCH")
        timing_valid = (type(physics_step) is int and physics_step > 0 and
                        physics_step > self.last_step and isinstance(simulation_time, (int, float)) and
                        math.isfinite(simulation_time) and
                        abs(simulation_time - physics_step*self.physics_dt) <= 1e-8 and
                        observation_phase == "post_physics_step")
        if not timing_valid:
            reasons.append("INVALID_OBSERVATION_TIMING")
        contiguous = timing_valid and physics_step == self.last_step + 1
        if timing_valid:
            self.last_step = physics_step
        if not contiguous:
            self.invalid_reasons["MISSING_OR_DUPLICATE_PHYSICS_STEP"] += 1
        if invalid_reason:
            reasons.append(str(invalid_reason))
        valid = not reasons
        if valid:
            self.valid_samples += 1
        applicability_reasons = source_reasons(self.source, applicability)
        comparable = valid and not applicability_reasons
        ratios = None
        if comparable:
            self.comparable_samples += 1
            ratios = [abs(v)/limit for v, limit in zip(values, self.limits_nm)]
            self.peak_ratio = max(max(ratios), self.peak_ratio or 0.)
            if max(ratios) > 1. and self.first_failure is None:
                i = max(range(len(ratios)), key=ratios.__getitem__)
                self.first_failure = dict(reason="JOINT_EFFORT_LIMIT_EXCEEDED",
                    joint=self.joint_names[i], value_nm=values[i], limit_nm=self.limits_nm[i],
                    physics_step=physics_step, simulation_time=simulation_time)
        self.invalid_reasons.update(reasons + applicability_reasons)
        return dict(physics_step=physics_step, simulation_time=simulation_time,
                    observation_phase=observation_phase, joint_names=list(joint_names),
                    raw_values=raw_values, converted_values=values if comparable else None,
                    valid=valid, invalid_reasons=reasons, applicability=applicability,
                    comparison_valid=comparable, comparison_invalid_reasons=applicability_reasons,
                    limit_ratios=ratios,
                    stop_reason=(self.first_failure["reason"] if self.first_failure else
                                 "JOINT_EFFORT_OBSERVATION_LOST" if not valid else None))

    def summary(self, required_steps: int) -> dict[str, Any]:
        coverage = (required_steps > 0 and self.samples == required_steps and
                    self.last_step == required_steps and self.comparable_samples == required_steps and
                    not self.invalid_reasons["MISSING_OR_DUPLICATE_PHYSICS_STEP"])
        status = "FAIL" if self.first_failure else "PASS" if coverage else "NOT_EVALUATED"
        readback = finite_vector(self.configured_readback_nm, len(self.joint_names))
        return dict(schema="joint_effort_evidence_v1", status=status,
                    reason="valid observed effort exceeds limit" if self.first_failure else
                           "valid comparable observation covers every required physics step" if coverage else
                           "observation semantics, applicability or required coverage incomplete",
                    joint_names=self.joint_names, source=self.source,
                    limits_nm=self.limits_nm, limit_source=self.limit_source,
                    configured_limit_readback_nm=self.configured_readback_nm,
                    configured_limits_read_back=readback == self.limits_nm,
                    physics_dt=self.physics_dt, observation_phase="post_physics_step",
                    comparison_scope="implicit drive output only under all documented source conditions",
                    required_steps=required_steps, observed_samples=self.samples,
                    valid_observation_samples=self.valid_samples,
                    comparable_samples=self.comparable_samples, full_coverage=coverage,
                    maximum_ratio=self.peak_ratio, first_failure=self.first_failure,
                    invalid_reasons=dict(self.invalid_reasons))


def safe_read_channel(reader, count):
    """Keep unavailability explicit; never substitute zeros for failed reads."""
    try:
        values = reader()
        finite = finite_vector(values, count)
        return (finite, None) if finite is not None else (None, "MISSING_OR_NONFINITE_VALUES")
    except Exception as exc:
        return None, f"CHANNEL_READ_ERROR:{type(exc).__name__}:{exc}"


def collect_isaac_effort_context(articulation, tensor_module, physics_context, joint_prims):
    """Optional adapter entry; dependencies supplied after Isaac starts."""
    import hashlib
    import importlib.metadata
    import inspect
    from pathlib import Path
    import tomllib
    tensor_path = Path(inspect.getfile(tensor_module.create_simulation_view)).resolve()
    tensor_version = "UNKNOWN"
    for ancestor in tensor_path.parents:
        config = ancestor / "config/extension.toml"
        if config.is_file():
            tensor_version = tomllib.loads(config.read_text())["package"]["version"]
            break
    names = list(articulation.dof_names)
    readback, error = safe_read_channel(lambda: articulation.get_dof_max_efforts().numpy()[0].tolist(), len(names))
    source = projected_source(isaac_version=importlib.metadata.version("isaacsim"),
        tensor_version=tensor_version, backend="GPU_PhysX" if physics_context.is_gpu_dynamics_enabled() else "CPU_PhysX")
    for key, path in [("implementation", Path(inspect.getfile(type(articulation)))), ("tensor_api", tensor_path)]:
        source[key + "_path"] = str(path)
        source[key + "_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    context = dict(joint_names=names, link_names=list(articulation.link_names),
        configured_limit_readback_nm=readback, readback_error=error, source=source,
        scope="post-step business actions; initialization/settling is outside this interval",
        comparison_applicability="multi-DOF contact/attachment drive decomposition not verified")
    try:
        context["dof_friction_properties"] = [x.numpy().tolist() for x in articulation.get_dof_friction_properties()]
    except Exception as exc:
        context["dof_friction_properties"] = None
        context["friction_read_error"] = f"{type(exc).__name__}:{exc}"
    context["joint_usd_attributes"] = {
        str(prim.GetPath()): {attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes()
            if any(key in attr.GetName().lower() for key in ("friction", "limit", "axis", "localrot", "localpos", "drive", "armature"))}
        for prim in joint_prims}
    return context


def collect_isaac_effort_sample(articulation, monitor, step, simulation_time, applicability=None):
    """Shared by the business adapter and independent physical probe."""
    count = len(monitor.joint_names)
    explicit, explicit_error = safe_read_channel(lambda: articulation.get_dof_efforts().numpy()[0].tolist(), count)
    projected, error = safe_read_channel(lambda: articulation.get_dof_projected_joint_forces().numpy()[0].tolist(), count)
    scope = dict(applicability or {})
    scope["zero_explicit_effort"] = explicit is not None and all(v == 0 for v in explicit)
    row = monitor.observe(joint_names=list(articulation.dof_names), physics_step=step,
        simulation_time=simulation_time, observation_phase="post_physics_step", raw_values=projected,
        applicability=scope, invalid_reason=error or explicit_error)
    row.update(explicit_input_nm=explicit, explicit_input_error=explicit_error)
    try:
        force, torque = articulation.get_link_incoming_joint_force()
        force, torque = force.numpy()[0].tolist(), torque.numpy()[0].tolist()
        if not all(math.isfinite(v) for array in (force, torque) for vector in array for v in vector):
            raise ValueError("nonfinite incoming wrench")
        row["incoming_wrench_child_joint_frame"] = dict(force_N=force, torque_Nm=torque)
    except Exception as exc:
        row["incoming_wrench_child_joint_frame"] = None
        row["incoming_wrench_error"] = f"{type(exc).__name__}:{exc}"
    return row
