"""Pure report-context helpers shared by qualification command wrappers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from .identity import load_tool_config, normalize_robot_model_id, sha256_file
from .robot_load.model import load_robot_limits


def resolve_robot_config(value: str | Path, root: str | Path) -> Path:
    root_path = Path(root).resolve()
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate.resolve()
    if candidate.suffix in {".yaml", ".yml"} or "/" in str(value) or "\\" in str(value):
        return (root_path / candidate).resolve()
    return (root_path / "configs" / "robots" / f"{normalize_robot_model_id(value)}.yaml").resolve()


def resolve_input_path(value: str | Path, root: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (Path(root).resolve() / path).resolve()


def build_report_context(
    *,
    robot: str | Path,
    tool: str | Path,
    root: str | Path,
    extra_inputs: Mapping[str, str | Path] | None = None,
) -> dict[str, Any]:
    """Resolve identity, paths, hashes, and evidence without running a study."""
    root_path = Path(root).resolve()
    robot_path = resolve_robot_config(robot, root_path)
    requested_tool_path = resolve_input_path(tool, root_path)
    robot_limits = load_robot_limits(robot_path)
    tool_config = load_tool_config(requested_tool_path)
    paths = {
        "robot_config": str(robot_path),
        "tool_requested_config": str(requested_tool_path),
        "tool_resolved_config": str(tool_config.config_path),
    }
    hashes = {
        "robot_config_sha256": sha256_file(robot_path),
        "tool_requested_config_sha256": sha256_file(requested_tool_path),
        "tool_resolved_config_sha256": tool_config.config_hash,
    }
    for name, value in (extra_inputs or {}).items():
        resolved = resolve_input_path(value, root_path)
        paths[str(name)] = str(resolved)
        hashes[f"{name}_sha256"] = sha256_file(resolved)
    return {
        "robot_model_id": robot_limits.model,
        "tool_name": tool_config.name,
        "tool_mass_kg": tool_config.mass_kg,
        "tcp_transform": tool_config.tcp_transform,
        "tool_mass_properties_source": tool_config.mass_properties_source,
        "tool_measured_status": tool_config.source.get("measured_status", "NOT_MEASURED"),
        "tool_manufacturer_qualification_status": tool_config.source.get(
            "manufacturer_load_certificate_status", "NOT_EVALUATED"
        ),
        "resolved_paths": paths,
        "input_hashes": hashes,
    }


def qualification_decision_reason(
    load_summary: Mapping[str, Any],
    reachability_summary: Mapping[str, Any],
    cycle_summary: Mapping[str, Any],
) -> str:
    """Describe the observed component status without fixed model assumptions."""
    failures = int(load_summary.get("required_25kg_known_failures", 0))
    required = int(load_summary.get("required_25kg_cases", 0))
    reach_rate = reachability_summary.get("task_reachable_rate", "NOT_EVALUATED")
    com_status = load_summary.get("official_com_curve_status", "NOT_EVALUATED")
    cycle_rate = cycle_summary.get("boxes_per_hour", "NOT_EVALUATED")
    return (
        f"load cases with known failures: {failures}/{required}; "
        f"official CoM criterion: {com_status}; task-reachable rate: {reach_rate}; "
        f"cycle estimate: {cycle_rate} boxes/h"
    )


def qualification_decision(
    load_summary: Mapping[str, Any],
    reachability_summary: Mapping[str, Any],
) -> str:
    """Return a fail-closed suite decision from recorded component results."""
    failures = int(load_summary.get("required_25kg_known_failures", 0))
    cases = int(load_summary.get("required_25kg_cases", 0))
    com_pass = load_summary.get("official_com_curve_status") == "PASS"
    reach_rate = reachability_summary.get("task_reachable_rate")
    fully_reachable = isinstance(reach_rate, (int, float)) and float(reach_rate) >= 1.0
    return "QUALIFIED" if cases > 0 and failures == 0 and com_pass and fully_reachable else "NOT_QUALIFIED"
