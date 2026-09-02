"""Generate an auditable FANUC payload/moment/inertia envelope."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Iterable, Sequence

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.robot_load import (  # noqa: E402
    BoxLoad,
    LoadCase,
    MotionLoad,
    ToolLoad,
    load_robot_limits,
    load_tool,
    qualify_load,
)


DEFAULT_BOX_SIZES_M = (
    (0.150, 0.150, 0.150),
    (0.300, 0.300, 0.300),
    (0.500, 0.400, 0.350),
    (0.600, 0.500, 0.500),
    (0.800, 0.600, 0.600),
    (0.800, 0.800, 0.800),
)
GRASPS = ("front_center", "front_offset", "side_center")
REGION_NAMES = ("NORMAL", "DERATED", "FAIL")
REGION_COLORS = ("#16a34a", "#f59e0b", "#dc2626")


def _inclusive(start: float, stop: float, step: float) -> np.ndarray:
    if not np.isfinite([start, stop, step]).all() or step <= 0.0 or stop < start:
        raise ValueError("scan range must be finite, increasing and use a positive step")
    count = int(np.floor((stop - start) / step + 1e-10))
    values = start + step * np.arange(count + 1)
    if values[-1] < stop - 1e-10:
        values = np.append(values, stop)
    return values


def _resolve_robot(value: str) -> Path:
    supplied = Path(value)
    if supplied.exists():
        return supplied.resolve()
    candidate = ROOT / "configs" / "robots" / f"{value}.yaml"
    if not candidate.exists():
        raise FileNotFoundError(f"robot config not found: {candidate}")
    return candidate


def _output_dir(value: str | None) -> Path:
    if value:
        result = Path(value).resolve()
    else:
        result = ROOT / "results" / datetime.now().strftime("%Y%m%dT%H%M%S_payload")
    result.mkdir(parents=True, exist_ok=False)
    return result


def _box(size: Sequence[float], mass: float, grasp: str) -> BoxLoad:
    depth, width, height = (float(value) for value in size)
    estimated_com = np.array([0.5 * depth, 0.0, 0.0])
    if grasp == "front_center":
        point = np.zeros(3)
    elif grasp == "front_offset":
        point = np.array([0.0, -0.20 * width, -0.20 * height])
    elif grasp == "side_center":
        point = np.array([0.5 * depth, -0.5 * width, 0.0])
    else:
        raise ValueError(f"unsupported grasp {grasp}")
    return BoxLoad(np.asarray(size, dtype=float), float(mass), grasp, point, estimated_com)


def _case(robot, tool: ToolLoad, size: Sequence[float], mass: float, grasp: str, linear_acceleration: float, angular_acceleration: float, safety_factor: float) -> LoadCase:
    return LoadCase(robot, tool, _box(size, mass, grasp), MotionLoad(linear_acceleration, angular_acceleration, safety_factor))


def _region(result) -> int:
    if result.qualification.startswith("FAIL_"):
        return 2
    utilization = float(result.intermediate["maximum_known_utilization"])
    return 1 if utilization > 0.80 else 0


def _region_name(result) -> str:
    return REGION_NAMES[_region(result)]


def _plot_region(path: Path, x: np.ndarray, y: np.ndarray, values: np.ndarray, title: str, xlabel: str, ylabel: str) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 5.4))
    cmap = ListedColormap(REGION_COLORS)
    mesh = ax.pcolormesh(x, y, values, shading="nearest", cmap=cmap, norm=BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N))
    colorbar = fig.colorbar(mesh, ax=ax, ticks=[0, 1, 2])
    colorbar.ax.set_yticklabels(REGION_NAMES)
    ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
    ax.grid(alpha=0.18)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _write_rows(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _scan_rows(robot, tool, masses: Iterable[float], linear_acceleration: float, angular_acceleration: float, safety_factor: float) -> tuple[list[dict], list]:
    rows: list[dict] = []
    results = []
    for size in DEFAULT_BOX_SIZES_M:
        for mass in masses:
            for grasp in GRASPS:
                result = qualify_load(_case(robot, tool, size, float(mass), grasp, linear_acceleration, angular_acceleration, safety_factor))
                intermediate = result.intermediate
                rows.append({
                    "box_size_depth_width_height_m": "x".join(f"{value:.3f}" for value in size),
                    "box_mass_kg": f"{mass:.6g}",
                    "tool_mass_kg": f"{tool.mass_kg:.6g}",
                    "tcp_xyz_m": ";".join(f"{value:.6g}" for value in tool.tcp_xyz_m),
                    "grasp": grasp,
                    "combined_com_xyz_m": ";".join(f"{value:.9g}" for value in intermediate["combined_com_flange_xyz_m"]),
                    "moment_nm_J4_J5_J6": ";".join(f"{value:.9g}" for value in intermediate["safety_factored_moment_nm_J4_J5_J6"]),
                    "inertia_kg_m2_J4_J5_J6": ";".join(f"{value:.9g}" for value in intermediate["safety_factored_inertia_kg_m2_J4_J5_J6"]),
                    "maximum_known_utilization": f"{intermediate['maximum_known_utilization']:.9g}",
                    "known_limits_region": _region_name(result),
                    "qualification": result.qualification,
                    "derating_reason": " | ".join(result.reasons),
                    "com_criterion": result.criteria["com"],
                    "method": intermediate["method"],
                })
                results.append(result)
    return rows, results


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="fanuc_m20id_35")
    parser.add_argument("--tool", default="configs/tools/unloading_gripper.yaml")
    parser.add_argument("--output-dir")
    parser.add_argument("--mass-range", nargs=3, type=float, metavar=("MIN_KG", "MAX_KG", "STEP_KG"), default=(1.0, 25.0, 1.0))
    parser.add_argument("--depth-range-mm", nargs=3, type=float, metavar=("MIN", "MAX", "STEP"), default=(150.0, 800.0, 25.0))
    parser.add_argument("--tcp-range-mm", nargs=3, type=float, metavar=("MIN", "MAX", "STEP"), default=(150.0, 500.0, 25.0))
    parser.add_argument("--tool-mass-range", nargs=3, type=float, metavar=("MIN_KG", "MAX_KG", "STEP_KG"), default=(5.0, 15.0, 1.0))
    parser.add_argument("--linear-acceleration", type=float, default=1.0)
    parser.add_argument("--angular-acceleration", type=float, default=1.0)
    parser.add_argument("--safety-factor", type=float, default=1.0)
    args = parser.parse_args(argv)

    robot_path = _resolve_robot(args.robot)
    tool_path = (ROOT / args.tool).resolve() if not Path(args.tool).is_absolute() else Path(args.tool).resolve()
    robot, tool = load_robot_limits(robot_path), load_tool(tool_path)
    output = _output_dir(args.output_dir)
    masses = _inclusive(*args.mass_range)
    depths = _inclusive(*args.depth_range_mm) * 1e-3
    tcps = _inclusive(*args.tcp_range_mm) * 1e-3
    tool_masses = _inclusive(*args.tool_mass_range)
    rows, results = _scan_rows(robot, tool, masses, args.linear_acceleration, args.angular_acceleration, args.safety_factor)
    _write_rows(output / "robot_load_results.csv", rows)

    representative_width_height = (0.5, 0.5)
    depth_mass = np.empty((len(masses), len(depths)), dtype=int)
    for yi, mass in enumerate(masses):
        for xi, depth in enumerate(depths):
            result = qualify_load(_case(robot, tool, (depth, *representative_width_height), mass, "front_center", args.linear_acceleration, args.angular_acceleration, args.safety_factor))
            depth_mass[yi, xi] = _region(result)
    _plot_region(output / "load_envelope.png", depths, masses, depth_mass, "Depth–payload envelope (published limits; CoM curve pending)", "box depth [m]", "box mass [kg]")

    tcp_mass = np.empty((len(masses), len(tcps)), dtype=int)
    for yi, mass in enumerate(masses):
        for xi, tcp in enumerate(tcps):
            varied = replace(tool, tcp_xyz_m=np.array([tcp, 0.0, 0.0]))
            tcp_mass[yi, xi] = _region(qualify_load(_case(robot, varied, (0.5, 0.4, 0.35), mass, "front_center", args.linear_acceleration, args.angular_acceleration, args.safety_factor)))
    _plot_region(output / "tcp_payload_envelope.png", tcps, masses, tcp_mass, "TCP–payload envelope (published limits; CoM curve pending)", "flange-to-TCP [m]", "box mass [kg]")

    tool_depth = np.empty((len(tool_masses), len(depths)), dtype=int)
    for yi, mass_tool in enumerate(tool_masses):
        varied = replace(tool, mass_kg=float(mass_tool), source={**tool.source, "mass_override": "continuous envelope scan"})
        for xi, depth in enumerate(depths):
            tool_depth[yi, xi] = _region(qualify_load(_case(robot, varied, (depth, 0.5, 0.5), 25.0, "front_center", args.linear_acceleration, args.angular_acceleration, args.safety_factor)))
    _plot_region(output / "tool_mass_envelope.png", depths, tool_masses, tool_depth, "25 kg payload by tool mass and box depth", "box depth [m]", "tool mass [kg]")

    fig, (moment_ax, inertia_ax) = plt.subplots(2, 1, figsize=(8.0, 9.0), sharex=True)
    for grasp in GRASPS:
        moment_values, inertia_values = [], []
        for depth in depths:
            result = qualify_load(_case(robot, tool, (depth, 0.5, 0.5), 25.0, grasp, args.linear_acceleration, args.angular_acceleration, args.safety_factor))
            moment_values.append(max(result.intermediate["moment_utilization_J4_J5_J6"]))
            inertia_values.append(max(result.intermediate["inertia_utilization_J4_J5_J6"]))
        moment_ax.plot(depths, moment_values, label=grasp)
        inertia_ax.plot(depths, inertia_values, label=grasp)
    for ax, ylabel in ((moment_ax, "max moment utilization"), (inertia_ax, "max inertia utilization")):
        ax.axhspan(0.0, robot.normal_utilization_limit, color=REGION_COLORS[0], alpha=0.10)
        ax.axhspan(robot.normal_utilization_limit, 1.0, color=REGION_COLORS[1], alpha=0.12)
        ax.axhspan(1.0, max(1.2, ax.get_ylim()[1]), color=REGION_COLORS[2], alpha=0.10)
        ax.axhline(1.0, color="black", linewidth=1.0)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.25)
        ax.legend()
    inertia_ax.set_xlabel("box depth [m]")
    fig.tight_layout()
    fig.savefig(output / "moment_inertia_envelope.png", dpi=160)
    fig.savefig(output / "moment_envelope.png", dpi=160, bbox_inches="tight")
    plt.close(fig)
    # Keep a separately named inertia artifact required by the review bundle.
    fig, ax = plt.subplots(figsize=(8.0, 5.4))
    for grasp in GRASPS:
        values = []
        for depth in depths:
            result = qualify_load(_case(robot, tool, (depth, 0.5, 0.5), 25.0, grasp, args.linear_acceleration, args.angular_acceleration, args.safety_factor))
            values.append(max(result.intermediate["inertia_utilization_J4_J5_J6"]))
        ax.plot(depths, values, label=grasp)
    ax.axhline(1.0, color="black")
    ax.set(title="25 kg wrist inertia exposure", xlabel="box depth [m]", ylabel="max inertia utilization")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "inertia_envelope.png", dpi=160)
    plt.close(fig)

    config_record = {
        "command": "analyze_payload_envelope",
        "robot_config": str(robot_path),
        "tool_config": str(tool_path),
        "arguments": vars(args),
        "units": "SI; CLI range suffixes marked mm are explicitly converted by 1e-3",
    }
    (output / "config.json").write_text(json.dumps(config_record, indent=2, ensure_ascii=False), encoding="utf-8")
    counts = {status: sum(result.qualification == status for result in results) for status in sorted({result.qualification for result in results})}
    mass_summary_rows: list[dict] = []
    for mass in sorted((float(value) for value in masses), reverse=True):
        selected = [(row, result) for row, result in zip(rows, results) if float(row["box_mass_kg"]) == mass]
        known_failures = sum(result.qualification.startswith("FAIL_") for _row, result in selected)
        max_utilization = max(float(result.intermediate["maximum_known_utilization"]) for _row, result in selected)
        normal_cases = sum(_region(result) == 0 for _row, result in selected)
        derated_cases = sum(_region(result) == 1 for _row, result in selected)
        worst_row, worst_result = max(selected, key=lambda item: float(item[1].intermediate["maximum_known_utilization"]))
        mass_summary_rows.append({
            "box_mass_kg": mass,
            "tool_mass_kg": tool.mass_kg,
            "case_count": len(selected),
            "known_failure_count": known_failures,
            "normal_case_count": normal_cases,
            "derated_case_count": derated_cases,
            "maximum_known_utilization": max_utilization,
            "all_published_hard_limits_pass": known_failures == 0,
            "all_known_limits_normal": normal_cases == len(selected),
            "formal_all_criteria_pass": all(result.qualification in {"PASS", "PASS_DERATED"} for _row, result in selected),
            "limiting_box_size_m": worst_row["box_size_depth_width_height_m"],
            "limiting_grasp": worst_row["grasp"],
            "limiting_qualification": worst_result.qualification,
        })
    _write_rows(output / "payload_descent_summary.csv", mass_summary_rows)
    hard_limit_pass_masses = [row["box_mass_kg"] for row in mass_summary_rows if row["all_published_hard_limits_pass"]]
    normal_masses = [row["box_mass_kg"] for row in mass_summary_rows if row["all_known_limits_normal"]]
    formal_pass_masses = [row["box_mass_kg"] for row in mass_summary_rows if row["formal_all_criteria_pass"]]
    highest_hard_limit_pass = max(hard_limit_pass_masses, default=None)
    highest_all_normal = max(normal_masses, default=None)
    highest_formal_pass = max(formal_pass_masses, default=None)
    required = [result for row, result in zip(rows, results) if float(row["box_mass_kg"]) == 25.0]
    required_failures = sum(result.qualification.startswith("FAIL_") for result in required)
    summary = {
        "model": robot.model,
        "rows": len(rows),
        "qualification_counts": counts,
        "required_25kg_cases": len(required),
        "required_25kg_known_failures": required_failures,
        "payload_plus_tool_kg": 25.0 + tool.mass_kg,
        "payload_descent": mass_summary_rows,
        "highest_box_mass_all_published_hard_limits_pass_kg": highest_hard_limit_pass,
        "highest_box_mass_all_known_limits_normal_kg": highest_all_normal,
        "highest_box_mass_formal_all_criteria_pass_kg": highest_formal_pass,
        "official_com_curve_status": robot.com_limit_status,
        "conclusion": "FAIL or NOT_EVALUATED; not vendor-qualified" if required_failures or robot.com_limit_status != "PASS" else "see row-level results",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    report = f"""# M-20iD/35 technical load qualification

## Decision

The 25 kg box plus the configured {tool.mass_kg:.3f} kg tool is **not qualified as a blanket PASS**. {required_failures} of {len(required)} required 25 kg size/grasp cases fail a known published payload, moment, or inertia limit. Cases without a known-limit failure remain `{robot.com_limit_status}` because the public product sheet does not contain a digitized payload/CoM curve.

In the configured 1 kg scan, the highest box mass for which all {len(DEFAULT_BOX_SIZES_M) * len(GRASPS)} size/grasp cases pass every available published hard limit is **{highest_hard_limit_pass} kg**. The highest mass for which every known-limit case remains in the NORMAL region is **{highest_all_normal} kg**. The highest formally all-criteria-qualified mass is **{highest_formal_pass}**, because the official CoM curve is still unavailable.

## Method and limits

- Total tool and box mass, both CoMs, solid-cuboid box inertia, the tool tensor, the parallel-axis theorem, gravity, configured linear/angular acceleration, and safety factor are included.
- Dynamic wrist moment is an `ENGINEERING_ESTIMATE`, not a FANUC controller certification.
- Published source: {robot.source['title']} ({robot.source['url']}).
- Tool mass properties: `{tool.source.get('file', tool.source.get('base_file', 'configured source'))}`; flange clocking is `{tool.source.get('flange_clocking_status', 'unknown')}`.

## Review boundary

This Phase 1 report does not claim collision-free extraction, full task reachability, a recommended Z axis, conveyor placement, or 900 boxes/hour. Those decisions require the corresponding geometry and cycle studies. Unknown criteria are not converted to PASS.
"""
    (output / "technical_qualification_report.md").write_text(report, encoding="utf-8")
    print(output)
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
