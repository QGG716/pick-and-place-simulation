"""Run a fail-closed trailer cross-section qualification screen.

This lightweight CI entry point evaluates trailer fit, a conservative flange
reach sphere, and the independent load model on the full 50 mm grid.  Checks
that require pose-specific IK, swept collision, extraction, and timed dynamics
remain NOT_EVALUATED.  Therefore this script never promotes a screen-only cell
to TASK_REACHABLE.  The existing full study remains available in
``studies/fanuc_m20id35_cross_section/run_study.py``.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Sequence

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.robot_load import BoxLoad, LoadCase, MotionLoad, load_robot_limits, load_tool, qualify_load  # noqa: E402
from unloading_sim.reporting import build_report_context  # noqa: E402


BOX_SIZES_M = ((0.150, 0.150, 0.150), (0.500, 0.400, 0.350), (0.800, 0.800, 0.800))
CHECK_NAMES = (
    "IK_FOUND", "JOINT_LIMIT_OK", "SINGULARITY_MARGIN_OK", "GRASP_NORMAL_OK",
    "PAYLOAD_OK", "COM_OK", "WRIST_MOMENT_OK", "WRIST_INERTIA_OK",
    "ROBOT_SELF_COLLISION_FREE", "ROBOT_TRUCK_COLLISION_FREE", "GRIPPER_COLLISION_FREE",
    "BOX_COLLISION_FREE", "EXTRACTION_PATH_FREE", "RETREAT_PATH_FREE", "TRAJECTORY_DYNAMIC_LIMIT_OK",
)
STATES = (
    "NORMAL_REACHABLE", "LOW_SPEED_REACHABLE", "UNREACHABLE_IK", "UNREACHABLE_COLLISION",
    "UNREACHABLE_LOAD", "UNREACHABLE_MOMENT", "UNREACHABLE_INERTIA", "UNREACHABLE_TRAJECTORY", "NOT_EVALUATED",
)
COLORS = ("#16a34a", "#f59e0b", "#64748b", "#7f1d1d", "#dc2626", "#ea580c", "#be123c", "#7e22ce", "#d1d5db")


def _output_dir(value: str | None) -> Path:
    output = Path(value).resolve() if value else ROOT / "results" / datetime.now().strftime("%Y%m%dT%H%M%S_reachability")
    output.mkdir(parents=True, exist_ok=False)
    return output


def _grid(lower: float, upper: float, step: float) -> np.ndarray:
    count = int(round((upper - lower) / step))
    values = lower + step * np.arange(count + 1)
    values[-1] = upper
    return values


def _box_load(size: Sequence[float], mass: float) -> BoxLoad:
    depth, _width, _height = size
    return BoxLoad(np.asarray(size), mass, "front_center", np.zeros(3), np.array([0.5 * depth, 0.0, 0.0]))


def _screen(robot, tool, truck: dict, y: float, z: float, size: Sequence[float], mass: float) -> dict:
    depth, width, height = size
    checks = {name: "NOT_EVALUATED" for name in CHECK_NAMES}
    reasons: list[str] = []
    inside = y - 0.5 * width >= -0.5 * truck["inside_width_m"] - 1e-12 and y + 0.5 * width <= 0.5 * truck["inside_width_m"] + 1e-12 and z - 0.5 * height >= -1e-12 and z + 0.5 * height <= truck["inside_height_m"] + 1e-12
    if not inside:
        checks["BOX_COLLISION_FREE"] = "FAIL"
        return {"state": "UNREACHABLE_COLLISION", "checks": checks, "reason": "real box OBB crosses trailer cross-section boundary", "load": None}

    # Conservative first screen: at least one configured longitudinal base
    # position must put the nominal flange point inside published maximum reach.
    j1_height = float(truck["base_mounting_surface_height_m"]) + 0.425
    contact_x = 1.0
    flange_x = contact_x - float(tool.tcp_xyz_m[0])
    distances = [np.linalg.norm([flange_x - base_x, y, z - j1_height]) for base_x in truck["base_longitudinal_x_scan_m"]]
    if min(distances) > robot.reach_m:
        checks["IK_FOUND"] = "FAIL"
        return {"state": "UNREACHABLE_IK", "checks": checks, "reason": f"nominal flange distance {min(distances):.3f} m exceeds published reach {robot.reach_m:.3f} m", "load": None}
    checks["IK_FOUND"] = "SCREEN_PASS_NOT_IK_PROOF"
    checks["GRASP_NORMAL_OK"] = "SCREEN_PASS"

    result = qualify_load(LoadCase(robot, tool, _box_load(size, mass), MotionLoad(1.0, 1.0, 1.0)))
    checks["PAYLOAD_OK"] = result.criteria["payload"]
    checks["COM_OK"] = result.criteria["com"]
    checks["WRIST_MOMENT_OK"] = result.criteria["wrist_moment"]
    checks["WRIST_INERTIA_OK"] = result.criteria["wrist_inertia"]
    if result.qualification == "FAIL_PAYLOAD":
        state = "UNREACHABLE_LOAD"
    elif result.qualification == "FAIL_WRIST_MOMENT":
        state = "UNREACHABLE_MOMENT"
    elif result.qualification == "FAIL_WRIST_INERTIA":
        state = "UNREACHABLE_INERTIA"
    else:
        state = "NOT_EVALUATED"
        reasons.append("pose-specific IK, joint/singularity, swept collision, extraction, retreat and trajectory dynamics are not evaluated by the lightweight screen")
        if result.qualification == "NOT_EVALUATED":
            reasons.extend(result.reasons)
    return {"state": state, "checks": checks, "reason": " | ".join(reasons or result.reasons), "load": result}


def _write_map(path: Path, title: str, y_values: np.ndarray, z_values: np.ndarray, state_grid: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 6.0))
    cmap = ListedColormap(COLORS)
    mesh = ax.pcolormesh(y_values, z_values, state_grid, shading="nearest", cmap=cmap, norm=BoundaryNorm(np.arange(-0.5, len(STATES) + 0.5), cmap.N))
    colorbar = fig.colorbar(mesh, ax=ax, ticks=np.arange(len(STATES)))
    colorbar.ax.set_yticklabels(STATES)
    ax.set(title=title, xlabel="trailer lateral Y [m] (+left)", ylabel="height Z [m]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="fanuc_m20id_35")
    parser.add_argument("--truck", default="configs/trucks/2p3x2p7.yaml")
    parser.add_argument("--tool", default="configs/tools/unloading_gripper.yaml")
    parser.add_argument("--payloads", nargs="+", type=float, default=(5.0, 15.0, 25.0))
    parser.add_argument("--output-dir")
    args = parser.parse_args(argv)
    context = build_report_context(
        robot=args.robot,
        tool=args.tool,
        root=ROOT,
        extra_inputs={"truck_config": args.truck},
    )
    robot_path = Path(context["resolved_paths"]["robot_config"])
    truck_path = (ROOT / args.truck).resolve() if not Path(args.truck).is_absolute() else Path(args.truck).resolve()
    tool_path = Path(context["resolved_paths"]["tool_resolved_config"])
    robot, tool = load_robot_limits(robot_path), load_tool(tool_path)
    truck = yaml.safe_load(truck_path.read_text(encoding="utf-8"))
    output = _output_dir(args.output_dir)
    spacing = float(truck["grid_spacing_m"])
    y_values = _grid(-0.5 * float(truck["inside_width_m"]), 0.5 * float(truck["inside_width_m"]), spacing)
    z_values = _grid(0.0, float(truck["inside_height_m"]), spacing)
    payload_summaries = {}
    geometry_grid = np.full((len(z_values), len(y_values)), STATES.index("NOT_EVALUATED"), dtype=int)
    for payload in args.payloads:
        rows: list[dict] = []
        grid = np.empty_like(geometry_grid)
        for zi, z in enumerate(z_values):
            for yi, y in enumerate(y_values):
                size_results = [_screen(robot, tool, truck, float(y), float(z), size, payload) for size in BOX_SIZES_M]
                # A grid point is reachable only when every required size passes.
                # No lightweight result can reach that state because some of the
                # 15 checks intentionally remain NOT_EVALUATED.
                state = "NOT_EVALUATED"
                priority = ("UNREACHABLE_LOAD", "UNREACHABLE_MOMENT", "UNREACHABLE_INERTIA", "UNREACHABLE_COLLISION", "UNREACHABLE_IK")
                for candidate in priority:
                    if any(item["state"] == candidate for item in size_results):
                        state = candidate
                        break
                grid[zi, yi] = STATES.index(state)
                geometry_grid[zi, yi] = STATES.index("UNREACHABLE_COLLISION") if all(item["state"] == "UNREACHABLE_COLLISION" for item in size_results) else STATES.index("NOT_EVALUATED")
                rows.append({
                    "y_m": f"{y:.3f}", "z_m": f"{z:.3f}", "payload_kg": f"{payload:g}",
                    "required_box_sizes_depth_width_height_m": "|".join("x".join(f"{v:.3f}" for v in size) for size in BOX_SIZES_M),
                    "task_state": state,
                    "size_states": "|".join(item["state"] for item in size_results),
                    "all_15_conditions_pass": False,
                    "reason": " || ".join(f"{BOX_SIZES_M[index]}: {item['reason']}" for index, item in enumerate(size_results)),
                    **{name: "|".join(item["checks"][name] for item in size_results) for name in CHECK_NAMES},
                })
        csv_path = output / f"reachability_{int(payload) if payload.is_integer() else payload:g}kg.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        _write_map(output / f"reachability_{int(payload) if payload.is_integer() else payload:g}kg.png", f"{payload:g} kg fail-closed task qualification", y_values, z_values, grid)
        counts = {state: int(np.count_nonzero(grid == index)) for index, state in enumerate(STATES)}
        yy, zz = np.meshgrid(y_values, z_values)
        masks = {
            "top_200mm": zz >= float(truck["inside_height_m"]) - 0.2 - 1e-12,
            "bottom_200mm": zz <= 0.2 + 1e-12,
            "left_150mm": yy >= 0.5 * float(truck["inside_width_m"]) - 0.15 - 1e-12,
            "right_150mm": yy <= -0.5 * float(truck["inside_width_m"]) + 0.15 + 1e-12,
        }
        region_counts = {
            name: {state: int(np.count_nonzero((grid == index) & mask)) for index, state in enumerate(STATES)}
            for name, mask in masks.items()
        }
        payload_summaries[str(payload)] = {
            "cell_count": len(rows), "state_counts": counts, "task_reachable_rate": 0.0,
            "region_state_counts": region_counts,
        }
    _write_map(output / "reachability_geometry.png", "Geometric screen (not task reachability)", y_values, z_values, geometry_grid)
    config_record = {"command": "run_reachability_study", "arguments": vars(args), "context": context, "grid_shape_z_y": [len(z_values), len(y_values)], "full_study_command": "python studies/fanuc_m20id35_cross_section/run_study.py --backend pybullet --workers <N> --audit-best"}
    (output / "config.json").write_text(json.dumps(config_record, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {
        "context": context,
        "qualification_scope": "LIGHTWEIGHT_FAIL_CLOSED_SCREEN", "payloads": payload_summaries,
        "task_reachable_rate": 0.0,
        "z_axis_used_ratio": "NOT_EVALUATED",
        "average_extra_action_seconds": "NOT_EVALUATED",
        "not_evaluated_checks": [name for name in CHECK_NAMES if name not in {"IK_FOUND", "GRASP_NORMAL_OK", "PAYLOAD_OK", "COM_OK", "WRIST_MOMENT_OK", "WRIST_INERTIA_OK", "BOX_COLLISION_FREE"}],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    report = f"""# {context['robot_model_id']} task reachability qualification

## Decision

No cross-section cell is reported as `NORMAL_REACHABLE` or `LOW_SPEED_REACHABLE` by this lightweight run for `{context['robot_model_id']}` with `{context['tool_name']}` ({context['tool_mass_kg']:.3f} kg). It is a full-resolution, fail-closed screen: trailer/box fit, published reach, and configured load limits are screened, while pose-specific IK, joint/singularity margins, robot/tool/carried-box swept collisions, 80/100/150 mm extraction, retreat, and timed dynamic limits remain `NOT_EVALUATED`.

The output therefore must not be used as evidence that the arm can extract a box. Run the existing PyBullet cross-section study and integrate pose-specific dynamic qualification before promoting any cell to task-reachable. A known screen failure remains a real exclusion and is not softened.
"""
    (output / "technical_qualification_report.md").write_text(report, encoding="utf-8")
    print(output)
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
