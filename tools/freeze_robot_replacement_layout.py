#!/usr/bin/env python3
"""Export the unchanged g700 environment without inventing an S50 model."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.robot_replacement_layout import (
    BASELINE_COMMIT, SOURCE_FILES, build_replacement_layout,
    compare_non_robot_scenes, freeze_non_robot_scene, frozen_baseline_layout,
)


def _write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definition", type=Path,
        help="Audited replacement definition JSON. Omit to export MODEL_NOT_READY.")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/s50_model_integration/layout_freeze")
    args = parser.parse_args(argv)
    definition = None if args.definition is None else json.loads(args.definition.read_text(encoding="utf-8"))
    reference = frozen_baseline_layout(ROOT)
    replacement = build_replacement_layout(definition, root=ROOT)
    before = freeze_non_robot_scene(reference)
    after = freeze_non_robot_scene(replacement)
    difference = compare_non_robot_scenes(before, after)
    # FANUC output keeps 42.5 kg; the S50 environment carries the S50 carton mass.
    from unloading_sim.s50_payload import apply_s50_carton_mass
    after = apply_s50_carton_mass(after, ROOT)
    report = dict(schema="robot_replacement_layout_report_v1", baseline_commit=BASELINE_COMMIT,
        baseline_sources=list(SOURCE_FILES), model_id=replacement.definition["model_id"],
        layout_status=difference["status"], non_robot_difference=difference,
        model_status="MODEL_BOUND_NOT_MOTION_VALIDATED" if replacement.full_scene_fingerprint else "MODEL_NOT_READY",
        replacement_complete=False, parameters=after["parameters"],
        initial_carton_count=len(after["initial_carton_ids"]), imported_completed_count=0,
        assumed_removed=[], configuration_identity=replacement.configuration_identity,
        full_scene_fingerprint=replacement.full_scene_fingerprint,
        baseline_layout_fingerprint=reference.layout_fingerprint,
        environment_fingerprint=after["environment_fingerprint"],
        qualification=dict(initial_ik="NOT_RUN", initial_collision="NOT_RUN", native_single_carton="NOT_RUN",
            dynamics="NOT_RUN", payload_qualification="NOT_EVALUATED"),
        notes=["只检查非机器人几何、机构行程和工具固有几何；不把布局相同当作模型接入成功。",
            "未知安装坐标变换不默认为单位矩阵；不会加载 FANUC 代替 S50。",
            "工具世界位姿不参与等值比较，真实安装面和安装轴线保持不变。"])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    _write(args.output_dir / "fanuc-frozen-environment.json", before)
    _write(args.output_dir / "replacement-frozen-environment.json", after)
    _write(args.output_dir / "layout-freeze-report.json", report)
    print(json.dumps(dict(layout_status=report["layout_status"], model_status=report["model_status"],
        environment_fingerprint=report["environment_fingerprint"],
        full_scene_fingerprint=report["full_scene_fingerprint"], output_dir=str(args.output_dir)), ensure_ascii=False))
    return 0 if difference["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
