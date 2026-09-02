"""Run the deterministic 100k-box unloading cycle Monte Carlo study."""

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

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.cycle import (  # noqa: E402
    draw_cycle_randomness,
    load_cycle_model,
    required_normal_cycle_seconds,
    simulate_cycles,
)


def _output_dir(value: str | None) -> Path:
    output = Path(value).resolve() if value else ROOT / "results" / datetime.now().strftime("%Y%m%dT%H%M%S_cycle")
    output.mkdir(parents=True, exist_ok=False)
    return output


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/cycle/unloading_900pph.yaml")
    parser.add_argument("--output-dir")
    parser.add_argument("--samples", type=int, help="explicit reproducible sample-count override")
    args = parser.parse_args(argv)
    config_path = (ROOT / args.config).resolve() if not Path(args.config).is_absolute() else Path(args.config).resolve()
    model, raw_config, _ = load_cycle_model(config_path)
    if args.samples is not None:
        from dataclasses import replace
        model = replace(model, sample_count=args.samples)
    output = _output_dir(args.output_dir)
    draws = draw_cycle_randomness(model)
    cycles, main_summary = simulate_cycles(model, model.normal_cycle_seconds, draws)
    comparisons = []
    for candidate in model.normal_cycle_candidates_seconds:
        _, summary = simulate_cycles(model, candidate, draws)
        comparisons.append({key: summary[key] for key in (
            "normal_cycle_seconds", "mean_cycle_time_seconds", "p50_seconds", "p90_seconds",
            "p95_seconds", "p99_seconds", "boxes_per_hour", "boxes_per_shift", "target_met"
        )})
    required = required_normal_cycle_seconds(model, draws)
    main_summary["required_normal_cycle_seconds_for_target"] = required
    main_summary["normal_cycle_candidate_comparison"] = comparisons
    main_summary["model_status"] = model.source.get("status", "NOT_EVALUATED")
    (output / "summary.json").write_text(json.dumps(main_summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "config.json").write_text(json.dumps({
        "command": "run_cycle_simulation", "config_path": str(config_path), "resolved_config": raw_config,
        "sample_count_override": args.samples,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    with (output / "cycle_time.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(comparisons[0]))
        writer.writeheader()
        writer.writerows(comparisons)

    fig, ax = plt.subplots(figsize=(8.0, 5.2))
    ax.hist(cycles, bins=120, color="#2563eb", alpha=0.82)
    for percentile, color in ((50, "#16a34a"), (90, "#f59e0b"), (99, "#dc2626")):
        value = float(__import__("numpy").percentile(cycles, percentile))
        ax.axvline(value, color=color, label=f"P{percentile}={value:.3f} s")
    ax.set(title=f"Cycle-time distribution ({model.sample_count:,} boxes, seed {model.seed})", xlabel="cycle time [s]", ylabel="count")
    ax.legend()
    ax.grid(alpha=0.20)
    fig.tight_layout()
    fig.savefig(output / "cycle_time_distribution.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.0, 5.2))
    x = [row["normal_cycle_seconds"] for row in comparisons]
    y = [row["boxes_per_hour"] for row in comparisons]
    ax.plot(x, y, marker="o", color="#2563eb")
    ax.axhline(model.target_boxes_per_hour, color="#dc2626", linestyle="--", label=f"target {model.target_boxes_per_hour:.0f} boxes/h")
    ax.axvline(required, color="#16a34a", linestyle=":", label=f"required normal {required:.3f} s")
    ax.set(title="Throughput versus normal cycle", xlabel="normal cycle [s]", ylabel="system boxes/hour")
    ax.legend()
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "throughput_vs_normal_cycle.png", dpi=160)
    plt.close(fig)

    loss_lines = "\n".join(
        f"- {name}: {detail['mean_seconds_per_box']:.4f} s/box, observed {detail['probability_observed']:.3%}"
        for name, detail in main_summary["event_loss_breakdown"].items()
    )
    report = f"""# Cycle-time technical qualification

## Result

At a configured normal cycle of {model.normal_cycle_seconds:.3f} s, the Monte Carlo mean is {main_summary['mean_cycle_time_seconds']:.3f} s, or {main_summary['boxes_per_hour']:.1f} boxes/hour and {main_summary['boxes_per_shift']:.0f} boxes per {model.shift_hours:g} h shift. P50/P90/P95/P99 are {main_summary['p50_seconds']:.3f}/{main_summary['p90_seconds']:.3f}/{main_summary['p95_seconds']:.3f}/{main_summary['p99_seconds']:.3f} s.

To average {model.target_boxes_per_hour:.0f} boxes/hour with this event model, the NORMAL cycle must be at most **{required:.3f} s**.

## Event loss breakdown

{loss_lines}

## Evidence status

The event model is `{main_summary['model_status']}` and deterministic for seed {model.seed}. It is an engineering throughput risk model, not a production-log calibration. The same draws are reused for all candidate normal cycles so their comparison is reproducible.
"""
    (output / "technical_qualification_report.md").write_text(report, encoding="utf-8")
    print(output)
    print(json.dumps({key: main_summary[key] for key in ("mean_cycle_time_seconds", "p95_seconds", "boxes_per_hour", "boxes_per_shift", "required_normal_cycle_seconds_for_target")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
