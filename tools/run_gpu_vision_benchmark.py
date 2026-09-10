"""Finite cold-process Mode-B benchmark over the one available fixed image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys
from time import perf_counter


ROOT = Path(__file__).resolve().parents[1]


def stats(values: list[float]) -> dict[str, float]:
    return {
        "minimum": min(values),
        "median": statistics.median(values),
        "maximum": max(values),
        "mean": statistics.fmean(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gpu-python", type=Path, required=True)
    parser.add_argument("--vision-root", type=Path, required=True)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--measurements", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=1800.0)
    args = parser.parse_args()
    if args.warmups < 0 or args.measurements <= 0 or args.warmups + args.measurements > 20:
        raise ValueError("benchmark requires 0..20 warmups and 1..20 measurements, at most 20 total")
    args.output_root.mkdir(parents=True, exist_ok=True)
    samples = []
    for index in range(args.warmups + args.measurements):
        result = args.output_root / f"request-{index:02d}.json"
        before = set(args.output_root.glob("worker-runs/*/metrics.json"))
        command = [
            str(args.gpu_python), str(ROOT / "tools" / "run_gpu_worker_smoke.py"),
            "--gpu-python", str(args.gpu_python), "--vision-root", str(args.vision_root),
            "--model-manifest", str(args.model_manifest),
            "--output-root", str(args.output_root / "worker-runs"),
            "--result", str(result), "--timeout", str(args.timeout),
        ]
        started = perf_counter()
        subprocess.run(command, cwd=ROOT, check=True)
        process_wall = perf_counter() - started
        created = set(args.output_root.glob("worker-runs/*/metrics.json")) - before
        if len(created) != 1:
            raise RuntimeError("each benchmark request must produce exactly one metrics file")
        metrics_path = created.pop()
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        samples.append({
            "index": index,
            "warmup": index < args.warmups,
            "process_wall_seconds": process_wall,
            "stage_timings_seconds": metrics["timings_seconds"],
            "peak_process_gpu_memory_mib_observed": metrics["peak_process_gpu_memory_mib_observed"],
            "metrics_path": str(metrics_path),
        })
    measured = [item for item in samples if not item["warmup"]]
    stages = sorted({name for item in measured for name in item["stage_timings_seconds"]})
    summary = {
        "benchmark_version": 1,
        "execution_model": "one bounded worker process and sequential model stages per request",
        "raw_image_automatic": False,
        "distinct_images": 1,
        "repeated_measurements": args.measurements,
        "warmups": args.warmups,
        "latency_seconds": stats([item["process_wall_seconds"] for item in measured]),
        "stage_latency_seconds": {
            stage: stats([item["stage_timings_seconds"][stage] for item in measured])
            for stage in stages
        },
        "peak_process_gpu_memory_mib_observed": max(
            item["peak_process_gpu_memory_mib_observed"] or 0 for item in measured
        ),
        "percentiles_reported": False,
        "percentiles_note": "sample count is too small for defensible P95/P99 claims",
        "samples": samples,
    }
    output = args.output_root / "benchmark-summary.json"
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
