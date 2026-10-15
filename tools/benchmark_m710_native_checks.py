"""Serial fixed-input native checker A/B; archived paths are never planning answers."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
from time import perf_counter


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-sha", required=True)
    p.add_argument("--pairs", type=int, choices=(1, 2, 3), default=3)
    p.add_argument("--narrow-only", action="store_true",
                   help="Keep all d33eabd optimizations ON; toggle only narrow qualification")
    p.add_argument("--baseline-sha")
    p.add_argument("--cpu", type=int, default=0)
    args = p.parse_args()
    if args.narrow_only and not args.baseline_sha:
        p.error("--narrow-only requires --baseline-sha")
    modes = ("narrow-baseline", "optimized") if args.narrow_only else ("reference", "optimized")
    args.output.mkdir(parents=True, exist_ok=False)
    os.sched_setaffinity(0, {args.cpu})
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               ROS_HOME=str(args.output / "ros"))
    data = dict(source_sha=args.source_sha, baseline_sha=args.baseline_sha, narrow_only=args.narrow_only, binary_sha256=sha(args.binary),
                model_sha256=sha(args.model), input_sha256=sha(args.input),
                machine=platform.platform(), cpu_affinity=sorted(os.sched_getaffinity(0)),
                thread_environment={k: env[k] for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS")},
                status="CHECKER_BENCHMARK_NOT_EXECUTABLE",
                qualification_status="NOT_EVALUATED", continuous_swept_proof=False, runs=[])
    canonical = None
    for pair in range(args.pairs):
        for mode in (modes if pair % 2 == 0 else tuple(reversed(modes))):
            command = [str(args.binary), str(args.model), str(args.input), mode]
            begin = perf_counter()
            run = subprocess.run(command, env=env, capture_output=True, text=True, timeout=120)
            process_s = perf_counter() - begin
            stem = args.output / f"pair-{pair+1}-{mode}"
            Path(str(stem)+".log").write_text(run.stderr)
            Path(str(stem)+".stdout").write_text(run.stdout)
            run.check_returncode()
            result = json.loads(run.stdout)
            with gzip.open(str(stem)+".json.gz", "wt") as f:
                json.dump(result, f)
            Path(str(stem)+".stdout").unlink()  # only our just-written duplicate serialization
            if args.narrow_only:
                for c in result["cases"]:
                    if not c["process"]["native_check_reuse"] or not c["clearance"]["native_check_reuse"]:
                        raise RuntimeError("PREVIOUS_OPTIMIZATIONS_DISABLED")
                    if c["clearance"]["fixed_narrow_qualification"] != (mode == "optimized"):
                        raise RuntimeError("WRONG_NARROW_DIAGNOSTIC_MODE")
            verdict = [(c["name"], c["verdicts"], c["process"]["last_seal"])
                       for c in result["cases"]]
            if canonical is None:
                canonical = verdict
            if verdict != canonical:
                raise RuntimeError("REFERENCE_OPTIMIZED_VERDICT_OR_WITNESS_DIFFERENCE")
            summary = {k: v for k, v in result.items() if k != "cases"}
            summary.update(pair=pair+1, command=command, process_wall_s=process_s, cases=[])
            for c in result["cases"]:
                compact = {k: v for k, v in c.items() if k != "verdicts"}
                # Raw gzip retains full policy; summary omits its repeated geometry.
                compact["clearance"] = {k: v for k, v in c["clearance"].items() if k != "policy"}
                summary["cases"].append(compact)
            data["runs"].append(summary)
            print(json.dumps(dict(pair=pair+1, mode=mode, cases=[
                dict(name=c["name"], setup_s=c["setup_s"], check_s=c["check_s"], total_s=c["total_s"])
                for c in result["cases"]])), flush=True)
    data["verdicts_and_witnesses_identical"] = True
    data["all_visited_states_valid"] = all(v["clearance"] and v["process"]
                                         for _, states, _ in canonical for v in states)
    data["medians"] = {}
    for name, _, _ in canonical:
        data["medians"][name] = {}
        for mode in modes:
            rows = [c for r in data["runs"] if r["mode"] == mode
                    for c in r["cases"] if c["name"] == name]
            data["medians"][name][mode] = {
                key: statistics.median(c[key] for c in rows)
                for key in ("setup_s", "check_s", "total_s", "process_s", "clearance_s")}
    (args.output / "comparison.json").write_text(json.dumps(data, indent=2)+"\n")


if __name__ == "__main__":
    main()
