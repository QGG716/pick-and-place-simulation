"""Record native package builds, binary provenance, platform and CPU authority."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--prefix", type=Path, required=True)
    p.add_argument("--worker", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    packages = []
    for path in sorted((args.prefix / "conda-meta").glob("*.json")):
        data = json.loads(path.read_text())
        packages.append({key: data.get(key) for key in ("name", "version", "build", "channel", "url", "sha256")})
    handshake = subprocess.run([str(args.worker.resolve())], input="", text=True, capture_output=True, timeout=60)
    linked = subprocess.run(["ldd", str(args.worker.resolve())], text=True, capture_output=True, timeout=30)
    result = dict(platform=platform.platform(), python=sys.version, python_executable=sys.executable,
        cpu_info=Path("/proc/cpuinfo").read_text() if Path("/proc/cpuinfo").exists() else None,
        cpu_affinity=sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
        threads={name: os.environ.get(name) for name in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]},
        native_packages=packages, worker_sha256=hashlib.sha256(args.worker.read_bytes()).hexdigest(),
        startup=dict(returncode=handshake.returncode, stdout=handshake.stdout, stderr=handshake.stderr),
        dynamic_libraries=linked.stdout, isaac_environment_modified=False)
    result["cpu_authority_packages"] = {}
    for name in ["pin", "coal", "numpy", "pytest"]:
        try:
            result["cpu_authority_packages"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result["cpu_authority_packages"][name] = None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result["startup"]))
    return handshake.returncode


if __name__ == "__main__":
    raise SystemExit(main())
