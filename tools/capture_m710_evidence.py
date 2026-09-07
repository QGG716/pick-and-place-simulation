"""Snapshot source/configuration evidence without copying private local files."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def capture(output: Path, command: str) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    paths = [ROOT / p for p in tracked if p and p.startswith(("src/", "tools/", "config/", "configs/", "assets/robots/fanuc_m710"))]
    manifest = {
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip(),
        "branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT).decode().strip(),
        "worktree_status": subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT).decode(),
        "command": command, "python": sys.version, "platform": platform.platform(),
        "numpy": np.__version__, "pyyaml": yaml.__version__,
        "file_sha256": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        "configuration_documents": {p.relative_to(ROOT).as_posix(): yaml.safe_load(p.read_text(encoding="utf-8")) for p in paths if p.suffix == ".yaml"},
    }
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--command", required=True)
    args = parser.parse_args()
    capture(args.output_dir, args.command)
