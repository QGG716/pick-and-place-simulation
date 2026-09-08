"""One-request worker boundary for the pinned visual pipeline environment.

This adapter consumes a published upstream JSON result. It deliberately does
not pretend that the repository exposes a single automatic real-time inference
entry point: proposal generation, SAM and MoGe prerequisites must be produced
in that independently provisioned environment first.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "unloading_contracts" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import ImageMapping, ResourceReference, SensorFrame, dumps
from unloading_perception.backends import CargoJsonReplayBackend, UPSTREAM_COMMIT


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream-root", type=Path, required=True)
    parser.add_argument("--published-json", type=Path, required=True)
    args = parser.parse_args()
    try:
        actual = subprocess.run(["git", "rev-parse", "HEAD"], cwd=args.upstream_root, text=True, capture_output=True, timeout=5, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        print(json.dumps({"schema_version": "1.0.0", "status": "BACKEND_UNAVAILABLE", "error": str(exc)}))
        return 3
    if actual != UPSTREAM_COMMIT:
        print(json.dumps({"schema_version": "1.0.0", "status": "BACKEND_UNAVAILABLE", "error": f"upstream SHA mismatch: {actual}"}))
        return 4
    request = json.loads(sys.stdin.readline())
    if request.get("schema_version") != "1.0.0" or request.get("op") != "infer":
        print(json.dumps({"schema_version": "1.0.0", "status": "ERROR", "error": "unsupported request schema/op"}))
        return 2
    frame_data = request["frame"]
    frame = SensorFrame(
        frame_data["source"], frame_data["stream"], frame_data["epoch"], int(frame_data["sequence"]),
        0.0, 0.0, "worker-request", "source_image", 1, 1, "controlled-reference",
        ResourceReference(frame_data["rgb_uri"]), image_mapping=ImageMapping(1, 1),
    )
    observation = CargoJsonReplayBackend(UPSTREAM_COMMIT).read(args.published_json, frame=frame)
    print(dumps(observation))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
