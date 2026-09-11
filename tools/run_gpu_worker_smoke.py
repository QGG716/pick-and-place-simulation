"""Run one real Mode-B request through the bounded domain IPC adapter."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter, time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "unloading_contracts" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import ImageMapping, ObservationStatus, ResourceReference, SensorFrame, to_wire
from unloading_perception.backends import CargoPipelineBackend


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def create_backend_and_frame(args, *, resident: bool = False):
    """Build one validated Mode-B request without running inference."""
    source_manifest = json.loads((ROOT / "integration" / "vision_mode_b_manifest.json").read_text(encoding="utf-8"))
    model_manifest = json.loads(args.model_manifest.read_text(encoding="utf-8"))
    vision_root = args.vision_root.resolve()
    source = vision_root / source_manifest["input"]["path"]
    proposal = vision_root / source_manifest["frozen_proposals"]["path"]
    people = vision_root / source_manifest["frozen_person_masks"]["path"]
    for path, expected in (
        (source, source_manifest["input"]["sha256"]),
        (proposal, source_manifest["frozen_proposals"]["sha256"]),
        (people, source_manifest["frozen_person_masks"]["sha256"]),
    ):
        if not path.is_file() or sha256(path) != expected:
            raise RuntimeError(f"fixed input missing or hash mismatch: {path}")
    sam = Path(model_manifest["sam"]["snapshot_path"])
    moge = Path(model_manifest["moge"]["model_path"])
    worker = "vision_resident_worker.py" if resident else "vision_worker_entry.py"
    command = (
        str(args.gpu_python), str(ROOT / "tools" / worker),
        "--upstream-root", str(vision_root), "--proposal-json", str(proposal),
        "--person-masks", str(people), "--output-root", str(args.output_root.resolve()),
        "--input-root", str(vision_root), "--sam-model", str(sam),
        "--sam-model-id", model_manifest["sam"]["repository"],
        "--sam-revision", model_manifest["sam"]["revision"], "--moge-model", str(moge),
        "--moge-model-id", model_manifest["moge"]["repository"],
        "--moge-revision", model_manifest["moge"]["revision"],
    )
    backend = CargoPipelineBackend(
        command, cwd=ROOT, timeout_seconds=args.timeout, worker_epoch=f"smoke-{uuid4()}",
        allowed_roots=(vision_root, ROOT, args.output_root.resolve(), args.model_manifest.parent.resolve()),
        resident=resident,
    )
    capture = source.stat().st_mtime
    frame = SensorFrame(
        "fixed-file-input", source.name, f"mode-b-{uuid4()}", 0,
        capture, max(capture, time()), "unix-file-mtime", "camera_optical_model",
        int(source_manifest["input"]["width"]), int(source_manifest["input"]["height"]),
        "png", ResourceReference(source.as_uri(), sha256(source), "image/png"),
        image_mapping=ImageMapping(int(source_manifest["input"]["width"]), int(source_manifest["input"]["height"])),
    )
    return source_manifest, backend, frame


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gpu-python", type=Path, required=True)
    parser.add_argument("--vision-root", type=Path, required=True)
    parser.add_argument("--model-manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--resident", action="store_true")
    args = parser.parse_args()
    source_manifest, backend, frame = create_backend_and_frame(args, resident=args.resident)
    started = perf_counter()
    try:
        observation = backend.infer(frame)
    finally:
        backend.shutdown()
    wall = perf_counter() - started
    result = {
        "mode": source_manifest["mode"],
        "raw_image_automatic": False,
        "wall_seconds": wall,
        "observation": to_wire(observation),
    }
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": observation.status.value,
        "failure_code": observation.failure_code,
        "observation_id": observation.observation_id,
        "cargo": len(observation.cargo),
        "unknown_regions": len(observation.unknown_regions),
        "wall_seconds": wall,
        "result": str(args.result),
    }, indent=2, sort_keys=True))
    expected_provider = "cargo-real-image-gpu-resident-worker" if args.resident else "cargo-real-image-gpu-worker"
    if observation.provider != expected_provider or observation.status not in (
        ObservationStatus.COMPLETE, ObservationStatus.PARTIAL,
    ):
        raise RuntimeError(f"real GPU worker failed closed: {observation.failure_code}: {observation.failure_message}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
