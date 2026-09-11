"""Resident Mode-B worker that reuses the pinned upstream SAM and MoGe models.

The adapter calls the upstream command entry points in-process with cached model
loaders.  Geometry and assembly continue to use the pinned upstream scripts;
no detection, segmentation, depth, or cuboid algorithm is reimplemented here.
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic, perf_counter, time
from typing import Any
from urllib.parse import unquote, urlparse


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "unloading_contracts" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import (  # noqa: E402
    ImageMapping, ObservationStatus, ResourceReference, SCHEMA_VERSION,
    SensorFrame, dumps,
)
from unloading_perception.backends import CargoJsonReplayBackend, UPSTREAM_COMMIT  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def controlled(root: Path, value: Path, *, file: bool = True) -> Path:
    root, value = root.resolve(), value.resolve()
    if value != root and root not in value.parents:
        raise ValueError(f"controlled path escapes {root}")
    if file and not value.is_file():
        raise FileNotFoundError(value)
    return value


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream entry point {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ResidentRuntime:
    def __init__(self, args: argparse.Namespace) -> None:
        started = perf_counter()
        self.args = args
        self.upstream = args.upstream_root.resolve()
        actual = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=self.upstream, text=True,
            capture_output=True, timeout=5.0, check=True,
        ).stdout.strip()
        if actual != UPSTREAM_COMMIT:
            raise RuntimeError(f"UPSTREAM_SHA_MISMATCH:expected {UPSTREAM_COMMIT}, got {actual}")
        self.upstream_commit = actual
        self.proposal = None if args.proposal_json is None else controlled(self.upstream, args.proposal_json)
        self.person_masks = None if args.person_masks is None else controlled(self.upstream, args.person_masks)
        self.output_root = args.output_root.resolve()
        self.output_root.mkdir(parents=True, exist_ok=True)
        if not args.allow_model_download and (
            not Path(args.sam_model).is_dir() or not Path(args.moge_model).is_file()
        ):
            raise RuntimeError("MODEL_MISSING:offline worker requires local SAM and MoGe weights")

        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("GPU_UNAVAILABLE:torch.cuda.is_available() is false")
        self.torch = torch
        import_started = perf_counter()
        self.sam_entry = load_module(
            "pinned_sam_from_generated_boxes",
            self.upstream / "pipeline/segmentation/sam_from_generated_boxes.py",
        )
        self.moge_entry = load_module(
            "pinned_infer_moge_pointmap",
            self.upstream / "pipeline/geometry/infer_moge_pointmap.py",
        )
        self.import_seconds = perf_counter() - import_started

        load_started = perf_counter()
        from transformers import SamModel, SamProcessor
        from moge.model.v2 import MoGeModel
        self.sam_processor = SamProcessor.from_pretrained(
            args.sam_model, local_files_only=not args.allow_model_download,
        )
        self.sam_model = SamModel.from_pretrained(
            args.sam_model, local_files_only=not args.allow_model_download,
        ).to("cuda").eval()
        self.moge_model = MoGeModel.from_pretrained(args.moge_model).to("cuda").eval()
        torch.cuda.synchronize()
        self.model_load_seconds = perf_counter() - load_started
        self.startup_seconds = perf_counter() - started

    def _sample_gpu(self, stage: str, samples: list[dict[str, Any]]) -> None:
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory", "--format=csv,noheader,nounits"],
                text=True, capture_output=True, timeout=2.0, check=True,
            )
            for line in result.stdout.splitlines():
                fields = [item.strip() for item in line.split(",")]
                if len(fields) == 2 and int(fields[0]) == os.getpid():
                    samples.append({"stage": stage, "used_gpu_memory_mib": int(fields[1])})
        except (FileNotFoundError, ValueError, subprocess.SubprocessError):
            pass

    def _upstream_main(self, name: str, module, argv: list[str], log: Path,
                       timings: dict[str, float], samples: list[dict[str, Any]]) -> None:
        before = perf_counter()
        old_argv = sys.argv
        output, errors = io.StringIO(), io.StringIO()
        self._sample_gpu(name + ":before", samples)
        try:
            sys.argv = argv
            with redirect_stdout(output), redirect_stderr(errors):
                module.main()
            self.torch.cuda.synchronize()
        finally:
            sys.argv = old_argv
        timings[name] = perf_counter() - before
        self._sample_gpu(name + ":after", samples)
        log.write_text(output.getvalue() + "\n--- stderr ---\n" + errors.getvalue(), encoding="utf-8")

    def _sam(self, source: Path, proposal: Path, sam_image: Path, masks: Path, instances: Path,
             log: Path, timings: dict[str, float], samples: list[dict[str, Any]]) -> None:
        runtime = self

        class CachedProcessor:
            @classmethod
            def from_pretrained(cls, *_args, **_kwargs):
                return runtime.sam_processor

        class CachedModel:
            @classmethod
            def from_pretrained(cls, *_args, **_kwargs):
                return runtime.sam_model

        old_processor, old_model = self.sam_entry.SamProcessor, self.sam_entry.SamModel
        self.sam_entry.SamProcessor, self.sam_entry.SamModel = CachedProcessor, CachedModel
        argv = [
            "sam_from_generated_boxes.py", str(source), str(proposal),
            "--output", str(sam_image), "--masks-output", str(masks),
            "--json-output", str(instances), "--sam-model", self.args.sam_model,
            "--device", "cuda",
        ]
        if not self.args.allow_model_download:
            argv.append("--local-files-only")
        if self.person_masks is not None:
            argv.extend(["--person-masks", str(self.person_masks)])
        try:
            self._upstream_main("sam", self.sam_entry, argv, log, timings, samples)
        finally:
            self.sam_entry.SamProcessor, self.sam_entry.SamModel = old_processor, old_model

    def _moge(self, source: Path, pointmap: Path, log: Path,
              timings: dict[str, float], samples: list[dict[str, Any]]) -> None:
        import moge.model.v2 as moge_v2
        runtime = self

        class CachedMoGeModel:
            @classmethod
            def from_pretrained(cls, *_args, **_kwargs):
                return runtime.moge_model

        original = moge_v2.MoGeModel
        moge_v2.MoGeModel = CachedMoGeModel
        try:
            self._upstream_main(
                "moge", self.moge_entry,
                ["infer_moge_pointmap.py", str(source), "--model", self.args.moge_model,
                 "--device", "cuda", "--num-tokens", str(self.args.num_tokens),
                 "--output", str(pointmap)],
                log, timings, samples,
            )
        finally:
            moge_v2.MoGeModel = original

    def _stage(self, name: str, command: list[str], run_dir: Path,
               timings: dict[str, float], samples: list[dict[str, Any]]) -> dict[str, Any]:
        started = perf_counter()
        environment = os.environ.copy()
        if not self.args.allow_model_download:
            environment["HF_HUB_OFFLINE"] = "1"
        process = subprocess.Popen(
            command, cwd=self.upstream, env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=os.name == "posix",
        )
        deadline = monotonic() + self.args.stage_timeout
        while True:
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                if os.name == "posix":
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                process.communicate()
                raise subprocess.TimeoutExpired(command, self.args.stage_timeout)
            try:
                stdout, stderr = process.communicate(timeout=min(0.25, remaining))
                break
            except subprocess.TimeoutExpired:
                self._sample_gpu(name, samples)
        timings[name] = perf_counter() - started
        log = run_dir / f"{name}.log"
        log.write_text(stdout + "\n--- stderr ---\n" + stderr, encoding="utf-8")
        if process.returncode:
            tail = (stderr or stdout)[-2000:]
            code = "WORKER_OOM" if "out of memory" in tail.lower() else "STAGE_FAILED"
            raise RuntimeError(f"{code}:{name}:{tail}")
        return {"path": str(log), "sha256": sha256(log), "returncode": process.returncode}

    def infer(self, request: dict[str, Any]) -> dict[str, Any]:
        request_started = perf_counter()
        request_id = str(request["request_id"])
        worker_epoch = str(request["worker_epoch"])
        frame_data = request["frame"]
        input_hash = str(frame_data["rgb_sha256"])
        if request.get("schema_version") != SCHEMA_VERSION or request.get("op") != "infer":
            raise ValueError("unsupported request schema/op")
        parsed = urlparse(str(frame_data["rgb_uri"]))
        if parsed.scheme != "file" or parsed.netloc not in ("", "localhost"):
            raise ValueError("worker accepts local file URIs only")
        source = Path(unquote(parsed.path)).resolve()
        roots = tuple(path.resolve() for path in self.args.input_root)
        if not any(source == root or root in source.parents for root in roots):
            raise ValueError("input escapes configured worker roots")
        if not source.is_file() or sha256(source) != input_hash:
            raise ValueError("input file missing or SHA-256 mismatch")
        proposal = self.proposal
        proposal_reference = request.get("proposal_reference")
        if proposal_reference is not None:
            if not isinstance(proposal_reference, dict):
                raise ValueError("proposal_reference must be a mapping")
            parsed_proposal = urlparse(str(proposal_reference.get("uri", "")))
            if parsed_proposal.scheme != "file" or parsed_proposal.netloc not in ("", "localhost"):
                raise ValueError("proposal_reference must use a local file URI")
            proposal = Path(unquote(parsed_proposal.path)).resolve()
            if not any(proposal == root or root in proposal.parents for root in roots):
                raise ValueError("proposal reference escapes configured worker roots")
            expected_proposal_hash = str(proposal_reference.get("sha256", ""))
            if not proposal.is_file() or len(expected_proposal_hash) != 64 or sha256(proposal) != expected_proposal_hash:
                raise ValueError("proposal file missing or SHA-256 mismatch")
        if proposal is None:
            raise ValueError("each request requires a validated proposal_reference")

        safe_id = "".join(character for character in request_id if character.isalnum() or character in "-_")
        run_dir = controlled(
            self.output_root,
            self.output_root / f"{frame_data['epoch']}-{int(frame_data['sequence']):06d}-{safe_id}",
            file=False,
        )
        run_dir.mkdir(parents=False, exist_ok=False)
        self.torch.cuda.reset_peak_memory_stats()
        timings: dict[str, float] = {}
        samples: list[dict[str, Any]] = []
        logs: dict[str, Any] = {}

        sam_image, masks = run_dir / "sam_crossvalidated.jpg", run_dir / "cargo_masks.npz"
        instances = run_dir / "cargo_instances.json"
        self._sam(source, proposal, sam_image, masks, instances, run_dir / "sam.log", timings, samples)
        logs["sam"] = {"path": str(run_dir / "sam.log"), "sha256": sha256(run_dir / "sam.log"), "returncode": 0}

        python = sys.executable
        faces_image, faces_json, faces_dir = run_dir / "box_geometry_2d.jpg", run_dir / "box_geometry_2d.json", run_dir / "face_crops"
        logs["geometry_2d"] = self._stage("geometry_2d", [
            python, "pipeline/geometry/recover_box_faces.py", str(source), str(masks),
            "--axis-mode", "none", "--refinement-fallback", "minrect",
            "--max-face-area-ratio", "0.15", "--min-mask-area", "50",
            "--min-iou", "0.45", "--min-line-support", "0.15",
            "--output", str(faces_image), "--json-output", str(faces_json),
            "--faces-dir", str(faces_dir),
        ], run_dir, timings, samples)

        pointmap = run_dir / "moge2_pointmap.npz"
        self._moge(source, pointmap, run_dir / "moge.log", timings, samples)
        logs["moge"] = {"path": str(run_dir / "moge.log"), "sha256": sha256(run_dir / "moge.log"), "returncode": 0}

        cuboids_json, cuboids_image = run_dir / "box_cuboids_instance_aware.json", run_dir / "box_cuboids_instance_aware.jpg"
        logs["geometry_3d"] = self._stage("geometry_3d", [
            python, "pipeline/geometry/recover_box_cuboids_3d.py", str(source),
            str(masks), str(pointmap), "--faces-json", str(faces_json),
            "--relative-threshold", "0.003", "--seed", "17",
            "--json-output", str(cuboids_json), "--output", str(cuboids_image),
        ], run_dir, timings, samples)

        final_json, final_image = run_dir / "final_instance_aware.json", run_dir / "final_instance_aware.jpg"
        logs["assembly"] = self._stage("assembly", [
            python, "pipeline/results/assemble_cargo_results.py", str(source),
            str(instances), str(masks), str(faces_json), "--cuboids-json",
            str(cuboids_json), "--output", str(final_image), "--json-output", str(final_json),
        ], run_dir, timings, samples)

        finished = time()
        frame = SensorFrame(
            frame_data["source"], frame_data["stream"], frame_data["epoch"], int(frame_data["sequence"]),
            float(frame_data["capture_time"]), max(float(frame_data["receive_time"]), finished),
            frame_data["clock_domain"], frame_data["frame_id"], int(frame_data["width"]),
            int(frame_data["height"]), frame_data["encoding"], ResourceReference(source.as_uri(), input_hash),
            image_mapping=ImageMapping(int(frame_data["width"]), int(frame_data["height"])),
        )
        observation = CargoJsonReplayBackend(UPSTREAM_COMMIT).read(final_json, frame=frame)
        sam_weight = next((path for name in ("model.safetensors", "pytorch_model.bin") if (path := Path(self.args.sam_model) / name).is_file()), None)
        config = {
            "mode": "REAL_IMAGE_WITH_FROZEN_PROPOSALS", "worker_mode": "resident",
            "sam_model_id": self.args.sam_model_id, "sam_revision": self.args.sam_revision,
            "sam_weight_sha256": None if sam_weight is None else sha256(sam_weight),
            "moge_model_id": self.args.moge_model_id, "moge_revision": self.args.moge_revision,
            "moge_weight_sha256": sha256(Path(self.args.moge_model)), "num_tokens": self.args.num_tokens,
            "proposal_sha256": sha256(proposal),
            "person_masks_sha256": None if self.person_masks is None else sha256(self.person_masks),
        }
        artifacts = {path.name: {"path": str(path), "sha256": sha256(path)} for path in (sam_image, masks, instances, faces_json, pointmap, cuboids_json, final_json, final_image)}
        observation = replace(
            observation, observation_id=f"gpu-{request_id}",
            processed_time=max(observation.capture_time, finished),
            provider="cargo-real-image-gpu-resident-worker",
            model_identity=f"{self.args.sam_model_id}@{self.args.sam_revision}+{self.args.moge_model_id}@{self.args.moge_revision}",
            config_identity=hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
            status=ObservationStatus.PARTIAL,
            coverage={**dict(observation.coverage), "mode": config["mode"], "worker_mode": "resident", "run_id": run_dir.name, "input_sha256": input_hash, "artifacts": artifacts, "stage_timings_seconds": timings, "absence_means_free_space": False},
        )
        timings["request_total"] = perf_counter() - request_started
        metrics = {
            "run_id": run_dir.name, "worker_mode": "resident", "upstream_commit": self.upstream_commit,
            "input_sha256": input_hash, "config": config,
            "startup_seconds": self.startup_seconds, "upstream_import_seconds": self.import_seconds,
            "model_load_seconds": self.model_load_seconds, "timings_seconds": timings,
            "torch_version": self.torch.__version__, "torch_cuda": self.torch.version.cuda,
            "gpu_name": self.torch.cuda.get_device_name(0),
            "adapter_peak_allocated_bytes": self.torch.cuda.max_memory_allocated(),
            "adapter_peak_reserved_bytes": self.torch.cuda.max_memory_reserved(),
            "peak_process_gpu_memory_mib_observed": max((item["used_gpu_memory_mib"] for item in samples), default=None),
            "gpu_memory_samples": samples, "artifacts": artifacts, "logs": logs,
            "timing_scope_note": "SAM and MoGe are upstream-entry totals; their internal preprocess/model/postprocess split is not exposed by the pinned upstream API.",
        }
        metrics_path = run_dir / "metrics.json"
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
        (run_dir / ".complete").write_text(sha256(final_json), encoding="ascii")
        return {
            "schema_version": SCHEMA_VERSION, "request_id": request_id,
            "worker_epoch": worker_epoch, "input_sha256": input_hash, "status": "COMPLETE",
            "observation": dumps(observation),
            "metrics_reference": {"path": str(metrics_path), "sha256": sha256(metrics_path)},
            "output_reference": {"path": str(final_json), "sha256": sha256(final_json)},
            "stage_timings_seconds": timings,
        }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--upstream-root", type=Path, required=True)
    result.add_argument(
        "--proposal-json", type=Path,
        help="Legacy fixed proposal file; per-request proposal_reference is preferred",
    )
    result.add_argument("--person-masks", type=Path)
    result.add_argument("--output-root", type=Path, required=True)
    result.add_argument("--input-root", type=Path, action="append", required=True)
    result.add_argument("--sam-model", required=True)
    result.add_argument("--sam-model-id", default="facebook/sam-vit-base")
    result.add_argument("--sam-revision", default="70c1a07f894ebb5b307fd9eaaee97b9dfc16068f")
    result.add_argument("--moge-model", required=True)
    result.add_argument("--moge-model-id", default="Ruicheng/moge-2-vits-normal")
    result.add_argument("--moge-revision", default="26b477f41595707c5db6770294c0d1721e8ed4ed")
    result.add_argument("--num-tokens", type=int, default=1800)
    result.add_argument("--stage-timeout", type=float, default=900.0)
    result.add_argument("--allow-model-download", action="store_true")
    return result


def emit(response: dict[str, Any]) -> None:
    print(json.dumps(response, sort_keys=True), flush=True)


def main() -> int:
    args = parser().parse_args()
    hello = json.loads(sys.stdin.readline())
    worker_epoch = str(hello.get("worker_epoch", ""))
    if hello.get("schema_version") != SCHEMA_VERSION or hello.get("op") != "hello" or not worker_epoch:
        emit({"schema_version": SCHEMA_VERSION, "op": "failed", "worker_epoch": worker_epoch, "error_code": "HANDSHAKE_INVALID"})
        return 2
    try:
        runtime = ResidentRuntime(args)
    except Exception as exc:
        message = str(exc)
        code = message.split(":", 1)[0] if ":" in message else "WORKER_START_FAILED"
        emit({"schema_version": SCHEMA_VERSION, "op": "failed", "worker_epoch": worker_epoch, "error_code": code, "error_message": message})
        return 2
    emit({
        "schema_version": SCHEMA_VERSION, "op": "ready", "worker_epoch": worker_epoch,
        "upstream_commit": runtime.upstream_commit, "startup_seconds": runtime.startup_seconds,
        "model_load_seconds": runtime.model_load_seconds,
    })
    for line in sys.stdin:
        request_id, input_hash = "unknown", None
        try:
            request = json.loads(line)
            if request.get("op") == "shutdown":
                if request.get("worker_epoch") != worker_epoch:
                    raise ValueError("shutdown epoch mismatch")
                return 0
            request_id = str(request.get("request_id", "unknown"))
            input_hash = request.get("frame", {}).get("rgb_sha256")
            if request.get("worker_epoch") != worker_epoch:
                raise ValueError("request worker epoch mismatch")
            emit(runtime.infer(request))
        except subprocess.TimeoutExpired as exc:
            emit({"schema_version": SCHEMA_VERSION, "request_id": request_id, "worker_epoch": worker_epoch, "input_sha256": input_hash, "status": "FAILED", "error_code": "WORKER_TIMEOUT", "error_message": f"stage timed out after {exc.timeout}s", "fatal": True})
            return 2
        except Exception as exc:
            message = str(exc)
            code = message.split(":", 1)[0] if message.startswith(("WORKER_OOM:", "STAGE_FAILED:")) else "WORKER_ERROR"
            fatal = code == "WORKER_OOM"
            print(f"resident vision worker failed: {message}", file=sys.stderr, flush=True)
            emit({"schema_version": SCHEMA_VERSION, "request_id": request_id, "worker_epoch": worker_epoch, "input_sha256": input_hash, "status": "FAILED", "error_code": code, "error_message": message[-2000:], "fatal": fatal})
            if fatal:
                return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
