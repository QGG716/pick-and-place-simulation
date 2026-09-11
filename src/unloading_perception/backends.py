"""Vision adapters. No detector or 3D recovery algorithm is implemented here."""

from __future__ import annotations

from dataclasses import replace
from collections import deque
import hashlib
import json
import os
from pathlib import Path
from queue import Empty, Queue
import subprocess
from threading import Lock, Thread
from typing import Any, Mapping, Sequence
from urllib.parse import unquote, urlparse
from urllib.request import url2pathname
from uuid import uuid4

from unloading_contracts import (
    SCHEMA_VERSION, CargoObservation, EvidenceKind, FaceEvidence, ImageMapping,
    ObservationStatus, PerceptionObservation, Pose3D, ResourceReference,
    SensorFrame, UnknownRegion, Validity, loads,
)

from .geometry import pose_from_axes_rows, quaternion_from_rotation


UPSTREAM_COMMIT = "1d208f2ed380a207e6e46b4a62d2ac640edfe477"


def resolve_controlled_reference(root: Path, reference: str) -> Path:
    root = root.resolve()
    candidate = (root / reference).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError("resource reference escapes the configured replay root") from exc
    if not candidate.is_file():
        raise FileNotFoundError(candidate)
    return candidate


def _optional_score(value: Any) -> float | None:
    return None if value is None else float(value)


def _contract_score(value: Any) -> float | None:
    """Map an upstream quality-head value to the contract score interval."""

    if value is None:
        return None
    number = float(value)
    return min(1.0, max(0.0, number))


def _face_evidence(geometry: Mapping[str, Any]) -> tuple[FaceEvidence, ...]:
    result = []
    evidence_map = {
        "registered_semantic_structural_corners": EvidenceKind.OBSERVED,
        "registered_sam_visible_face": EvidenceKind.OBSERVED,
        "monocular_depth_plane": EvidenceKind.MODEL_ESTIMATED,
        "depth_supported_threeface_pnp": EvidenceKind.MODEL_ESTIMATED,
        "joint_sam_depth_multiplane_pnp": EvidenceKind.MODEL_ESTIMATED,
        "mask_supported_cuboid_side": EvidenceKind.CONSTRAINT_COMPLETED,
        "bounded_cuboid_completion": EvidenceKind.CONSTRAINT_COMPLETED,
        "cuboid_constraint_completion": EvidenceKind.CONSTRAINT_COMPLETED,
        "person_occlusion_completed_visible_face": EvidenceKind.CONSTRAINT_COMPLETED,
    }
    for index, face in enumerate(geometry.get("camera_facing_faces") or ()):
        source = str(face.get("evidence") or "unknown")
        result.append(FaceEvidence(
            f"axis-{face.get('axis_index', 'unknown')}-{face.get('side', index)}",
            evidence_map.get(source, EvidenceKind.MODEL_ESTIMATED),
            source,
            tuple(tuple(float(value) for value in point) for point in face.get("corners_2d") or ()),
        ))
    return tuple(result)


def _cargo_from_upstream(instance: Mapping[str, Any]) -> CargoObservation:
    source_id = str(instance.get("instance_id", instance.get("id", "unknown")))
    label = str(instance.get("label") or "unknown")
    bbox = tuple(float(value) for value in instance.get("bbox") or ())
    geometry = instance.get("geometry_3d")
    reasons = []
    pose = None
    dimensions = corners = axes = None
    pose_evidence = size_evidence = depth_evidence = scale_evidence = None
    geometry_validity = Validity.NOT_EVALUATED
    metric_validity = Validity.UNKNOWN
    if isinstance(geometry, Mapping) and geometry.get("accepted"):
        corners = tuple(tuple(float(value) for value in point) for point in geometry.get("corners_3d") or ()) or None
        axes = tuple(tuple(float(value) for value in axis) for axis in geometry.get("orthogonal_axes_3d") or ()) or None
        dimensions = tuple(float(value) for value in geometry.get("shape_dimensions") or ()) or None
        if corners and len(corners) == 8 and axes and len(axes) == 3 and dimensions and len(dimensions) == 3:
            center = tuple(sum(point[index] for point in corners) / len(corners) for index in range(3))
            pose = pose_from_axes_rows(center, axes, "camera_optical_model", EvidenceKind.MODEL_ESTIMATED)
            pose_evidence = EvidenceKind.MODEL_ESTIMATED
            size_evidence = EvidenceKind.CONSTRAINT_COMPLETED if "single" in str(geometry.get("completion_mode")) else EvidenceKind.MODEL_ESTIMATED
            depth_evidence = EvidenceKind.MODEL_ESTIMATED
            scale_evidence = EvidenceKind.MODEL_ESTIMATED
            geometry_validity = Validity.VALID
        else:
            geometry_validity = Validity.INVALID
            reasons.append("GEOMETRY_3D_INCOMPLETE")
        # MoGe is explicitly monocular pseudo-3D in the fixed upstream. The
        # field name 'metric' and a 2D certificate do not validate scale.
        reasons.extend(("MONOCULAR_SCALE_UNVERIFIED", "CAMERA_EXTRINSICS_MISSING", "GEOMETRY_UNCERTAINTY_MISSING"))
    elif geometry is None:
        geometry_validity = Validity.NOT_EVALUATED
        reasons.append("GEOMETRY_3D_MISSING")
    else:
        geometry_validity = Validity.INVALID
        reasons.append("GEOMETRY_3D_REJECTED")
    if label not in ("box", "cardboard_box"):
        reasons.append("NON_BOX_CARGO")
    if not reasons:
        reasons.append("EXECUTION_EVIDENCE_NOT_EVALUATED")
    return CargoObservation(
        source_id, None, None, label, bbox, None,
        _optional_score(instance.get("score")),
        _contract_score(instance.get("sam_iou_score")),
        _optional_score(geometry.get("mask_reprojection_iou") if isinstance(geometry, Mapping) else None),
        _optional_score(geometry.get("plane_residual_mean") if isinstance(geometry, Mapping) else None),
        pose, dimensions, corners, axes,
        pose_evidence, size_evidence, depth_evidence, scale_evidence,
        metric_validity, geometry_validity, False, tuple(dict.fromkeys(reasons)),
        False, "SOURCE_FRAME_ONLY", _face_evidence(geometry or {}),
        {
            "id": instance.get("id"),
            "instance_id": instance.get("instance_id"),
            "simulation_object_id": instance.get("simulation_object_id"),
            "oracle_proposal_source_id": instance.get("oracle_proposal_source_id"),
            "proposal_source": instance.get("proposal_source"),
            "sam_iou_score_raw": instance.get("sam_iou_score"),
            "sam_iou_score_clipped_to_contract": (
                instance.get("sam_iou_score") is not None
                and not 0.0 <= float(instance["sam_iou_score"]) <= 1.0
            ),
            "geometry_status": instance.get("geometry_status"),
            "geometry_3d_status": instance.get("geometry_3d_status"),
            "completion_mode": geometry.get("completion_mode") if isinstance(geometry, Mapping) else None,
            "certificate_status": geometry.get("certificate_status") if isinstance(geometry, Mapping) else None,
            "validation_status": instance.get("validation_status"),
        },
    )


class CargoJsonReplayBackend:
    """Read actual upstream stage/final JSON without upgrading its evidence."""

    provider_name = "cargo-json-replay"

    def __init__(self, upstream_commit: str = UPSTREAM_COMMIT) -> None:
        if len(upstream_commit) != 40:
            raise ValueError("replay backend requires a fixed 40-character upstream commit")
        self.upstream_commit = upstream_commit

    def read(self, path: Path, *, frame: SensorFrame | None = None, replay_time: float = 0.0, replay_sequence: int = 0, replay_epoch: str = "replay-file") -> PerceptionObservation:
        payload = json.loads(path.read_text(encoding="utf-8"))
        instances = payload.get("instances")
        if not isinstance(instances, list):
            raise ValueError("upstream JSON must contain an instances array")
        size = payload.get("source_size") or (1, 1)
        if frame is None:
            frame = SensorFrame(
                "cargo-detection-and-6D-estimation", str(path), replay_epoch, replay_sequence,
                float(replay_time), float(replay_time), "replay", "source_image",
                int(size[0]), int(size[1]), "external-json-reference",
                ResourceReference(path.resolve().as_uri(), media_type="application/json"),
                image_mapping=ImageMapping(int(size[0]), int(size[1])),
            )
        cargo = tuple(_cargo_from_upstream(item) for item in instances)
        unknown = tuple(
            UnknownRegion(f"cargo-{item.source_instance_id}", "source_image", ";".join(item.eligibility_reasons), item.bbox_xyxy)
            for item in cargo if item.pose is None or item.metric_scale_validity is not Validity.VALID
        )
        return PerceptionObservation(
            SCHEMA_VERSION, f"replay-{frame.epoch}-{frame.sequence}", frame.epoch, frame.sequence,
            frame.capture_time, max(frame.capture_time, frame.receive_time), frame.clock_domain,
            self.provider_name, self.upstream_commit, "published-upstream-json", "fixture-or-manifest-bound",
            ObservationStatus.PARTIAL, None, None, cargo, unknown,
            {"coordinate_space_2d": payload.get("coordinate_space"), "source_size": size, "absence_means_free_space": False},
            False,
        )


class SimGroundTruthBackend:
    """Explicit simulation-only wrapper around baseline OBB detections."""

    def read(self, detections: Sequence[Any], frame: SensorFrame) -> PerceptionObservation:
        cargo = []
        for detection in detections:
            obb = detection.obb
            rotation = tuple(tuple(float(obb.rotation[row, col]) for col in range(3)) for row in range(3))
            dimensions = tuple(float(value * 2.0) for value in obb.half_extents)
            pose = Pose3D(tuple(float(value) for value in obb.center), quaternion_from_rotation(rotation), "world", "rotation_columns_are_local_axes", EvidenceKind.SYNTHETIC, tuple(0.0 for _ in range(36)))
            cargo.append(CargoObservation(
                str(detection.name), str(detection.name), str(detection.name), "box", (0.0, 0.0, 1.0, 1.0), None,
                float(detection.score), None, None, 0.0, pose, dimensions, None,
                tuple(tuple(rotation[row][col] for row in range(3)) for col in range(3)),
                EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC,
                Validity.VALID, Validity.VALID, True, (), False, "SIMULATION_IDENTITY", (),
                {"source": "simulation_ground_truth"},
            ))
        return PerceptionObservation(
            SCHEMA_VERSION, f"sim-{frame.epoch}-{frame.sequence}", frame.epoch, frame.sequence,
            frame.capture_time, frame.receive_time, frame.clock_domain, "simulation-ground-truth",
            "baseline:" + UPSTREAM_COMMIT, "deterministic-scene-obb", "simulation",
            ObservationStatus.COMPLETE, None, None, tuple(cargo), (),
            {"frame_id": "world", "absence_means_free_space": False}, True,
        )


class CargoPipelineBackend:
    """Bounded JSON-lines subprocess adapter for a separately managed worker."""

    def __init__(self, command: Sequence[str], *, cwd: Path, timeout_seconds: float = 60.0,
                 worker_epoch: str | None = None, allowed_roots: Sequence[Path] = (),
                 resident: bool = False) -> None:
        if not command or timeout_seconds <= 0.0:
            raise ValueError("worker command and positive timeout are required")
        self.command = tuple(str(item) for item in command)
        self.cwd = cwd.resolve()
        self.timeout_seconds = float(timeout_seconds)
        self.worker_epoch = worker_epoch or str(uuid4())
        self.allowed_roots = tuple(path.resolve() for path in (allowed_roots or (self.cwd,)))
        self.latest_epoch: str | None = None
        self.latest_sequence = -1
        self.resident = bool(resident)
        self._process: subprocess.Popen[str] | None = None
        self._stdout_lines: Queue[str] | None = None
        self._stderr_tail: deque[str] = deque(maxlen=64)
        self._request_lock = Lock()

    def capability(self) -> tuple[bool, str]:
        if not self.cwd.is_dir():
            return False, "BACKEND_WORKDIR_MISSING"
        executable = Path(self.command[0])
        if executable.is_absolute() and not executable.is_file():
            return False, "BACKEND_INTERPRETER_MISSING"
        return True, "AVAILABLE"

    def infer(self, frame: SensorFrame) -> PerceptionObservation:
        available, reason = self.capability()
        if not available:
            return self._failure(frame, "BACKEND_UNAVAILABLE", reason, ObservationStatus.BACKEND_UNAVAILABLE)
        request_id = str(uuid4())
        try:
            image_path = self._validated_image(frame.rgb)
        except (ValueError, FileNotFoundError) as exc:
            return self._failure(frame, "INPUT_REFERENCE_INVALID", str(exc), ObservationStatus.FAILED)
        request = json.dumps({
            "schema_version": SCHEMA_VERSION, "op": "infer", "request_id": request_id,
            "worker_epoch": self.worker_epoch,
            "frame": {
                "source": frame.source, "stream": frame.stream, "epoch": frame.epoch,
                "sequence": frame.sequence, "capture_time": frame.capture_time,
                "receive_time": frame.receive_time, "clock_domain": frame.clock_domain,
                "frame_id": frame.frame_id, "width": frame.width, "height": frame.height,
                "encoding": frame.encoding, "rgb_uri": image_path.as_uri(),
                "rgb_sha256": frame.rgb.sha256,
            },
        }, sort_keys=True) + "\n"
        if not self._request_lock.acquire(blocking=False):
            return self._failure(frame, "WORKER_BUSY", "resident worker queue capacity is one", ObservationStatus.FAILED)
        try:
            try:
                returncode, stdout, stderr = self._exchange(request)
            except subprocess.TimeoutExpired:
                self._stop_process(force=True)
                if self.resident:
                    self.worker_epoch = str(uuid4())
                return self._failure(frame, "WORKER_TIMEOUT", "vision worker exceeded configured timeout", ObservationStatus.FAILED)
            except (OSError, ValueError) as exc:
                self._stop_process(force=True)
                if self.resident:
                    self.worker_epoch = str(uuid4())
                return self._failure(frame, "WORKER_START_FAILED", str(exc), ObservationStatus.BACKEND_UNAVAILABLE)
        finally:
            self._request_lock.release()
        if returncode != 0:
            if self.resident:
                self._stop_process(force=True)
                self.worker_epoch = str(uuid4())
            try:
                response = json.loads(stdout.strip().splitlines()[-1])
            except (ValueError, IndexError, json.JSONDecodeError):
                return self._failure(frame, "WORKER_CRASH", f"worker exited {returncode}: {stderr[-500:]}", ObservationStatus.FAILED)
            code = str(response.get("error_code") or "WORKER_CRASH")
            status = ObservationStatus.BACKEND_UNAVAILABLE if code in ("MODEL_MISSING", "GPU_UNAVAILABLE") else ObservationStatus.FAILED
            return self._failure(frame, code, str(response.get("error_message") or stderr[-500:]), status)
        try:
            response = json.loads(stdout.strip().splitlines()[-1])
            if response.get("schema_version") != SCHEMA_VERSION or response.get("request_id") != request_id or response.get("worker_epoch") != self.worker_epoch:
                raise ValueError("worker response envelope identity mismatch")
            if response.get("input_sha256") != frame.rgb.sha256:
                raise ValueError("worker response input hash mismatch")
            if self.resident and bool(response.get("fatal", False)):
                self._stop_process(force=True)
                self.worker_epoch = str(uuid4())
            if response.get("status") != "COMPLETE":
                code = str(response.get("error_code") or "WORKER_FAILED")
                status = ObservationStatus.BACKEND_UNAVAILABLE if code in ("MODEL_MISSING", "GPU_UNAVAILABLE") else ObservationStatus.FAILED
                return self._failure(frame, code, str(response.get("error_message") or code), status)
            self._validated_output_reference(response.get("metrics_reference"), name="metrics_reference")
            self._validated_output_reference(response.get("output_reference"), name="output_reference")
            observation = loads(response["observation"])
        except (ValueError, KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            return self._failure(frame, "WORKER_SCHEMA_ERROR", str(exc), ObservationStatus.FAILED)
        if not isinstance(observation, PerceptionObservation):
            return self._failure(frame, "WORKER_SCHEMA_ERROR", "worker did not return PerceptionObservation", ObservationStatus.FAILED)
        if observation.source_epoch != frame.epoch or observation.source_sequence != frame.sequence:
            return self._failure(frame, "WORKER_IDENTITY_MISMATCH", "worker response does not match request frame", ObservationStatus.FAILED)
        try:
            artifacts = observation.coverage.get("artifacts", {})
            if not isinstance(artifacts, Mapping):
                raise ValueError("worker artifacts must be a mapping")
            for name, reference in artifacts.items():
                self._validated_output_reference(reference, name=f"artifact:{name}")
        except (ValueError, FileNotFoundError) as exc:
            return self._failure(frame, "WORKER_SCHEMA_ERROR", str(exc), ObservationStatus.FAILED)
        if self.latest_epoch != frame.epoch:
            self.latest_epoch, self.latest_sequence = frame.epoch, -1
        if frame.sequence <= self.latest_sequence:
            return self._failure(frame, "LATE_WORKER_RESULT", "late result cannot replace a newer frame", ObservationStatus.STALE)
        self.latest_sequence = frame.sequence
        return observation

    def shutdown(self) -> None:
        process = self._process
        if self.resident and process is not None and process.poll() is None and process.stdin is not None:
            try:
                process.stdin.write(json.dumps({
                    "schema_version": SCHEMA_VERSION, "op": "shutdown",
                    "worker_epoch": self.worker_epoch,
                }, sort_keys=True) + "\n")
                process.stdin.flush()
            except OSError:
                pass
        self._stop_process(force=False)

    def _exchange(self, request: str) -> tuple[int, str, str]:
        if not self.resident:
            process = subprocess.Popen(
                self.command, cwd=self.cwd, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            self._process = process
            stdout, stderr = process.communicate(request, timeout=self.timeout_seconds)
            self._process = None
            return int(process.returncode), stdout, stderr
        self._ensure_resident()
        assert self._process is not None and self._process.stdin is not None
        assert self._stdout_lines is not None
        self._process.stdin.write(request)
        self._process.stdin.flush()
        try:
            response = self._stdout_lines.get(timeout=self.timeout_seconds)
        except Empty as exc:
            raise subprocess.TimeoutExpired(self.command, self.timeout_seconds) from exc
        returncode = self._process.poll()
        if response == "" and returncode is not None:
            return int(returncode or 1), "", "".join(self._stderr_tail)
        if response == "":
            try:
                returncode = self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                returncode = 1
            return int(returncode), "", "".join(self._stderr_tail)
        return 0, response, "".join(self._stderr_tail)

    def _ensure_resident(self) -> None:
        if self._process is not None and self._process.poll() is None:
            return
        self._stderr_tail.clear()
        process = subprocess.Popen(
            self.command, cwd=self.cwd, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        self._process = process
        stdout_lines: Queue[str] = Queue(maxsize=2)
        self._stdout_lines = stdout_lines
        assert process.stdout is not None and process.stderr is not None and process.stdin is not None

        def read_stdout() -> None:
            for line in process.stdout:
                stdout_lines.put(line)
            stdout_lines.put("")

        def read_stderr() -> None:
            for line in process.stderr:
                self._stderr_tail.append(line)

        Thread(target=read_stdout, name="cargo-worker-stdout", daemon=True).start()
        Thread(target=read_stderr, name="cargo-worker-stderr", daemon=True).start()
        process.stdin.write(json.dumps({
            "schema_version": SCHEMA_VERSION, "op": "hello",
            "worker_epoch": self.worker_epoch,
        }, sort_keys=True) + "\n")
        process.stdin.flush()
        try:
            line = self._stdout_lines.get(timeout=self.timeout_seconds)
        except Empty as exc:
            raise subprocess.TimeoutExpired(self.command, self.timeout_seconds) from exc
        try:
            ready = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError("resident worker returned an invalid ready envelope") from exc
        if (
            ready.get("schema_version") != SCHEMA_VERSION
            or ready.get("op") != "ready"
            or ready.get("worker_epoch") != self.worker_epoch
        ):
            raise ValueError("resident worker ready envelope identity mismatch")

    def _stop_process(self, *, force: bool) -> None:
        process = self._process
        self._process = None
        self._stdout_lines = None
        if process is None or process.poll() is not None:
            return
        if force:
            process.kill()
            process.wait()
            return
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def _validated_image(self, reference: ResourceReference) -> Path:
        parsed = urlparse(reference.uri)
        if parsed.scheme != "file" or parsed.netloc not in ("", "localhost"):
            raise ValueError("pipeline RGB input must be a local file URI")
        local_path = url2pathname(unquote(parsed.path))
        if os.name == "nt" and len(local_path) >= 3 and local_path[0] in ("/", "\\") and local_path[2] == ":":
            local_path = local_path[1:]
        candidate = Path(local_path).resolve()
        if not any(candidate == root or root in candidate.parents for root in self.allowed_roots):
            raise ValueError("pipeline RGB input escapes configured roots")
        if not candidate.is_file():
            raise FileNotFoundError(candidate)
        if reference.sha256 is None:
            raise ValueError("pipeline RGB input requires a SHA-256")
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        if digest != reference.sha256.lower():
            raise ValueError("pipeline RGB input SHA-256 mismatch")
        return candidate

    def _validated_output_reference(self, reference: Any, *, name: str) -> Path:
        if not isinstance(reference, Mapping):
            raise ValueError(f"{name} must be a path/hash mapping")
        raw_path, expected = reference.get("path"), reference.get("sha256")
        if not isinstance(raw_path, str) or not raw_path or not isinstance(expected, str) or len(expected) != 64:
            raise ValueError(f"{name} requires path and SHA-256")
        candidate = Path(raw_path).resolve()
        if not any(candidate == root or root in candidate.parents for root in self.allowed_roots):
            raise ValueError(f"{name} escapes configured roots")
        if not candidate.is_file():
            raise FileNotFoundError(candidate)
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        if digest != expected.lower():
            raise ValueError(f"{name} SHA-256 mismatch")
        return candidate

    def _failure(self, frame: SensorFrame, code: str, message: str, status: ObservationStatus) -> PerceptionObservation:
        return PerceptionObservation(
            SCHEMA_VERSION, f"worker-failure-{frame.epoch}-{frame.sequence}", frame.epoch, frame.sequence,
            frame.capture_time, frame.receive_time, frame.clock_domain, "cargo-pipeline-worker",
            UPSTREAM_COMMIT, "unavailable", "external-worker", status, code, message, (),
            (UnknownRegion("worker-coverage-unknown", frame.frame_id, code),),
            {"absence_means_free_space": False}, False,
        )
