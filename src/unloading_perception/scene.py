"""Conservative admission, association, lifecycle, and snapshot assembly."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
import json
import math
from typing import Any, Mapping

from unloading_contracts import (
    CargoObservation, PerceptionObservation, PerceptionSceneUpdate,
    PlanningWorldSnapshot, RobotStateRevision, SceneRevision, UnknownRegion,
    ObservationStatus,
    Validity, canonical_fingerprint,
)


def bbox_iou(first: tuple[float, ...], second: tuple[float, ...]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right-left) * max(0.0, bottom-top)
    first_area = (first[2]-first[0]) * (first[3]-first[1])
    second_area = (second[2]-second[0]) * (second[3]-second[1])
    return intersection / max(first_area + second_area - intersection, 1e-12)


class SourceEpochGuard:
    """Monotonic source ownership with explicit, bounded restart takeover."""

    def __init__(self, *, retired_capacity: int = 16) -> None:
        if retired_capacity <= 0:
            raise ValueError("retired epoch capacity must be positive")
        self.retired_capacity = int(retired_capacity)
        self.current_epoch: str | None = None
        self.sequence = -1
        self._retired: OrderedDict[str, None] = OrderedDict()

    def accept(self, epoch: str, sequence: int, *, restart: bool = False) -> None:
        epoch, sequence = str(epoch), int(sequence)
        if not epoch or sequence < 0:
            raise ValueError("source epoch and sequence are invalid")
        if self.current_epoch is None:
            self.current_epoch, self.sequence = epoch, sequence
            return
        if epoch == self.current_epoch:
            if restart:
                raise ValueError("restart cannot reuse the current source epoch")
            if sequence <= self.sequence:
                raise ValueError("duplicate or out-of-order source sequence")
            self.sequence = sequence
            return
        if epoch in self._retired:
            raise ValueError("retired source epoch cannot take ownership again")
        if not restart or sequence != 0:
            raise ValueError("new source epoch requires an explicit sequence-zero restart")
        self._retired[self.current_epoch] = None
        while len(self._retired) > self.retired_capacity:
            self._retired.popitem(last=False)
        self.current_epoch, self.sequence = epoch, sequence


def parse_mechanism_bundle(
    *, tool_identity: str, tool_json: str, payload_identity: str,
    payload_json: str, base_identity: str, base_json: str,
    conveyor_identity: str, conveyor_json: str, source_epoch: str,
    source_sequence: int, sample_time: float,
) -> dict[str, Mapping[str, Any]]:
    """Parse and validate every mechanism field before returning any state."""

    def document(identity: str, raw: str, name: str) -> dict[str, Any]:
        if not identity:
            raise ValueError(f"{name} identity is required")
        value = json.loads(raw)
        if not isinstance(value, dict) or not value:
            raise ValueError(f"{name} state must be a non-empty JSON object")
        return value

    tool = document(tool_identity, tool_json, "tool")
    payload = document(payload_identity, payload_json, "payload")
    base = document(base_identity, base_json, "base")
    conveyor = document(conveyor_identity, conveyor_json, "conveyor")
    if not tool.get("verified"):
        raise ValueError("tool state requires affirmative verified evidence")
    if "object_id" not in payload:
        raise ValueError("payload state requires explicit object_id, including null")
    position = base.get("position_m")
    if not isinstance(position, list) or len(position) != 3 or not all(
        isinstance(value, (int, float)) and math.isfinite(float(value)) for value in position
    ):
        raise ValueError("base state requires three finite position_m values")
    if not isinstance(conveyor.get("running"), bool):
        raise ValueError("conveyor state requires boolean running")

    common = {
        "confirmed": True,
        "source": "mechanism_state_topic",
        "source_epoch": source_epoch,
    }
    return {
        "tool_attachment": {**common, "identity": tool_identity, "details": tool},
        "payload_attachment": {**common, "identity": payload_identity, "details": payload},
        "base_state": {**common, "identity": base_identity, "details": base},
        "conveyor_state": {**common, "identity": conveyor_identity, "details": conveyor},
    }


class ObservationTracker:
    def __init__(self, *, association_iou: float = 0.5, stale_after_misses: int = 1) -> None:
        self.association_iou = float(association_iou)
        self.stale_after_misses = int(stale_after_misses)
        self.epoch: str | None = None
        self.sequence = -1
        self._next_track = 1
        self._tracks: dict[str, tuple[CargoObservation, int]] = {}

    def update(self, observation: PerceptionObservation) -> tuple[CargoObservation, ...]:
        if self.epoch != observation.source_epoch:
            self.epoch, self.sequence, self._tracks = observation.source_epoch, -1, {}
        if observation.source_sequence <= self.sequence:
            raise ValueError("duplicate or out-of-order observation")
        self.sequence = observation.source_sequence
        unused = set(self._tracks)
        updated: dict[str, tuple[CargoObservation, int]] = {}
        for cargo in observation.cargo:
            matches = [
                (bbox_iou(cargo.bbox_xyxy, previous.bbox_xyxy), track_id)
                for track_id, (previous, _) in self._tracks.items()
                if track_id in unused and previous.category == cargo.category
            ]
            matches = [(score, track_id) for score, track_id in matches if score >= self.association_iou]
            matches.sort(reverse=True)
            ambiguous = len(matches) > 1 and abs(matches[0][0] - matches[1][0]) < 0.05
            if matches and not ambiguous:
                track_id = matches[0][1]
                unused.remove(track_id)
                status = "ASSOCIATED"
            elif ambiguous:
                track_id = f"ambiguous-{observation.source_epoch}-{observation.source_sequence}-{cargo.source_instance_id}"
                status = "AMBIGUOUS"
            else:
                track_id = f"track-{self._next_track}"
                self._next_track += 1
                status = "NEW"
            reasons = cargo.eligibility_reasons
            eligible = cargo.candidate_eligible
            if ambiguous:
                eligible = False
                reasons = tuple(dict.fromkeys(reasons + ("ASSOCIATION_AMBIGUOUS",)))
            item = replace(cargo, object_id=track_id, track_id=track_id, association_status=status, candidate_eligible=eligible, eligibility_reasons=reasons)
            updated[track_id] = (item, 0)
        for track_id in sorted(unused):
            previous, misses = self._tracks[track_id]
            misses += 1
            stale = replace(previous, candidate_eligible=False, eligibility_reasons=tuple(dict.fromkeys(previous.eligibility_reasons + ("MISSED_OR_OCCLUDED",))), association_status="STALE_OCCLUDED", occluded=True)
            updated[track_id] = (stale, misses)
        self._tracks = updated
        return tuple(item for item, _ in updated.values())


def build_scene_update(observation: PerceptionObservation, tracked: tuple[CargoObservation, ...] | None = None, *, now: float | None = None, max_age_seconds: float | None = None) -> PerceptionSceneUpdate:
    cargo = observation.cargo if tracked is None else tracked
    unknown = list(observation.unknown_regions)
    blocking = []
    if observation.status in (ObservationStatus.FAILED, ObservationStatus.BACKEND_UNAVAILABLE, ObservationStatus.STALE):
        unknown.append(UnknownRegion(f"observation-{observation.observation_id}", "coverage", observation.failure_code or observation.status.value))
        blocking.append("PERCEPTION_NOT_USABLE")
    if not cargo and not unknown:
        unknown.append(UnknownRegion(f"empty-{observation.observation_id}", "coverage", "EMPTY_OBSERVATION_DOES_NOT_PROVE_FREE_SPACE"))
        blocking.append("EMPTY_OBSERVATION")
    if max_age_seconds is not None:
        if max_age_seconds <= 0.0:
            raise ValueError("max_age_seconds must be positive")
        if now is None:
            raise ValueError("freshness evaluation requires now")
        if now - observation.capture_time > max_age_seconds:
            blocking.append("OBSERVATION_STALE")
    for item in cargo:
        if item.pose is None:
            unknown.append(UnknownRegion(f"object-{item.source_instance_id}", "source_image", "OBJECT_WITHOUT_WORLD_GEOMETRY", item.bbox_xyxy))
        elif item.pose.frame_id != "world":
            unknown.append(UnknownRegion(f"object-{item.source_instance_id}", item.pose.frame_id, "WORLD_TRANSFORM_MISSING", item.bbox_xyxy))
        elif item.full_dimensions_m is None:
            unknown.append(UnknownRegion(f"object-{item.source_instance_id}", item.pose.frame_id, "OBJECT_WITHOUT_CONSERVATIVE_VOLUME", item.bbox_xyxy))
    if unknown:
        blocking.append("UNKNOWN_OR_UNTRANSFORMED_REGIONS")
    geometry_fingerprint = canonical_fingerprint({"obstacles": cargo, "unknown_regions": tuple(unknown)})
    return PerceptionSceneUpdate(observation, tuple(cargo), tuple(item for item in cargo if item.candidate_eligible), tuple(unknown), geometry_fingerprint, not blocking, tuple(dict.fromkeys(blocking)))


@dataclass(frozen=True)
class SnapshotAssemblyResult:
    snapshot: PlanningWorldSnapshot | None
    status: str
    missing: tuple[str, ...]


class SnapshotAssembler:
    """Owns no mechanism state; it only combines externally confirmed state."""

    def __init__(self) -> None:
        self.update: PerceptionSceneUpdate | None = None
        self.robot_state: RobotStateRevision | None = None
        self.tool_attachment: Mapping[str, Any] | None = None
        self.payload_attachment: Mapping[str, Any] | None = None
        self.base_state: Mapping[str, Any] | None = None
        self.conveyor_state: Mapping[str, Any] | None = None
        self.config_identity: Mapping[str, Any] | None = None
        self._revision: SceneRevision | None = None

    def assemble(self) -> SnapshotAssemblyResult:
        names = ("update", "robot_state", "tool_attachment", "payload_attachment", "base_state", "conveyor_state", "config_identity")
        missing = tuple(name for name in names if getattr(self, name) is None)
        if missing:
            return SnapshotAssemblyResult(None, "INCOMPLETE_STATE", missing)
        assert self.update is not None and self.robot_state is not None
        scene = {
            "geometry_fingerprint": self.update.geometry_fingerprint,
            "obstacles": tuple({
                "object_id": item.object_id,
                "source_instance_id": item.source_instance_id,
                "category": item.category,
                "pose": None if item.pose is None else {
                    "position_m": item.pose.position_m,
                    "orientation_xyzw": item.pose.orientation_xyzw,
                    "frame_id": item.pose.frame_id,
                },
                "full_dimensions_m": item.full_dimensions_m,
                "corners_3d_m": item.corners_3d_m,
                "axes_3d_rows": item.axes_3d_rows,
                "pose_evidence": None if item.pose_evidence is None else item.pose_evidence.value,
                "size_evidence": None if item.size_evidence is None else item.size_evidence.value,
                "depth_evidence": None if item.depth_evidence is None else item.depth_evidence.value,
                "scale_evidence": None if item.scale_evidence is None else item.scale_evidence.value,
                "metric_scale_validity": item.metric_scale_validity.value,
                "geometry_validity": item.geometry_validity.value,
                "candidate_eligible": item.candidate_eligible,
                "eligibility_reasons": item.eligibility_reasons,
                "association_status": item.association_status,
                "occluded": item.occluded,
                "raw_result": item.raw_result,
            } for item in self.update.accepted_obstacles),
            "unknown_regions": tuple({"region_id": item.region_id, "frame_id": item.frame_id, "reason": item.reason, "bbox_xyxy": item.bbox_xyxy} for item in self.update.unknown_regions),
            "planning_admissible": self.update.planning_admissible,
            "blocking_reasons": self.update.blocking_reasons,
        }
        fingerprint = self.update.geometry_fingerprint
        if self._revision is None or self._revision.fingerprint != fingerprint:
            sequence = 0 if self._revision is None else self._revision.sequence + 1
            self._revision = SceneRevision(sequence, fingerprint, "perception-integration", None if self._revision is None else self._revision.fingerprint)
        snapshot = PlanningWorldSnapshot(self._revision, scene, self.robot_state, self.tool_attachment, self.payload_attachment, self.base_state, self.conveyor_state, self.config_identity)
        return SnapshotAssemblyResult(snapshot, "READY" if self.update.planning_admissible else "COMPLETE_BUT_NOT_PLANNABLE", ())
