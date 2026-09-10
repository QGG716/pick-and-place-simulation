"""Lossless ROS mappings for the shared domain contracts."""

from __future__ import annotations

import json
from collections.abc import Mapping

from unloading_contracts import (
    CargoObservation,
    EvidenceKind,
    PerceptionObservation,
    PlanningWorldSnapshot,
    Pose3D,
    ResourceReference,
    UnknownRegion,
    Validity,
    canonical_fingerprint,
    dumps,
    from_wire,
    loads,
    to_wire,
)
from unloading_interfaces.msg import CargoObservation as CargoObservationMsg
from unloading_interfaces.msg import PerceptionObservation as PerceptionObservationMsg
from unloading_interfaces.msg import PlanningWorldSnapshot as PlanningWorldSnapshotMsg
from unloading_interfaces.msg import UnknownRegion as UnknownRegionMsg

from .common import float_to_time, time_to_float


def _evidence(value: str) -> EvidenceKind | None:
    return None if not value else EvidenceKind(value)


def _triples(values) -> tuple[tuple[float, float, float], ...] | None:
    if not values:
        return None
    if len(values) % 3:
        raise ValueError("flattened 3D data must be divisible by three")
    return tuple(tuple(float(item) for item in values[index:index + 3]) for index in range(0, len(values), 3))


def cargo_to_msg(cargo: CargoObservation) -> CargoObservationMsg:
    message = CargoObservationMsg()
    message.source_instance_id = cargo.source_instance_id
    message.object_id = cargo.object_id or ""
    message.track_id = cargo.track_id or ""
    message.category = cargo.category
    message.bbox_xyxy = list(cargo.bbox_xyxy)
    if cargo.mask_reference is not None:
        message.mask_uri = cargo.mask_reference.uri
        message.mask_sha256 = cargo.mask_reference.sha256 or ""
        message.mask_media_type = cargo.mask_reference.media_type or ""
    for source_name, has_name, target_name in (
        ("detection_score", "has_detection_score", "detection_score"),
        ("contour_score", "has_contour_score", "contour_score"),
        ("reprojection_score", "has_reprojection_score", "reprojection_score"),
        ("geometry_error_m", "has_geometry_error", "geometry_error_m"),
    ):
        value = getattr(cargo, source_name)
        setattr(message, has_name, value is not None)
        if value is not None:
            setattr(message, target_name, value)
    message.has_pose = cargo.pose is not None
    if cargo.pose is not None:
        message.pose.position.x, message.pose.position.y, message.pose.position.z = cargo.pose.position_m
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = cargo.pose.orientation_xyzw
        message.pose_frame_id = cargo.pose.frame_id
        message.pose_axis_convention = cargo.pose.axis_convention
        message.pose_evidence = cargo.pose.evidence.value
        message.has_pose_covariance = cargo.pose.covariance is not None
        if cargo.pose.covariance is not None:
            message.pose_covariance = list(cargo.pose.covariance)
    message.has_full_dimensions = cargo.full_dimensions_m is not None
    if cargo.full_dimensions_m is not None:
        message.full_dimensions_m = list(cargo.full_dimensions_m)
    message.corners_3d_m = [] if cargo.corners_3d_m is None else [value for point in cargo.corners_3d_m for value in point]
    message.axes_3d_rows = [] if cargo.axes_3d_rows is None else [value for point in cargo.axes_3d_rows for value in point]
    message.size_evidence = "" if cargo.size_evidence is None else cargo.size_evidence.value
    message.depth_evidence = "" if cargo.depth_evidence is None else cargo.depth_evidence.value
    message.scale_evidence = "" if cargo.scale_evidence is None else cargo.scale_evidence.value
    message.metric_scale_validity = cargo.metric_scale_validity.value
    message.geometry_validity = cargo.geometry_validity.value
    message.candidate_eligible = cargo.candidate_eligible
    message.eligibility_reasons = list(cargo.eligibility_reasons)
    message.occluded = cargo.occluded
    message.association_status = cargo.association_status
    message.face_evidence_json = json.dumps(to_wire(cargo.face_evidence), sort_keys=True, separators=(",", ":"))
    message.raw_result_json = json.dumps(to_wire(cargo.raw_result), sort_keys=True, separators=(",", ":"))
    return message


def cargo_from_msg(message: CargoObservationMsg) -> CargoObservation:
    pose = None
    if message.has_pose:
        pose = Pose3D(
            (message.pose.position.x, message.pose.position.y, message.pose.position.z),
            (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w),
            message.pose_frame_id,
            message.pose_axis_convention,
            EvidenceKind(message.pose_evidence),
            tuple(message.pose_covariance) if message.has_pose_covariance else None,
        )
    mask = None
    if message.mask_uri:
        mask = ResourceReference(message.mask_uri, message.mask_sha256 or None, message.mask_media_type or None)
    optional = lambda present, value: float(value) if present else None
    face_evidence = tuple(from_wire(json.loads(message.face_evidence_json))) if message.face_evidence_json else ()
    raw_result = from_wire(json.loads(message.raw_result_json)) if message.raw_result_json else {}
    return CargoObservation(
        message.source_instance_id, message.object_id or None, message.track_id or None,
        message.category, tuple(message.bbox_xyxy), mask,
        optional(message.has_detection_score, message.detection_score),
        optional(message.has_contour_score, message.contour_score),
        optional(message.has_reprojection_score, message.reprojection_score),
        optional(message.has_geometry_error, message.geometry_error_m),
        pose, tuple(message.full_dimensions_m) if message.has_full_dimensions else None,
        _triples(message.corners_3d_m), _triples(message.axes_3d_rows),
        _evidence(message.pose_evidence), _evidence(message.size_evidence),
        _evidence(message.depth_evidence), _evidence(message.scale_evidence),
        Validity(message.metric_scale_validity), Validity(message.geometry_validity),
        message.candidate_eligible, tuple(message.eligibility_reasons), message.occluded,
        message.association_status, face_evidence, raw_result,
    )


def unknown_to_msg(region: UnknownRegion) -> UnknownRegionMsg:
    message = UnknownRegionMsg(region_id=region.region_id, frame_id=region.frame_id, reason=region.reason, has_bbox=region.bbox_xyxy is not None)
    if region.bbox_xyxy is not None:
        message.bbox_xyxy = list(region.bbox_xyxy)
    return message


def unknown_from_msg(message: UnknownRegionMsg) -> UnknownRegion:
    return UnknownRegion(message.region_id, message.frame_id, message.reason, tuple(message.bbox_xyxy) if message.has_bbox else None)


def observation_to_msg(observation: PerceptionObservation) -> PerceptionObservationMsg:
    message = PerceptionObservationMsg()
    for name in ("schema_version", "observation_id", "source_epoch", "clock_domain", "provider", "upstream_commit", "model_identity", "config_identity"):
        setattr(message, name, getattr(observation, name))
    message.source_sequence = observation.source_sequence
    message.capture_time = float_to_time(observation.capture_time)
    message.processed_time = float_to_time(observation.processed_time)
    message.status = observation.status.value
    message.failure_code = observation.failure_code or ""
    message.failure_message = observation.failure_message or ""
    message.synthetic = observation.synthetic
    message.cargo = [cargo_to_msg(item) for item in observation.cargo]
    message.unknown_regions = [unknown_to_msg(item) for item in observation.unknown_regions]
    message.coverage_json = json.dumps(to_wire(observation.coverage), sort_keys=True, separators=(",", ":"))
    return message


def observation_from_msg(message: PerceptionObservationMsg) -> PerceptionObservation:
    coverage = from_wire(json.loads(message.coverage_json)) if message.coverage_json else {"absence_means_free_space": False}
    return PerceptionObservation(
        message.schema_version, message.observation_id, message.source_epoch,
        int(message.source_sequence), time_to_float(message.capture_time),
        time_to_float(message.processed_time), message.clock_domain, message.provider,
        message.upstream_commit, message.model_identity, message.config_identity,
        message.status, message.failure_code or None, message.failure_message or None,
        tuple(cargo_from_msg(item) for item in message.cargo),
        tuple(unknown_from_msg(item) for item in message.unknown_regions), coverage,
        message.synthetic,
    )


def _state_identity(value, *, label: str) -> str:
    """Return an explicit identity, or a stable identity of the exact state value."""
    if isinstance(value, Mapping) and value.get("identity"):
        return str(value["identity"])
    return canonical_fingerprint({"state_kind": label, "state": value})


def snapshot_to_msg(snapshot: PlanningWorldSnapshot, *, source_epoch: str, source_capture_time: float, blocking_reasons=(), obstacles=(), unknown_regions=(), publisher_epoch: str = "legacy-publisher", publisher_sequence: int = 0, publisher_restart: bool = False, published_time: float | None = None, robot_sample_time: float | None = None, mechanism_sample_time: float | None = None, mechanism_revision_sequence: int = 0) -> PlanningWorldSnapshotMsg:
    message = PlanningWorldSnapshotMsg()
    message.schema_version = "1.1.0"
    message.publisher_epoch = publisher_epoch
    message.publisher_sequence = int(publisher_sequence)
    message.publisher_restart = bool(publisher_restart)
    message.published_time = float_to_time(source_capture_time if published_time is None else published_time)
    message.robot_sample_time = float_to_time(0.0 if robot_sample_time is None else robot_sample_time)
    message.mechanism_sample_time = float_to_time(0.0 if mechanism_sample_time is None else mechanism_sample_time)
    message.scene_revision_sequence = snapshot.scene_revision.sequence
    message.scene_fingerprint = snapshot.scene_revision.fingerprint
    message.robot_state_revision_sequence = snapshot.robot_state_revision.sequence
    message.robot_state_fingerprint = snapshot.robot_state_revision.fingerprint
    message.mechanism_revision_sequence = int(mechanism_revision_sequence)
    message.mechanism_fingerprint = snapshot.mechanism_fingerprint
    message.world_fingerprint = snapshot.fingerprint
    message.source_epoch = source_epoch
    message.source_capture_time = float_to_time(source_capture_time)
    message.planning_admissible = bool(snapshot.scene_snapshot["planning_admissible"])
    message.blocking_reasons = list(blocking_reasons)
    message.obstacles = [cargo_to_msg(item) for item in obstacles]
    message.unknown_regions = [unknown_to_msg(item) for item in unknown_regions]
    message.joint_names = list(snapshot.robot_state_revision.robot_state["joint_names"])
    message.actual_joint_positions = list(snapshot.current_q)
    message.robot_model_fingerprint = snapshot.robot_model_fingerprint
    message.world_model_fingerprint = snapshot.world_model_fingerprint
    message.tool_state_identity = _state_identity(snapshot.tool_attachment, label="tool")
    message.payload_state_identity = _state_identity(snapshot.payload_attachment, label="payload")
    message.base_state_identity = _state_identity(snapshot.base_state, label="base")
    message.conveyor_state_identity = _state_identity(snapshot.conveyor_state, label="conveyor")
    message.config_identity = _state_identity(snapshot.config_identity, label="config")
    message.domain_payload_json = dumps(snapshot)
    return message


def snapshot_from_msg(message: PlanningWorldSnapshotMsg) -> PlanningWorldSnapshot:
    snapshot = loads(message.domain_payload_json)
    if not isinstance(snapshot, PlanningWorldSnapshot):
        raise ValueError("ROS snapshot payload is not a PlanningWorldSnapshot")
    if snapshot.fingerprint != message.world_fingerprint or snapshot.scene_revision.fingerprint != message.scene_fingerprint:
        raise ValueError("ROS snapshot envelope fingerprint mismatch")
    if (
        snapshot.robot_state_revision.sequence != int(message.robot_state_revision_sequence)
        or snapshot.robot_state_revision.fingerprint != message.robot_state_fingerprint
        or snapshot.mechanism_fingerprint != message.mechanism_fingerprint
    ):
        raise ValueError("ROS snapshot state-revision envelope mismatch")
    return snapshot
