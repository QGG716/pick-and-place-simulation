from __future__ import annotations

from unloading_interfaces.msg import CargoObservation as CargoObservationMsg
from unloading_interfaces.msg import PerceptionObservation as PerceptionObservationMsg
from unloading_interfaces.msg import UnknownRegion as UnknownRegionMsg

from .common import float_to_time


def cargo_to_msg(cargo):
    message = CargoObservationMsg()
    message.source_instance_id = cargo.source_instance_id
    message.object_id = cargo.object_id or ""
    message.track_id = cargo.track_id or ""
    message.category = cargo.category
    message.bbox_xyxy = list(cargo.bbox_xyxy)
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
        message.pose_evidence = cargo.pose.evidence.value
        message.has_pose_covariance = cargo.pose.covariance is not None
        if cargo.pose.covariance is not None:
            message.pose_covariance = list(cargo.pose.covariance)
    message.has_full_dimensions = cargo.full_dimensions_m is not None
    if cargo.full_dimensions_m is not None:
        message.full_dimensions_m = list(cargo.full_dimensions_m)
    message.size_evidence = "" if cargo.size_evidence is None else cargo.size_evidence.value
    message.metric_scale_validity = cargo.metric_scale_validity.value
    message.geometry_validity = cargo.geometry_validity.value
    message.candidate_eligible = cargo.candidate_eligible
    message.eligibility_reasons = list(cargo.eligibility_reasons)
    message.occluded = cargo.occluded
    message.association_status = cargo.association_status
    return message


def observation_to_msg(observation):
    message = PerceptionObservationMsg()
    message.schema_version = observation.schema_version
    message.observation_id = observation.observation_id
    message.source_epoch = observation.source_epoch
    message.source_sequence = observation.source_sequence
    message.capture_time = float_to_time(observation.capture_time)
    message.processed_time = float_to_time(observation.processed_time)
    message.clock_domain = observation.clock_domain
    message.provider = observation.provider
    message.upstream_commit = observation.upstream_commit
    message.model_identity = observation.model_identity
    message.config_identity = observation.config_identity
    message.status = observation.status.value
    message.failure_code = observation.failure_code or ""
    message.failure_message = observation.failure_message or ""
    message.synthetic = observation.synthetic
    message.cargo = [cargo_to_msg(item) for item in observation.cargo]
    message.unknown_regions = []
    for region in observation.unknown_regions:
        item = UnknownRegionMsg(region_id=region.region_id, frame_id=region.frame_id, reason=region.reason, has_bbox=region.bbox_xyxy is not None)
        if region.bbox_xyxy is not None:
            item.bbox_xyxy = list(region.bbox_xyxy)
        message.unknown_regions.append(item)
    return message
