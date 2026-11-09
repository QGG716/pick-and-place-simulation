"""Existing GroundingDINO + SAM outputs bound to the original RGB-D observation."""
from time import monotonic_ns
import cv2
import numpy as np

from .contracts import DetectionResult
from .reference.camera_geometry import ProjectionInfo, opencv_brown_coefficients
from .reference.rgbd_geometry import validate_input


def metric_points(frame):
    k = frame.intrinsics
    info = ProjectionInfo("realsense", k["width"], k["height"], k["fx"], k["fy"], k["ppx"], k["ppy"],
                          distortion=tuple(k["coeffs"]), distortion_model=k["model"])
    if k["model"] == "SYNTHETIC_PINHOLE":
        coefficients = (0.,)*5
    else:
        coefficients = opencv_brown_coefficients(info)
    K = np.array([[k["fx"], 0, k["ppx"]], [0, k["fy"], k["ppy"]], [0, 0, 1]], float)
    y, x = np.indices(frame.depth.shape)
    rays = cv2.undistortPoints(np.stack((x, y), -1).astype(np.float32).reshape(-1, 1, 2), K,
                               np.asarray(coefficients)).reshape(*frame.depth.shape, 2)
    depth_m = frame.depth.astype(np.float32)*frame.depth_scale_m
    points = np.concatenate((rays*depth_m[..., None], depth_m[..., None]), -1)
    # Original reference validator is reused; identity means optical coordinates here.
    return validate_input(frame.rgb, points, np.eye(4), frame.observation_id, virtual_pose=True).points_m


class ExistingGroundingSAM:
    def __init__(self, dino_config, dino_weights, sam_weights, sam_type="vit_b", prompt="carton . box ."):
        from .reference.arith_grounding_dino import GroundingDino
        from .reference.arith_sam_v1 import SamV1
        self.dino, self.sam = GroundingDino(), SamV1()
        self.dino.create_model(dino_config, dino_weights)
        self.sam.create_model(sam_type, sam_weights)
        self.prompt = prompt

    def detect(self, frame):
        bgr = frame.rgb[:, :, ::-1].copy()
        result = self.dino.run_inference(bgr, self.prompt)
        if result is None:
            return DetectionResult(frame.observation_id, frame.stamp.source, monotonic_ns(), ())
        contours = self.sam.run_inference(bgr, result.boxes)
        points = metric_points(frame)
        items, masks = [], []
        for box, score, label, contour in zip(result.boxes.detach().cpu().numpy(), result.scores, result.labels, contours):
            mask = np.zeros(frame.depth.shape, np.uint8)
            if len(contour) >= 3:
                cv2.fillPoly(mask, [contour.astype(np.int32)], 1)
            valid = (mask != 0) & (frame.depth > 0)
            center = np.median(points[valid], axis=0).tolist() if np.any(valid) else None
            items.append({"label": label, "score": float(score), "bbox_xyxy": box.tolist(),
                          "center_m": center, "frame_id": "camera_optical",
                          "geometry_kind": "MASK_MEDIAN_OBSERVATION_NOT_GRASP_POSE"})
            masks.append(mask.astype(bool))
        return DetectionResult(frame.observation_id, frame.stamp.source, monotonic_ns(), tuple(items), tuple(masks))
