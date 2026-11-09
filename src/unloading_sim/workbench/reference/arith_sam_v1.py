"""Segment Anything adapter for GroundingDINO boxes."""

from __future__ import annotations

import cv2
import numpy as np
import torch


class SamV1:
    """Load SAM lazily and convert masks into ordered contours."""

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = None

    def create_model(self, model_type: str, model_path: str) -> None:
        from segment_anything import SamPredictor, sam_model_registry

        model = sam_model_registry[model_type](checkpoint=model_path)
        self.model = SamPredictor(model.eval().requires_grad_(False).to(self.device))

    @torch.inference_mode()
    def run_inference(self, image: np.ndarray, boxes: torch.Tensor) -> list[np.ndarray]:
        if boxes is None or len(boxes) == 0:
            return []
        if self.model is None:
            raise RuntimeError("SAM 模型尚未加载")
        # The ViT image encoder is ~95 % of SAM's time; half precision nearly halves it.
        with torch.autocast("cuda", dtype=torch.float16, enabled=self.device == "cuda"):
            self.model.set_image(image, image_format="BGR")
            transformed = self.model.transform.apply_boxes_torch(boxes, image.shape[:2])
            masks, _, _ = self.model.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed,
                multimask_output=False,
            )
        return self.masks2segments(masks.squeeze(1).detach().cpu().numpy())

    @staticmethod
    def masks2segments(masks: np.ndarray) -> list[np.ndarray]:
        segments: list[np.ndarray] = []
        for mask in masks.astype(np.uint8):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                segments.append(np.empty((0, 2), dtype=np.float32))
                continue
            contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
            segments.append(contour.astype(np.float32))
        return segments
