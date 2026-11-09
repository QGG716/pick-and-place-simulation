"""GroundingDINO adapter used by the optional open-vocabulary detector."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch


@dataclass(frozen=True)
class GroundingResult:
    """Box coordinates and their matching model scores and text phrases."""

    boxes: torch.Tensor
    scores: np.ndarray
    labels: tuple[str, ...]


def _cached_text_encoder(name: str) -> str:
    """Use a complete local HF snapshot without HTTP freshness checks on startup."""
    from huggingface_hub import try_to_load_from_cache

    paths = [try_to_load_from_cache(name, file) for file in
             ("config.json", "tokenizer_config.json", "vocab.txt")]
    weights = try_to_load_from_cache(name, "model.safetensors")
    if not isinstance(weights, str):
        weights = try_to_load_from_cache(name, "pytorch_model.bin")
    paths.append(weights)
    if all(isinstance(path, str) and Path(path).is_file() for path in paths):
        roots = {Path(path).parent for path in paths}
        if len(roots) == 1:
            return str(roots.pop())
    return name  # A first-time install still uses the normal official download.


DEFAULT_SIZE = 800  # GroundingDINO's own resize target for the shorter image side.


def _native_size_model():
    """The official Model, but never upscaling small camera frames to 800 px.

    A 640 x 480 RealSense frame resized to 1067 x 800 costs ~2x the backbone time
    and finds the same boxes (IoU > 0.98 on recorded carton walls).
    """
    import cv2
    import groundingdino.datasets.transforms as T
    from groundingdino.util.inference import Model, predict
    from PIL import Image

    class NativeSizeModel(Model):
        def predict_with_caption(self, image, caption, box_threshold=0.35, text_threshold=0.25):
            source_h, source_w = image.shape[:2]
            transform = T.Compose([T.RandomResize([min(DEFAULT_SIZE, source_h, source_w)], max_size=1333),
                                   T.ToTensor(), T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
            processed, _ = transform(Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), None)
            boxes, logits, phrases = predict(model=self.model, image=processed.to(self.device), caption=caption,
                                             box_threshold=box_threshold, text_threshold=text_threshold,
                                             device=self.device)
            return Model.post_process_result(source_h, source_w, boxes.float(), logits.float()), phrases

    return NativeSizeModel


class GroundingDino:
    """Load GroundingDINO lazily and retain each box's model metadata."""

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = None

    def create_model(self, config_path: str, model_path: str) -> None:
        # This detector uses PyTorch only. Avoid Transformers probing an unrelated
        # (possibly incompatible) TensorFlow installation during tokenizer.decode.
        # This is process-local and does not uninstall/change TensorFlow itself.
        os.environ.setdefault("USE_TF", "0")
        from groundingdino.models import build_model
        from groundingdino.util.slconfig import SLConfig
        from groundingdino.util.misc import clean_state_dict

        args = SLConfig.fromfile(config_path)
        args.device = self.device
        args.use_checkpoint = False
        args.use_transformer_ckpt = False
        args.text_encoder_type = _cached_text_encoder(args.text_encoder_type)
        network = build_model(args)
        checkpoint = torch.load(model_path, map_location="cpu")
        network.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
        # Preserve the official Model postprocessing implementation;
        # construct its network explicitly to pass inference-only configuration.
        model_class = _native_size_model()
        self.model = model_class.__new__(model_class)
        self.model.model = network.eval().requires_grad_(False).to(self.device)
        self.model.device = self.device

    @torch.inference_mode()
    def run_inference(
        self,
        image: np.ndarray,
        text_prompt: str,
        box_thres: float = 0.35,
        text_thres: float = 0.25,
    ) -> GroundingResult | None:
        if self.model is None:
            raise RuntimeError("GroundingDINO 模型尚未加载")
        # Half precision on the GPU: boxes match fp32 to IoU > 0.999.
        with torch.autocast("cuda", dtype=torch.float16, enabled=self.device == "cuda"):
            detections, phrases = self.model.predict_with_caption(
                image=image,
                caption=text_prompt,
                box_threshold=box_thres,
                text_threshold=text_thres,
            )
        if len(detections.xyxy) == 0:
            return None
        boxes = torch.as_tensor(detections.xyxy, dtype=torch.float32, device=self.device)
        scores = np.asarray(detections.confidence, dtype=np.float32)
        if scores.shape != (len(boxes),) or len(phrases) != len(boxes):
            raise RuntimeError("GroundingDINO 返回的检测框、类别和分数数量不一致")
        return GroundingResult(boxes, scores, tuple(phrases))
