"""Grounding DINO + SAM inference core shared by the field PC and the GPU server.

Standalone on purpose: it imports only numpy, torch and transformers, so the
same file (verified by SHA-256) runs locally and on the remote inference
server. It returns plain Python/numpy values; callers bind them to frames.
Models load from local Hugging Face directories only (no downloads).
"""
import hashlib
import json
from pathlib import Path
import numpy as np

MAX_TARGETS = 20


def carton_nms(boxes, scores, threshold=.98):
    """Class-agnostic: 'box' and 'carton' can name the same physical object."""
    boxes = np.asarray(boxes, float); scores = np.asarray(scores, float); keep = []
    for i in np.argsort(-scores, kind='stable'):
        a = boxes[i]; area = max(0, a[2]-a[0])*max(0, a[3]-a[1])
        if area <= 0 or not np.isfinite(a).all(): continue
        duplicate = False
        for j in keep:
            b = boxes[j]; overlap = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
            other = max(0, b[2]-b[0])*max(0, b[3]-b[1])
            if overlap/(area+other-overlap) >= threshold: duplicate = True; break
        if not duplicate: keep.append(int(i))
    return keep


def directory_hashes(path):
    """SHA-256 of every regular file in one model directory, by file name."""
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path(path).iterdir()) if p.is_file()}


class GroundingSamCore:
    def __init__(self, dino_directory, sam_directory, device='cpu', prompt='carton . box .',
                 box_threshold=.35, text_threshold=.30, cpu_threads=2):
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection, SamModel, SamProcessor
        self.torch = torch; self.device = device; self.prompt = prompt
        if type(cpu_threads) != int or not 1 <= cpu_threads <= 8: raise ValueError('INVALID_INFERENCE_CPU_THREAD_BUDGET')
        torch.set_num_threads(cpu_threads)
        self.box_threshold = box_threshold; self.text_threshold = text_threshold
        for path, kind in ((dino_directory, 'grounding-dino'), (sam_directory, 'sam')):
            p = Path(path)
            if not (p/'model.safetensors').is_file() or kind not in json.loads((p/'config.json').read_text()).get('model_type', ''):
                raise ValueError('CHECKPOINT_FORMAT_OR_MODEL_MISMATCH:'+str(p))
        # These are HF directories, never renamed upstream .pth checkpoints.
        self.dino_processor = AutoProcessor.from_pretrained(dino_directory, local_files_only=True)
        self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(dino_directory, local_files_only=True).to(device).eval()
        self.sam_processor = SamProcessor.from_pretrained(sam_directory, local_files_only=True)
        self.sam = SamModel.from_pretrained(sam_directory, local_files_only=True).to(device).eval()

    def detect(self, rgb):
        """Boxes after class-agnostic NMS: list of {label, score, bbox_xyxy}."""
        torch = self.torch
        with torch.inference_mode():
            inputs = self.dino_processor(images=rgb, text=self.prompt, return_tensors='pt').to(self.device)
            outputs = self.dino(**inputs)
            r = self.dino_processor.post_process_grounded_object_detection(outputs, inputs.input_ids,
                box_threshold=self.box_threshold, text_threshold=self.text_threshold, target_sizes=[rgb.shape[:2]])[0]
            boxes = r['boxes'].cpu().numpy(); scores = r['scores'].cpu().numpy()
            keep = carton_nms(boxes, scores)
            if len(keep) > MAX_TARGETS: raise ValueError('TOO_MANY_TARGETS_FOR_SINGLE_CARTON_OBSERVATION')
            return [dict(label=str(r['labels'][i]), score=float(scores[i]), bbox_xyxy=boxes[i].tolist()) for i in keep]

    def segment(self, rgb, boxes):
        """One boolean mask per box (highest predicted IoU of three) and that IoU."""
        if not len(boxes): return [], []
        torch = self.torch
        with torch.inference_mode():
            inp = self.sam_processor(images=rgb, input_boxes=[list(map(list, boxes))], return_tensors='pt').to(self.device)
            out = self.sam(**inp, multimask_output=True)
            masks = self.sam_processor.image_processor.post_process_masks(out.pred_masks.cpu(), inp['original_sizes'].cpu(), inp['reshaped_input_sizes'].cpu())[0]
            choices = out.iou_scores[0].argmax(-1).cpu().numpy()
            return ([masks[i, int(j)].numpy().astype(bool) for i, j in enumerate(choices)],
                    [float(out.iou_scores[0, i, int(choices[i])]) for i in range(len(choices))])
