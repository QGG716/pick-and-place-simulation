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


STACK_CHILD_INSIDE = .90     # each child box at least 90 % inside the parent box
STACK_UNION_COVERAGE = .60   # children's union covers most of the parent box area


def _area(b):
    return max(0., b[2]-b[0])*max(0., b[3]-b[1])


def _union_area(boxes):
    """Exact area of a union of axis-aligned rectangles (coordinate compression)."""
    if not len(boxes): return 0.
    xs = sorted({float(v) for b in boxes for v in (b[0], b[2])}); ys = sorted({float(v) for b in boxes for v in (b[1], b[3])})
    total = 0.
    for x0, x1 in zip(xs, xs[1:]):
        for y0, y1 in zip(ys, ys[1:]):
            if any(b[0] <= x0 and x1 <= b[2] and b[1] <= y0 and y1 <= b[3] for b in boxes): total += (x1-x0)*(y1-y0)
    return total


def carton_nms(boxes, scores, threshold=.98, record=None):
    """Class-agnostic NMS plus whole-stack box suppression.

    'box' and 'carton' can name the same physical object (IoU >= threshold).
    A box that contains two or more kept boxes (each >= 90 % inside it) whose
    union covers most of its area is a stack/group box, not one carton: it is
    dropped and, when ``record`` is a list, appended there. A box containing
    only one other box is kept; nested-mask fitting handles that case.
    """
    boxes = np.asarray(boxes, float); scores = np.asarray(scores, float); keep = []
    for i in np.argsort(-scores, kind='stable'):
        a = boxes[i]; area = _area(a)
        if area <= 0 or not np.isfinite(a).all(): continue
        duplicate = False
        for j in keep:
            b = boxes[j]; overlap = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
            other = _area(b)
            if overlap/(area+other-overlap) >= threshold: duplicate = True; break
        if not duplicate: keep.append(int(i))
    dropped = set()
    for i in keep:
        a = boxes[i]; area = _area(a); children = []
        for j in keep:
            if j == i: continue
            b = boxes[j]; inside = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
            if _area(b) < area and inside/max(_area(b), 1e-9) >= STACK_CHILD_INSIDE: children.append(j)
        if len(children) < 2: continue
        clipped = [[max(a[0], boxes[j][0]), max(a[1], boxes[j][1]), min(a[2], boxes[j][2]), min(a[3], boxes[j][3])] for j in children]
        coverage = _union_area(clipped)/area
        if coverage >= STACK_UNION_COVERAGE:
            dropped.add(i)
            if record is not None:
                record.append(dict(index=int(i), bbox_xyxy=a.tolist(), score=float(scores[i]), children=[int(j) for j in children],
                                   children_union_coverage=float(coverage), reason='STACK_BOX_CONTAINS_TWO_OR_MORE_CARTONS',
                                   child_inside_min=STACK_CHILD_INSIDE, union_coverage_min=STACK_UNION_COVERAGE))
    return [i for i in keep if i not in dropped]


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
            suppressed = []; keep = carton_nms(boxes, scores, record=suppressed)
            if len(keep) > MAX_TARGETS: raise ValueError('TOO_MANY_TARGETS_FOR_SINGLE_CARTON_OBSERVATION')
            for item in suppressed: item['label'] = str(r['labels'][item['index']])
            return [dict(label=str(r['labels'][i]), score=float(scores[i]), bbox_xyxy=boxes[i].tolist(),
                         suppressed_stack_boxes=suppressed) for i in keep]

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
