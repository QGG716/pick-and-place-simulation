"""Small RGB proposal contract. No detector, GT, depth or ROS dependencies."""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import numpy as np

DETECTOR_SOURCE = 'GROUNDING_DINO'
ORACLE_SOURCE = 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL'


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class DetectorConfig:
    text: str = 'cardboard box.'
    box_threshold: float = .25
    text_threshold: float = .25
    duplicate_iou: float = .9
    shortest_edge: int = 800
    longest_edge: int = 1333
    max_candidates: int = 300
    seed: int = 17

    def __post_init__(self):
        if not isinstance(self.text, str) or not self.text.strip():
            raise ValueError('empty detector text')
        for name in ('box_threshold', 'text_threshold', 'duplicate_iou'):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError('invalid detector threshold: ' + name)
        if (self.shortest_edge <= 0 or self.longest_edge < self.shortest_edge or self.max_candidates <= 0
                or any(type(getattr(self, k)) is not int for k in ('shortest_edge', 'longest_edge', 'max_candidates', 'seed'))):
            raise ValueError('invalid detector size/count/seed')

    def to_dict(self):
        return asdict(self)


def box_iou(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    overlap = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])).prod()
    return float(overlap / max(1e-12, (a[2:]-a[:2]).prod() + (b[2:]-b[:2]).prod() - overlap))


def filter_candidates(boxes, scores, labels, width, height, config):
    """Only near-identical boxes are duplicates; no object-count prior or filling."""
    if not (len(boxes) == len(scores) == len(labels)) or width <= 0 or height <= 0:
        raise ValueError('invalid candidate/image dimensions')
    rows, eligible, kept = [], [], []
    for index, (box, score, label) in enumerate(zip(boxes, scores, labels)):
        box, score = np.asarray(box, float), float(score)
        row = dict(proposal_id=index + 1, raw_text_label=str(label), detector_score=score if np.isfinite(score) else None,
                   original_bbox_xyxy=[float(v) if np.isfinite(v) else None for v in box], status='REJECTED')
        rows.append(row)
        if box.shape != (4,) or not np.isfinite(box).all() or not np.isfinite(score):
            row['reason'] = 'NONFINITE_OR_INVALID_BOX_SCORE'; continue
        if np.any(box[2:] <= box[:2]):
            row['reason'] = 'DEGENERATE_BOX'; continue
        clipped = np.clip(box, [0, 0, 0, 0], [width, height, width, height])
        row.update(bbox_xyxy=clipped.tolist(), clipped=not np.array_equal(clipped, box))
        if np.any(clipped[2:] <= clipped[:2]):
            row['reason'] = 'DEGENERATE_AFTER_CLIP'; continue
        if score <= config.box_threshold:
            row['reason'] = 'BELOW_BOX_THRESHOLD'; continue
        if not str(label).strip():
            row['reason'] = 'EMPTY_TEXT_LABEL'; continue
        eligible.append(row)
    for row in sorted(eligible, key=lambda r: (-r['detector_score'], r['proposal_id'])):
        duplicate = next((k for k in kept if box_iou(k['bbox_xyxy'], row['bbox_xyxy']) >= config.duplicate_iou), None)
        if duplicate is not None:
            row.update(reason='NEAR_IDENTICAL_BOX', suppressed_by=duplicate['proposal_id']); continue
        if len(kept) >= config.max_candidates:
            row['reason'] = 'CANDIDATE_LIMIT'; continue
        row.update(status='KEPT', reason='ACCEPTED'); kept.append(row)
    return rows, sorted(kept, key=lambda r: r['proposal_id'])


def validate_detector_document(document, *, rgb_sha256, width, height, capture_id, module_id,
                               sensor_epoch, frame_sequence, config_identity=None):
    expected = dict(rgb_sha256=rgb_sha256, source_size=[width, height], capture_id=capture_id,
                    module_id=module_id, sensor_epoch=sensor_epoch, frame_sequence=frame_sequence,
                    source=DETECTOR_SOURCE, coordinate_space='source_image')
    if any(document.get(k) != v for k, v in expected.items()):
        raise ValueError('DETECTOR_PROPOSAL_BINDING_MISMATCH')
    actual = fingerprint(document['detector']['config'])
    if document['detector']['config_identity'] != actual or config_identity not in (None, actual):
        raise ValueError('DETECTOR_CONFIG_MISMATCH')
    seen = set()
    for item in document['instances']:
        box = np.asarray(item['bbox'], float)
        if (type(item['id']) is not int or item['id'] in seen or item['proposal_id'] != item['id']
                or box.shape != (4,) or not np.isfinite(box).all() or np.any(box[2:] <= box[:2])
                or np.any(box < 0) or np.any(box > [width, height, width, height])
                or item['bbox_xyxy'] != item['bbox'] or 'simulation_object_id' in item
                or item['proposal_source'] != DETECTOR_SOURCE):
            raise ValueError('INVALID_DETECTOR_PROPOSAL')
        seen.add(item['id'])


def proposal_path(folder):
    """New results select an explicit reference; legacy archives remain untouched."""
    folder = Path(folder)
    reference = folder/'proposal_binding.json'
    if reference.exists():
        item = json.loads(reference.read_text(encoding='utf-8'))
        if item['filename'] not in ('detector_proposals.json', 'oracle_proposals.json'):
            raise ValueError('INVALID_PROPOSAL_REFERENCE')
        path = folder/item['filename']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('PROPOSAL_REFERENCE_HASH_MISMATCH')
        return path
    # Compatibility for immutable historical archives, never detector failure recovery.
    if (folder/'detector_proposals.json').exists():
        raise ValueError('DETECTOR_PROPOSAL_REFERENCE_MISSING')
    return folder/'oracle_proposals.json'
