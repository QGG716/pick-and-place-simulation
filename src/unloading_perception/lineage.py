"""Validate pinned SAM file identities before joining geometry to proposals.

SAM instance IDs are frame-local keys, not original proposal or object IDs.
The NPZ stores validation_score in scores (not the SAM IoU prediction).
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from urllib.parse import quote

import numpy as np


def instance_identity(epoch: str, module: str, capture: str, instance: int) -> str:
    return "/".join(quote(str(v), safe="") for v in (epoch, module, capture, instance))


def _index(records: list[dict], key: str) -> dict[int, dict]:
    result = {}
    for record in records:
        value = record[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0 or value in result:
            raise ValueError(f"invalid or duplicate {key}: {value}")
        result[value] = record
    return result


@dataclass(frozen=True)
class InstanceLineage:
    identity: str
    mask_id: int
    proposal: dict
    sam: dict
    mask: np.ndarray
    audit: dict


def load_instance_lineage(
    masks_path: Path, instances_path: Path, proposals: dict, geometry: dict,
    *, sensor_epoch: str, module_id: str, capture_id: str,
    source_path: Path, proposal_path: Path,
) -> dict[int, InstanceLineage]:
    """Fail closed on corrupt joins; retain genuine NMS entity ambiguity.

Legacy pinned JSON has source/box_source paths, not capture fields. Bind those
to this capture's exact input paths and check namespace fields when present.
Callers must also verify worker artifact digests before accepting cached files.
"""
    payload = json.loads(instances_path.read_text(encoding="utf-8"))
    namespace = dict(sensor_epoch=sensor_epoch, module_id=module_id, capture_id=capture_id)
    for key, expected in (("source", source_path), ("box_source", proposal_path)):
        if Path(payload[key]).resolve() != expected.resolve():
            raise ValueError(f"SAM {key} belongs to another capture")
    for container in (payload, proposals, geometry):
        for key, value in namespace.items():
            if key in container and container[key] != value:
                raise ValueError(f"cross-capture {key}")
    originals = _index(proposals["instances"], "id")
    sam_records = _index(payload["instances"], "instance_id")
    geometry_records = _index(geometry["instances"], "mask_id")
    if not set(geometry_records).issubset(sam_records):
        raise ValueError("geometry references missing SAM instance")
    with np.load(masks_path, allow_pickle=False) as archive:
        ids = archive["mask_ids"]
        if ids.ndim != 1 or ids.dtype.kind not in "iu" or len(set(ids.tolist())) != len(ids):
            raise ValueError("invalid or duplicate NPZ mask_ids")
        if set(ids.tolist()) != set(sam_records):
            raise ValueError("NPZ/JSON mask identity mismatch")
        arrays = {key: archive[key] for key in ("masks", "boxes", "scores", "labels", "sources")}
    n = len(ids)
    if any(len(a) != n for a in arrays.values()) or arrays["masks"].ndim != 3 or arrays["boxes"].shape != (n, 4):
        raise ValueError("NPZ array shape/index mismatch")
    if not np.isin(arrays["masks"], (0, 1)).all():
        raise ValueError("NPZ masks must be binary")
    audits = payload.get("proposal_audit", [])
    results = {}
    used_proposals = set()
    for row, value in enumerate(ids):
        mask_id = int(value)
        record = sam_records[mask_id]
        for container in (record, geometry_records.get(mask_id, {})):
            for key, expected in namespace.items():
                if key in container and container[key] != expected:
                    raise ValueError(f"instance cross-capture {key}")
        proposal_id = record["proposal_id"]
        supporting = record.get("supporting_proposal_ids", [])
        joined_ids = [proposal_id, *supporting]
        if any(isinstance(i, bool) or not isinstance(i, int) or i not in originals for i in joined_ids):
            raise ValueError("missing original proposal mapping")
        if len(set(joined_ids)) != len(joined_ids) or used_proposals.intersection(joined_ids):
            raise ValueError("duplicate proposal ownership")
        used_proposals.update(joined_ids)
        if not any(a.get("status") == "accepted" and a.get("proposal_id") == proposal_id and a.get("instance_id") == mask_id for a in audits):
            raise ValueError("missing accepted proposal audit")
        for supporting_id in supporting:
            if not any(a.get("status") == "merged_duplicate" and a.get("proposal_id") == supporting_id and a.get("accepted_proposal_id") == proposal_id for a in audits):
                raise ValueError("unsupported NMS merge mapping")
        original = originals[proposal_id]
        box = np.asarray(record["bbox"], dtype=float)
        if box.shape != (4,) or not np.isfinite(box).all() or np.any(box[2:] <= box[:2]):
            raise ValueError("invalid source bbox")
        if not np.allclose(box, original["bbox"], rtol=0, atol=1e-3) or not np.allclose(box, arrays["boxes"][row], rtol=0, atol=1e-3):
            raise ValueError("NPZ/JSON/proposal boxes mismatch")
        for key in ("validation_score", "sam_iou_score", "sam_prompt_stability"):
            # SAM's IoU head is an unbounded regression output, not a probability.
            if not np.isfinite(record[key]) or (key != "sam_iou_score" and not 0 <= record[key] <= 1):
                raise ValueError(f"invalid {key}")
        if not np.isclose(arrays["scores"][row], record["validation_score"], rtol=0, atol=1e-6):
            raise ValueError("NPZ/JSON score mismatch")
        if arrays["labels"][row] != record["label"] or arrays["sources"][row] != record["boundary_source"]:
            raise ValueError("NPZ/JSON label/source mismatch")
        entity_ids = sorted({str(originals[i]["simulation_object_id"]) for i in joined_ids if originals[i].get("simulation_object_id")})
        mask = arrays["masks"][row].astype(bool)
        y, x = np.nonzero(mask)
        if not len(x) or int(mask.sum()) != record["mask_area"]:
            raise ValueError("NPZ/JSON mask area mismatch")
        identity = instance_identity(sensor_epoch, module_id, capture_id, mask_id)
        audit = {
            **namespace, "sam_instance_id": mask_id, "mask_id": mask_id,
            "geometry_instance_identity": identity, "proposal_id": proposal_id,
            "supporting_proposal_ids": supporting, "evaluation_entity_ids": entity_ids,
            "identity_status": "AMBIGUOUS_MERGED_ENTITIES" if len(entity_ids) > 1 else "RESOLVED",
            "geometry_status": "PRESENT" if mask_id in geometry_records else "MISSING",
            "original_bbox": box.tolist(),
            "mask_bbox": [int(x.min()), int(y.min()), int(x.max()+1), int(y.max()+1)],
            "sam_iou_score": record["sam_iou_score"],
            "sam_prompt_stability": record["sam_prompt_stability"],
            "validation_score": record["validation_score"],
            "proposal_source": original.get("proposal_source", "UNSPECIFIED"),
        }
        results[mask_id] = InstanceLineage(identity, mask_id, original, record, mask, audit)
    return results
