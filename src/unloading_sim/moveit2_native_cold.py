"""Strict native provenance gates, usable by the standalone pre-Isaac loader.

Only standard-library imports belong here.  Coverage is computed from the
actual returned native points; neither labels nor caller-supplied percentages
are evidence of a generated edge.
"""
from __future__ import annotations

import hashlib
import json
import math


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode()).hexdigest()


def _path(value):
    if hasattr(value, "tolist"):
        value = value.tolist()
    result = [[float(x) for x in row] for row in value]
    if not result or any(len(row) != 6 or not all(math.isfinite(x) for x in row) for row in result):
        raise ValueError("INVALID_NATIVE_AUDIT_PATH")
    return result


def audit_native_motion_coverage(path, records, *, task_id=None):
    """Bind complete native stage paths to exact ranges in the delivered path."""
    full = _path(path)
    nonzero = [i for i in range(len(full)-1) if full[i] != full[i+1]]
    covered = set()
    used, rejected = [], []
    for source in records:
        try:
            candidate = _path([point["q"] for point in source["points"]])
            times = [float(point["t"]) for point in source["points"]]
            if len(candidate) < 2 or not all(math.isfinite(t) for t in times) or any(
                    b <= a for a, b in zip(times, times[1:])):
                raise ValueError("INVALID_NATIVE_TIMES")
            for key in ("stage_id", "request_id", "solver", "input_state_sha256", "constraints_sha256"):
                if not source.get(key):
                    raise ValueError("MISSING_NATIVE_" + key.upper())
            if source.get("authoritative_status") != "PASS" or source.get("mtc_generation") is not True:
                raise ValueError("NATIVE_GENERATION_OR_AUTHORITY_NOT_PROVEN")
            if task_id is not None and source.get("task_id") != task_id:
                raise ValueError("NATIVE_TASK_ID_MISMATCH")
            if source.get("path_sha256") != _digest(candidate):
                raise ValueError("NATIVE_OUTPUT_DIGEST_MISMATCH")
            if source.get("input_state_sha256") != _digest(candidate[0]):
                raise ValueError("NATIVE_INPUT_DIGEST_MISMATCH")
            matches = [offset for offset in range(len(full)-len(candidate)+1)
                       if full[offset:offset+len(candidate)] == candidate]
            if len(matches) > 1:
                raise ValueError("AMBIGUOUS_NATIVE_STAGE_PATH_RANGE")
            if not matches:  # Valid subsolutions from unselected candidates are not final coverage.
                continue
            offset = matches[0]
            end = offset + len(candidate)-1
            if "path_range" in source and source["path_range"] != [offset, end]:
                raise ValueError("NATIVE_STAGE_RANGE_MISMATCH")
            covered.update(range(offset, end))
            used.append({**source, "path_range": [offset, end]})
        except (ValueError, KeyError, TypeError) as exc:
            rejected.append({"stage_id": source.get("stage_id"), "reason": str(exc)})
    used.sort(key=lambda item: (item["path_range"][0], item["path_range"][1]))
    uncovered = [i for i in nonzero if i not in covered]
    return {"schema": "m710_native_motion_coverage_v1", "passed": not uncovered and not rejected,
        "task_id": task_id, "path_sha256": _digest(full), "nonzero_edge_count": len(nonzero),
        "covered_nonzero_edge_count": len(nonzero)-len(uncovered),
        "coverage_fraction": (len(nonzero)-len(uncovered))/len(nonzero) if nonzero else 1.0,
        "uncovered_edges": uncovered, "rejected_records": rejected, "records": used,
        "zero_length_edge_indices": [i for i in range(len(full)-1) if i not in nonzero]}


def verify_native_cold_segment(segment):
    """Return a concrete fail-closed diagnostic, or None for a valid contract."""
    backend = segment.get("native_backend") or {}
    strict = any((segment.get("native_cold"), segment.get("require_native_motion"),
                  backend.get("native_cold"), backend.get("require_native_motion")))
    if not strict:
        return None
    def failure(reason, **details):
        return {"reason": reason, "stage": "native_cold_final_contract", **details}
    if not backend or backend.get("native_cold") is not True or backend.get("require_native_motion") is not True:
        return failure("NATIVE_COLD_BACKEND_CONTRACT_MISSING")
    if segment.get("native_cold") is not True or segment.get("require_native_motion") is not True:
        return failure("NATIVE_COLD_REQUEST_MARKER_MISSING")
    audit = backend.get("cold_audit") or {}
    for key, expected in (("history_enabled", False), ("history_inputs_read", 0),
                          ("legacy_motion_generator_calls", 0)):
        if key not in audit or type(audit[key]) is not type(expected) or audit[key] != expected:
            return failure("NATIVE_COLD_FORBIDDEN_SOURCE", field=key, observed=audit.get(key))
    task_id = backend.get("task_id")
    if not isinstance(task_id, str) or not task_id:
        return failure("NATIVE_COLD_TASK_ID_MISSING")
    try:
        coverage = audit_native_motion_coverage(segment["path"], backend.get("stages", []), task_id=task_id)
    except (ValueError, TypeError, KeyError) as exc:
        return failure("NATIVE_COLD_COVERAGE_INVALID", detail=str(exc))
    if not coverage["passed"]:
        return failure("NATIVE_COLD_MOTION_SOURCE_GAP", coverage=coverage)
    task = backend.get("mtc_task_audit") or {}
    ids = [r["stage_id"] for r in coverage["records"]]
    if (task.get("status") != "SUCCESS" or task.get("task_id") != task_id or
            task.get("generated_during_task") is not True or task.get("complete_task") is not True or
            task.get("stage_ids") != ids):
        return failure("NATIVE_COLD_COMPLETE_MTC_TASK_NOT_PROVEN", observed=task, expected_stage_ids=ids)
    if not ids:
        return failure("NATIVE_COLD_EMPTY_TASK")
    source_rows=task.get("stage_sources")
    if not isinstance(source_rows,list) or len(source_rows)!=len(ids):
        return failure("NATIVE_COLD_TASK_SOURCE_LOG_MISSING")
    previous=""
    for record,row in zip(coverage["records"],source_rows):
        if (row.get("stage_id")!=record["stage_id"] or row.get("request_id")!=record["request_id"] or
                row.get("parent_stage_id")!=previous or record.get("parent_stage_id")!=previous or
                row.get("native_solver_calls")!=record.get("native_solver_calls")):
            return failure("NATIVE_COLD_TASK_SOURCE_LOG_MISMATCH",stage_id=record["stage_id"])
        calls=record.get("native_solver_calls")
        if not isinstance(calls,dict) or not calls or any(type(v) is not int or v<0 for v in calls.values()) or sum(calls.values())<1:
            return failure("NATIVE_COLD_SOLVER_CALLS_MISSING",stage_id=record["stage_id"])
        request=record.get("submitted_request")
        if (not isinstance(request,dict) or _digest(request)!=record.get("request_fingerprint") or
                _digest({k:v for k,v in request.items() if k not in {"q_start","request_id"}})!=record["constraints_sha256"] or
                request.get("task_id")!=task_id or request.get("stage_id")!=record["stage_id"]):
            return failure("NATIVE_COLD_REQUEST_LOG_MISMATCH",stage_id=record["stage_id"])
        previous=record["stage_id"]
    return None
