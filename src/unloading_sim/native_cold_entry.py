"""Strict ordinary-entry contracts; no historical solution loaders."""
from __future__ import annotations

import math
import json
import hashlib
from pathlib import Path
from contextvars import ContextVar
from functools import wraps

_active_trace = ContextVar("native_cold_run_trace", default=None)


def bind_native_run(connector):
    trace = _active_trace.get()
    if trace is not None:
        trace["connector"] = connector


def preserve_native_failure(function):
    """Keep actual worker/authority records even if the ordinary entry raises."""
    @wraps(function)
    def run(*args, **kwargs):
        if not kwargs.get("native_cold", False):
            return function(*args, **kwargs)
        trace = {}
        token = _active_trace.set(trace)
        try:
            return function(*args, **kwargs)
        except BaseException as exc:
            connector = trace.get("connector")
            if connector is not None:
                exc.native_cold_failure_evidence = {
                    "audit": connector.native_cold_evidence(),
                    "native_stages": connector.native_evidence,
                    "native_ik": connector.native_ik_evidence,
                    "authority_paths": connector.authority_path_evidence,
                }
            raise
        finally:
            try:
                if "connector" in trace:
                    trace["connector"].native.close()
            finally:
                _active_trace.reset(token)
    return run


DEFAULT_NATIVE_BUDGET = {
    "task_wall_time_s": 3600.0,
    "stage_wall_time_s": 300.0,
    "ipc_timeout_s": 360.0,
}


def native_budget_config(value=None):
    supplied = dict(value or {})
    if set(supplied) - set(DEFAULT_NATIVE_BUDGET):
        raise ValueError("NATIVE_COLD_UNKNOWN_BUDGET")
    result = {**DEFAULT_NATIVE_BUDGET, **supplied}
    for name, number in result.items():
        if isinstance(number, bool) or not math.isfinite(float(number)) or float(number) <= 0:
            raise ValueError("NATIVE_COLD_INVALID_BUDGET: " + name)
        result[name] = float(number)
    if result["ipc_timeout_s"] <= result["stage_wall_time_s"]:
        raise ValueError("NATIVE_COLD_IPC_BUDGET_SHORTER_THAN_STAGE")
    return result


def validate_native_cold_inputs(policy, *, backend, target_id, fixed_history_fixture=None):
    """Reject forbidden sources before constructing even a history reader."""
    if backend != "moveit2":
        raise ValueError("NATIVE_COLD_REQUIRES_MOVEIT2")
    if target_id != "carton_l07_c02":
        raise ValueError("NATIVE_COLD_REQUIRES_TARGET_carton_l07_c02")
    if fixed_history_fixture is not None:
        raise ValueError("NATIVE_COLD_FORBIDDEN_HISTORY_FIXTURE")
    strategy = policy.get("search_strategy", {})
    history = strategy.get("history", {}) or {}
    forbidden = [name for name in ("source", "register_directory") if history.get(name) is not None]
    forbidden += [name for name in ("history_hint", "roadmap", "experience_database",
                                   "trajectory_cache", "reuse_motion", "fixed_history_fixture")
                  if strategy.get(name) is not None]
    if forbidden:
        raise ValueError("NATIVE_COLD_FORBIDDEN_INPUT: " + ",".join(forbidden))


def read_native_initial_state(actual_path, ready_path):
    """Read only the freshly retained world's identity-bound initial snapshot."""
    if actual_path is None or ready_path is None:
        raise ValueError("NATIVE_COLD_REQUIRES_ACTUAL_STATE_AND_INITIAL_READY")
    ready = json.loads(Path(ready_path).read_text(encoding="utf-8"))
    if (ready.get("status") != "INITIAL_WORLD_RETAINED_AWAITING_NATIVE_PLAN"
            or ready.get("completed_segments") != 0 or ready.get("target") != "carton_l07_c02"
            or ready.get("physics_time_paused_for_offline_planning") is not True
            or ready.get("motion_execution_permitted") is not False):
        raise ValueError("NATIVE_COLD_INITIAL_WORLD_NOT_READY")
    raw = Path(actual_path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != ready.get("actual_state_sha256"):
        raise ValueError("NATIVE_COLD_INITIAL_STATE_HASH_MISMATCH")
    state = json.loads(raw)
    if (state.get("schema") != "m710id70_actual_motion_state_v1"
            or not state.get("world_session_id")
            or state["world_session_id"] != ready.get("world_session_id")
            or state.get("attached") is not False
            or len(state.get("cartons", [])) != 40
            or state.get("initialization_provenance", {}).get("world_scope") != "NEW_WORLD_NATIVE_COLD"):
        raise ValueError("NATIVE_COLD_INVALID_ACTUAL_INITIAL_STATE")
    if any(state.get(key) for key in ("completed_carton_ids", "processed_carton_ids",
            "ideal_received_ids", "handed_off_ids", "inactive_carton_ids", "receiver_transport_state",
            "attachment_target", "attachments", "path", "joint_path", "trajectory", "history_hint")):
        raise ValueError("NATIVE_COLD_INITIAL_STATE_CONTAINS_HISTORY_OR_ATTACHMENT")
    return state, raw


class DisabledHistorySource:
    """An auditable absence of a loader, rather than a reader with empty data."""
    def evidence(self):
        return {"source": None, "status": "DISABLED_NATIVE_COLD", "files": [],
                "history_enabled": False, "history_inputs_read": 0,
                "screening_seconds": 0.0, "old_validation_inherited": False}

    def candidates(self, *args, **kwargs):
        raise RuntimeError("NATIVE_COLD_HISTORY_ENTRY_FORBIDDEN")
