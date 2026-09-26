"""Serializable stage boundary and fail-closed candidate acceptance (no GPU imports).

Schema v1 uses SI, world-from-local matrices and named scalar joints. Deadline
is a local monotonic clock value, never a timestamp to compare across hosts.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
from time import perf_counter
from typing import Callable

import numpy as np

SCHEMA_VERSION = "unloading_stage_v1"


class Status(str, Enum):
    ACCEPTED = "AUTHORITY_ACCEPTED"
    DEPENDENCY_UNAVAILABLE = "DEPENDENCY_UNAVAILABLE"
    MODEL_MISMATCH = "MODEL_MISMATCH"
    UNSUPPORTED_STAGE = "UNSUPPORTED_STAGE"
    INVALID_INPUT = "INVALID_INPUT"
    START_INVALID = "START_INVALID"
    GOAL_INVALID = "GOAL_INVALID"
    NO_CANDIDATE = "BACKEND_NO_CANDIDATE"
    BACKEND_ERROR = "BACKEND_ERROR"
    AUTHORITY_REJECTED = "AUTHORITY_REJECTED"
    RESOURCE_EXHAUSTED = "RESOURCE_EXHAUSTED"
    TIMEOUT = "TIMEOUT"
    CANCELLED = "CANCELLED"
    SCENE_STALE = "SCENE_STALE"


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def transform(value):
    m = np.asarray(value, dtype=float)
    if (m.shape != (4, 4) or not np.isfinite(m).all()
            or not np.allclose(m[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(m[:3, :3]), 1, atol=1e-8)):
        raise ValueError("invalid world-from-local SE(3)")
    return m


def pose_wxyz(value):
    """Explicit matrix -> [x,y,z,w,x,y,z], including 180 degree rotations."""
    m = transform(value)
    r = m[:3, :3]
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    xyzw = np.linalg.eigh(k)[1][:, -1]
    q = xyzw[[3, 0, 1, 2]]
    if q[0] < 0:
        q = -q
    return [*m[:3, 3].tolist(), *q.tolist()]


@dataclass(frozen=True)
class StageRequest:
    data: dict

    def __post_init__(self):
        d = deepcopy(self.data)
        fingerprint(d)  # Reject non-JSON values, NaN and infinity.
        required = {"request_id", "schema_version", "stage", "scene_revision",
                    "scene_fingerprint", "robot_model_fingerprint", "tool_fingerprint",
                    "payload_fingerprint", "collision_policy_fingerprint", "joint_names",
                    "q_start", "q_goal", "limits", "transforms", "payload", "seed",
                    "resources", "collision_policy", "goal_tolerance_rad"}
        if required - d.keys():
            raise ValueError(f"missing fields: {sorted(required - d.keys())}")
        if d["schema_version"] != SCHEMA_VERSION:
            raise ValueError("unsupported schema_version")
        names = d["joint_names"]
        if not names or any(not isinstance(n,str) or not n for n in names) or len(set(names)) != len(names):
            raise ValueError("joint names must be nonempty and unique")
        for key in ("q_start", "q_goal"):
            if np.asarray(d[key], float).shape != (len(names),):
                raise ValueError(f"invalid {key}")
        for m in d["transforms"].values():
            transform(m)
        transform(d["payload"]["flange_from_object"])
        if d["payload"]["mass_kg"] <= 0:
            raise ValueError("payload mass must be positive")
        payload=d['payload']
        dimensions=np.asarray(payload['dimensions_m'],float)
        com=np.asarray(payload['com_xyz_m'],float)
        inertia=np.asarray(payload['inertia_tensor_com_kg_m2'],float)
        if (not payload['object_id'] or dimensions.shape!=(3,) or np.any(dimensions<=0)
                or com.shape!=(3,) or inertia.shape!=(3,3)
                or not np.allclose(inertia,inertia.T,atol=1e-12)
                or np.min(np.linalg.eigvalsh(inertia))<=0):
            raise ValueError('invalid payload geometry or physical inertia')
        if type(d['seed']) is not int or not 0<=d['seed']<2**32:
            raise ValueError('invalid deterministic seed')
        for name in ('boundary_velocity','boundary_acceleration'):
            if d.get(name) is not None and np.asarray(d[name],float).shape!=(2,len(names)):
                raise ValueError('invalid '+name)
        limits = d["limits"]
        for key in ("lower", "upper", "velocity", "effort", "acceleration", "jerk"):
            if np.asarray(limits[key]).shape != (len(names),):
                raise ValueError(f"invalid limits: {key}")
        if np.any(np.asarray(limits["lower"]) >= limits["upper"]):
            raise ValueError("invalid position limits")
        if np.any(np.asarray(limits["velocity"]) <= 0) or np.any(np.asarray(limits["effort"]) <= 0):
            raise ValueError("finite positive velocity and effort required")
        if any(np.any(np.asarray(limits[key])<=0) for key in ('acceleration','jerk')):
            raise ValueError('positive finite derivative limits required')
        if not 0 < d["goal_tolerance_rad"] <= .01:
            raise ValueError("invalid endpoint tolerance")
        for key in ("attempts", "num_seeds"):
            if type(d["resources"][key]) is not int or not 1 <= d["resources"][key] <= 64:
                raise ValueError(f"invalid finite resource: {key}")
        object.__setattr__(self, "data", d)

    def to_dict(self):
        return deepcopy(self.data)


def trajectory_failure(request, trajectory):
    d = request.data
    try:
        q = np.asarray(trajectory["q"], float)
        t = np.asarray(trajectory["time_s"], float)
        if trajectory["joint_names"] != d["joint_names"]:
            return {"reason": "JOINT_ORDER_MISMATCH"}
        if (q.ndim != 2 or q.shape[1] != len(d["joint_names"]) or len(q) < 2
                or t.shape != (len(q),) or not np.isfinite(q).all() or not np.isfinite(t).all()
                or t[0] != 0 or np.any(np.diff(t) <= 0)
                or trajectory["valid_length"] != len(q)):
            return {"reason": "INVALID_TRAJECTORY_SHAPE_OR_TIME"}
        if trajectory["interpolation"] != "linear_joint_samples":
            return {"reason": "UNSUPPORTED_EXECUTION_INTERPOLATION"}
        if any(np.max(np.abs(q[i] - d[key])) > d["goal_tolerance_rad"]
               for i, key in ((0, "q_start"), (-1, "q_goal"))):
            return {"reason": "ENDPOINT_MISMATCH"}
        if np.any(q < np.asarray(d["limits"]["lower"]) - 1e-9) or np.any(q > np.asarray(d["limits"]["upper"]) + 1e-9):
            return {"reason": "JOINT_LIMIT"}
        speed = np.abs(np.diff(q, axis=0) / np.diff(t)[:, None])
        if np.any(speed > np.asarray(d["limits"]["velocity"]) + 1e-5):
            return {"reason": "REFERENCE_VELOCITY_LIMIT"}
        for key in ("dq", "ddq"):
            if trajectory.get(key) is not None:
                arr = np.asarray(trajectory[key], float)
                if arr.shape != q.shape or not np.isfinite(arr).all():
                    return {"reason": "INVALID_DERIVATIVE", "field": key}
                bound = d['limits'].get('velocity' if key == 'dq' else 'acceleration')
                if bound is not None and np.any(np.abs(arr) > np.asarray(bound) + 1e-5):
                    return {"reason": "DERIVATIVE_LIMIT", "field": key}
    except (KeyError, ValueError, TypeError) as exc:
        return {"reason": "INVALID_TRAJECTORY", "detail": str(exc)}
    return None


def run_stage(request: StageRequest, candidate: Callable, authority: Callable,
              state_authority: Callable, current_revision: Callable, cancelled=lambda: False):
    """Serial ownership; rejected candidates consume resources but do not early-exit.

    Cancellation drains an in-flight synchronous call then discards its result.
    This does not claim to interrupt CUDA kernels. No implicit CPU fallback.
    """
    start = perf_counter()
    d = request.data
    result = dict(schema_version=SCHEMA_VERSION, request=request.to_dict(),
                  request_fingerprint=fingerprint(d), status=None, attempts=[], trajectory=None,
                  candidate_generated=False, authority_accepted=False, delivered=False,
                  isaac_execution_completed=False, fallback=False, actual_backend=None,
                  timings={"authority_s": 0., "end_to_end_s": None}, first_failure=None,
                  cancellation_semantics="drain_inflight_then_discard_no_kernel_interrupt")

    def obsolete():
        if cancelled():
            return Status.CANCELLED.value
        if d.get("deadline_monotonic") is not None and perf_counter() >= d["deadline_monotonic"]:
            return Status.TIMEOUT.value
        if current_revision() != (d["scene_revision"], d["scene_fingerprint"]):
            return Status.SCENE_STALE.value
        return None

    def finish(status):
        result["status"] = status
        result["timings"]["end_to_end_s"] = perf_counter() - start
        return result

    if d["stage"] != "TRANSIT":
        return finish(Status.UNSUPPORTED_STAGE.value)
    if stale := obsolete():
        return finish(stale)
    for key, status in (("q_start", Status.START_INVALID), ("q_goal", Status.GOAL_INVALID)):
        before = perf_counter()
        failure = state_authority(d[key])
        result["timings"]["authority_s"] += perf_counter() - before
        if failure:
            result["first_failure"] = failure
            return finish(status.value)
    rejected = set()
    for attempt in range(d["resources"]["attempts"]):
        if stale := obsolete():
            return finish(stale)
        try:
            native = candidate(request, attempt)
        except (ImportError, FileNotFoundError) as exc:
            native=dict(status=Status.DEPENDENCY_UNAVAILABLE.value,trajectory=None,
                        error=dict(type=type(exc).__name__,message=str(exc)))
        except Exception as exc:
            native=dict(status=Status.BACKEND_ERROR.value,trajectory=None,
                        error=dict(type=type(exc).__name__,message=str(exc)))
        result["attempts"].append(native)
        result["actual_backend"] = native.get("backend")
        if stale := obsolete():
            return finish(stale)
        if native["status"] in {Status.DEPENDENCY_UNAVAILABLE.value, Status.MODEL_MISMATCH.value, Status.BACKEND_ERROR.value}:
            result["first_failure"] = native.get("error")
            return finish(native["status"])
        trajectory = native.get("trajectory")
        if trajectory is None:
            result["first_failure"] = result["first_failure"] or native.get("error")
            continue
        result["candidate_generated"] = True
        key = fingerprint(trajectory["q"])
        if key in rejected:
            native["authority"] = {"reason": "DUPLICATE_REJECTED_CANDIDATE"}
            continue
        before = perf_counter()
        failure = trajectory_failure(request, trajectory) or authority(trajectory)
        result["timings"]["authority_s"] += perf_counter() - before
        native["authority"] = failure or {"accepted": True}
        if failure:
            result["first_failure"] = result["first_failure"] or failure
            rejected.add(key)
            continue
        if stale := obsolete():
            return finish(stale)
        result.update(trajectory=trajectory, authority_accepted=True)
        return finish(Status.ACCEPTED.value)
    return finish(Status.AUTHORITY_REJECTED.value if result["candidate_generated"] else Status.RESOURCE_EXHAUSTED.value)
