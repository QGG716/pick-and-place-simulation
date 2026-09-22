"""Preserve native timing as edge-duration lower bounds in the C2 executor.

The existing controller uses stopped quintics, not ROS trajectory splines.
Changing that time law is explicit: no joint edge changes, no native edge is
sped up, and the existing analytic velocity/acceleration/jerk audit still runs.
"""
import numpy as np

from .timing import _quintic_from_durations


def native_timing_floor(path, records):
    path = np.asarray(path, dtype=float)
    floors = np.zeros(len(path)-1)
    used = []
    for record in records:
        points = record["points"]
        candidate = np.asarray([p["q"] for p in points], dtype=float)
        times = np.asarray([p["t"] for p in points], dtype=float)
        if len(candidate)<2 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
            raise ValueError("INVALID_NATIVE_TIMES")
        for offset in range(len(path)-len(candidate)+1):
            if np.array_equal(path[offset:offset+len(candidate)],candidate):
                last=offset+len(candidate)-1
                floors[offset:last]=np.maximum(floors[offset:last],np.diff(times))
                used.append({**record,"path_range":[offset,last]})
                break
    return floors.tolist(),used


def preserve_native_durations(trajectory, segment, source_indices):
    if "native_backend" not in segment:
        return trajectory
    floors=np.asarray(segment["native_backend"]["minimum_edge_seconds"],dtype=float)
    if floors.shape!=(len(segment["path"])-1,) or not np.isfinite(floors).all() or np.any(floors<0):
        raise ValueError("INVALID_NATIVE_DURATION_FLOOR")
    indices=np.asarray(source_indices,dtype=int)
    if len(indices)!=len(trajectory.positions) or np.any(np.diff(indices)<=0):
        raise ValueError("NATIVE_DURATION_INDEX_MISMATCH")
    minimum=np.array([np.sum(floors[a:b]) for a,b in zip(indices[:-1],indices[1:])])
    durations=np.maximum(np.diff(trajectory.time_from_start),minimum)
    if len(durations)==0: return trajectory
    return _quintic_from_durations(trajectory.positions,durations,trajectory.iterations)
