"""scene_build=SPLIT_PARALLEL (user decision 2026-11-25): the mask-independent
part of the site scene in one resident child process.

site_scene.precompute_scene (voxel free space from depth, robot self-ownership
of every valid return and every voxel at q_capture) runs here as soon as a
frame is frozen, while detection and SAM segmentation run on the same frame.
The child holds its own RobotSelfFilter built from the same robot description
and coordinate model; the function and its inputs are identical, so the
returned arrays are the ones build_scene would compute itself (equivalence:
tools/check_scene_split_equivalence.py). Spawned, not forked, and never given
any device handle: nothing here can move the robot or write a valve.
"""
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor
from types import SimpleNamespace
import numpy as np

_FILTER = None


def _init(robot_description, model):
    global _FILTER
    from .self_filter import RobotSelfFilter
    _FILTER = RobotSelfFilter(robot_description, model) if robot_description is not None else None


def _ready():
    return os.getpid(), _FILTER is not None


def _precompute(frame, world_camera, q_capture, site):
    from .site_scene import precompute_scene
    return precompute_scene(frame, world_camera, q_capture, site, _FILTER)


def frame_inputs(frame):
    """Only what precompute_scene reads (metric_points validates RGB with depth); no device stamp."""
    return SimpleNamespace(observation_id=frame.observation_id, rgb=np.asarray(frame.rgb), depth=np.asarray(frame.depth),
                           depth_scale_m=frame.depth_scale_m, intrinsics=dict(frame.intrinsics))


class ScenePrecompute:
    """One resident spawn child; submit() returns a Future of precompute_scene's dict."""
    def __init__(self, robot_description, model, *, timeout_s=120):
        self.executor = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                            initializer=_init, initargs=(robot_description, model))
        self.pid, self.self_filter_applied = self.executor.submit(_ready).result(timeout=timeout_s)

    def submit(self, frame, world_camera, q_capture, site):
        return self.executor.submit(_precompute, frame_inputs(frame), world_camera, tuple(q_capture), site)

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
