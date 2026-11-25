"""Offline end-to-end check of observation_mode=CONTINUOUS_STANDING (+ scene_build).

Feeds one saved field record through the production TaskManager standing
observation: fake camera frames (the record's RGB-D re-stamped every 33 ms in
this process, labelled SYNTHETIC_IN_PROCESS_LIVE_STAMP), a fake stationary
joint stream at the record's q_start, the real continuous LiveVision with a
fake model that returns the record's own detections and SAM masks after a
configurable inference delay, the real planner build, self filter, scene
precompute child, box recovery, site scene and layout registration. It times
gate -> installed standing observation (cold start, then an operator refresh)
and binds the last one to a task record in a scratch record root.

Development diagnostic only: no device, SDK, serial or camera import is
opened, no permit is issued, no motion or valve command exists in this path.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.contracts import RGBDFrame, DeviceTime, Stamp, Source, DetectionResult
from unloading_sim.workbench.single_carton.config import load, observation_mode, scene_build
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.live import LiveVision
from unloading_sim.workbench.single_carton.runtime import TaskManager
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel


class RecordModel:
    """Fake detector: the record's own detections and masks, after a fixed delay."""
    def __init__(self, detections, masks, detect_s, segment_s):
        self.detections, self.masks, self.detect_s, self.segment_s = detections, masks, detect_s, segment_s
        self.description = 'saved record masks (offline)'; self.calls = 0
    def detect_boxes(self, f):
        time.sleep(self.detect_s); self.calls += 1
        return DetectionResult(f.observation_id, f.stamp.source, time.monotonic_ns(), tuple(self.detections), ())
    def segment(self, f, d):
        time.sleep(self.segment_s)
        return replace(d, masks=tuple(self.masks))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--detect-s', type=float, default=1.2)
    parser.add_argument('--segment-s', type=float, default=.35)
    parser.add_argument('--timeout-s', type=float, default=180)
    args = parser.parse_args()
    record = Path(args.record)
    if not record.is_dir(): record = ROOT/'outputs/single_carton'/args.record
    config = load(args.config)
    if observation_mode(config)['mode'] != 'CONTINUOUS_STANDING': raise SystemExit('CONFIG_IS_NOT_CONTINUOUS_STANDING')
    args.output.mkdir(parents=True, exist_ok=False)
    config['record_root'] = str((args.output/'records').resolve())
    obs = json.loads((record/'observation.json').read_text())
    with np.load(record/'observation.npz', allow_pickle=False) as saved: rgb, depth = saved['rgb'], saved['depth']
    with np.load(record/'segmentation.npz', allow_pickle=False) as saved:
        masks = [saved[f'mask_{i}'].astype(bool) for i in range(len(saved.files))]
    detections = json.loads((record/'detections.json').read_text())['detections']
    installation = Installation.load(config['execution']['installation'])
    real = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    # The record's own camera pose (its calibration and mast at capture time).
    model = SimpleNamespace(camera=lambda q, c: {'T_world_camera': np.asarray(obs['T_world_camera'], float)},
                            tcp=real.tcp, frames=real.frames, world_from_base=real.world_from_base)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), real)
    q_start = list(obs['q_start'])
    counter = [0]; lock = threading.Lock()

    class Backend:
        observation_generation = 1
        vision = None
        def __init__(self): self.current = None
        @property
        def frame(self): return self.current
        def sample_state(self, stamp, device): return 'LIVE_VALID'
    backend = Backend()
    stop = threading.Event()

    def camera():
        while not stop.wait(1/30):
            with lock:
                counter[0] += 1; n = counter[0]
            backend.current = RGBDFrame(f'offline-{record.name[:8]}-{n}', 'OFFLINE_REPLAY', dict(obs['stream']), n, n,
                                        DeviceTime(n*33.3, 'camera'), DeviceTime(n*33.3, 'camera'), Stamp.receive(Source.LIVE),
                                        obs['intrinsics'], obs['depth_scale_m'], rgb, depth)
            if backend.vision is not None: backend.vision.offer(backend.current, None, 1)
    sequence = [0]
    def snapshot():
        sequence[0] += 1
        return dict(source='LIVE', q=q_start, received_s=time.monotonic(), sequence=sequence[0], alarm=False, enabled=True,
                    velocity=[0.]*6, velocity_source='MEASURED_UDP_SPEED')
    service = SimpleNamespace(snapshot=snapshot, request_stop=lambda: None, close=lambda: None)
    events = []
    t0 = time.monotonic()
    manager = TaskManager(backend, service, SimpleNamespace(holding=False, inhibit=lambda: None), planner, model,
                          SimpleNamespace(parent_frame='link_1'), config,
                          lambda kind, value: events.append((round(time.monotonic()-t0, 3), kind)))
    fake = RecordModel(detections, masks, args.detect_s, args.segment_s)
    vision = LiveVision({}, factory=lambda **_: fake); backend.vision = vision; vision.start()
    threading.Thread(target=camera, daemon=True).start()
    manager.start_standing()
    report = dict(scope='OFFLINE_STANDING_OBSERVATION_CHECK_NOT_EXECUTABLE', record=str(record), config=str(args.config),
                  observation_mode=observation_mode(config), scene_build=scene_build(config),
                  frame_stamp='SYNTHETIC_IN_PROCESS_LIVE_STAMP', detect_s=args.detect_s, segment_s=args.segment_s,
                  hardware_execution='NOT_RUN', physical_result='NOT_RUN', permits_issued=0, runs=[])
    try:
        for label in ('cold_start', 'operator_refresh', 'operator_refresh_2'):
            if label != 'cold_start':
                manager.request_standing_refresh()
            started = time.monotonic(); seen = None; deadline = started+args.timeout_s
            while time.monotonic() < deadline:
                s = manager.standing
                if s is not None and (seen is None or s is not seen) and s.standing_id >= (1 if label == 'cold_start' else 2 if label == 'operator_refresh' else 3):
                    seen = s; break
                time.sleep(.01)
            if seen is None: raise SystemExit(f'NO_STANDING_OBSERVATION_{label}: {manager.standing_status()} / {manager.reason} / {manager.prewarm_error}')
            report['runs'].append(dict(label=label, request_to_installed_s=round(time.monotonic()-started, 3),
                build_s=round(seen.build_s, 3), phase=seen.phase, boxes=len(seen.boxes), rejected=len(seen.rejected),
                standing_id=seen.standing_id, world_objects=len((seen.scene or {}).get('world', []))))
            print(json.dumps(report['runs'][-1], ensure_ascii=False), flush=True)
        manager._enter()
        try:
            bind_started = time.monotonic(); bound = manager._bind_standing(); bind_s = time.monotonic()-bind_started
            manager.evidence_flush(); manager.journal.flush()
        finally:
            manager.busy = False
        report.update(bind_s=round(bind_s, 3), bound_task=manager.task_id, bound_files=sorted(p.name for p in manager.folder.iterdir()),
                      bound_phase=bound.phase, vision_inferences=fake.calls, events=events[-12:])
    finally:
        stop.set(); vision.close()
        try: manager.close()
        except Exception as e: report['close'] = str(e)
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    print(json.dumps({k: v for k, v in report.items() if k not in ('events',)}, ensure_ascii=False, default=str))


if __name__ == '__main__':
    main()
