"""Remote GPU detector self-test on a saved real field frame. NOT field acceptance.

Runs the configured remote_ssh backend (the exact code path the workbench uses)
on the saved RGB of one record, repeats it to separate warm latency from model
load, and compares boxes and masks with the record's own CPU inference. No
device, robot, valve or motion code is imported.
"""
import json, sys, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.perception import make_detector

from unloading_sim.workbench.ui import ROOT
record = ROOT/'outputs/single_carton'/(sys.argv[1] if len(sys.argv) > 1 else 'bb88010f39ae4abdbccfeac31472d687')
out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT/'outputs/workbench/remote_detector'/f'selftest-{record.name[:8]}.json'
out.parent.mkdir(parents=True, exist_ok=True)
config = load(ROOT/'configs/workbench/single_carton.example.json')
obs = json.loads((record/'observation.json').read_text()); old = json.loads((record/'geometry.json').read_text())
rgb = np.load(record/'observation.npz')['rgb']; saved = np.load(record/'segmentation.npz')
field = [saved[f'mask_{i}'] for i in range(len(saved.files))]
frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=rgb, stamp=SimpleNamespace(source='SAVED_RECORD_OFFLINE_SELFTEST'))
t = time.monotonic(); model = make_detector(**config['detector']); connect_s = time.monotonic()-t
runs = []
try:
    for i in range(5):
        t = time.monotonic(); detection = model.detect_boxes(frame); d = time.monotonic()-t
        t = time.monotonic(); result = model.segment(frame, detection); s = time.monotonic()-t
        runs.append(dict(detect_round_trip_s=round(d, 3), segment_round_trip_s=round(s, 3),
                         server_detect_s=round(detection.detections[0]['inference']['server_inference_s'], 3) if detection.detections else None,
                         server_segment_s=round(result.detections[0]['inference']['server_inference_s'], 3) if result.detections else None))
finally:
    info = dict(model.info or {}); model.close()
iou = lambda a, b: float((a & b).sum()/max(1, (a | b).sum()))
report = dict(scope='REMOTE_GPU_DETECTOR_SELFTEST_ON_SAVED_FIELD_FRAME_NOT_FIELD_ACCEPTANCE', record=record.name,
    backend=config['detector']['backend'], connect_verify_load_s=round(connect_s, 2),
    server=dict((k, info.get(k)) for k in ('gpu', 'torch', 'cuda', 'transformers', 'python', 'model_files_verified', 'core_sha256', 'server_sha256', 'load_s')),
    runs=runs, provenance=result.provenance,
    detections=[dict(label=x['label'], score=round(x['score'], 4), bbox_xyxy=[round(v, 2) for v in x['bbox_xyxy']]) for x in result.detections],
    field_cpu_detections=[dict(label=x['label'], score=round(x['score'], 4), bbox_xyxy=[round(v, 2) for v in x['bbox_xyxy']]) for x in old['detections']],
    mask_iou_vs_field_cpu=[round(max((iou(m, f) for f in field), default=0.), 5) for m in result.masks],
    hardware_execution='NOT_RUN')
out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
print(json.dumps(report, ensure_ascii=False))
