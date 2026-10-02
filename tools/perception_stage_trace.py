"""Optional stage-boundary snapshots. Never changes arrays, RNG or algorithms."""
from dataclasses import asdict
from pathlib import Path
import hashlib
import os
import tempfile
from time import perf_counter
import numpy as np
from unloading_perception.finite_sequence import atomic_json


def reference(path):
    path = Path(path)
    return {'path': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


class StageTrace:
    def __init__(self, directory, *, source, masks, metadata, config):
        self.directory = Path(directory)
        self.directory.mkdir(exist_ok=False)
        self.report = dict(schema='perception_stage_trace_v1', status='RUNNING',
            capture_id=metadata.capture_id, camera_frame=metadata.rgb_frame_id,
            calibration_identity=metadata.calibration_identity,
            source=reference(source), masks=reference(masks), config=asdict(config),
            events=[], observer_seconds=0., clock='perf_counter elapsed seconds, not source time')
        self.save()

    def save(self):
        atomic_json(self.directory/'index.json', self.report)

    def extracted(self, identity, labels, seeds, audit, seconds):
        start = perf_counter()
        path = self.directory/f'mask-{identity}-initial.npz'
        with tempfile.NamedTemporaryFile(dir=self.directory, suffix='.npz', delete=False) as f:
            temporary = Path(f.name)
        try:
            np.savez_compressed(temporary, labels=labels)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        document = self.directory/f'mask-{identity}-initial.json'
        atomic_json(document, dict(mask_id=identity, planes=seeds, audit=audit,
            labels=reference(path), capture_id=self.report['capture_id'],
            description='Actual extract_observation_labels return, captured before fit_metric_faces'))
        self.report['events'].append(dict(mask_id=identity, stage='initial_planes', seconds=seconds,
                                         result=reference(document)))
        self.report['observer_seconds'] += perf_counter()-start
        self.save()

    def fitted(self, identity, record, seconds):
        start = perf_counter()
        path = self.directory/f'mask-{identity}-fitted.json'
        atomic_json(path, record)
        self.report['events'].append(dict(mask_id=identity, stage='fitted_and_independently_validated',
                                         seconds=seconds, result=reference(path)))
        self.report['observer_seconds'] += perf_counter()-start
        self.save()

    def complete(self, ids):
        actual=[e['mask_id'] for e in self.report['events'] if e['stage']=='fitted_and_independently_validated']
        if ids != actual or len(ids)!=len(set(ids)):
            raise ValueError('TRACE_INSTANCE_ROSTER_MISMATCH')
        self.report.update(status='COMPLETED', mask_ids=ids)
        self.save()
