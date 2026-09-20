"""Bounded latest-frame scheduling and continuous-recording identity checks."""
from pathlib import Path
from .finite_sequence import read_json
from .isaac_validation import sha256_file


class LatestFrameSlot:
    def __init__(self): self.pending=None; self.dropped=0
    def offer(self,frame):
        if self.pending is not None: self.dropped+=1
        self.pending=frame
    def take(self):
        frame,self.pending=self.pending,None
        return frame


def worker_failure(result):
    """Retain both shared initialization errors and per-module technical errors."""
    summary = result.get('summary', {})
    errors = [row.get('error', '') for row in summary.get('runs', [])
              if row.get('status') == 'TECHNICAL_FAILURE']
    errors += [f"{row.get('stage', 'run')}: {row.get('error_type', '')}: {row.get('error', '')}"
               for row in summary.get('errors', [])]
    return result.get('error') or '; '.join(filter(None, errors)) or 'algorithm failed'


def recording_frames(path):
    path=Path(path).resolve()
    record=read_json(path)
    if record['schema_version']!='continuous_rgbd_recording_v1' or len(record['frames'])<2:
        raise ValueError('a genuine continuous dual RGB-D recording is required')
    previous=(-1,float('-inf')); frames=[]
    for row in record['frames']:
        folder=(path.parent/row['path']).resolve()
        if not folder.is_relative_to(path.parent): raise ValueError('recording path escapes source')
        if sha256_file(folder/'manifest.json')!=row['manifest_sha256']: raise ValueError('recording manifest hash mismatch')
        manifest=read_json(folder/'manifest.json')
        timing=manifest['timing']
        if (timing['simulation_epoch']!=record['sensor_epoch'] or timing['simulation_frame']!=row['frame_sequence']
            or timing['simulation_time']!=row['source_time'] or len(manifest['cameras'])!=2
            or row['frame_sequence']<=previous[0] or row['source_time']<=previous[1]):
            raise ValueError('recording has mixed epochs, unpaired modules or nonmonotonic source frames')
        previous=(row['frame_sequence'],row['source_time'])
        frames.append({**row,'capture':str(folder)})
    return record,frames
