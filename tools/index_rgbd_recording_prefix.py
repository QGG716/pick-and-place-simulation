"""Index a verified contiguous prefix after interrupted acquisition; never repair bindings."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import atomic_json,read_json,verify_capture
from unloading_perception.isaac_validation import sha256_file


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path)
    a=p.parse_args();root=a.recording.resolve();output=root/'sequence-prefix.json'
    if output.exists():raise FileExistsError(output)
    frames=[];warning=None;epoch=None;start=None;end=None
    for folder in sorted((root/'frames').iterdir()):
        try:
            verify_capture(folder)
            manifest=read_json(folder/'manifest.json');t=manifest['timing']
            if epoch is not None and epoch!=t['simulation_epoch']:raise ValueError('epoch changed')
            if frames and (t['simulation_frame']<=frames[-1]['frame_sequence'] or t['simulation_time']<=frames[-1]['source_time']):
                raise ValueError('source time or frame did not advance')
            epoch=t['simulation_epoch']
            if start is None:start=(folder/'manifest.json').stat().st_mtime
            end=max(p.stat().st_mtime for p in folder.rglob('capture_binding.json'))
            frames.append({'ordinal':int(folder.name),'path':str(folder.relative_to(root)),
                'frame_sequence':t['simulation_frame'],'source_time':t['simulation_time'],
                'manifest_sha256':sha256_file(folder/'manifest.json')})
        except Exception as exc:
            warning=f'Acquisition interrupted: {folder.name}: {type(exc).__name__}: {exc}'
            break  # Never skip a bad middle frame and silently concatenate later captures.
    if len(frames)<2:raise ValueError('not enough complete original paired captures')
    duration=frames[-1]['source_time']-frames[0]['source_time']
    atomic_json(output,{'schema_version':'continuous_rgbd_recording_v1','mode':'RGB-D RECORDING',
        'sensor_epoch':epoch,'fps':(len(frames)-1)/duration,'frames':frames,
        'algorithm_resolution':manifest['cameras'][0]['resolution'],'preview_resolution':[640,480],
        'simulation_real_time_factor':duration/(end-start),
        'rtf_measurement':'source first-to-last interval / filesystem wall interval from first manifest to last binding; excludes engine startup',
        'capture_wall_seconds':end-start,'simulation_seconds':duration,
        'source_warning':warning,'index_kind':'VERIFIED_CONTIGUOUS_PREFIX_OF_INTERRUPTED_NEW_ACQUISITION',
        'original_bindings_modified':False})
    print(output,len(frames),duration,flush=True)


if __name__=='__main__':main()
