"""Package sealed, actual Isaac segments for offline download; never reconstruct a world."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import subprocess


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def build(world, output, ffmpeg=None, interruption_report=None):
    import cv2
    output.mkdir(parents=True, exist_ok=True)
    scope = read(world / 'initial_highest_row.json', {})
    files, segments = [], []
    session_offset = read(world / 'archive_initialization.json', {}).get('time_s', 0.0)
    latest_transport = {}
    interruption = read(interruption_report, {}) if interruption_report else {}
    if interruption:
        assert interruption['world_session_id'] == scope['world_session_id']

    def copy(source, relative):
        dest = output / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
        sha = digest(source)
        assert sha == digest(dest)
        record = dict(path=relative, source=str(source), bytes=dest.stat().st_size,
                      sha256=sha, server_copy_verified=True, kind='original')
        files.append(record)
        return record

    if (world / 'initial.png').exists():
        copy(world / 'initial.png', 'initial.png')
        copy(world / 'initial.png.json', 'initial.png.json')
    if (world / 'initial_highest_row.json').exists():
        copy(world / 'initial_highest_row.json', 'initial_highest_row.json')
    if interruption:
        copy(interruption_report, 'continuation_interruption.json')
    for index, folder in enumerate([world] + sorted(world.glob('segment_*')), 1):
        status = read(folder / 'run_status.json', {})
        if status.get('status') not in ('complete', 'failed'):
            continue  # Do not copy a writer which is still open.
        result = read(folder / 'result.json', {})
        planning_dir = world.parent / ('initial_plan' if index == 1 else f'plan_{index:03d}')
        motion = read(planning_dir / 'motion.json', {})
        planned_release = motion.get('selected_trajectory_segment', {}).get('place', {}).get('release_prediction', {})
        capture = read(folder / 'initial.png.json', {})
        target = capture.get('target', result.get('target', 'unknown'))
        if scope and capture:
            assert capture['world_session_id'] == scope['world_session_id'], 'mixed physical worlds'
            assert target in scope['carton_ids'], 'target outside initialized highest row'
        prefix = f'segments/{index:03d}_{target}'
        for path in sorted(folder.iterdir()):
            if path.is_file() and path.suffix in ('.json', '.csv', '.png'):
                record = copy(path, f'{prefix}/{path.name}')
                if path.suffix == '.png':
                    pixels = cv2.imread(str(path))
                    assert pixels is not None, path
                    record['image_decoded'] = True
        video = folder / 'replay.mp4'
        video_audit = dict(valid=False, decoded_frames=0)
        frames = read(folder / 'actual_frame_states.json', {}).get('states', [])
        events = read(folder / 'execution_events.json', {}).get('events', [])
        transport = read(folder / 'ideal_transport_events.json', {})
        if transport:
            latest_transport = transport
        for event in transport.get('events', []):
            if event not in events and session_offset <= event.get('time_s', -1):
                events.append(event)
        wanted = []
        for number, event in enumerate(events):
            name = str(event.get('event', ''))
            event_time = event.get('simulation_time_s', event.get('time_s'))
            if event_time is not None and name in ('IDEAL_RECEPTION_ACCEPTED', 'LANDED_IDEAL_TRANSPORT', 'OUTFED_ASSUMED'):
                event_time -= session_offset  # Transport registry uses cumulative world task time.
            if event_time is not None and frames and any(s in name.lower() for s in
                    ('grasp', 'attach', 'release', 'reception', 'takeover', 'departure', 'outfed', 'free_transit', 'free_space', 'stop')):
                if event_time < 0 or event_time > frames[-1]['time_s'] + 0.2:
                    continue
                frame_index = min(range(len(frames)), key=lambda i: abs(frames[i]['time_s'] - event_time))
                wanted.append((frame_index, number, event, event_time))
        keyframes = []
        if video.exists():
            cap = cv2.VideoCapture(str(video))
            fps = cap.get(cv2.CAP_PROP_FPS)
            width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            metadata_aligned = len(frames) == int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if not metadata_aligned:
                wanted = []  # Retain video without inventing a frame/event mapping.
            count = 0
            while True:
                ok, pixels = cap.read()
                if not ok:
                    break
                for frame_index, number, event, event_time in wanted:
                    if frame_index != count:
                        continue
                    relative = f'{prefix}/event_{number:03d}_{event["event"]}.png'
                    assert cv2.imwrite(str(output / relative), pixels)
                    keyframes.append(dict(path=relative, event=event['event'],
                        event_target=event.get('carton_id', event.get('target', target)),
                        event_time_s=event_time, original_event_time_s=event.get('simulation_time_s', event.get('time_s')),
                        session_offset_s=session_offset, frame_index=count, video_time_s=count / fps,
                        frame_physics_time_s=frames[count]['time_s'],
                        frame_minus_event_s=frames[count]['time_s'] - event_time,
                        source='NEAREST_RECORDED_FRAME_NOT_EXACT_EVENT_STEP'))
                    dest = output / relative
                    files.append(dict(path=relative, source=str(video), bytes=dest.stat().st_size,
                                      sha256=digest(dest), kind='extracted_frame', image_decoded=True))
                count += 1
            expected = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()
            video_audit = dict(valid=count > 0 and count == expected, decoded_frames=count,
                               container_frames=expected, width=width, height=height, fps=fps,
                               frame_metadata_aligned=metadata_aligned)
            if video_audit['valid']:
                assert (width, height, fps) == (640, 360, 5.0)
                copy(video, f'{prefix}/replay.mp4')['video_audit'] = video_audit
                if ffmpeg:
                    relative = f'{prefix}/replay_browser.mp4'
                    playback = output / relative
                    subprocess.run([str(ffmpeg), '-nostdin', '-v', 'error', '-y', '-i', str(video),
                        '-map', '0:v:0', '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
                        '-fps_mode', 'passthrough', '-movflags', '+faststart', str(playback)], check=True)
                    reader = cv2.VideoCapture(str(playback))
                    decoded = 0
                    while reader.read()[0]:
                        decoded += 1
                    assert decoded == count and reader.get(cv2.CAP_PROP_FPS) == fps
                    reader.release()
                    files.append(dict(path=relative, source=str(video), bytes=playback.stat().st_size,
                        sha256=digest(playback), kind='playback_copy', video_audit=video_audit,
                        codec='H264', unchanged_frame_count_and_time_scale=True))
        stop = result.get('runtime_stop_reason') or status.get('primary_runtime_stop_reason') or status.get('exception')
        segment = dict(index=index, target=target, prefix=prefix, stop_reason=stop,
                       workflow_completed=result.get('workflow_cycle_completed', False),
                       counts=result.get('execution_counts', {}), video=video_audit,
                       actual_release_prediction=result.get('actual_release_prediction'),
                       planned_release_height_m=planned_release.get('height_m'),
                       replayed_simulation_seconds=result.get('replayed_simulation_seconds'),
                       keyframes=keyframes, events=events)
        segments.append(segment)
        remaining = read(folder / 'actual_remaining_state.json', {})
        session_offset = remaining.get('time_s', session_offset + result.get('replayed_simulation_seconds', 0))
        if stop:
            for name in ('failure.png', 'failure.png.json', 'run_status.json', 'execution_events.json'):
                if (folder / name).exists():
                    copy(folder / name, f'failure/{index:03d}_{target}_{name}')
    rows = []
    for target in scope.get('carton_ids', []):
        segment = next((s for s in segments if s['target'] == target), {})
        counts = segment.get('counts', {})
        prediction = segment.get('actual_release_prediction') or {}
        actual_grasp = bool(counts.get('actual_grasp')) or any(
            e.get('event') == 'grasp_contact_attempt' and e.get('accepted') for e in segment.get('events', []))
        actual_release = bool(counts.get('actual_release')) or any(
            e.get('event') == 'release_constraint_removal_confirmed' for e in segment.get('events', []))
        row = dict(target=target, execution_order=segment.get('index'), actual_grasp=actual_grasp,
            actual_release=actual_release,
            actual_physical_reception=target in latest_transport.get('actual_received_ids', []),
            ideal_reception=target in latest_transport.get('ideal_received_ids', []),
            withdrawal_completed=bool(segment.get('workflow_completed')),
            ideal_outfeed=target in latest_transport.get('ideal_outfed_ids', []),
            actual_release_height_m=prediction.get('height_m') if actual_release else None,
            planned_release_height_m=segment.get('planned_release_height_m'),
            first_takeover_height_m=next((e.get('first_takeover_height_m') for e in segment.get('events', [])
                if e.get('event') == 'IDEAL_RECEPTION_ACCEPTED' and e.get('carton_id') == target), None),
            stop_reason=segment.get('stop_reason'), media_directory=segment.get('prefix'))
        row['full_cycle_completed'] = all(row[k] for k in
            ('actual_grasp', 'actual_release', 'ideal_reception', 'withdrawal_completed', 'ideal_outfeed'))
        row['status'] = ('COMPLETED' if row['full_cycle_completed'] else 'FAILED' if row['stop_reason']
                         else 'SEGMENT_FINISHED_AWAITING_OUTFEED' if segment.get('workflow_completed')
                         else 'NOT_EXECUTED_PREDECESSOR_BLOCKED' if not segment and any(s['stop_reason'] for s in segments)
                         else 'NOT_EXECUTED' if not segment else 'NOT_COMPLETED')
        if interruption and not segment:
            row['status'] = ('PLANNING_INTERRUPTED_NOT_EXECUTED'
                if target == interruption.get('interrupted_planning_target') else 'NOT_EXECUTED_WORLD_ENDED')
        rows.append(row)
    summary = dict(world=scope, segments=segments, rows=rows,
                   full_cycles_completed=sum(r['full_cycle_completed'] for r in rows),
                   workflow_completed=sum(s['workflow_completed'] for s in segments),
                   denominator=scope.get('denominator'),
                   continuation_interruption=interruption or None,
                   actual_drive_effort_qualification='NOT_EVALUATED',
                   video_excludes_offline_planning_pauses=True)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    lines = [f'# Actual world {scope.get("world_session_id")}',
             f'Full cycles including outfeed: {summary["full_cycles_completed"]}/{summary["denominator"]}.',
             'Ideal reception is not measured physical reception. Drive effort: NOT_EVALUATED.']
    if interruption:
        lines.append('World and planner processes were absent on reconnect. Exit reason is unknown; '
                     'the next carton was not physically executed. This is not a physical failure or an infeasibility result.')
    for s in segments:
        lines += [f'\n## {s["index"]}: {s["target"]}', f'Stop: {s["stop_reason"]}',
                  'Raw execution_counts mix segment and cumulative registries; use the per-carton table below.',
                  f'Video: {json.dumps(s["video"])}']
    lines += ['\n## Per-carton results', '| Target | Grasp | Release | Ideal reception | Withdrawal | Ideal outfeed | Release mm | Status |',
              '|---|---|---|---|---|---|---|---|']
    for row in rows:
        cells = [str(row[k]) for k in ('target', 'actual_grasp', 'actual_release', 'ideal_reception',
                                      'withdrawal_completed', 'ideal_outfeed')]
        cells += [str(None if row['actual_release_height_m'] is None else 1000 * row['actual_release_height_m']), row['status']]
        lines.append('| ' + ' | '.join(cells) + ' |')
    (output / 'summary.md').write_text('\n\n'.join(lines), encoding='utf-8')
    body = ['<!doctype html><meta charset="utf-8"><title>M710 actual execution</title>',
            '<style>body{font:16px system-ui;max-width:1000px;margin:32px auto;background:#17202b;color:#eee}a{color:#89c8ff}img,video{max-width:100%}pre{white-space:pre-wrap}figure{display:inline-block;margin:8px;width:300px}</style>',
            '<h1>M710 actual execution</h1>', '<pre>' + html.escape('\n\n'.join(lines)) + '</pre>',
            '<p><a href="summary.json">Full event summary</a> | <a href="delivery_manifest.json">File manifest</a></p>']
    if (output / 'initial.png').exists():
        body.append('<h2>Actual initialized world</h2><img src="initial.png">')
    for s in segments:
        body.append(f'<h2>{s["index"]}: {html.escape(s["target"])}</h2>')
        if s['video']['valid']:
            p = s['prefix'] + '/replay.mp4'
            playback = s['prefix'] + '/replay_browser.mp4'
            if not (output / playback).exists():
                playback = p
            body.append(f'<video controls preload="metadata" src="{playback}"></video><p><a href="{p}">Original MP4</a></p>')
        for frame in s['keyframes']:
            body.append(f'<figure><img src="{frame["path"]}"><figcaption>{html.escape(frame["event_target"])}: {html.escape(frame["event"])}; frame − event: {frame["frame_minus_event_s"]:.6f} s</figcaption></figure>')
        for name in ('failure.png', 'final_actual.png'):
            p = s['prefix'] + '/' + name
            if (output / p).exists():
                body.append(f'<p>{name}</p><img src="{p}">')
    (output / 'index.html').write_text('\n'.join(body), encoding='utf-8')
    for name in ('summary.json', 'summary.md', 'index.html'):
        p = output / name
        files.append(dict(path=name, source='delivery_builder', bytes=p.stat().st_size,
                          sha256=digest(p), kind='generated_index'))
    manifest = dict(world_session_id=scope.get('world_session_id'), files=files,
                    videos=sum(f['path'].endswith('.mp4') for f in files),
                    original_videos=sum(s['video']['valid'] for s in segments),
                    png_files=sum(f['path'].endswith('.png') for f in files),
                    unique_png_images=len({f['sha256'] for f in files if f['path'].endswith('.png')}),
                    extracted_event_frames=sum(f['kind'] == 'extracted_frame' for f in files))
    (output / 'delivery_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'files'}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ffmpeg', type=Path, help='Existing encoder for a separate browser playback copy')
    parser.add_argument('--interruption-report', type=Path, help='Explicit observed interruption evidence for this world')
    args = parser.parse_args()
    build(args.world.resolve(), args.output.resolve(), args.ffmpeg, args.interruption_report)
