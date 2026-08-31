"""Render every segment of an unloading plan and concatenate the Isaac Sim videos.

This runner is intentionally outside the core package: Isaac Sim remains an optional
backend.  A completed segment is reused when its result and accelerated replay exist,
so an interrupted remote run can be resumed safely.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--project-root", default=Path.cwd(), type=Path)
    parser.add_argument("--width", default=1280, type=int)
    parser.add_argument("--height", default=720, type=int)
    parser.add_argument("--physics-hz", default=60.0, type=float)
    parser.add_argument("--render-every", default=12, type=int)
    parser.add_argument("--video-preview-speed", default=6.0, type=float)
    parser.add_argument("--start-segment", default=0, type=int)
    parser.add_argument("--end-segment", type=int)
    parser.add_argument("--force", action="store_true")
    return parser


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run(command: list[str], cwd: Path) -> None:
    env = os.environ.copy()
    source_root = str(cwd / "src")
    existing_pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = (
        source_root + os.pathsep + existing_pythonpath if existing_pythonpath else source_root
    )
    subprocess.run(command, cwd=cwd, env=env, check=True)


def _segment_count(plan: dict[str, object]) -> int:
    for key in ("segments", "plans", "trajectories"):
        value = plan.get(key)
        if isinstance(value, list):
            return len(value)
    raise ValueError("plan does not contain a segment list")


def _target(plan: dict[str, object], index: int) -> str | None:
    for key in ("segments", "plans", "trajectories"):
        value = plan.get(key)
        if isinstance(value, list) and index < len(value) and isinstance(value[index], dict):
            segment = value[index]
            for target_key in ("target", "target_name", "box_name", "carton_name"):
                if target_key in segment:
                    return str(segment[target_key])
    return None


def _write_concat_list(paths: list[Path], destination: Path) -> None:
    lines = []
    for path in paths:
        escaped = str(path.resolve()).replace("'", "'\\''")
        lines.append(f"file '{escaped}'")
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _concatenate_videos(
    videos: list[Path], destination: Path, concat_list: Path, fps: float, size: tuple[int, int]
) -> str:
    """Concatenate with ffmpeg when present, otherwise stream frames through OpenCV."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        _run(
            [
                ffmpeg, "-y", "-f", "concat", "-safe", "0", "-i", str(concat_list),
                "-c", "copy", "-movflags", "+faststart", str(destination),
            ],
            Path.cwd(),
        )
        return "ffmpeg_concat_copy"

    import cv2

    writer = cv2.VideoWriter(
        str(destination), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), size
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the full-unload MP4 writer")
    try:
        for video in videos:
            capture = cv2.VideoCapture(str(video))
            if not capture.isOpened():
                raise RuntimeError(f"OpenCV could not open segment video: {video}")
            try:
                actual_size = (
                    int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                )
                if actual_size != size:
                    raise RuntimeError(f"segment video {video} is {actual_size}, expected {size}")
                while True:
                    ok, frame = capture.read()
                    if not ok:
                        break
                    writer.write(frame)
            finally:
                capture.release()
    finally:
        writer.release()
    if not destination.is_file() or destination.stat().st_size == 0:
        raise RuntimeError("full-unload MP4 was not created")
    return "opencv_stream_reencode"


def main() -> None:
    args = _parser().parse_args()
    project_root = args.project_root.resolve()
    plan_path = args.plan.resolve()
    config_path = args.config.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    count = _segment_count(plan)
    end = count if args.end_segment is None else min(args.end_segment, count)
    if not (0 <= args.start_segment < end <= count):
        raise ValueError(f"invalid segment range [{args.start_segment}, {end}) for {count} segments")
    if args.physics_hz <= 0 or args.render_every <= 0 or args.video_preview_speed < 1:
        raise ValueError("physics/render rates and preview speed must be positive")

    speed_label = f"{args.video_preview_speed:g}"
    segment_summaries: list[dict[str, object]] = []
    started = time.time()
    for index in range(args.start_segment, end):
        segment_dir = output_dir / "segments" / f"segment_{index:02d}"
        segment_dir.mkdir(parents=True, exist_ok=True)
        bundle_path = segment_dir / "replay_bundle.json"
        result_path = segment_dir / "result.json"
        video_path = segment_dir / f"replay_{speed_label}x.mp4"
        reused = result_path.is_file() and video_path.is_file() and video_path.stat().st_size > 0
        print(
            f"FULL_UNLOAD_PROGRESS segment={index + 1}/{count} "
            f"target={_target(plan, index) or 'unknown'} status={'reuse' if reused else 'start'}",
            flush=True,
        )
        if args.force or not reused:
            _run(
                [
                    sys.executable,
                    str(project_root / "scripts" / "export_isaac_fanuc_replay.py"),
                    "--plan", str(plan_path),
                    "--config", str(config_path),
                    "--segment", str(index),
                    "--output", str(bundle_path),
                ],
                project_root,
            )
            _run(
                [
                    sys.executable,
                    str(project_root / "scripts" / "isaacsim_fanuc_replay.py"),
                    "--bundle", str(bundle_path),
                    "--project-root", str(project_root),
                    "--usd-directory", str(output_dir / "usd_cache"),
                    "--output", str(segment_dir),
                    "--physics-hz", str(args.physics_hz),
                    "--render-every", str(args.render_every),
                    "--width", str(args.width),
                    "--height", str(args.height),
                    "--record-video",
                    "--video-preview-speed", str(args.video_preview_speed),
                ],
                project_root,
            )
        if not result_path.is_file() or not video_path.is_file():
            raise RuntimeError(f"segment {index} did not produce its result and preview video")
        result = json.loads(result_path.read_text(encoding="utf-8"))
        qualification_checks = result.get("qualification_checks") or {}
        segment_summaries.append(
            {
                "segment_index": index,
                "target": result.get("target", _target(plan, index)),
                "reused": reused and not args.force,
                "result_path": str(result_path),
                "video_path": str(video_path),
                "video_sha256": _sha256(video_path),
                "command_schedule_duration_seconds": result.get("command_schedule_duration_seconds"),
                "replayed_simulation_seconds": result.get("replayed_simulation_seconds"),
                "replay_wall_seconds": result.get("replay_wall_seconds"),
                "grasp_command_succeeded": result.get("payload_grasp_command_succeeded"),
                "payload_attachment_intact": result.get("payload_attachment_intact"),
                "release_command_succeeded": result.get("payload_release_command_succeeded"),
                "placement_within_tolerance": qualification_checks.get("placement_within_tolerance"),
                "unexpected_robot_collision_count": len(
                    result.get("unexpected_robot_scene_contacts") or []
                ),
                "premature_payload_conveyor_contact_count": result.get(
                    "premature_payload_conveyor_contact_count"
                ),
                "conveyor_transport_engaged": result.get("conveyor_transport_engaged"),
                "conveyor_transport_projected_speed_m_s": result.get(
                    "conveyor_transport_projected_speed_m_s"
                ),
                "conveyor_transport_speed_within_tolerance": result.get(
                    "conveyor_transport_speed_within_tolerance"
                ),
                "qualification_passed": result.get("qualification_passed"),
            }
        )
        print(f"FULL_UNLOAD_PROGRESS segment={index + 1}/{count} status=complete", flush=True)

    videos = [Path(item["video_path"]) for item in segment_summaries]
    concat_list = output_dir / "concat_6x.txt"
    _write_concat_list(videos, concat_list)
    final_video = output_dir / f"full_unload_{speed_label}x.mp4"
    output_fps = args.physics_hz / args.render_every * args.video_preview_speed
    concatenation_backend = _concatenate_videos(
        videos,
        final_video,
        concat_list,
        output_fps,
        (args.width, args.height),
    )
    manifest = {
        "format": "isaacsim_full_unload_manifest_v1",
        "scope": "visual_and_dynamic_replay_baseline_not_production_qualification",
        "plan_path": str(plan_path),
        "plan_sha256": _sha256(plan_path),
        "config_path": str(config_path),
        "segment_count_in_plan": count,
        "rendered_segment_range": [args.start_segment, end],
        "rendered_segment_count": len(segment_summaries),
        "resolution": [args.width, args.height],
        "physics_hz": args.physics_hz,
        "render_every": args.render_every,
        "physical_render_rate_hz": args.physics_hz / args.render_every,
        "viewing_speed": args.video_preview_speed,
        "output_video_fps": output_fps,
        "concatenation_backend": concatenation_backend,
        "full_video_path": str(final_video),
        "full_video_sha256": _sha256(final_video),
        "wall_seconds": time.time() - started,
        "segments": segment_summaries,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": "complete", "video": str(final_video), "manifest": str(manifest_path)}), flush=True)


if __name__ == "__main__":
    main()
