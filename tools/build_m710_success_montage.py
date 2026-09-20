"""Concatenate verified successful recordings without implying one physical world."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def build(source_list: Path, output: Path, ffmpeg: Path, font: Path) -> dict:
    sources = read(source_list)
    clips = []
    for source in sources:
        root = Path(source["delivery_directory"])
        summary, manifest = read(root / "summary.json"), read(root / "delivery_manifest.json")
        row = next(r for r in summary["rows"] if r["target"] == source["target"])
        assert row["full_cycle_completed"] and row["status"] == "COMPLETED", row
        relative = row["media_directory"] + "/replay.mp4"
        entry = next(f for f in manifest["files"] if f["path"] == relative)
        video = root / relative
        assert entry["video_audit"]["valid"] and sha256(video) == entry["sha256"]
        capture = read(root / row["media_directory"] / "initial.png.json")
        assert capture["world_session_id"] == manifest["world_session_id"]
        assert capture["target"] == source["target"]
        clips.append(dict(target=source["target"], world_session_id=capture["world_session_id"],
                          source=str(video), source_sha256=entry["sha256"],
                          expected_frames=entry["video_audit"]["decoded_frames"],
                          actual_release_height_m=row["actual_release_height_m"]))
    assert len(clips) == 5 and len({c["target"] for c in clips}) == 5
    worlds = list(dict.fromkeys(c["world_session_id"] for c in clips))
    output.mkdir(parents=True, exist_ok=False)
    movie = output / "five_successful_pick_place_3worlds.mp4"
    encoder = subprocess.Popen([str(ffmpeg), "-nostdin", "-v", "error", "-f", "rawvideo",
        "-pixel_format", "bgr24", "-video_size", "640x416", "-framerate", "5", "-i", "pipe:0",
        "-an", "-c:v", "libx264", "-threads", "2", "-crf", "18", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(movie)], stdin=subprocess.PIPE)
    total = 0
    try:
        for index, clip in enumerate(clips, 1):
            cap = cv2.VideoCapture(clip["source"])
            assert (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                    cap.get(cv2.CAP_PROP_FPS)) == (640, 360, 5.0)
            band = Image.new("RGB", (640, 56), "#14202b")
            draw = ImageDraw.Draw(band)
            world_index = worlds.index(clip["world_session_id"]) + 1
            draw.text((10, 5), f"SUCCESS {index}/5 | {clip['target']} | WORLD {world_index}/{len(worlds)}",
                      font=ImageFont.truetype(str(font), 16), fill="#ffffff")
            draw.text((10, 30), f"{len(worlds)}-world compilation | original speed | ideal reception / outfeed",
                      font=ImageFont.truetype(str(font), 12), fill="#b7d5e8")
            frame_out = np.empty((416, 640, 3), dtype=np.uint8)
            frame_out[360:] = np.asarray(band)[:, :, ::-1]
            clip["start_frame"] = total
            count = 0
            try:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        break
                    assert frame.shape == (360, 640, 3)
                    frame_out[:360] = frame
                    encoder.stdin.write(frame_out.tobytes())
                    if total == 200:
                        assert cv2.imwrite(str(output / "poster.png"), frame_out)
                    count += 1
                    total += 1
            finally:
                cap.release()
            assert count == clip["expected_frames"] and count > 0
            clip.update(frames=count, start_seconds=clip["start_frame"] / 5,
                        end_seconds=total / 5, duration_seconds=count / 5)
        encoder.stdin.close()
        assert encoder.wait() == 0
    except BaseException:
        encoder.stdin.close()
        encoder.wait()
        raise
    cap = cv2.VideoCapture(str(movie))
    decoded = 0
    while cap.read()[0]:
        decoded += 1
    assert decoded == total == int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    assert (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            cap.get(cv2.CAP_PROP_FPS)) == (640, 416, 5.0)
    cap.release()
    manifest = dict(schema="verified_five_successful_picks_montage_v1", clips=clips,
        world_session_ids=worlds, same_world_continuous_trial=False, new_physics_execution=False,
        source_frames_omitted=0, inserted_frames=0, playback_speed=1.0,
        source_pixel_rectangle=[0, 0, 640, 360], label_band_rectangle=[0, 360, 640, 56],
        video=dict(path=movie.name, sha256=sha256(movie), bytes=movie.stat().st_size,
                   width=640, height=416, fps=5, frames=total, decoded_frames=decoded,
                   duration_seconds=total / 5),
        poster=dict(path="poster.png", sha256=sha256(output / "poster.png")))
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    rows = "".join(f"<tr><td>{i}</td><td>{html.escape(c['target'])}</td>"
        f"<td>{html.escape(c['world_session_id'])}</td><td>{c['start_seconds']:.1f}–{c['end_seconds']:.1f} s</td></tr>"
        for i, c in enumerate(clips, 1))
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>五箱成功抓放连续剪辑</title>
<style>body{{font:16px/1.7 system-ui;max-width:950px;margin:30px auto;padding:0 20px;background:#14202b;color:#edf4fa}}a{{color:#8acbff}}video{{width:100%}}table{{width:100%;border-collapse:collapse}}td,th{{text-align:left;padding:8px;border-bottom:1px solid #567}}</style>
<h1>五箱成功抓放连续剪辑</h1><p>本轮五段成功实跑录像，顺序 c02 → c01 → c03 → c04 → c00。
来源于三个物理世界，属于成功片段拼接，不是同一世界连续五箱验收。正常速度，完整保留各段原始帧；没有新增仿真或补造动作。</p>
<video controls preload="metadata" poster="poster.png" src="{movie.name}"></video>
<p>{total / 5:.1f} 秒，{total} 帧，5 fps；原始 640×360 画面完整保留，底部另加来源标识，总尺寸 640×416。
接收与送出使用已批准的理想模式，不能计为真实物理接收。</p>
<p><a href="{movie.name}">直接打开 MP4</a> · <a href="manifest.json">来源、时间段和哈希清单</a></p>
<table><tr><th>顺序</th><th>目标</th><th>来源世界</th><th>视频时间段</th></tr>{rows}</table></html>"""
    (output / "index.html").write_text(page, encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sources", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", type=Path, required=True)
    parser.add_argument("--font", type=Path, required=True)
    args = parser.parse_args()
    result = build(args.sources, args.output, args.ffmpeg, args.font)
    print(json.dumps(result["video"]))
