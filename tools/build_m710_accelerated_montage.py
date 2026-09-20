"""Create a labeled 16x 720P derivative of the verified five-pick montage."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(source, output, ffmpeg, font):
    manifest = json.loads((source.parent / "manifest.json").read_text(encoding="utf-8"))
    assert digest(source) == manifest["video"]["sha256"]
    assert manifest["video"]["fps"] == 5 and manifest["video"]["frames"] == 3113
    output.mkdir(parents=True, exist_ok=False)
    movie = output / "five_successful_pick_place_3worlds_16x_720p.mp4"
    # Keep every recorded frame: 5 fps * 16 = 80 fps, with no interpolation.
    encoder = subprocess.Popen([str(ffmpeg), "-nostdin", "-v", "error", "-f", "rawvideo",
        "-pixel_format", "bgr24", "-video_size", "640x416", "-framerate", "80", "-i", "pipe:0",
        "-vf", "scale=-2:720:flags=lanczos,pad=1280:720:(ow-iw)/2:0,setsar=1",
        "-an", "-c:v", "libx264", "-preset", "fast", "-threads", "2", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(movie)], stdin=subprocess.PIPE)
    footer = Image.new("RGB", (640, 28), "#14202b")
    ImageDraw.Draw(footer).text((10, 2), "16x speed | 3-world compilation | ideal reception / outfeed",
        font=ImageFont.truetype(str(font), 13), fill="#b7d5e8")
    footer_bgr = np.asarray(footer)[:, :, ::-1]
    cap = cv2.VideoCapture(str(source))
    count = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            assert frame.shape == (416, 640, 3)
            frame[388:] = footer_bgr  # Replace the obsolete "original speed" label only.
            encoder.stdin.write(frame.tobytes())
            count += 1
    finally:
        cap.release()
        encoder.stdin.close()
    assert encoder.wait() == 0 and count == 3113
    cap = cv2.VideoCapture(str(movie))
    assert (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            cap.get(cv2.CAP_PROP_FPS)) == (1280, 720, 80.0)
    decoded = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if decoded == 200:
            assert cv2.imwrite(str(output / "poster.png"), frame)
        decoded += 1
    cap.release()
    assert decoded == count
    report = dict(schema="accelerated_montage_v1", source=str(source), source_sha256=digest(source),
        video=movie.name, sha256=digest(movie), bytes=movie.stat().st_size,
        width=1280, height=720, fps=80, decoded_frames=decoded,
        duration_seconds=count / 80, playback_speed=16, source_frames_omitted=0,
        interpolated_frames=0, aspect_ratio_preserved=True, pillarboxed=True,
        upscaled_from=[640, 416], historical_burned_in_hud_preserved=True,
        same_world_continuous_trial=False, new_physics_execution=False,
        poster_sha256=digest(output / "poster.png"))
    (output / "manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (output / "index.html").write_text(f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>五箱抓放 · 16倍速 · 720P</title><style>body{{background:#14202b;color:#edf4fa;font:16px/1.7 system-ui;max-width:1280px;margin:24px auto;padding:0 16px}}video{{width:100%}}a{{color:#8acbff}}</style>
<h1>五箱抓放 · 16倍速 · 720P</h1><video controls preload="metadata" poster="poster.png" src="{movie.name}"></video>
<p>38.9125 秒 · 1280×720 · 80 fps。保留全部 3113 帧，等比放大并加左右黑边。原录像左上角文字已嵌入画面，本版保留；后续新录像采用左下角精简参数。</p>
<p>三个世界的成功片段拼接；接收与送出采用理想模式。没有新增物理执行。</p>
<p><a href="{movie.name}">打开 MP4</a> · <a href="manifest.json">校验清单</a></p></html>''', encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("source", "output", "ffmpeg", "font"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output, args.ffmpeg, args.font)))
