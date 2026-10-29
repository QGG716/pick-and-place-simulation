"""Render a verified batch replay to MP4 at a fixed presentation speed.

Optional dependencies: playwright, imageio-ffmpeg. A Chromium browser is required.
This samples the existing replay clock; it does not execute or resimulate physics.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--browser", help="Chromium/Edge executable; otherwise Playwright Chromium")
    parser.add_argument("--speed", type=float, default=4.0)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--required-count", type=int, default=40)
    args = parser.parse_args()
    if args.speed <= 0 or args.fps <= 0:
        parser.error("speed and fps must be positive")
    if args.output.exists():
        parser.error("output already exists; choose a new path")

    import imageio_ffmpeg
    from playwright.sync_api import sync_playwright

    replay = args.replay.resolve()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    checks_path = output.with_suffix(".checks.json")
    log_path = output.with_suffix(".ffmpeg.log")
    started = time.monotonic()
    errors: list[str] = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=args.browser, headless=True,
            args=["--enable-unsafe-swiftshader", "--use-angle=swiftshader"],
        )
        page = browser.new_page(viewport={"width": 640, "height": 360}, device_scale_factor=1)
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(replay.as_uri())
        page.wait_for_function("window.previewState !== undefined")
        page.add_style_tag(content="""
          header, aside, #source {display:none!important}
          main {display:block!important;height:360px!important}
          #view {width:640px;height:360px}
          #hud {left:8px;bottom:8px;padding:5px 8px;font-size:11px;
                line-height:1.4;border-radius:4px;max-width:620px}
        """)
        page.evaluate("resizeBatch()")
        metadata = page.evaluate("""() => ({duration:batchData.duration,
          completed:batchData.result.completed_count,
          initial_count:batchData.segments[0].cartons.length,
          targets:[...new Set(batchData.segments.map(s=>s.target).filter(Boolean))],
          segments:batchData.segments.length,
          moves:batchData.segments.filter(s=>s.kind==='base_xy').map(s=>({start:s.start,duration:s.duration})),
          assumed_removed:batchData.assumed_removed_target_ids||[]})""")
        assert metadata["completed"] == metadata["initial_count"] == args.required_count, metadata
        assert len(metadata["targets"]) == args.required_count and not metadata["assumed_removed"]
        frame_count = math.ceil(metadata["duration"] * args.fps / args.speed) + 1
        page.evaluate("""speed => {window.drawVideoFrame=t=>{
          setBatchTime(t);
          const state=previewState;
          const phases={pregrasp:'接近',contact:'接触',top_lift:'顶吸抬箱',
            top_transfer:'顶吸转移',extraction:'抽离',withdrawal:'撤离',residence:'空载停靠',
            drop:'名义下落',nominal_drop:'名义下落',outfeed:'理想出料',
            landed:'名义接收',complete:'完成',completed:'完成',base_xy:'空载平移'};
          const oldHud=document.getElementById('hud').innerText;
          const cup=oldHud.split('吸盘 ')[1]?.split('\\n')[0]||'关闭';
          document.getElementById('hud').innerText=
            `规划回放 · ${speed}× · ${state.target||'空载/结束'}\\n`+
            `${phases[state.phase]||state.phase} · 回放 ${t.toFixed(1)} s\\n`+
            `附着 ${state.attached?'是':'否'} · 吸盘 ${cup}\\n`+
            `名义接收 ${state.nominal_count} · 理想出料 ${state.drain_count}/40 · 实测接收 0`;
          return structuredClone(state);
        }}""", args.speed)
        command = [ffmpeg, "-hide_banner", "-loglevel", "warning", "-f", "image2pipe",
                   "-vcodec", "mjpeg", "-framerate", str(args.fps), "-i", "pipe:0", "-an",
                   "-c:v", "libx264", "-preset", "medium", "-crf", "23", "-pix_fmt", "yuv420p",
                   "-movflags", "+faststart", str(output)]
        seen_targets: set[str] = set()
        seen_counts: set[int] = set()
        previous = 0
        samples = []
        move_frames = {round((m["start"] + m["duration"] / 2) * args.fps / args.speed)
                       for m in metadata["moves"]}
        sample_frames = {0, frame_count // 2, frame_count - 1} | move_frames
        with log_path.open("wb") as log:
            encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=log, stderr=log)
            try:
                for index in range(frame_count):
                    replay_time = min(index * args.speed / args.fps, metadata["duration"])
                    state = page.evaluate("t=>drawVideoFrame(t)", replay_time)
                    assert previous <= state["completed_count"] <= args.required_count
                    assert state["physical_execution"] == "NOT_RUN" and state["physical_count"] == 0
                    previous = state["completed_count"]
                    seen_counts.add(previous)
                    if state["target"]:
                        seen_targets.add(state["target"])
                    encoder.stdin.write(page.screenshot(type="jpeg", quality=90))
                    if index in sample_frames:
                        page.screenshot(path=str(output.with_name(f"video-frame-{index:05d}.png")))
                        samples.append(dict(frame=index, replay_time_s=replay_time, state=state))
                    if index % 100 == 0 or index == frame_count - 1:
                        print(json.dumps(dict(frame=index, total=frame_count, completed=previous,
                                              elapsed_s=round(time.monotonic()-started, 1))), flush=True)
                assert not errors, errors
                assert state["completed_count"] == args.required_count and state["carton_count"] == 0
                assert seen_targets == set(metadata["targets"])
                assert seen_counts == set(range(args.required_count + 1)), seen_counts
            finally:
                encoder.stdin.close()
                exit_code = encoder.wait()
            assert exit_code == 0, log_path
        browser.close()
    # Fully decode the encoded result: detect corruption and count actual frames.
    decoded = subprocess.run([ffmpeg, "-v", "error", "-progress", "pipe:1", "-i", str(output),
                              "-map", "0:v:0", "-f", "null", "-"],
                             capture_output=True, text=True, check=True)
    assert not decoded.stderr.strip(), decoded.stderr
    decoded_frames = int([line.split("=", 1)[1] for line in decoded.stdout.splitlines()
                          if line.startswith("frame=")][-1])
    assert decoded_frames == frame_count, (decoded_frames, frame_count)
    checks = dict(status="PASS_COMPLETE_40_CARTON_VIDEO", replay_sha256=sha256(replay),
                  video_sha256=sha256(output), video_bytes=output.stat().st_size,
                  width=640, height=360, fps=args.fps, playback_speed=args.speed,
                  frame_count=frame_count, decoded_frames=decoded_frames,
                  video_duration_s=frame_count/args.fps,
                  frame_time_rule="min(frame_index * speed / fps, replay_duration)",
                  final_state_sampled=True, physical_execution="NOT_RUN",
                  all_targets_seen=sorted(seen_targets), completed_counts_seen=sorted(seen_counts),
                  source=metadata, samples=samples, browser_errors=errors,
                  render_elapsed_s=time.monotonic()-started)
    checks_path.write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in checks.items() if k not in ("source", "samples")}), flush=True)


if __name__ == "__main__":
    main()
