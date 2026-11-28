"""Offline check of the Qwen VLM advisor on saved frames (no device, no UI, no motion).

    python tools/brain/assess_records.py <task_id | image.png | observation.npz> ... [--ask 问题] [--out report.json]

Each input's colour frame goes through the same assessment prompt and parser
as the workbench panel; prints one line per frame and optionally writes a
JSON report (verdicts, raw replies, timings, image SHA-256).
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))

from unloading_sim.workbench.brain import settings as brain_settings  # noqa: E402
from unloading_sim.workbench.brain.assessment import (assessment_messages, chat_messages, clean_reply,  # noqa: E402
                                                      parse_assessment, prompt_sha256)
from unloading_sim.workbench.brain.vlm_client import RemoteQwenVL, encode_jpeg  # noqa: E402


def load_rgb(item):
    p = Path(item)
    if not p.exists():
        p = ROOT / 'outputs/single_carton' / item / 'observation.npz'
    if p.suffix == '.npz':
        with np.load(p, allow_pickle=False) as z:
            return np.ascontiguousarray(z['rgb']), str(p)
    from PIL import Image
    return np.asarray(Image.open(p).convert('RGB')), str(p)


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('inputs', nargs='+')
    a.add_argument('--config', default=str(brain_settings.DEFAULT_PATH))
    a.add_argument('--ask', default=None, help='同时问一个问题（对话提示词）')
    a.add_argument('--out', default=None)
    args = a.parse_args()
    s, issues = brain_settings.load(args.config)
    problems = brain_settings.vlm_issues(s)
    if problems:
        raise SystemExit('远程 VLM 配置问题：' + '；'.join(problems + issues))
    v = s['vlm']
    vlm = RemoteQwenVL(v['remote'], dict(relative_directory=v['model']['relative_directory'], files=v['model_files']),
                       max_pixels=v.get('max_pixels', 640 * 480))
    started = time.monotonic()
    info = vlm.connect()
    report = dict(model=v['model']['name'], gpu=info.get('gpu'), connect_s=time.monotonic() - started,
                  verify_s=info.get('verify_s'), load_s=info.get('load_s'), warm_up_s=info.get('warm_up_s'), file_hash_states=info.get('file_hash_states'),
                  authority=s['authority'], frames=[])
    print(f"connected {info.get('gpu')} in {report['connect_s']:.1f} s (verify {info.get('verify_s', 0):.1f} s, load {info.get('load_s', 0):.1f} s, warm-up {info.get('warm_up_s') or 0:.1f} s)")
    context = dict(note='离线回放：保存的冻结帧，没有实时设备状态')
    for item in args.inputs:
        rgb, source = load_rgb(item)
        data, digest = encode_jpeg(rgb, v.get('jpeg_quality', 90))
        messages = assessment_messages(context)
        reply = vlm.generate(messages, [data], v.get('assess_max_new_tokens', 320))
        result = parse_assessment(reply['text'], image_sha256=digest, stamp=time.time(),
                                  timing={k: reply.get(k) for k in ('round_trip_s', 'inference_s', 'input_tokens', 'output_tokens')})
        entry = dict(source=source, prompt_sha256=prompt_sha256(messages), **result.record())
        print(f"{Path(source).parent.name or source}: [{result.level}] {result.headline()} | {result.summary} | "
              f"{reply['round_trip_s']:.2f} s ({reply['input_tokens']}→{reply['output_tokens']} tok)")
        if args.ask:
            m = chat_messages([], args.ask, dict(context, last_vlm_assessment=result.headline()), True)
            r = vlm.generate(m, [data], v.get('chat_max_new_tokens', 160))
            entry['chat'] = dict(question=args.ask, reply=clean_reply(r['text']), round_trip_s=r['round_trip_s'])
            print(f"  问：{args.ask}\n  答：{entry['chat']['reply']}（{r['round_trip_s']:.2f} s）")
        report['frames'].append(entry)
    vlm.close()
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str))


if __name__ == '__main__':
    main()
