"""Generate a separate reference-limited S50 model, or retain its precise gaps.

Unknown controller sign/offset never produces a task URDF. Exit 0 denotes
generation only, not installed collision/task qualification; exit 2 is NOT_READY.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_task_model import generate_task_model, INPUT_CONFIG, TASK_DIRECTORY


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/INPUT_CONFIG)
    parser.add_argument('--model-directory',type=Path,default=ROOT/TASK_DIRECTORY)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/s50_task_collision_20261111/task-generation.json')
    args=parser.parse_args(argv)
    report=generate_task_model(args.config,root=ROOT,output_directory=args.model_directory)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('status','task_model_generated','task_definition_sha256','missing')},ensure_ascii=False))
    return 0 if report['task_model_generated'] else 2


if __name__=='__main__':
    raise SystemExit(main())
