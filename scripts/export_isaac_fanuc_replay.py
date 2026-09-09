"""Export a planned FANUC segment to a backend-neutral Isaac replay bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.m710_replay_contract import verify_m710_preflight_contract
from unloading_sim.scene import load_scene_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path)
    parser.add_argument(
        "--preflight",
        type=Path,
        help="verified M-710 preflight; consumes its exact bound plan/config/trajectory",
    )
    parser.add_argument("--config", default=Path("config/fanuc_m20id35.yaml"), type=Path)
    parser.add_argument("--segment", default=0, type=int)
    parser.add_argument("--period", type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    preflight = None
    if args.preflight is not None:
        if args.plan is not None:
            parser.error("--preflight and --plan are mutually exclusive")
        if args.segment != 0:
            parser.error("an M-710 preflight binds exactly segment 0")
        preflight = json.loads(args.preflight.read_text(encoding="utf-8"))
        verify_m710_preflight_contract(preflight, require_ready=True)
        adapter = preflight["replay_adapter_inputs"]
        plan = dict(adapter["plan_common"])
        plan["segments"] = [adapter["trajectory_segment"]]
        cfg = adapter["configuration"]
    else:
        if args.plan is None:
            parser.error("one of --plan or --preflight is required")
        plan = json.loads(args.plan.read_text(encoding="utf-8"))
        _, cfg = load_scene_config(args.config)
    bundle = build_fanuc_isaac_replay_bundle(
        plan,
        cfg,
        segment_index=args.segment,
        controller_period_seconds=args.period,
        preflight=preflight,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle.to_dict(), indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "commands": len(bundle.timestamps_seconds),
                "duration_seconds": bundle.metadata["duration_seconds"],
                "target": bundle.metadata["target"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
