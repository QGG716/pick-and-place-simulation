"""Export a planned FANUC segment to a backend-neutral Isaac replay bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.scene import load_scene_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--config", default=Path("config/fanuc_m20id35.yaml"), type=Path)
    parser.add_argument("--segment", default=0, type=int)
    parser.add_argument("--period", type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    _, cfg = load_scene_config(args.config)
    bundle = build_fanuc_isaac_replay_bundle(
        plan,
        cfg,
        segment_index=args.segment,
        controller_period_seconds=args.period,
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
