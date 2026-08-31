"""Compare FANUC capsule planning collisions with Pinocchio/Coal URDF meshes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from unloading_sim.collision_backend import (
    CapsuleCollisionBackend,
    CollisionQuery,
    compare_collision_backends,
)
from unloading_sim.online_unload import docked_robot_and_scene
from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
from unloading_sim.scene import load_scene_config


class _PinocchioCollisionAdapter:
    name = "pinocchio_coal_urdf_mesh"

    def __init__(self, backend: PinocchioHppFclBackend) -> None:
        self.backend = backend

    def query(self, q: np.ndarray, obstacles) -> CollisionQuery:
        result = self.backend.collision_result(q, obstacles, margin=0.0)
        return CollisionQuery(
            result.in_collision,
            result.reason,
            result.first_link,
            result.first_obstacle,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--segment", default=0, type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    manifest = json.loads(args.plan.read_text(encoding="utf-8"))
    scene, cfg = load_scene_config(args.config)
    segment = manifest["segments"][args.segment]
    dock = np.asarray(segment["amr_dock_position"], dtype=float)
    robot, docked_scene, docked_cfg = docked_robot_and_scene(scene, cfg, dock)
    target = str(segment["target"])
    obstacles = docked_scene.obstacles_without({target})
    urdf_path = Path(docked_cfg["robot"]["urdf_path"])
    exact = PinocchioHppFclBackend(
        urdf_path,
        tip_frame=str(docked_cfg["robot"].get("tip_link", "tool0")),
        package_dirs=(urdf_path.parent,),
        srdf_path=urdf_path.with_suffix(".srdf"),
        base_transform=robot.base_transform,
        tool_length=float(docked_cfg["robot"].get("tool_length", 0.0)),
        name="fanuc_m20id35_exact",
    )
    report = compare_collision_backends(
        segment["path"],
        obstacles,
        CapsuleCollisionBackend(robot, margin=0.015),
        _PinocchioCollisionAdapter(exact),
    )
    report.update(
        {
            "segment": args.segment,
            "target": target,
            "amr_dock_position": dock.tolist(),
        }
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
