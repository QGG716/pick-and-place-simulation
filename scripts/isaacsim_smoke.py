"""Headless Isaac Sim smoke test for GPU rendering and rigid-body dynamics."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


OUTPUT_DIR = Path(
    os.environ.get("ISAACSIM_SMOKE_OUTPUT", "outputs/isaacsim_smoke")
).resolve()


def _array_data(value):
    if isinstance(value, dict) and "data" in value:
        return value["data"]
    return value


process_started_at = time.perf_counter()
simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": 320,
        "height": 240,
    }
)
app_startup_wall_s = time.perf_counter() - process_started_at

try:
    import numpy as np
    import omni.replicator.core as rep
    from PIL import Image
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
    world.scene.add_default_ground_plane()
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/FallingCube",
            name="falling_cube",
            position=np.array([0.0, 0.0, 2.0]),
            scale=np.array([0.5, 0.5, 0.5]),
            color=np.array([0.85, 0.1, 0.05]),
            mass=2.0,
        )
    )

    camera = rep.create.camera(position=(3.0, 3.0, 2.4), look_at=(0.0, 0.0, 0.4))
    rep.create.light(light_type="distant", intensity=3000.0, rotation=(315.0, 0.0, 0.0))
    render_product = rep.create.render_product(camera, (320, 240))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    rgb_annotator.attach(render_product)
    depth_annotator.attach(render_product)

    world.reset()
    initial_position, _ = cube.get_world_pose()
    step_started_at = time.perf_counter()
    for _ in range(180):
        world.step(render=True)
    step_wall_s = time.perf_counter() - step_started_at
    final_position, _ = cube.get_world_pose()

    rgba = np.asarray(_array_data(rgb_annotator.get_data()))
    depth = np.asarray(_array_data(depth_annotator.get_data()))
    rgb = rgba[..., :3]
    Image.fromarray(rgb.astype(np.uint8)).save(OUTPUT_DIR / "rgb.png")
    np.save(OUTPUT_DIR / "depth_m.npy", depth)

    finite_depth = np.isfinite(depth)
    result = {
        "renderer": "RaytracedLighting",
        "frames": 180,
        "physics_dt_s": 1.0 / 60.0,
        "simulated_time_s": 3.0,
        "app_startup_wall_s": app_startup_wall_s,
        "step_wall_s": step_wall_s,
        "rendered_step_rate_hz": 180.0 / step_wall_s,
        "simulation_realtime_factor": 3.0 / step_wall_s,
        "initial_cube_z_m": float(initial_position[2]),
        "final_cube_z_m": float(final_position[2]),
        "cube_drop_m": float(initial_position[2] - final_position[2]),
        "rgb_shape": list(rgba.shape),
        "rgb_std": float(np.std(rgb)),
        "depth_shape": list(depth.shape),
        "finite_depth_fraction": float(np.mean(finite_depth)),
        "rgb_path": str(OUTPUT_DIR / "rgb.png"),
        "depth_path": str(OUTPUT_DIR / "depth_m.npy"),
    }
    (OUTPUT_DIR / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True), encoding="utf-8"
    )
    print("ISAACSIM_SMOKE_RESULT=" + json.dumps(result, sort_keys=True), flush=True)

    physics_ok = result["cube_drop_m"] > 1.0 and 0.20 <= result["final_cube_z_m"] <= 0.30
    camera_ok = rgba.shape == (240, 320, 4) and result["rgb_std"] > 1.0 and depth.size > 0
    if not (physics_ok and camera_ok):
        print(
            f"Smoke validation failed: physics_ok={physics_ok}, camera_ok={camera_ok}",
            file=sys.stderr,
        )
        raise SystemExit(2)
finally:
    simulation_app.close()
