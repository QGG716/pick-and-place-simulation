"""Build the new ECO65 coupled receiver from one verified small-carton snapshot."""
from __future__ import annotations
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter

from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.coupled_scene import (CONFIG, build_coupled_scene,
    coupled_conveyor_state, receiving_height_candidates, export_state_usd)
from unloading_sim.moveit2_backend import digest


def build(source, home, output, *, config=CONFIG, native_asset_root="/work-eco65-20261015", usd=False):
    started = perf_counter()
    old = verify_snapshot(load_json(source))
    out = Path(output).resolve()
    out.relative_to(ROOT/"outputs")
    out.mkdir(parents=True, exist_ok=False)
    scene = build_coupled_scene(old["scene"], config)
    config = Path(config).resolve()
    hashes = dict(old["config_sha256"])
    hashes[config.relative_to(ROOT).as_posix()] = file_hash(config)
    scene["config_sha256"] = hashes
    scene["scene_fingerprint"] = digest({k: v for k, v in scene.items() if k != "scene_fingerprint"})
    verify_scene(scene)
    model = build_native_model(scene, out/"model", native_asset_root=native_asset_root)
    assets = dict(old["asset_sha256"])
    assets.update({row["convex_path"]: row["convex_sha256"] for row in model["convex_bindings"]})
    snapshot = dict(schema="eco65_native_snapshot_v1", scene=scene, tool=model["tool"],
                    native_model=model, asset_sha256=assets, config_sha256=hashes)
    snapshot["fingerprint"] = digest(snapshot)
    verify_snapshot(snapshot)
    save_json(out/"snapshot.json", snapshot)
    save_json(out/"native_model_prepared.json", model)
    h = load_json(home)
    candidate = make_candidate(h["q_rad"], model["tool"], scene,
        dict(source="COUPLED_LAYOUT_HOME_PROPOSAL_REQUIRES_NEW_NATIVE_CHECK",
             source_home_sha256=file_hash(home)))
    save_json(out/"home_candidate.json", candidate)
    spec = scene["coupled_conveyor_spec"]
    candidates = receiving_height_candidates(spec, scene["box"])
    heights = [("low", spec["low_height_m"])]
    heights += [(row["name"], row["height_m"]) for row in candidates if row["in_hardware_range"]]
    heights += [("hardware_max", spec["high_height_m"])]
    previews = []
    for name, height in heights:
        state = coupled_conveyor_state(spec, height)
        save_json(out/("mechanism_"+name+".json"), state)
        if usd:
            export_state_usd(state, out/("mechanism_"+name+".usda"))
        previews.append(dict(name=name, height_m=height, scene_fingerprint=state["scene_fingerprint"],
                             object_count=len(state["world_objects"]),
                             longitudinal_angle_rad=state["tilt_rad"]))
    report = dict(status="BUILT_NOT_RUNTIME_VALIDATED", planning_only=True,
                  physical_execution="NOT_RUN", execution_ready=False,
                  snapshot_fingerprint=snapshot["fingerprint"],
                  source_snapshot_fingerprint=old["fingerprint"],
                  geometry_revision=spec["geometry_revision"],
                  configuration_sha256=hashes[config.relative_to(ROOT).as_posix()],
                  box_ids=[row["id"] for row in scene["boxes"]],
                  target_id=scene["box"]["id"], box_size_m=scene["box"]["size_m"],
                  base_position_m=deepcopy(scene["base_position_m"]),
                  source_belt_bounds_D_m=spec["source_belt_bounds_D_m"],
                  receiving_height_candidates=candidates, previews=previews,
                  replaced_object_ids=spec["replaced_object_ids"],
                  replaced_obstacle_ids=spec["replaced_obstacle_ids"],
                  engineering_geometry=spec["engineering_geometry"],
                  wall_s=perf_counter()-started)
    save_json(out/"build_report.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "home", "output"):
        parser.add_argument("--"+key, type=Path, required=True)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--native-asset-root", default="/work-eco65-20261015")
    parser.add_argument("--usd", action="store_true")
    args = parser.parse_args()
    print(build(args.source, args.home, args.output, config=args.config,
                native_asset_root=args.native_asset_root, usd=args.usd))
