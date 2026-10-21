"""Build the new ECO65 coupled receiver from one verified small-carton snapshot."""
from __future__ import annotations
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import yaml

from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot, file_hash
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.coupled_scene import (CONFIG, build_coupled_scene,
    coupled_conveyor_state, receiving_height_candidates, export_state_usd)
from unloading_sim.moveit2_backend import digest
from unloading_sim.eco65.layout_installation import CONFIG as LAYOUT_CONFIG


def build(source, home, output, *, config=CONFIG, native_asset_root="/work-eco65-20261015", usd=False,
          camera_config=None, layout_config=None):
    started = perf_counter()
    old = verify_snapshot(load_json(source))
    out = Path(output).resolve()
    out.relative_to(ROOT/"outputs")
    out.mkdir(parents=True, exist_ok=False)
    scene = build_coupled_scene(old["scene"], config)
    # A grasp contract is independent of the optional camera remount.
    scene['grasp_edge_alignment_required'] = True
    config = Path(config).resolve()
    hashes = dict(old["config_sha256"])
    extra_assets = {}
    if camera_config:
        from unloading_sim.eco65.camera_installation import apply_camera_installation
        from unloading_sim.eco65.collision_policy import effective_collision_policy
        prior_pairs=effective_collision_policy(scene)['robot_mast_exemption']['exempt_pairs']
        camera_config = Path(camera_config).resolve()
        scene, extra_assets = apply_camera_installation(scene,
            yaml.safe_load(camera_config.read_text(encoding='utf-8')), out/'camera_attachments', ROOT)
        hashes[camera_config.relative_to(ROOT).as_posix()] = file_hash(camera_config)
        camera_pairs_unchanged = prior_pairs == effective_collision_policy(scene)['robot_mast_exemption']['exempt_pairs']
        camera_ids_unchanged = ([r['id'] for r in old['scene']['moving_attachments']]
                               == [r['id'] for r in scene['moving_attachments']])
    if layout_config:
        from unloading_sim.eco65.layout_installation import apply_layout_installation
        layout_config = Path(layout_config).resolve()
        scene, layout_assets = apply_layout_installation(scene,
            yaml.safe_load(layout_config.read_text(encoding='utf-8')), out/'layout_attachments', ROOT)
        extra_assets.update(layout_assets)
        hashes[layout_config.relative_to(ROOT).as_posix()] = file_hash(layout_config)
        save_json(out/'layout_installation.json', scene['layout_installation'])
    hashes[config.relative_to(ROOT).as_posix()] = file_hash(config)
    scene["config_sha256"] = hashes
    scene["scene_fingerprint"] = digest({k: v for k, v in scene.items() if k != "scene_fingerprint"})
    verify_scene(scene)
    model = build_native_model(scene, out/"model", native_asset_root=native_asset_root)
    assets = dict(old["asset_sha256"])
    assets.update(extra_assets)
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
                  base_rpy_rad=scene.get('base_rpy_rad',[0.,0.,0.]),
                  layout_installation=scene.get('layout_installation'),
                  source_belt_bounds_D_m=spec["source_belt_bounds_D_m"],
                  receiving_height_candidates=candidates, previews=previews,
                  replaced_object_ids=spec["replaced_object_ids"],
                  replaced_obstacle_ids=spec["replaced_obstacle_ids"],
                  engineering_geometry=spec["engineering_geometry"],
                  wall_s=perf_counter()-started)
    save_json(out/"build_report.json", report)
    if camera_config:
        import numpy as np
        from eco65_camera_mount import mount_check
        camera=scene['permanent_camera']
        installation=dict(axis_rotation=camera['axis_rotation'],mount=camera['mount'],
            mount_check=mount_check(np.asarray(camera['T_link1_optical']),np.eye(4),camera),
            nominal_extrinsics=scene['camera_extrinsics'],
            prior_mast_exemption_pairs_unchanged=camera_pairs_unchanged)
        # The original small-carton source may precede the mast exemption.
        # Current ID inventory still must be unchanged by camera remounting.
        installation['moving_attachment_ids_unchanged']=camera_ids_unchanged
        installation['inventory_check_scope']='CAMERA_REMOUNT_BEFORE_OPTIONAL_BASE_MAST_LAYOUT_CHANGE'
        if (not installation['mount_check']['accepted'] or not installation['moving_attachment_ids_unchanged']
                or not installation['prior_mast_exemption_pairs_unchanged']):
            raise ValueError('CAMERA_INSTALLATION_BINDING_INVALID')
        save_json(out/'camera_installation.json',installation)
        save_json(out/'nominal_camera_extrinsics.json',scene['camera_extrinsics'])
    elif layout_config:
        save_json(out/'nominal_camera_extrinsics.json',scene['camera_extrinsics'])
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "home", "output"):
        parser.add_argument("--"+key, type=Path, required=True)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--native-asset-root", default="/work-eco65-20261015")
    parser.add_argument("--usd", action="store_true")
    parser.add_argument("--camera-config", type=Path)
    parser.add_argument("--layout-config", type=Path,
                        help='Fixed base/mast installation; use '+str(LAYOUT_CONFIG.relative_to(ROOT)))
    args = parser.parse_args()
    print(build(args.source, args.home, args.output, config=args.config,
                native_asset_root=args.native_asset_root, usd=args.usd, camera_config=args.camera_config,
                layout_config=args.layout_config))
