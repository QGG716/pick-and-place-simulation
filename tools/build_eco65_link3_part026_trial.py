"""Bind the user-authorized exact pair to an existing scene; geometry stays fixed."""
import argparse
from copy import deepcopy
from pathlib import Path
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.home_candidate import verify_candidate
from unloading_sim.eco65.collision_policy import ROBOT_TOOL_MODE
from unloading_sim.moveit2_backend import digest


def build(a):
    source = verify_snapshot(load_json(a.snapshot))
    home = verify_candidate(load_json(a.home_candidate), source['tool'], source['scene'])
    out = a.output.resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True, exist_ok=False)
    scene = deepcopy(source['scene'])
    scene['robot_tool_exemption_mode'] = ROBOT_TOOL_MODE
    scene['collision_policy_revision'] = dict(
        source_snapshot_fingerprint=source['fingerprint'], physical_layout_changed=False,
        calibration_reused_without_changes=True, exact_added_pairs=[['link_3', 'part_026']],
        reason='2026-10-22 explicit user simulation authorization')
    scene['scene_fingerprint'] = digest({k:v for k,v in scene.items() if k!='scene_fingerprint'})
    verify_scene(scene)
    model = build_native_model(scene, out/'model', native_asset_root=a.native_asset_root)
    assets = dict(source['asset_sha256'])
    assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    snapshot = dict(schema=source['schema'], scene=scene, tool=model['tool'], native_model=model,
                    asset_sha256=assets, config_sha256=source['config_sha256'])
    snapshot['fingerprint'] = digest(snapshot); verify_snapshot(snapshot)
    verify_candidate(home, snapshot['tool'], scene)
    save_json(out/'snapshot.json', snapshot); save_json(out/'home_candidate.json', home)
    report = dict(status='BUILT_REQUIRES_NATIVE_RUN', snapshot_fingerprint=snapshot['fingerprint'],
        **scene['collision_policy_revision'], planning_only=True, physical_execution='NOT_RUN', execution_ready=False)
    save_json(out/'build_report.json', report); print(report)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', required=True); p.add_argument('--home-candidate', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--native-asset-root', default='/work-eco65-20261015')
    build(p.parse_args())
