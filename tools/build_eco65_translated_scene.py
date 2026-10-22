"""Remount an existing approved scene by a finite translation, preserving geometry."""
import argparse
from pathlib import Path
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.base_translation import translate_installation
from unloading_sim.moveit2_backend import digest


def build(a):
    old = verify_snapshot(load_json(a.source))
    out = a.output.resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True,exist_ok=False)
    scene = translate_installation(old['scene'], a.delta, a.target)
    verify_scene(scene)
    model = build_native_model(scene, out/'model', native_asset_root=a.native_asset_root)
    assets = dict(old['asset_sha256'])
    assets.update({r['convex_path']:r['convex_sha256'] for r in model['convex_bindings']})
    snapshot = dict(schema=old['schema'],scene=scene,tool=model['tool'],native_model=model,
                    asset_sha256=assets,config_sha256=old['config_sha256'])
    snapshot['fingerprint'] = digest(snapshot); verify_snapshot(snapshot)
    save_json(out/'snapshot.json', snapshot)
    report = dict(status='BUILT_REQUIRES_NEW_NATIVE_CHECK',**scene['base_translation'],
        source_snapshot_fingerprint=old['fingerprint'],snapshot_fingerprint=snapshot['fingerprint'],
        target_id=a.target,world_cartons=len(scene['boxes']),planning_only=True,
        physical_execution='NOT_RUN',execution_ready=False)
    save_json(out/'build_report.json',report); print(report)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--delta',type=float,nargs=3,required=True); p.add_argument('--target',required=True)
    p.add_argument('--native-asset-root',default='/work-eco65-20261015')
    build(p.parse_args())
