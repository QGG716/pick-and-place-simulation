"""Run separately on Linux host and inside ROS rootfs after unpacking assets."""
import argparse
import hashlib
import json
import platform
import xml.etree.ElementTree as ET
from urllib.parse import urlparse, unquote
from pathlib import Path
from time import perf_counter
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.eco65.asset_inventory import ROOT, load_json, asset_inventory


def save_json(path, value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def verify(root, manifest, scope):
    started = perf_counter(); data = load_json(manifest)
    if data['schema'] != 'eco65_private_transfer_v1' or not data['files']:
        raise ValueError('TRANSFER_MANIFEST_INVALID')
    failures = []
    for name, expected in data['files'].items():
        try:
            actual = hashlib.sha256(resolve_asset(root, name).read_bytes()).hexdigest()
            if actual != expected: failures.append(dict(path=name, reason='HASH_MISMATCH'))
        except (ValueError, OSError) as error: failures.append(dict(path=name, reason=str(error)))
    audit = asset_inventory(root)
    # A truncated package manifest cannot pass merely because those few files exist.
    for name, expected in audit['asset_sha256'].items():
        if data['files'].get(name) != expected: failures.append(dict(path=name, reason='TRANSFER_COVERAGE_MISMATCH'))
    if data.get('native_model_path'):
        model=load_json(resolve_asset(root,data['native_model_path']))
        if model['asset_audit']['asset_sha256'] != audit['asset_sha256']:
            failures.append(dict(reason='MODEL_ASSET_BINDING_MISMATCH'))
        for row in model['convex_bindings']:
            for path_key, hash_key in [('source','source_sha256'),('convex_path','convex_sha256')]:
                if data['files'].get(row[path_key])!=row[hash_key]:
                    failures.append(dict(path=row[path_key],reason='CONVEX_BINDING_MISMATCH'))
        if scope=='ROS_ROOTFS':
            urdf=ET.fromstring(model['init']['parameters']['robot_description'])
            for mesh in urdf.findall('.//mesh'):
                uri=urlparse(mesh.get('filename')); path=Path(unquote(uri.path))
                if uri.scheme!='file' or uri.netloc: raise ValueError('UNEXPECTED_RESOURCE_URI')
                rel=path.resolve().relative_to(Path(root).resolve()).as_posix()
                if hashlib.sha256(path.read_bytes()).hexdigest()!=data['files'].get(rel):
                    failures.append(dict(path=rel,reason='ROOTFS_RESOURCE_IDENTITY_MISMATCH'))
    if data.get('home_candidate_path') and scope!='ROS_ROOTFS':
        from unloading_sim.eco65.home_candidate import verify_candidate
        from unloading_sim.eco65.native_scene import combined_scene
        from unloading_sim.eco65.native_scene import CONFIG
        from unloading_sim.eco65.native_model import load_robot_config
        verify_candidate(load_json(resolve_asset(root,data['home_candidate_path'])),
            load_json(resolve_asset(root,load_robot_config()['tool_config'])),
            combined_scene(config=resolve_asset(root,CONFIG.relative_to(ROOT).as_posix()),root=root))
    if scope in ('LINUX_HOST', 'ROS_ROOTFS') and platform.system() != 'Linux':
        failures.append(dict(reason='REQUESTED_SCOPE_IS_NOT_LINUX'))
    return dict(status='PASS' if not failures and audit['status']=='PASS' else 'FAIL',
        scope=scope, platform=platform.system(), local_mount=str(Path(root).resolve()),
        manifest_sha256=hashlib.sha256(Path(manifest).read_bytes()).hexdigest(),
        files_read=len(data['files']), failures=failures, asset_audit=audit,
        home_candidate_check='BYTE_IDENTITY_ONLY' if scope=='ROS_ROOTFS' else 'SEMANTIC_AND_BYTE_IDENTITY',
        wall_s=perf_counter()-started)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=ROOT); p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--scope', choices=['LOCAL', 'LINUX_HOST', 'ROS_ROOTFS'], required=True)
    p.add_argument('--output', type=Path, required=True); a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    result=verify(a.root, a.manifest, a.scope); save_json(a.output, result)
    print(result['status'], result['files_read'], result['wall_s']); raise SystemExit(0 if result['status']=='PASS' else 2)
