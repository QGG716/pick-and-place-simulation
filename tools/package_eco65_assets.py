"""Prepare a private portable bundle; preparation is not target verification."""
import argparse
import hashlib
from pathlib import Path
import tarfile
from time import perf_counter
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.eco65.native_model import asset_inventory


def package(output, model_path=None, home_path=None):
    started=perf_counter(); out=Path(output).resolve(); out.relative_to(ROOT/'outputs')
    out.mkdir(parents=True,exist_ok=False); audit=asset_inventory()
    if audit['status']!='PASS': raise ValueError(audit)
    files=dict(audit['asset_sha256']); manifest=dict(schema='eco65_private_transfer_v1',files=files)
    def add(path, expected=None):
        rel=Path(path).resolve().relative_to(ROOT).as_posix()
        actual=hashlib.sha256(resolve_asset(ROOT,rel).read_bytes()).hexdigest()
        if expected and actual!=expected: raise ValueError('PACKAGE_BINDING_CHANGED:'+rel)
        files[rel]=actual; return rel
    if model_path:
        manifest['native_model_path']=add(model_path)
        for row in load_json(model_path)['convex_bindings']:
            add(resolve_asset(ROOT,row['source']),row['source_sha256'])
            add(resolve_asset(ROOT,row['convex_path']),row['convex_sha256'])
    if home_path: manifest['home_candidate_path']=add(home_path)
    save_json(out/'private_asset_manifest.json',manifest)
    archive=out/'private_assets.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        for rel in sorted(files): tar.add(resolve_asset(ROOT,rel),arcname=rel,recursive=False)
        tar.add(out/'private_asset_manifest.json',arcname='private_asset_manifest.json',recursive=False)
    result=dict(status='PREPARED_NOT_SENT',files=len(files),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        linux_host_verification='NOT_RUN',ros_rootfs_verification='NOT_RUN',wall_s=perf_counter()-started)
    save_json(out/'transfer_preparation.json',result); print(result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',type=Path); p.add_argument('--home-candidate',type=Path)
    a=p.parse_args(); package(a.output,a.model,a.home_candidate)
