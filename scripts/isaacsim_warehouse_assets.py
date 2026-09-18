"""Resolve and cache the installed Isaac release's official Warehouse assets.

Run with the existing Isaac Python; never upgrades packages or edits source USDs.
The cache and detailed manifest must be in an ignored/private output directory.
"""
import argparse
import concurrent.futures
import hashlib
import json
import re
import time
from pathlib import Path
from urllib.parse import urljoin, urlparse, unquote
from urllib.request import urlopen


def cache_assets(root, relative_paths, directory):
    from pxr import UsdUtils
    directory = Path(directory).resolve()
    base = root.rstrip('/') + '/'
    seen, records, pending = set(), [], [urljoin(base, p.lstrip('/')) for p in relative_paths]

    def fetch(url):
        print("FETCH", url, flush=True)
        parsed = urlparse(url)
        if parsed.scheme != 'https' or parsed.hostname != urlparse(base).hostname:
            raise ValueError(f'Unexpected dependency origin: {url}')
        dest = directory / unquote(parsed.path).lstrip('/')
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            for attempt in range(5):
                try:
                    with urlopen(url, timeout=30) as r:
                        data = r.read()
                    break
                except Exception as error:
                    print('RETRY', attempt + 1, url, type(error).__name__, flush=True)
                    if attempt == 4: raise
                    time.sleep(1 + attempt)
            temp = dest.with_suffix(dest.suffix + '.part')
            temp.write_bytes(data)
            temp.replace(dest)
        data = dest.read_bytes()
        if not data or data[:5].lower() == b'<html':
            raise ValueError(f'Invalid asset response: {url}')
        return url, dest, dict(url=url, path=dest.relative_to(directory).as_posix(), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())

    while pending:
        batch = sorted(set(pending) - seen)
        pending = []
        if not batch:
            break
        seen.update(batch)
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            futures = [executor.submit(fetch, url) for url in batch]
            for future in concurrent.futures.as_completed(futures):
                try:
                    url, dest, record = future.result()
                except Exception as error:
                    print('DOWNLOAD_FAILED', repr(error), flush=True)
                    raise
                records.append(record)
                print('CACHED', record['path'], record['bytes'], flush=True)
                if dest.suffix.lower() in ('.usd', '.usda', '.usdc'):
                    sublayers, references, payloads = UsdUtils.ExtractExternalReferences(str(dest))
                    for reference in (*sublayers, *references, *payloads):
                        if not reference:
                            continue
                        # MDL modules shipped with Kit are resolved by its search path.
                        if reference.endswith('.mdl') and '/' not in reference:
                            record.setdefault('kit_mdl_modules', []).append(reference)
                        else:
                            pending.append(urljoin(url, reference))
                elif dest.suffix.lower() == '.mdl':
                    source = dest.read_text(encoding='utf-8-sig')
                    for texture in re.findall(r'texture_\w+\(\s*"([^"\n]+)"', source):
                        pending.append(urljoin(url, texture))
                    for module in re.findall(r'(?:using|import)\s+\.::([A-Za-z0-9_:]+)', source):
                        pending.append(urljoin(url, module.replace('::', '/') + '.mdl'))
    records.sort(key=lambda item: item['path'])
    manifest = dict(asset_root=base.rstrip('/'), files=records, file_count=len(records), total_bytes=sum(r['bytes'] for r in records))
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return manifest


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 640, 'height': 480})
    try:
        import carb.settings
        import omni.kit.app
        import importlib.metadata
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Usd, UsdGeom, UsdPhysics
        root = get_assets_root_path()
        if not root:
            raise RuntimeError('Installed Isaac asset root unavailable')
        runtime = dict(isaacsim=importlib.metadata.version('isaacsim'), kit=omni.kit.app.get_app().get_build_version(), asset_root=root,
                       configured_root=carb.settings.get_settings().get('/persistent/isaac/asset_root/default'))
        (args.output / 'runtime.json').write_text(json.dumps(runtime, indent=2))
        print('RUNTIME', json.dumps(runtime), flush=True)
        relatives = ['/Isaac/Environments/Simple_Warehouse/warehouse.usd', '/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd']
        manifest = cache_assets(root, relatives, args.output / 'official')
        results = []
        for relative in relatives:
            path = args.output / 'official' / urlparse(root + relative).path.lstrip('/')
            stage = Usd.Stage.Open(str(path))
            if stage and stage.GetCompositionErrors():
                raise RuntimeError(str(stage.GetCompositionErrors()))
            if not stage:
                raise RuntimeError(f'Failed loading {path}')
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'], useExtentsHint=False)
            objects = []
            for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
                if prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
                    b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                    objects.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName(), collision=prim.HasAPI(UsdPhysics.CollisionAPI),
                                        bounds=[list(b.GetMin()), list(b.GetMax())]))
            results.append(dict(source=root+relative, path=str(path), default_prim=str(stage.GetDefaultPrim().GetPath()),
                                up_axis=str(UsdGeom.GetStageUpAxis(stage)), meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage), objects=objects))
        (args.output / 'asset_inspection.json').write_text(json.dumps(results, indent=2))
        print('ASSETS_LOADED', len(results), 'FILES', manifest['file_count'], flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
