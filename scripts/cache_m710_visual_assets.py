"""Prepare external visual caches using installed Isaac; never vendor asset bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fetch(url,path,expected=None):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists() and (expected is None or sha(path)==expected): return
    temporary=path.with_suffix(path.suffix+'.part')
    urllib.request.urlretrieve(url.replace(' ','%20'),temporary)
    if expected and sha(temporary)!=expected: raise ValueError('asset SHA mismatch: '+url)
    temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--cache',type=Path,required=True)
    args=parser.parse_args();config=json.loads(args.config.read_text())
    for field,folder in [('cartons','cartons'),('conveyors','workcell')]:
        manifest=json.loads((args.config.parent/config[field]).read_text());archives={a['id']:a for a in manifest.get('archives',[])}
        for entry in manifest['dependencies']:
            target=(args.cache/folder/entry['path']).resolve()
            assert target.is_relative_to((args.cache/folder).resolve())
            if target.exists() and sha(target)==entry['sha256']:continue
            if 'url' in entry: fetch(entry['url'],target,entry['sha256']);continue
            archive=archives[entry['archive']];archive_path=args.cache/folder/archive['path']
            fetch(archive['url'],archive_path,archive['sha256'])
            with zipfile.ZipFile(archive_path) as z:
                data=z.read(entry['member']);assert hashlib.sha256(data).hexdigest()==entry['sha256']
                target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True})
    from isaacsim.storage.native import get_assets_root_path
    import omni.client
    from pxr import Usd,UsdGeom
    root=get_assets_root_path()+'/Isaac'
    relative='Robots/Idealworks/iwhub/iw_hub_static.usd';url=root+'/'+relative
    assert omni.client.stat(url)[0]==omni.client.Result.OK,url
    local=args.cache/config['chassis']['relative_path']
    if not local.exists():
        result=omni.client.copy(root+'/Robots/Idealworks/iwhub',str(local.parent.resolve()))
        assert result==omni.client.Result.OK,result
    stage=Usd.Stage.Open(str(local));assert stage and stage.GetDefaultPrim()
    files=[dict(path=str(p.relative_to(args.cache)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(local.parent.rglob('*')) if p.is_file()]
    evidence=dict(url=url,asset_root=root,default_prim=str(stage.GetDefaultPrim().GetPath()),
        meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),up_axis=UsdGeom.GetStageUpAxis(stage),files=files)
    (args.cache/'iwhub_cache_manifest.json').write_text(json.dumps(evidence,indent=2))
    app.close()


if __name__=='__main__': main()
