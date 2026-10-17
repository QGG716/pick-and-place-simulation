"""Build/check the finite-datum native model in the existing authorized rootfs."""
import argparse,shutil,time
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import ROOT,load_json,save_json
from unloading_sim.eco65.native_io import file_hash
from eco65_touch_native import build,NativeTouch


def prepare(configuration,output,worker,rootfs):
    c=load_json(configuration);started=time.perf_counter();ctx,g=build(c,output)
    resources=[];native=rootfs/'work-eco65-20261015'
    for row in ctx['bindings']:
        src=ROOT/row['path'];dst=native/row['path'];dst.resolve().relative_to(native.resolve());dst.parent.mkdir(parents=True,exist_ok=True)
        if dst.exists() and file_hash(dst)!=row['sha256']:raise ValueError('NATIVE_ASSET_ALREADY_EXISTS_WITH_DIFFERENT_HASH')
        shutil.copyfile(src,dst)
        if file_hash(dst)!=row['sha256']:raise ValueError('NATIVE_MESH_COPY_HASH')
        resources.append(dict(row,rootfs_readable=True))
    save_json(output/'rootfs_assets.json',resources)
    client=NativeTouch(ctx,c,g,output,worker)
    try:
        checks=[]
        if not any(p['split']=='validation' for p in c['photos']):raise ValueError('HELDOUT_PHOTO_STATES_REQUIRED')
        for photo in c['photos']:
            if photo['split']!='validation':continue
            value=client.check(photo['q_rad']);checks.append(dict(sample=photo['sample_id'],**value))
            if not value['accepted']:break
        save_json(output/'native_check.json',dict(status='PASS' if all(x['accepted'] for x in checks) else 'FAIL',checks=checks,seconds=time.perf_counter()-started))
        return all(x['accepted'] for x in checks)
    finally:client.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['configuration','output','rootfs']:p.add_argument('--'+k,type=Path,required=True)
    p.add_argument('--worker-command',required=True);a=p.parse_args()
    raise SystemExit(0 if prepare(a.configuration,a.output,a.worker_command,a.rootfs) else 1)
