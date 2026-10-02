"""Read-only fixed-resolution / encoder experiment review; no fitted tolerances."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def checked(ref):
    raw=Path(ref['path']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=ref['sha256']:raise ValueError('HASH_MISMATCH')
    return raw


def load(folder):
    response=json.loads((folder/'mode_b1_worker_response.json').read_text())
    metrics=json.loads(checked(response['metrics_reference']))
    for ref in metrics['artifacts'].values():checked(ref)
    artifacts=metrics['artifacts']
    masks=np.load(artifacts['cargo_masks.npz']['path'])
    return dict(masks={int(i):m for i,m in zip(masks['mask_ids'],masks['masks'])},
        entries=json.loads(checked(artifacts['cargo_instances.json']))['instances'],
        records=json.loads((folder/'rgbd_cuboids.json').read_text())['instances'],
        session=json.loads(checked(artifacts['sam_image_session.json'])),metrics=metrics,
        module_timings=json.loads((folder/'rgbd_stage_timing.json').read_text()))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('a','b','c','plan','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args();a,b,c=[load(getattr(args,k)) for k in ('a','b','c')]
    gates=json.loads(args.plan.read_text())['resolution_gates']
    from scipy.ndimage import binary_erosion,distance_transform_edt
    mask_exact=list(a['masks'])==list(b['masks']) and all(np.array_equal(m,b['masks'][i]) for i,m in a['masks'].items())
    reuse=dict(masks_exact=mask_exact,entries_exact=a['entries']==b['entries'],geometry_exact=a['records']==b['records'])
    rows=[];roster=list(b['masks'])==list(c['masks'])
    for identity,mask in b['masks'].items():
        other=c['masks'].get(identity);row=dict(mask_id=identity,pass_quality=False);rows.append(row)
        if other is None:row['reason']='MISSING_INSTANCE';continue
        x=mask&~binary_erosion(mask);y=other&~binary_erosion(other)
        distances=np.r_[distance_transform_edt(~x)[y],distance_transform_edt(~y)[x]]
        row.update(mask_iou=float(np.count_nonzero(mask&other)/max(1,np.count_nonzero(mask|other))),
            boundary_p95_native_px=float(np.quantile(distances,.95)) if len(distances) else None)
        old=next(r for r in b['records'] if r['mask_id']==identity)
        new=next(r for r in c['records'] if r['mask_id']==identity)
        row.update(faces_full=len(old['camera_facing_faces']),faces_half=len(new['camera_facing_faces']))
        main_ok=True
        if old['camera_facing_faces']:
            def main_face(r):return max(r['camera_facing_faces'],key=lambda f:f['quality']['retained_pixels'])
            if not new['camera_facing_faces']:main_ok=False
            else:
                f,g=main_face(old),main_face(new)
                row.update(main_normal_degrees=float(np.degrees(np.arccos(np.clip(np.dot(f['plane_normal'],g['plane_normal']),-1,1)))),
                    main_offset_m=abs(f['plane_offset_m']-g['plane_offset_m']),
                    main_support_ratio=g['quality']['retained_pixels']/f['quality']['retained_pixels'])
                main_ok=(row['main_normal_degrees']<=gates['maximum_main_normal_degrees'] and
                    row['main_offset_m']<=gates['maximum_main_offset_m'] and row['main_support_ratio']>=gates['minimum_main_support_ratio'])
        row['pass_quality']=bool(row['mask_iou']>=gates['minimum_mask_iou'] and row['boundary_p95_native_px'] is not None and
            row['boundary_p95_native_px']<=gates['maximum_boundary_p95_native_px'] and main_ok and row['faces_half']>=row['faces_full'])
    report=dict(encoder_reuse=reuse,resolution=dict(gates=gates,roster_exact=roster,instances=rows,
        pass_quality=roster and all(r['pass_quality'] for r in rows)),
        arms=[dict(name=k,session={key:value for key,value in v['session'].items() if key not in ('actual_prompts','prompts_source_coordinates')},
            sam_timings=v['metrics']['timings_seconds'],geometry_timings=v['module_timings']) for k,v in zip('ABC',(a,b,c))],
        note='A restarted only after wrapper failure, its completed SAM reused; failed C mapping attempt retained separately. No fastest-run selection.')
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(encoder_reuse=reuse,resolution_pass=report['resolution']['pass_quality'],failed_masks=[r['mask_id'] for r in rows if not r['pass_quality']]),indent=2))
    return 0 if all(reuse.values()) else 1


if __name__=='__main__':raise SystemExit(main())
