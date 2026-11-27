"""Convert the single private real-site STEP; never selects a demo layout.

Mesh and assembly metadata retain CAD design geometry provenance. Analytic
BRep bounding boxes conservatively enclose curved surfaces and tessellation.
No part is automatically excluded based on its name, shape, or dimensions.
"""
import argparse
import hashlib
import json
from pathlib import Path
import convert_static_cad

ROOT=Path(__file__).resolve().parents[1]


SURFACE_BOX='analytic_surface_body_aabb'


def represent_unenclosed_surface_bodies(report,out):
    """Give every unenclosed degenerate/invalid surface body its own box part.

    The analytic BRep bounds enclose the body; the box mesh serves as both
    visual and collision so hashes bind it like every other part."""
    import numpy as np
    import trimesh
    made=[]
    for k,body in enumerate(b for b in report['enclosed_surface_bodies'] if b['enclosed_by'] is None):
        lo,hi=np.asarray(body['analytic_bounds_m'],float)
        if lo.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<lo):raise ValueError('SITE_LAYOUT_SURFACE_BODY_BOUNDS_INVALID')
        hi=np.maximum(hi,lo+1e-6)  # a planar sheet still gets a finite box
        pid='layout_surface_%03d'%k
        mesh=trimesh.creation.box(extents=hi-lo);mesh.apply_translation((lo+hi)/2)
        path=out/(pid+'.obj');mesh.export(path)
        rel=path.relative_to(ROOT).as_posix();digest=hashlib.sha256(path.read_bytes()).hexdigest()
        report['parts'].append(dict(id=pid,instance=body['instance'],name='SURFACE_BODY_AABB',bounds_m=[lo.tolist(),hi.tolist()],
            size_m=(hi-lo).tolist(),rgb=None,topology_valid=False,visual=rel,visual_sha256=digest,collision=rel,
            collision_sha256=digest,collision_type=SURFACE_BOX,collision_bounds_m=[lo.tolist(),hi.tolist()],planar_faces=[],
            body_kind=body['body_kind'],body_name=body['body_name'],step_entity_type=body['step_entity_type'],
            untriangulated_faces=body['untriangulated_faces'],collision_obb=None))
        body.update(enclosed_by=pid,representation='OWN_ANALYTIC_AABB_FIXED_BOX');made.append(pid)
    return made


def convert(output=None,source=None):
    raw=ROOT/'assets/layout/raw'
    paths=sorted(p for p in raw.rglob('*') if p.is_file() and p.suffix.lower() in ('.step','.stp'))
    if not paths:raise FileNotFoundError('WAITING_FOR_REAL_STEP: assets/layout/raw contains no STEP; no demo fallback')
    if source is not None:
        source=Path(source).resolve()
        if source not in [p.resolve() for p in paths]:raise ValueError('SITE_LAYOUT_SOURCE_NOT_IN_PRIVATE_RAW')
    elif len(paths)!=1:raise ValueError('MULTIPLE_SITE_LAYOUT_STEPS: pass --source to select one explicitly')
    else:source=paths[0].resolve()
    if not source.is_relative_to(raw.resolve()):raise ValueError('SITE_LAYOUT_SOURCE_OUTSIDE_PRIVATE_RAW')
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    out=Path(output).resolve() if output else ROOT/'assets/layout/derived'/sha[:16]
    if not out.is_relative_to((ROOT/'assets/layout/derived').resolve()):raise ValueError('PRIVATE_LAYOUT_DERIVED_REQUIRED')
    report=convert_static_cad.convert(source,out,'layout',collision_mode='cad_aabb')
    if not report['parts'] or any(not p['topology_valid'] for p in report['parts']):
        raise ValueError('SITE_LAYOUT_INVALID_SOLID_TOPOLOGY')
    if any(n.get('non_surface_shapes') for n in report['assembly_nodes']):
        raise ValueError('SITE_LAYOUT_UNREPRESENTED_NON_SURFACE_PART')
    own_boxes=represent_unenclosed_surface_bodies(report,out)
    if any(b['enclosed_by'] is None for b in report['enclosed_surface_bodies']):
        raise ValueError('SITE_LAYOUT_UNREPRESENTED_SURFACE_BODY')
    report.update(schema='eco65_site_layout_cad_v1',basis='CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT',
        exclusion_policy='EXPLICIT_CONFIG_PART_IDS_WITH_BASIS_ONLY',registration='REQUIRED_NOT_RUN',
        physical_validation='NOT_RUN',source_kind='USER_SUPPLIED_PRIVATE_STEP',
        enclosure_policy='DEGENERATE_OR_INVALID_SURFACE_BODY_INSIDE_VALID_SOLID_AABB_FOLLOWS_HOST_PART',
        unenclosed_surface_policy=dict(mode=SURFACE_BOX,parts=own_boxes,
            basis='A degenerate/invalid surface body that no single valid solid AABB encloses keeps its own analytic '
                  'BRep AABB as a fixed box part (conservative, never dropped); first needed for layout 20261127 '
                  '(30L compressor internals)'))
    (out/'site-layout.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path)
    p.add_argument('--source',type=Path,help='STEP under assets/layout/raw (required when several exist)')
    a=p.parse_args();convert(a.output,a.source)
