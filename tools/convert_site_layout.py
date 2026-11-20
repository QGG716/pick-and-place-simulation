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


def convert(output=None):
    raw=ROOT/'assets/layout/raw'
    paths=sorted(p for p in raw.rglob('*') if p.is_file() and p.suffix.lower() in ('.step','.stp'))
    if not paths:raise FileNotFoundError('WAITING_FOR_REAL_STEP: assets/layout/raw contains no STEP; no demo fallback')
    if len(paths)!=1:raise ValueError('MULTIPLE_SITE_LAYOUT_STEPS: exactly one input required')
    source=paths[0].resolve()
    if not source.is_relative_to(raw.resolve()):raise ValueError('SITE_LAYOUT_SOURCE_OUTSIDE_PRIVATE_RAW')
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    out=Path(output).resolve() if output else ROOT/'assets/layout/derived'/sha[:16]
    if not out.is_relative_to((ROOT/'assets/layout/derived').resolve()):raise ValueError('PRIVATE_LAYOUT_DERIVED_REQUIRED')
    report=convert_static_cad.convert(source,out,'layout',collision_mode='cad_aabb')
    if not report['parts'] or any(not p['topology_valid'] for p in report['parts']):
        raise ValueError('SITE_LAYOUT_INVALID_SOLID_TOPOLOGY')
    if any(n.get('non_surface_shapes') for n in report['assembly_nodes']):
        raise ValueError('SITE_LAYOUT_UNREPRESENTED_NON_SURFACE_PART')
    if any(b['enclosed_by'] is None for b in report['enclosed_surface_bodies']):
        raise ValueError('SITE_LAYOUT_UNREPRESENTED_SURFACE_BODY')
    report.update(schema='eco65_site_layout_cad_v1',basis='CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT',
        exclusion_policy='EXPLICIT_CONFIG_PART_IDS_WITH_BASIS_ONLY',registration='REQUIRED_NOT_RUN',
        physical_validation='NOT_RUN',source_kind='USER_SUPPLIED_PRIVATE_STEP',
        enclosure_policy='DEGENERATE_OR_INVALID_SURFACE_BODY_INSIDE_VALID_SOLID_AABB_FOLLOWS_HOST_PART')
    (out/'site-layout.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path)
    convert(p.parse_args().output)
