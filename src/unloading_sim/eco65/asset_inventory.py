"""Asset byte/identity checks without geometry or simulator dependencies."""
import hashlib
import json
from pathlib import Path
import yaml
from .asset_paths import resolve_asset

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT/'configs/robots/eco65_b_native.yaml'


def load_json(path): return json.loads(Path(path).read_text(encoding='utf-8'))


def load_robot_config(path=CONFIG):
    c = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if (c['schema'] != 'eco65_native_robot_v1' or c['robot_variant'] != 'ECO65-B' or
            c['joint_names'] != [f'joint_{i}' for i in range(1, 7)] or
            c['flange_link'] != 'link_6' or c['geometry_scale'] != 1 or
            c['required_seal_count'] != 12 or c['contact_geometry_type'] != 'cad_seal_contours_v1'):
        raise ValueError('ECO65_ROBOT_CONFIG_MISMATCH')
    return c


def asset_inventory(root=ROOT, config=None):
    root = Path(root); c = config or load_robot_config()
    manifest = load_json(root/c['source_manifest'])
    if hashlib.sha256((root/c['source_manifest']).read_bytes()).hexdigest() != c['source_manifest_sha256']:
        raise ValueError('PINNED_OFFICIAL_MANIFEST_CHANGED')
    official = root/Path(c['source_manifest']).parent
    entries = [e for e in manifest['entries'] if e['local_path'].startswith(
        'description/rm_models/ECO65/urdf/ECO65-B/')]
    if len(entries) != 16: raise ValueError('PINNED_OFFICIAL_INVENTORY_INCOMPLETE')
    failures, hashes = [], {}
    for e in entries:
        p = official/e['local_path']; relative = p.relative_to(root).as_posix()
        if not p.is_file(): failures.append(dict(path=relative, reason='MISSING')); continue
        sha = hashlib.sha256(p.read_bytes()).hexdigest()
        if sha != e['sha256']: failures.append(dict(path=relative, reason='HASH_MISMATCH'))
        hashes[relative] = sha
    required = [c['tool_config'], 'assets/tools/desktop_suction/derived/manifests/cad.json']
    for relative in required:
        if not (root/relative).is_file(): failures.append(dict(path=relative, reason='MISSING'))
    if all((root/p).is_file() for p in required):
        tool = load_json(root/required[0]); cad = load_json(root/required[1])
        private = {cad['source'], *required}
        private.update(p['visual'] for p in tool['visual_parts'])
        private.update(p[k] for p in cad['parts'] for k in ('visual', 'collision'))
        private.update(p['mesh'] for p in tool['collisions'])
        for relative in sorted(private):
            p = resolve_asset(root, relative)
            if not p.is_file(): failures.append(dict(path=relative, reason='MISSING')); continue
            hashes[relative] = hashlib.sha256(p.read_bytes()).hexdigest()
        if hashes.get(cad['source']) != cad['source_sha256'] or tool['source_sha256'] != cad['source_sha256']:
            failures.append(dict(path=cad['source'], reason='CAD_SOURCE_IDENTITY_MISMATCH'))
        if tool.get('cad_manifest_sha256') and hashes.get(required[1]) != tool['cad_manifest_sha256']:
            failures.append(dict(path=required[1], reason='CAD_MANIFEST_IDENTITY_MISMATCH'))
        for row in cad['parts']:
            for field in ('visual', 'collision'):
                if hashes.get(row[field]) != row[field+'_sha256']:
                    failures.append(dict(path=row[field], reason='CAD_MESH_IDENTITY_MISMATCH'))
        for row in tool['visual_parts']:
            if row.get('visual_sha256') and hashes.get(row['visual']) != row['visual_sha256']:
                failures.append(dict(path=row['visual'], reason='VISUAL_IDENTITY_MISMATCH'))
        for row in tool['collisions']:
            if row.get('mesh_sha256') and hashes.get(row['mesh']) != row['mesh_sha256']:
                failures.append(dict(path=row['mesh'], reason='COLLISION_IDENTITY_MISMATCH'))
        if ({p['id'] for p in cad['parts']} != {p['id'] for p in tool['visual_parts']} or
                {p['id'] for p in cad['parts']} != {p['source_part'] for p in tool['collisions']}):
            failures.append(dict(path=required[0], reason='TOOL_SOLID_COVERAGE_MISMATCH'))
    return dict(status='PASS' if not failures else 'BLOCKED_ASSETS', failures=failures,
                asset_sha256=hashes, robot_variant=c['robot_variant'],
                private_geometry_verified=False, dynamics='NOT_EVALUATED')


