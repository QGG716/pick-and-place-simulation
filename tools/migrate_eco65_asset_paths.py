"""Hash-checked metadata migration only. Never re-convert STEP or rewrite meshes."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter

from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.asset_paths import asset_relative, resolve_asset
from unloading_sim.eco65.mount_alignment import relative_roll, check_alignment
from unloading_sim.eco65.native_model import asset_inventory

CAD = 'assets/tools/desktop_suction/derived/manifests/cad.json'
TOOL = 'configs/local/eco65_desktop/tool.json'


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def migrate(output, root=ROOT):
    started = perf_counter(); root = Path(root).resolve(); out = Path(output).resolve()
    out.relative_to(root/'outputs'); out.mkdir(parents=True, exist_ok=False)
    cad_path, tool_path = resolve_asset(root, CAD), resolve_asset(root, TOOL)
    old_cad, old_tool = load_json(cad_path), load_json(tool_path)
    if old_tool['cad_manifest_sha256'] != sha(cad_path): raise ValueError('OLD_CAD_BINDING_INVALID')
    cad, tool = deepcopy(old_cad), deepcopy(old_tool)
    before = {}; changes = []; rebound = []
    collision_bindings = {asset_relative(p['mesh'], migrate_windows=True): p['mesh_sha256'] for p in tool['collisions']}

    def field(row, key, expected=None):
        old = row[key]; new = asset_relative(old, migrate_windows=True)
        path = resolve_asset(root, new); actual = sha(path)
        if expected and actual != expected:
            # The historical assembly builder replaced same-name convex files.
            # Only its independently stored, exact tool binding can authorize
            # repairing the stale conversion hash; arbitrary changed bytes fail.
            if key != 'collision' or collision_bindings.get(new) != actual:
                raise ValueError('PRE_MIGRATION_HASH_MISMATCH:'+new)
            rebound.append(dict(path=new, old_sha256=expected, new_sha256=actual,
                                authority='existing_tool_collision_mesh_sha256'))
            row['collision_sha256'] = actual
        before[new] = actual; row[key] = new
        if old != new: changes.append(dict(old=old, new=new))

    field(cad, 'source', cad['source_sha256'])
    if tool['source_sha256'] != cad['source_sha256']: raise ValueError('OLD_SOURCE_BINDING_INVALID')
    for row in cad['parts']+tool['visual_parts']:
        for key in ('visual', 'collision'): field(row, key, row[key+'_sha256'])
    for row in tool['collisions']: field(row, 'mesh', row['mesh_sha256'])
    # Audit the existing transform first. Metadata migration must not move geometry.
    e = tool['mounting_evidence']
    official = root/'assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/meshes/link_6.STL'
    if sha(official) != e['official_flange_mesh_sha256']: raise ValueError('OFFICIAL_FLANGE_CHANGED')
    alignment = check_alignment(e['official_bolt_centers_xy_m'], e['bolt_centers_cad_xz_m'],
        e['mount_point_cad_m'][1], tool['T_flange_tool_cad'])
    if alignment['status'] != 'PASS': raise ValueError('EXISTING_MOUNT_REQUIRES_GEOMETRY_REVIEW:'+str(alignment))
    alignment['relative_roll_rad'] = relative_roll(e['official_bolt_centers_xy_m'], e['bolt_centers_cad_xz_m'])
    alignment['transform_changed'] = False
    e['final_alignment'] = alignment
    # Preserve exact old bytes, not just parsed values, before either write.
    backup = out/'before'; backup.mkdir()
    shutil.copy2(cad_path, backup/'cad.json'); shutil.copy2(tool_path, backup/'tool.json')
    new_cad = json.dumps(cad, ensure_ascii=False, indent=2).encode('utf-8')
    tool['cad_manifest_sha256'] = hashlib.sha256(new_cad).hexdigest()
    new_tool = json.dumps(tool, ensure_ascii=False, indent=2).encode('utf-8')
    try:
        cad_path.write_bytes(new_cad); tool_path.write_bytes(new_tool)
        after = {p: sha(resolve_asset(root, p)) for p in before}
        if before != after: raise ValueError('ASSET_BYTES_CHANGED_DURING_MIGRATION')
        audit = asset_inventory(root)
        if audit['status'] != 'PASS': raise ValueError('POST_MIGRATION_INVENTORY_FAILED:'+str(audit))
    except Exception:
        shutil.copy2(backup/'cad.json', cad_path); shutil.copy2(backup/'tool.json', tool_path)
        raise
    report = dict(status='MIGRATED_METADATA_ONLY', changed_paths=changes, collision_hash_rebindings=rebound, unchanged_asset_sha256=before,
        old_cad_sha256=sha(backup/'cad.json'), new_cad_sha256=sha(cad_path),
        old_tool_sha256=sha(backup/'tool.json'), new_tool_sha256=sha(tool_path),
        mount_alignment=alignment, downstream='Regenerate derived model and HOME identity; old outputs remain historical',
        step_reconverted=False, mesh_bytes_changed=False, wall_s=perf_counter()-started)
    save_json(out/'migration.json', report); save_json(out/'asset_inventory.json', audit)
    print(json.dumps({k:report[k] for k in ('status', 'mount_alignment', 'wall_s')}))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', type=Path, required=True)
    migrate(p.parse_args().output)
