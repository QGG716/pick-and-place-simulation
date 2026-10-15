"""Rebind a historical joint proposal to current assets, with fresh local checks."""
import argparse
import hashlib
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.asset_paths import asset_relative, resolve_asset
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.eco65.native_model import asset_inventory, build_native_model
from unloading_sim.eco65.native_scene import combined_scene
from unloading_sim.eco65.unloading_layout import UnloadingWorld
from inspect_eco65_assembly import manual_fk


def prepare(report_path, migration_path, output, native_asset_root=None):
    started=perf_counter(); out=Path(output).resolve(); out.relative_to(ROOT/'outputs')
    out.mkdir(parents=True, exist_ok=False)
    report=load_json(report_path); migration=load_json(migration_path)
    inventory=asset_inventory()
    if inventory['status']!='PASS': raise ValueError(inventory)
    current=inventory['asset_sha256']; old=report['asset_inventory']['asset_sha256']
    tool_path='configs/local/eco65_desktop/tool.json'
    if old[tool_path]!=migration['old_tool_sha256'] or current[tool_path]!=migration['new_tool_sha256']:
        raise ValueError('HOME_MIGRATION_TOOL_CHAIN_INVALID')
    for name, expected in old.items():
        name=asset_relative(name, migrate_windows=True)
        if name in (tool_path, 'assets/tools/desktop_suction/derived/manifests/cad.json'): continue
        if hashlib.sha256(resolve_asset(ROOT,name).read_bytes()).hexdigest()!=expected:
            raise ValueError('HOME_SOURCE_GEOMETRY_CHANGED:'+name)
    tool=load_json(ROOT/tool_path); scene=combined_scene(); q=np.asarray(report['home_q'],float)
    candidate=make_candidate(q,tool,scene,dict(source_report=Path(report_path).resolve().relative_to(ROOT).as_posix(),
        source_report_sha256=hashlib.sha256(Path(report_path).read_bytes()).hexdigest(),
        migration_sha256=hashlib.sha256(Path(migration_path).read_bytes()).hexdigest(),
        historical_pass_is_not_acceptance=True))
    world=UnloadingWorld(tool,scene)
    try:
        limits=bool(np.all(q>=world.robot.joint_limits[:,0]) and np.all(q<=world.robot.joint_limits[:,1]))
        valid=world.valid(q,'pose'); frames=world.robot.named_link_frames(q); direct=manual_fk(q,scene)
        fk_error=max(float(np.max(abs(frames[k]-direct[k]))) for k in frames)
        tcp=frames['link_6']@np.asarray(tool['T_flange_tool_cad'])@np.asarray(tool['T_tool_cad_tcp'])
        tcp_error=float(np.max(abs(tcp-world.robot.fk(q))))
        check=dict(scope='EQUIPMENT_ONLY_DIAGNOSTIC', joint_limits=limits, collision_and_clearance=valid,
            failure=world.last_failure, fk_matrix_error=fk_error, tcp_matrix_error=tcp_error,
            mount_alignment=tool['mounting_evidence']['final_alignment'],
            native_state='NOT_RUN', full_warehouse='NOT_EVALUATED', homing_motion='NOT_EVALUATED')
    finally: world.close()
    save_json(out/'local_home_check.json',check)
    if not limits or not valid or max(fk_error,tcp_error)>1e-8: raise ValueError('LOCAL_HOME_RECHECK_FAILED')
    save_json(out/'home_candidate.json',candidate)
    local_s=perf_counter()-started; begin=perf_counter()
    model=build_native_model(scene,out/'model',native_asset_root=native_asset_root)
    save_json(out/'native_model_prepared.json',model)
    save_json(out/'report.json',dict(scope='EQUIPMENT_ONLY_DIAGNOSTIC', candidate_fingerprint=candidate['fingerprint'],
        native_worker='NOT_RUN', local_check_s=local_s, model_preparation_s=perf_counter()-begin,
        initialization_parameters='PREPARED_NOT_RUNTIME_VALIDATED', environment_status=scene['environment_status']))
    print(check)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True); p.add_argument('--migration',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--native-asset-root')
    a=p.parse_args(); prepare(a.report,a.migration,a.output,a.native_asset_root)
