"""Run finite real-mesh S50 Coal diagnostics in the unchanged 40-carton layout.

This is neither a legal initial-state search nor single-carton task planning.
It does not require a repaired whole CAD solid in order to query its surfaces.
"""
from pathlib import Path
from time import perf_counter
import argparse
import hashlib
import json
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_cad_model import ensure_preview_assets,load_definition,PREVIEW_CONFIG
from unloading_sim.s50_collision_diagnostics import (audit_model_meshes,diagnose_pose,diagnostic_poses,
    diagnostic_environment,diagnostic_obstacle_record,SCOPE)


def save(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8')
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp');temporary.write_bytes(raw);temporary.replace(path)
    return hashlib.sha256(raw).hexdigest()


def run(output,*,pose_ids=None,mesh_only=False):
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout
    from unloading_sim.s50_payload import freeze_s50_environment
    from unloading_sim.robot_model_binding import RobotModelBinding
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    ensure_preview_assets(ROOT);definition=load_definition(ROOT)
    audit=audit_model_meshes(ROOT)
    audit_hash=save(output/'mesh-surface-audit.json',audit)
    if mesh_only:return dict(status='MESH_SURFACE_AUDIT_RECORDED_NOT_TASK_QUALIFIED',mesh_audit_sha256=audit_hash)
    reference=frozen_baseline_layout(ROOT);frozen=freeze_s50_environment(reference)
    mount=reference.robot_base_transform()
    binding=RobotModelBinding.from_config(ROOT/PREVIEW_CONFIG)
    world_from_base=mount@binding.mounting_from_base
    belt_height=frozen['parameters']['initial_belt_height_m']
    scene=diagnostic_environment(reference,belt_height)
    obstacles=scene.boxes
    report=dict(schema='s50_runtime_collision_diagnostics_v1',qualification_scope=SCOPE,status='RUNNING',
        source_step_sha256=definition['source_step_sha256'],model_definition_sha256=definition['definition_sha256'],
        model_binding=binding.identity(),environment_fingerprint=frozen['environment_fingerprint'],
        initial_carton_ids=frozen['initial_carton_ids'],imported_completed_count=0,assumed_removed=[],
        world_from_physical_mount=mount.tolist(),world_from_sim_mount=mount.tolist(),
        world_from_model_base=world_from_base.tolist(),
        conveyor_height_m=belt_height,conveyor_configuration='FROZEN_INITIAL_HEIGHT_UNCHANGED_FOR_EVERY_POSE',
        scene_context=dict(environment_fingerprint=frozen['environment_fingerprint'],
            world_from_physical_mount=mount.tolist(),world_from_model_base=world_from_base.tolist(),
            initial_carton_ids=frozen['initial_carton_ids'],conveyor_height_m=belt_height),
        installation_scope='FROZEN_PHYSICAL_INSTALLATION_POSITION_WITH_DECLARED_CAD_SIM_CLOCKING_ONLY',
        actual_mount_clocking_verified=False,physical_mount_validation_scope='DIAGNOSTIC_ONLY_NOT_TASK_ADMISSION',
        tool_installed=False,tool_exemptions=[],self_collision_exemptions=[],mount_contact_exemptions=[],
        mechanical_limits_verified=False,dynamics_verified=False,planning_attempts=0,
        mesh_audit=dict(path='mesh-surface-audit.json',sha256=audit_hash),
        historical_j1_brep_issue='RETAINED_SEPARATELY_DOES_NOT_BLOCK_TRIANGLE_SURFACE_QUERY',
        obstacle_count=len(obstacles),obstacles=[diagnostic_obstacle_record(b) for b in obstacles],
        obstacle_geometry_scope='FULL_COUPLED_FROZEN_SCENE_WITH_DECLARED_SOLIDS',
        coupled_scene_fingerprint=scene.scene_fingerprint,
        poses=[],all_requested_pairs_queried=False,task_acceptance_evaluated=False,
        self_collision_interpretation='Adjacent and owned-body contacts remain visible; no all-pair count is a task failure')
    save(output/'run-manifest.json',report)
    try:
        from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
        backend=PinocchioHppFclBackend.from_binding(binding,output_dir=output/'pinocchio-model',project_root=ROOT,
            world_from_base=world_from_base)
        counts={}
        inventory=[]
        for shape_index,shape in enumerate(backend.geometry_model.geometryObjects):
            link=str(backend.model.frames[int(shape.parentFrame)].name)
            counts[link]=counts.get(link,0)+1
            declared=next(row for row in definition['links'] if row['name']==link)['collision_meshes'][counts[link]-1]
            triangle_count=getattr(shape.geometry,'num_tris',None)
            if callable(triangle_count):triangle_count=triangle_count()
            triangle_count=None if triangle_count is None else int(triangle_count)
            vertices=backend._geometry_local_vertices[shape_index]
            inventory.append(dict(name=str(shape.name),link=link,mesh_path=str(shape.meshPath),
                geometry_class=type(shape.geometry).__name__,source_mesh_sha256=declared['sha256'],
                source_vertex_count=declared['vertices'],source_triangle_count=declared['triangles'],
                coal_vertex_count=None if vertices is None else len(vertices),coal_triangle_count=triangle_count,
                reader_vertex_count_matches_source=None if vertices is None else len(vertices)==declared['vertices'],
                reader_triangle_count_matches_source=None if triangle_count is None else triangle_count==declared['triangles']))
        expected={row['name']:len(row['collision_meshes']) for row in definition['links']}
        if counts!=expected:raise ValueError('COAL_COLLISION_SHAPE_OWNERSHIP_COUNT_MISMATCH')
        report['coal_geometry_inventory']=inventory
        report['coal_version']=str(getattr(backend.coal,'__version__','UNKNOWN'))
        report['pinocchio_version']=str(getattr(backend.pin,'__version__','UNKNOWN'))
        selected=diagnostic_poses()
        if pose_ids:
            selected=diagnostic_poses(include_mount_probes=True)
            unknown=set(pose_ids)-{p['pose_id'] for p in selected}
            if unknown:raise ValueError('UNKNOWN_DIAGNOSTIC_POSE:'+','.join(sorted(unknown)))
            selected=[p for p in selected if p['pose_id'] in pose_ids]
        report['requested_pose_ids']=[p['pose_id'] for p in selected]
        began=perf_counter()
        for index,pose in enumerate(selected):
            row=diagnose_pose(backend,pose['q_rad'],obstacles,mount,pose_id=pose['pose_id'],conveyor_height_m=belt_height)
            row.update(label=pose['label'],description=pose['description'])
            row['record_sha256']=save(output/('pose-%02d.json'%index),row)
            report['poses'].append(row)
            print(json.dumps(dict(pose_id=row['pose_id'],pair_count=row['queried_pair_count'],
                unknown=row['unknown_pair_count'],colliding_pairs=len(row['collision_pairs']),
                scope=SCOPE,elapsed_s=perf_counter()-began)),flush=True)
            save(output/'runtime-collision-report.json',report)
        report['all_requested_pairs_queried']=all(p['unknown_pair_count']==0 for p in report['poses'])
        from unloading_sim.s50_compactness import summarize_samples
        report['compactness_summary']=summarize_samples(p['compactness'] for p in report['poses'] if p.get('compactness'))
        report['elapsed_s']=perf_counter()-began
        report['status']=('DIAGNOSTIC_QUERIES_COMPLETED_NOT_TASK_QUALIFIED' if report['all_requested_pairs_queried']
            else 'DIAGNOSTIC_QUERY_UNCERTAINTY_RETAINED')
    except Exception as exc:
        import traceback
        report.update(status='DIAGNOSTIC_FAILURE_RETAINED',failure=dict(type=type(exc).__name__,detail=str(exc)))
        (output/'runtime-exception.txt').write_text(traceback.format_exc(),encoding='utf8')
    report_hash=save(output/'runtime-collision-report.json',report)
    save(output/'runtime-collision-report.sha256.json',dict(path='runtime-collision-report.json',sha256=report_hash))
    return dict(status=report['status'],poses=len(report['poses']),sha256=report_hash,
        report=str(output/'runtime-collision-report.json'))


def run_mount_cases(output):
    """Negative fixtures change only a diagnostic copy, never the task layout."""
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout
    from unloading_sim.s50_payload import freeze_s50_environment
    from unloading_sim.robot_model_binding import RobotModelBinding
    from unloading_sim.robot import URDFRobot
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    from unloading_sim.mount_contact import bind_mount_contact,use_actual_mesh_backend,mount_contact_evidence
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    ensure_preview_assets(ROOT);definition=load_definition(ROOT)
    layout=frozen_baseline_layout(ROOT);frozen=freeze_s50_environment(layout)
    mount=layout.robot_base_transform();plinth=next(b for b in layout.fixed_components() if b.name=='mount_plinth')
    binding=RobotModelBinding.from_config(ROOT/PREVIEW_CONFIG)
    core=URDFRobot.from_config(ROOT/PREVIEW_CONFIG,mounting_world_transform=mount)
    backend=PinocchioHppFclBackend.from_binding(binding,output_dir=output/'pinocchio-model',project_root=ROOT,
        world_from_base=core.base_transform)
    bind_mount_contact(core,definition,ROOT,world_from_physical_mount=mount,expected_plinth=plinth)
    use_actual_mesh_backend(core,backend)
    initial_base=core.base_transform.copy()
    report=dict(schema='s50_coal_mount_contact_cases_v1',qualification_scope=SCOPE,
        source_step_sha256=definition['source_step_sha256'],model_definition_sha256=definition['definition_sha256'],
        environment_fingerprint=frozen['environment_fingerprint'],initial_carton_ids=frozen['initial_carton_ids'],
        world_from_physical_mount=mount.tolist(),obstacle_scope='MOUNT_PLINTH_ONLY_DIAGNOSTIC_CASES',
        source_mount_sha256=hashlib.sha256((ROOT/'assets/robots/huayan_s50_2000/generated/mount_contact_source.json').read_bytes()).hexdigest(),
        task_layout_modified=False,tool_installed=False,installation_verified=False,cases=[])
    def case(name,q,z_delta=0.):
        transform=initial_base.copy();transform[2,3]+=z_delta
        core.base_transform=transform.copy();backend.base_transform=transform.copy()
        row=diagnose_pose(backend,q,(plinth,),mount,pose_id=name,
            conveyor_height_m=frozen['parameters']['initial_belt_height_m'])
        row.update(label=name,description='Independent diagnostic fixture; no task installation change',
            world_from_model_base=transform.tolist(),diagnostic_installation_z_delta_m=z_delta,
            mount_contact=mount_contact_evidence(backend,(plinth,)))
        row['active_link_plinth_collisions']=[p for p in row['collision_pairs'] if p['kind']=='robot_environment'
            and p['first']['link']!='s50_base' and p['second']['object']=='mount_plinth']
        row['no_active_link_exemption']=not row['mount_contact'].get('active_links_exempted') and all(
            pair[0]=='s50_base' and pair[1]=='mount_plinth' for pair in row['mount_contact'].get('allowed_pairs',[]))
        report['cases'].append(row)
        print(json.dumps(dict(case=name,mount_status=row['mount_contact']['status'],
            mount_passed=row['mount_contact']['passed'],active_plinth_collisions=len(row['active_link_plinth_collisions']))),flush=True)
        save(output/'mount-contact-cases.json',report)
        return row
    normal=case('source_mount_normal',np.zeros(6))
    penetration=case('negative_fixture_base_down_1mm',np.zeros(6),-.001)
    selected=None
    for angle in (2.,2.4,2.8,3.,float(np.pi)):
        q=np.zeros(6);q[1]=angle
        candidate=case('active_link_probe_j2_'+str(angle),q)
        if candidate['active_link_plinth_collisions']:
            selected=candidate['pose_id'];break
    core.base_transform=initial_base.copy();backend.base_transform=initial_base.copy()
    report.update(normal_mount_proof_passed=normal['mount_contact']['passed'],
        shifted_mount_proof_rejected=not penetration['mount_contact']['passed'],
        actual_active_link_collision_case=selected,
        no_active_link_exemption=all(row['no_active_link_exemption'] for row in report['cases']),
        all_raw_coal_queries_completed=all(row['unknown_pair_count']==0 for row in report['cases']))
    complete=(report['normal_mount_proof_passed'] and report['shifted_mount_proof_rejected'] and selected is not None
        and report['no_active_link_exemption'] and report['all_raw_coal_queries_completed'])
    report['status']='THREE_MOUNT_CASES_RECORDED_NOT_TASK_QUALIFIED' if complete else 'MOUNT_DIAGNOSTIC_GAP_RETAINED'
    digest=save(output/'mount-contact-cases.json',report)
    save(output/'mount-contact-cases.sha256.json',dict(path='mount-contact-cases.json',sha256=digest))
    return dict(status=report['status'],sha256=digest,report=str(output/'mount-contact-cases.json'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/s50_task_collision_20261111')
    parser.add_argument('--pose',action='append',help='Explicit diagnostic subset; repeat to select several')
    parser.add_argument('--mesh-only',action='store_true')
    parser.add_argument('--mount-cases-only',action='store_true',help='Source mount, rejected 1mm shift, and finite active-link/plinth fixtures')
    args=parser.parse_args()
    if args.mount_cases_only and (args.pose or args.mesh_only):parser.error('--mount-cases-only is independent of --pose/--mesh-only')
    try:
        result=run_mount_cases(args.output) if args.mount_cases_only else run(args.output,pose_ids=args.pose,mesh_only=args.mesh_only)
    except Exception as exc:
        import traceback
        args.output.mkdir(parents=True,exist_ok=True)
        result=dict(status='DIAGNOSTIC_SETUP_FAILURE_RETAINED',qualification_scope=SCOPE,
            failure=dict(type=type(exc).__name__,detail=str(exc)),task_acceptance_evaluated=False)
        save(args.output/'diagnostic-setup-failure.json',result)
        (args.output/'diagnostic-setup-exception.txt').write_text(traceback.format_exc(),encoding='utf8')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['status'] in ('DIAGNOSTIC_QUERIES_COMPLETED_NOT_TASK_QUALIFIED','MESH_SURFACE_AUDIT_RECORDED_NOT_TASK_QUALIFIED',
        'THREE_MOUNT_CASES_RECORDED_NOT_TASK_QUALIFIED') else 2


if __name__=='__main__':raise SystemExit(main())
