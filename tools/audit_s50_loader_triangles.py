"""Audit actual Pin/Coal S50 loaded triangles against hash-bound source OBJ.

Run in the existing optional planning environment; no robot/task is moved.
Detailed actual arrays and bidirectional face-index maps remain beside the report.
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
from unloading_sim.s50_cad_model import ensure_preview_assets,load_definition,read_obj,PREVIEW_CONFIG
from unloading_sim.s50_loader_audit import audit_triangle_correspondence,read_actual_coal_mesh


def save(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    raw=(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf-8')
    path.write_bytes(raw)
    return dict(path=str(path),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())


def run(output,selected_names=None):
    from unloading_sim.robot_model_binding import RobotModelBinding
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    began=perf_counter();ensure_preview_assets(ROOT);definition=load_definition(ROOT)
    binding=RobotModelBinding.from_config(ROOT/PREVIEW_CONFIG)
    backend=PinocchioHppFclBackend.from_binding(binding,output_dir=output/'pinocchio-model',project_root=ROOT,
        world_from_base=np.eye(4))
    declared={row['name']:row['collision_meshes'] for row in definition['links']}
    counts={};names={str(obj.name) for obj in backend.geometry_model.geometryObjects}
    if selected_names and set(selected_names)-names:raise ValueError('UNKNOWN_LOADER_AUDIT_MESH')
    report=dict(schema='s50_actual_loader_triangle_audit_v1',
        scope='ACTUAL_COAL_IMPORT_POINT_AND_TRIANGLE_CORRESPONDENCE_NOT_TASK_QUALIFICATION',
        status='RUNNING',source_step_sha256=definition['source_step_sha256'],
        model_definition_sha256=definition['definition_sha256'],model_binding=binding.identity(),
        coal_version=str(getattr(backend.coal,'__version__','UNKNOWN')),
        pinocchio_version=str(getattr(backend.pin,'__version__','UNKNOWN')),
        selected_geometry_names=sorted(selected_names or names),
        source_code_sha256={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in
            ['tools/audit_s50_loader_triangles.py','src/unloading_sim/s50_loader_audit.py',
             'src/unloading_sim/s50_cad_model.py','src/unloading_sim/pinocchio_backend.py']},
        results=[],collision_coverage_qualified=False,task_planning_performed=False,
        scene_modified=False,model_modified=False)
    for index,obj in enumerate(backend.geometry_model.geometryObjects):
        link=str(backend.model.frames[int(obj.parentFrame)].name)
        ordinal=counts.get(link,0);counts[link]=ordinal+1
        record=declared[link][ordinal]
        if selected_names and str(obj.name) not in selected_names:continue
        mesh_path=ROOT/record['path'];sv,st=read_obj(mesh_path,record['sha256'])
        scale=np.asarray(obj.meshScale,dtype=float)
        if not np.array_equal(scale,np.ones(3)):raise ValueError('S50_SOURCE_SCALE_CHANGED')
        loaded_path=Path(str(obj.meshPath)).resolve()
        if loaded_path!=mesh_path.resolve():raise ValueError('LOADED_MESH_SOURCE_PATH_MISMATCH')
        lv,lt=read_actual_coal_mesh(obj.geometry)
        result,arrays=audit_triangle_correspondence(sv,st,lv,lt)
        artifact=output/(str(obj.name)+'-actual-correspondence.npz')
        np.savez_compressed(artifact,**arrays)
        result.update(geometry_index=index,geometry_name=str(obj.name),physical_link=link,
            geometry_class=type(obj.geometry).__name__,source_mesh_path=record['path'],
            source_mesh_sha256=record['sha256'],actual_mesh_path=str(obj.meshPath),
            urdf_mesh_scale=scale.tolist(),geometry_placement=backend._matrix(obj.placement).tolist(),
            evidence_arrays=dict(path=artifact.name,bytes=artifact.stat().st_size,
                sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
                contents=sorted(arrays),source_indices_zero_based=True),
            source_coordinate_frame='LINK_LOCAL_MESH_COORDINATES',
            actual_coordinate_frame='ACTUAL_COAL_LOCAL_MESH_COORDINATES_NO_EXTRA_SCALE_OR_PLACEMENT')
        report['results'].append(result)
        save(output/'loader-triangle-report.json',report)
        print(json.dumps(dict(mesh=str(obj.name),status=result['status'],
            source_triangles=len(st),loaded_triangles=len(lt),
            removed=result.get('removed_source_triangle_count'),elapsed_s=perf_counter()-began)),flush=True)
    if counts!={name:len(items) for name,items in declared.items()}:
        raise ValueError('ACTUAL_COAL_SOURCE_GEOMETRY_OWNERSHIP_COUNT_CHANGED')
    binding.verify_source_unchanged()
    complete=all(row['passed'] for row in report['results'])
    report.update(status='EXACT_LOADER_CORRESPONDENCE_VERIFIED' if complete else 'LOADER_DIFFERENCE_RETAINED',
        all_selected_meshes_audited=len(report['results'])==len(report['selected_geometry_names']),
        entire_declared_robot_audited=len(report['results'])==sum(map(len,declared.values())),
        all_selected_correspondence_passed=complete,
        source_triangle_total=sum(row['source_triangle_count'] for row in report['results']),
        actual_triangle_total=sum(row['loaded_triangle_count'] for row in report['results']),
        removed_triangle_total=sum(row.get('removed_source_triangle_count',0) for row in report['results']),
        source_zero_area_triangle_total=sum(row['source_zero_area_triangle_count'] for row in report['results']),
        elapsed_s=perf_counter()-began)
    receipt=save(output/'loader-triangle-report.json',report)
    save(output/'loader-triangle-report.sha256.json',receipt)
    return report,receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/s50_mount_entry_20261111/loader-audit')
    parser.add_argument('--mesh',action='append',dest='meshes')
    args=parser.parse_args()
    try:
        report,receipt=run(args.output,args.meshes)
        print(json.dumps(dict(status=report['status'],report=receipt)),flush=True)
        return 0 if report['all_selected_correspondence_passed'] else 2
    except Exception as exc:
        import traceback
        save(args.output/'loader-audit-failure.json',dict(status='AUDIT_FAILED_RETAINED',
            exception=type(exc).__name__,detail=str(exc),task_planning_performed=False))
        (args.output/'loader-audit-exception.txt').write_text(traceback.format_exc(),encoding='utf-8')
        raise


if __name__=='__main__':raise SystemExit(main())
