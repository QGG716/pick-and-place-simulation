"""Prepare the retained terminal and report actual geometry, without native search."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from min250_continuation import restore_population_terminal
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.coupled_batch import BatchState
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.conveyor_lift import coupled_conveyor_state,ScissorGeometryRangeError
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.lifted_reception import StartHeight
from unloading_sim.low_top_receiver import uses_low_top,BOX_TOP_LOW_DEFINITION
from unloading_sim.moveit2_backend import box_message


def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def prepare(previous,output):
    output.mkdir(parents=True,exist_ok=False)
    archive=previous/'formal-raw.tar.xz'
    archive_sha=hashlib.sha256(archive.read_bytes()).hexdigest()
    if archive_sha!='4f354b08b02382d31711377b83c756968e94c1394293c35be4ed4bae9a8dabc2':
        raise ValueError('MIN250_RETAINED_RAW_ARCHIVE_CHANGED')
    baseline=output/'previous-formal-v1'
    # Only the previously verified result/lineage inputs; never feed native
    # trajectories into search. The full raw source remains separately retained.
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            name=member.name
            wanted=name in {f'formal-v1/{n}.json' for n in ('summary','protocol','verification')}
            wanted=wanted or (name.startswith('formal-v1/task-') and name.endswith('/result.json'))
            if not wanted:continue
            destination=(baseline/Path(name).relative_to('formal-v1')).resolve()
            if not destination.is_relative_to(baseline.resolve()) or not member.isfile():
                raise ValueError('MIN250_ARCHIVE_PATH_INVALID')
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(tar.extractfile(member).read())
    layout=load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250.yaml')
    original=layout.cartons();frozen=FrozenCartonPopulation.capture(original)
    summary=json.loads((baseline/'summary.json').read_text())
    if summary['initial']['world_cartons']!=[box_message(b) for b in original]:
        raise ValueError('MIN250_ORIGINAL_WORLD_CHANGED')
    state=BatchState(frozen,tuple(original),np.array(summary['initial']['q_rad']),.6)
    predecessor=restore_population_terminal(baseline,state,'x_minus_500_y_minus_100')
    save(output/'predecessor-check.json',predecessor)
    boxes=[]
    for b in state.cartons:
        h=StartHeight.capture(b)
        boxes.append(dict(target_id=b.name,box_top_world_m=h.z_b+h.h,Hmin_world_m=h.hmin,Hmax_world_m=h.hmax,
            use_top=uses_low_top(h,.25,BOX_TOP_LOW_DEFINITION),face_rule=BOX_TOP_LOW_DEFINITION))
    heights=[]
    contacts=[['chassis','conveyor_transverse'],['chassis','conveyor_longitudinal'],['conveyor_transverse','conveyor_longitudinal']]
    for height in (.25,.55,.85,1.15):
        try:
            geometry=coupled_conveyor_state(layout,height,state.cartons)
            check=check_coupled_state(layout,height,state.cartons,designed_contacts=contacts)
            save(output/f'geometry-{round(height*1000)}mm.json',check)
            heights.append(dict(height_m=height,constructible=True,geometry_check_passed=check['passed'],
                failure_count=len(check['failures']),failures=check['failures'],
                object_count=len(geometry.boxes),tilt_rad=geometry.mechanism['tilt_rad']))
        except ScissorGeometryRangeError as exc:
            heights.append(dict(height_m=height,constructible=False,reason=str(exc),evidence=exc.evidence))
    report=dict(status='PREPARED_NOT_NATIVE_PLANNED',previous_raw_sha256=archive_sha,
        requested_minimum_m=.25,nominal_longitudinal_rear_surface_m=.6,mechanism_dimensions_changed=False,
        historical_completed_count=20,continuation_target_count=20,top_selected_count=sum(b['use_top'] for b in boxes),
        boxes=boxes,geometry=heights,layout_audit=audit_layout_constraints(layout),
        initial_root=state.root(),production_backend_check='PENDING_GPU',
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    save(output/'preparation.json',report)
    print(json.dumps({k:report[k] for k in ('status','historical_completed_count','continuation_target_count','top_selected_count')}))
    print(json.dumps([dict(height_m=h['height_m'],constructible=h['constructible'],failures=h.get('failure_count')) for h in heights]))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--previous',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepare(a.previous,a.output)
