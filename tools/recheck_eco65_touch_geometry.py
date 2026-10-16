"""Recheck existing discrete calibration states only; no IK, search or rendering."""
import argparse,copy,time
from pathlib import Path
from unloading_sim.eco65.model import ROOT,load_json,save_json
from unloading_sim.eco65.native_io import file_hash
from build_eco65_full_face_layout import verify_static
from eco65_touch_sources import bind_scene_sources,derived_scene_identity
from eco65_touch_geometry import StaticGeometry


def recheck(prepared,snapshot,composed_usd,geometry_manifest,output):
    output=output.resolve();output.relative_to(ROOT/'outputs');output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    try:
        original=load_json(prepared/'configuration.json');s=verify_static(load_json(snapshot))
        if file_hash(snapshot)!=original['snapshot_sha256'] or s['fingerprint']!=original['source_fingerprint']:
            raise ValueError('EXISTING_CALIBRATION_SNAPSHOT_MISMATCH')
        c=copy.deepcopy(original);c.pop('base_composed_usd',None)
        c['scene_sources']=bind_scene_sources(snapshot,composed_usd,geometry_manifest)
        c['derived_scene_identity']=derived_scene_identity(c)
        save_json(output/'configuration.json',c)
        geometry=StaticGeometry(s,c,output);placements={p['board_placement_id']:p['T_D_P'] for p in c['placements']};records=[]
        for kind,items in [('touch',c['touches']),('photo',c['photos'])]:
            for row in items:
                check=geometry.check(row['q_rad'],placements[row['board_placement_id']],row['point_id'] if kind=='touch' else None)
                record=dict(sample_id=row['sample_id'],kind=kind,board_placement_id=row['board_placement_id'],q_rad=row['q_rad'],
                    previous_check=row['geometry' if kind=='touch' else 'static_check'],current_check=check)
                records.append(record);save_json(output/(row['sample_id']+'.json'),record)
                print(row['sample_id'],'PASS' if check['accepted'] else 'FAIL',check.get('failure_count',len(check['failures'])),flush=True)
        passed=all(row['current_check']['accepted'] for row in records)
        by_id={row['sample_id']:row['current_check'] for row in records}
        for key,check_key in [('touches','geometry'),('photos','static_check')]:
            for row in c[key]:row[check_key]=by_id[row['sample_id']]
        c['static_recheck_status']='PASS' if passed else 'FAIL'
        # A derivative configuration cannot retain the obsolete accepted flags.
        save_json(output/'configuration.json',c)
        report=dict(status='PASS' if passed else 'FAIL',scope='EXISTING_DISCRETE_STATES_ONLY',
            source_configuration_sha256=file_hash(prepared/'configuration.json'),scene_sources=c['scene_sources'],derived_scene_identity=c['derived_scene_identity'],
            samples=[dict(sample_id=row['sample_id'],accepted=row['current_check']['accepted'],failures=row['current_check']['failures']) for row in records],
            calibration_estimation_quality='SEPARATE_IMAGE_RECOMPUTATION_RESULT',
            next_single_point_path_stage='STATIC_PRECONDITIONS_ONLY_PASSED_PATH_NOT_PLANNED' if passed else 'BLOCKED_BY_STATIC_GEOMETRY',
            motion='NO_MOTION_EXECUTED',dynamics='NOT_EVALUATED',seconds=time.perf_counter()-started)
    except Exception as error:
        report=dict(status='FAIL',error=type(error).__name__+': '+str(error),seconds=time.perf_counter()-started)
    save_json(output/'geometry_report.json',report)
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['prepared','snapshot','composed-usd','geometry-manifest','output']:p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args(argv)
    try:return 0 if recheck(a.prepared,a.snapshot,a.composed_usd,a.geometry_manifest,a.output)['status']=='PASS' else 1
    except Exception as error:print('FAIL',type(error).__name__,str(error));return 2


if __name__=='__main__':raise SystemExit(main())
