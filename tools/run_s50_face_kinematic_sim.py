"""Run the complete face at the configured fixed layout; explicit gap trials are optional."""
from pathlib import Path
import sys,argparse,yaml,copy,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,save
from unloading_sim.s50_face_kinematic import FaceRun


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gaps',type=float,nargs='+')
    p.add_argument('--output',type=Path,default=ROOT/'outputs/s50_full_face');p.add_argument('--max-cartons',type=int)
    p.add_argument('--no-viewer',action='store_true')
    p.add_argument('--config',type=Path,default=ROOT/'configs/robots/huayan_s50_2000_kinematic_sim.yaml')
    p.add_argument('--allow-side-suction',action='store_true',help='Use one explicit pose on each side after front/top fail; collision checks remain active.')
    p.add_argument('--j1-empty-reposition-weight',type=float,help='Soft cost for empty J1 relocation; does not change geometry or limits.')
    p.add_argument('--resume-cycle',type=Path,help='Resume a retained empty boundary in the same recorded world; restart numerical search.')
    a=p.parse_args();base=yaml.safe_load(a.config.read_text(encoding='utf8'))
    base['face_run']=dict(release_drop_m=.05,maximum_release_drop_m=.1,scope='SAME_LAYOUT_COMPLETE_40_CARTON_FRONT_FACE')
    if a.resume_cycle:
        base=json.loads(a.resume_cycle.read_text(encoding='utf8'))['settings']
        if a.gaps and a.gaps!=[base['layout_gap_m']]:raise ValueError('RESUME_CANNOT_CHANGE_LAYOUT')
    if a.allow_side_suction:base['face_run'].update(allow_side_suction=True,side_contact_from_front_m=.15)
    if a.j1_empty_reposition_weight is not None:
        if a.j1_empty_reposition_weight<0:raise ValueError('NEGATIVE_PREFERENCE_WEIGHT')
        base['posture_preferences']['j1_empty_reposition_weight']=a.j1_empty_reposition_weight
    attempts=[]
    for gap in a.gaps or [base['layout_gap_m']]:
        settings=copy.deepcopy(base);settings['layout_gap_m']=gap
        folder=a.output/f'gap_{round(gap*1000):03d}';c=make_context(ROOT,folder,settings)
        run=FaceRun.resume_empty_boundary(c,a.resume_cycle) if a.resume_cycle else FaceRun(c)
        if a.resume_cycle and a.allow_side_suction:
            run.report.setdefault('planning_policy_updates',[]).append(dict(at_sample=len(run.samples),
                change='USER_AUTHORIZED_SIDE_SUCTION_WITH_UNCHANGED_SCENE_AND_FIXED_TARGET_ORDER',
                face_run=copy.deepcopy(base['face_run'])))
            run.report['settings']['face_run']=copy.deepcopy(base['face_run'])
        if a.resume_cycle and a.j1_empty_reposition_weight is not None:
            run.report.setdefault('planning_policy_updates',[]).append(dict(at_sample=len(run.samples),
                change='EMPTY_J1_LARGE_RELOCATION_SOFT_COST',previous=run.report['settings']['posture_preferences'].get('j1_empty_reposition_weight'),
                value=a.j1_empty_reposition_weight))
            run.report['settings']['posture_preferences']['j1_empty_reposition_weight']=a.j1_empty_reposition_weight
        result=run.run(a.max_cartons)
        attempts.append(dict(gap_m=gap,status=result['status'],completed_cartons=result['completed_cartons'],report=str(folder/'cycle.json'),failure=result.get('failure')))
        save(a.output/'layout-attempts.json',dict(attempts=attempts,counts_are_separate_worlds=True))
        if result['samples'] and not a.no_viewer:
            from export_s50_kinematic_viewer import export
            export(folder/'cycle.json',folder/'viewer.html')
        print(json.dumps(attempts[-1]),flush=True)
        if result['status'] in ['COMPLETED_KINEMATIC_FULL_FACE','COMPLETED_KINEMATIC_SINGLE_CARTON','COMPLETED_REQUESTED_PREFIX_NOT_FULL_FACE']:return 0
    return 1

if __name__=='__main__':raise SystemExit(main())
