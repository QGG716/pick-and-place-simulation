"""Directed mechanism sweep only; robot/native checks remain separate gates."""
import argparse,json
from pathlib import Path
from time import perf_counter
from unloading_sim.coupled_lift_checks import check_coupled_interval
from unloading_sim.workcell_layout import load_workcell_layout

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/
        'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile.yaml')
    started=perf_counter()
    check=check_coupled_interval(layout,.25,2.1,[],height_step_m=.01,
        designed_contacts=[['chassis','conveyor_transverse']])
    check.update(layout_fingerprint=layout.layout_fingerprint,wall_s=perf_counter()-started,
        scope='MECHANISM_AND_FIXED_WORLD_ONLY; ACTUAL_CARTONS_AND_ROBOT_CHECKED_IN_FORMAL_RUN')
    args.output.write_text(json.dumps(check,separators=(',',':'))+'\n')
    print(json.dumps(dict(status=check['status'],samples=check['sample_count'],wall_s=check['wall_s'],
        rejected=[dict(height=s['height_m'],failures=s['failures']) for s in check['samples'] if not s['passed']])))
    if any(not s['passed'] for s in check['samples']):raise ValueError('LOWPROFILE_GEOMETRY_REJECTED')

if __name__=='__main__':main()
