"""Offline solve from observations only; this CLI has no truth/scene argument."""
import argparse
import time,json
from pathlib import Path
from unloading_sim.touch_calibration import solve,save_failure

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observations',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);started=time.perf_counter()
    if a.output.exists():print('FAIL OUTPUT_ALREADY_EXISTS');return 2
    try:r=solve(a.observations,a.output)
    except Exception as error:r=save_failure(a.output,error,'failure.json')
    (a.output/'timing.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,command='solve_touch_calibration.py --observations '+str(a.observations)+' --output '+str(a.output)),indent=2))
    print(r['status'])
    return 0 if r['status']=='SOLVED_INDEPENDENT_VALIDATION_REQUIRED' else 1


if __name__=='__main__':raise SystemExit(main())
