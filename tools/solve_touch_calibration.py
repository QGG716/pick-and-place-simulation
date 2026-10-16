"""Offline solve from observations only; this CLI has no truth/scene argument."""
import argparse
import time,json
from pathlib import Path
from unloading_sim.touch_calibration import solve

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observations',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();started=time.perf_counter();r=solve(a.observations,a.output)
    (a.output/'timing.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,command='solve_touch_calibration.py --observations '+str(a.observations)+' --output '+str(a.output)),indent=2))
    print(r['status'])
