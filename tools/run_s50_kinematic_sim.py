"""Compute and export an actual S50 single-carton kinematic cycle."""
from pathlib import Path
import argparse,sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,ik_candidates,target_contact,save

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'outputs/s50_kinematic_sim_20261112')
    p.add_argument('--config',type=Path)
    p.add_argument('--probe-ik',action='store_true')
    p.add_argument('--probe-geometry',action='store_true')
    p.add_argument('--no-viewer',action='store_true',help='Only compute; default also embeds the offline full-task viewer.')
    a=p.parse_args();c=make_context(ROOT,a.output,a.config)
    if a.probe_ik or a.probe_geometry:
        import numpy as np
        from unloading_sim.geometry import rotation_matrix_from_rpy
        checker=None
        if a.probe_geometry:
            from unloading_sim.s50_sim_collision import SimCollision
            checker=SimCollision(c)
        target,pose=target_contact(c)
        release=pose.copy();release[:3,:3]=rotation_matrix_from_rpy(0,0,np.pi/2)@pose[:3,:3];release[:3,3]=[-1.05,-.3,2.27]
        found={}
        for name,T in [('home',np.asarray(c.frozen['initial_physical_contact_target'])),('contact',pose),('release',release)]:
            solutions=ik_candidates(c,T,stage=name)
            found[name]=[q.tolist() for q in solutions]
            print(name,len(solutions),flush=True)
            if checker:
                for q in solutions:
                    good=checker.check(q,contact=name=='contact',phase=name)
                    print(json.dumps(dict(stage=name,q_deg=np.rad2deg(q).tolist(),passed=good,failure=checker.last)),flush=True)
        save(a.output/('geometry-probe.json' if checker else 'ik-probe.json'),dict(solutions=found,attempts=c.attempts));return
    from unloading_sim.s50_kinematic_cycle import run
    result=run(c)
    if result['samples'] and not a.no_viewer:
        from export_s50_kinematic_viewer import export
        export(a.output/'cycle.json',a.output/'viewer.html')
    print(json.dumps(dict(status=result['status'],report=str(a.output/'cycle.json'),viewer=str(a.output/'viewer.html'))))
    return 0 if result['status']=='COMPLETED_KINEMATIC_SINGLE_CARTON' else 1

if __name__=='__main__':raise SystemExit(main())
