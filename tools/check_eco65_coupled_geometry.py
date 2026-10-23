"""Finite convex mechanism checks using the same world solids as native/replay."""
import argparse
from itertools import combinations, product
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from unloading_sim.eco65.model import load_json,save_json
from unloading_sim.eco65.coupled_scene import coupled_conveyor_state


def vertices(row):
    local=(np.asarray(list(product(*[(-v/2,v/2) for v in row['size']])))
           if 'size' in row else np.asarray(row['convex_mesh']['vertices']))
    t=np.asarray(row['pose']);return local@t[:3,:3].T+t[:3,3]


def check(snapshot,heights):
    spec=snapshot['scene']['coupled_conveyor_spec'];reports=[];fixed=None
    for height in heights:
        state=coupled_conveyor_state(spec,height);m=state['mechanism'];rows=state['world_objects']
        named={frozenset(p[:2]) for p in m['additional_mates']}
        solids={r['id']:vertices(r) for r in rows};overlaps=[];mates=[]
        fixed_now={r['id']:r for r in rows if r['id'] in m['fixed_structure_ids']}
        if fixed is None:fixed=fixed_now
        assert fixed==fixed_now,'FIXED_STRUCTURE_MOVED'
        assert abs(np.linalg.norm(np.asarray(m['upper_endpoint_D_m'])-m['lower_endpoint_D_m'])-spec['longitudinal_length_m'])<1e-9
        for stack in m['stacks']:
            for bar in stack['bars']:
                assert abs(np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))-bar['length_m'])<1e-9
        floor=spec['config'].get('floor_z_m',0.)
        below=[name for name,v in solids.items() if v[:,2].min()<floor-1e-9]
        hulls={name:ConvexHull(v).equations for name,v in solids.items()}
        for (a,va),(b,vb) in combinations(solids.items(),2):
            if np.any(np.minimum(va.max(0),vb.max(0))-np.maximum(va.min(0),vb.min(0))<=1e-8):continue
            e=np.vstack([hulls[a],hulls[b]])
            lp=linprog([0,0,0,-1],A_ub=np.c_[e[:,:3],np.ones(len(e))],b_ub=-e[:,3],
                       bounds=[(None,None)]*3+[(0,None)],method='highs')
            if lp.success and lp.x[3]>1e-7:
                (mates if frozenset((a,b)) in named else overlaps).append(dict(pair=[a,b],intersection_radius_m=float(lp.x[3])))
        reports.append(dict(height_m=height,tilt_rad=state['tilt_rad'],nonassembly_overlaps=overlaps,
                            named_assembly_proxy_contacts=mates,below_floor=below))
    return dict(status='PASS' if all(not r['nonassembly_overlaps'] and not r['below_floor'] for r in reports) else 'FAIL',
        scope='FINITE_STATIC_CONVEX_MECHANISM_SAMPLES_NOT_CONTINUOUS_OR_MANUFACTURING_PROOF',
        snapshot_fingerprint=snapshot['fingerprint'],samples=reports)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot',required=True)
    p.add_argument('--heights',nargs='+',type=float,default=[.1,.18,.33,.43,.45]);p.add_argument('--output',required=True)
    a=p.parse_args();result=check(load_json(a.snapshot),a.heights);save_json(a.output,result)
    print(result['status'],[(r['height_m'],len(r['nonassembly_overlaps'])) for r in result['samples']])
    raise SystemExit(result['status']!='PASS')
