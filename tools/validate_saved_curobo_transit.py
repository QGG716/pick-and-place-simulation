"""Isolated authority replay of an immutable saved candidate; never runs a planner."""
import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import sys
from time import perf_counter, process_time
import numpy as np
from run_curobo_v2_transit import context
from unloading_sim.stage_export import export_request, file_hash
from unloading_sim.stage_backend import fingerprint
import unloading_sim.layout_trajectory as lt


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result', required=True)
    p.add_argument('--motion', default='fixtures/curobo_v2/source_motion.json')
    p.add_argument('--state', default='fixtures/curobo_v2/actual_remaining_state.json')
    p.add_argument('--config', default='configs/validation/m710id70_handoff_continuation.yaml')
    p.add_argument('--output', required=True)
    p.add_argument('--warm-runs', type=int, default=1)
    p.add_argument('--fixture',choices=['business','direct_unit'],default='business')
    a=p.parse_args(); out=Path(a.output); out.mkdir(parents=True,exist_ok=True)
    saved=json.loads(Path(a.result).read_text()); trajectory=saved['trajectory']
    t=perf_counter(); _,scene,connector,attachment,historical,target=context(a.motion,a.state,a.config)
    setup=perf_counter()-t
    if a.fixture=='direct_unit':
        start=historical[-1].copy();goal=start.copy();goal[2]+=.001
        historical=np.array([start,goal])
    q=np.asarray(trajectory['q'], dtype=float)
    request,bundle=export_request(scene,connector,attachment,historical[0],historical[-1],request_id=target.name+'-transit')
    # Paths in provenance are intentionally machine-specific. These are the physical bindings.
    fields=('q_start','q_goal','scene_fingerprint','robot_model_fingerprint','tool_fingerprint',
            'payload_fingerprint','collision_policy_fingerprint','joint_names','stage')
    for name in fields:
        if request.data[name]!=saved['request'][name]: raise ValueError('saved request mismatch: '+name)
    if trajectory['interpolation']!='linear_joint_samples': raise ValueError('unsupported delivered interpolation')
    obstacles=[x for x in scene.all_obstacles if x.name!=target.name]
    validator=connector.robot_state_validator
    counters={}
    # Inclusive timings: these overlap validator counters and MUST NOT be summed with them.
    for name in ('obb_surface_distance','obb_pair_failure','possible_inflated_obb_pairs'):
        original=getattr(lt,name)
        def wrapper(*args,_fn=original,_name=name,**kwargs):
            started=perf_counter()
            try: return _fn(*args,**kwargs)
            finally:
                c=counters.setdefault(_name,dict(calls=0,inclusive_s=0.))
                c['calls']+=1;c['inclusive_s']+=perf_counter()-started
        setattr(lt,name,wrapper)
    checked=connector._state_failure
    sample_hash=hashlib.sha256();unique=set();samples=0
    def state(q,*args,**kwargs):
        nonlocal samples
        key=np.asarray(q,dtype=float).tobytes();unique.add(key);sample_hash.update(key);samples+=1
        if samples%1000==0: print('samples',samples, 'unique',len(unique),flush=True)
        return checked(q,*args,**kwargs)
    connector._state_failure=state
    report=dict(saved_result_sha256=file_hash(a.result),motion_sha256=file_hash(a.motion),
        state_sha256=file_hash(a.state),request_fingerprint=fingerprint(request.data),
        physical_binding_fields=fields,hardware=dict(platform=platform.platform(),cpu=platform.processor(),
        logical_cpus=os.cpu_count(),affinity=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None,
        threads={k:os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS')}),
        context_construction_s=setup,original_points=len(q),edges=max(0,len(q)-1),
        interpolation=trajectory['interpolation'],source_hashes={str(f):file_hash(f) for f in
        [Path('src/unloading_sim/layout_trajectory.py'),Path('src/unloading_sim/pinocchio_backend.py')]},
        timing_semantics='Counters are inclusive/nested; do not sum. Legacy context/key build and per-state tool pose are not separately measured.',
        runs=[])
    for run in range(1+a.warm_runs):
        before=dict(connector._statistics); vb=dict(validator.performance_counters)
        mesh=validator.mesh_robot; mb=dict(getattr(mesh,'performance_counters',{}))
        cb=dict(getattr(connector,'context_statistics',{}))
        edge_cache=getattr(connector,'_validation_edges',None)
        eb={} if edge_cache is None else {k:getattr(edge_cache,k) for k in ('hits','misses','evictions')}
        samples=0; unique.clear();sample_hash=hashlib.sha256();counters.clear()
        started=perf_counter();cpu=process_time()
        failure=connector._path_failure(q,obstacles,attachment=attachment,stage='transit')
        cpu=process_time()-cpu;wall=perf_counter()-started
        delta=lambda after,old:{k:v-old.get(k,0) for k,v in after.items() if isinstance(v,(int,float))}
        result=dict(kind='cold' if run==0 else 'same_context_warm',failure=failure,accepted=failure is None,
          wall_s=wall,cpu_s=cpu,expanded_samples=samples,unique_states=len(unique),sample_sha256=sample_hash.hexdigest(),
          connector=delta(connector._statistics,before),exact_validator=delta(validator.performance_counters,vb),
          mesh=delta(getattr(mesh,'performance_counters',{}),mb),obb_inclusive=dict(counters),
          state_cache_size=len(connector._state_cache),state_cache_capacity=connector._state_cache_limit,
          edge_cache=('not implemented' if edge_cache is None else delta({k:getattr(edge_cache,k) for k in eb},eb)),
          context=delta(getattr(connector,'context_statistics',{}),cb),authority_calls=dict(path=1,state=samples))
        report['runs'].append(result)
        (out/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps(result),flush=True)

if __name__=='__main__': main()
