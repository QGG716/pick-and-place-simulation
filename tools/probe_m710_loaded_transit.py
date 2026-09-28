"""Offline replay of an evidenced prefix, then ONE production loaded connection.

This is not a new full-cycle plan or a measured physical extraction state.
Missing extraction nodes are rebuilt by the original controlled producer once;
saved nodes on subsequent comparisons are rechecked with a fresh real tracker.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
from time import perf_counter

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.probe_m710_free_approach import rebuild, write
from unloading_sim.layout_single_carton import motion_implementation_identity
from unloading_sim.stage_motion_policy import failure_status


def main(args):
    if args.output.exists(): raise FileExistsError(args.output)
    case = json.loads(args.case.read_text())
    prior = json.loads(args.motion.read_text())
    old_trace = prior['tasks'][0]['attempts'][0]['trajectory_search']['attempts'][0]['trace']
    approach = json.loads(args.approach.read_text())
    expected_start = old_trace['stages']['transit']['attempts'][0]['connection']['failure']['motion_start']
    c, scene, target = rebuild(case)
    root = Path(sys.modules[type(c).__module__].__file__).resolve().parents[2]
    source = motion_implementation_identity(root)
    inputs = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (args.case, args.motion, args.approach, *([args.frozen] if args.frozen else []))}
    data = dict(scope='OFFLINE_LOADED_FRAGMENT_WITH_RECHECKED_PREFIX', source=source,
        inputs=inputs, historical_prefix_reused=True, new_full_cycle=False, isaac_executed=False,
        original_missing=['extraction_path_nodes', 'request_validation_checks_remaining'],
        original_request_remaining=None)
    started = perf_counter()
    progress = args.output.with_suffix('.progress.jsonl')
    progress.parent.mkdir(parents=True, exist_ok=True)
    class Finished(BaseException): pass
    class ResourceStop(BaseException): pass
    def stop(*_): raise ResourceStop()
    signal.signal(signal.SIGALRM, stop)
    result = None
    with progress.open('x') as log:
        def record(event):
            data['last_progress'] = event
            log.write(json.dumps(dict(elapsed_s=perf_counter()-started, **event), default=str)+'\n')
            log.flush()
        c.progress_callback = record
        record(dict(event='FIXED_INPUT', source=source, inputs=inputs))
        # The previous successful approach is input evidence, not replanned here.
        c._approach = lambda *a, **k: ([np.asarray(q) for q in approach['prefix']],
            [np.asarray(q) for q in approach['contact']], None, approach['evidence'])
        if args.frozen:
            frozen = json.loads(args.frozen.read_text())
            def extraction(start, attachment, obstacles, tracker, outward, *, seed):
                path = [np.asarray(q) for q in frozen['extraction']]
                np.testing.assert_array_equal(start, path[0])
                branch = tracker.clone()
                failure = c._path_failure(path, obstacles, attachment=attachment,
                    initial_proximity=branch, stage='extraction')
                if failure: raise ValueError(str(failure))
                if not branch.fully_released: raise ValueError('tracker did not release')
                if c._extraction_reserve_failure(path[-1], attachment, obstacles):
                    raise ValueError('actual planned extraction reserve failed')
                record(dict(event='RECHECKED_SAVED_EXTRACTION', tracker=branch.evidence()))
                yield path, branch, None, frozen['extraction_evidence']
            c._extraction_options = extraction
        original_finish = c._finish_place_branch
        def finish(**kw):
            np.testing.assert_allclose(kw['extraction'][-1], expected_start, atol=1e-12, rtol=0)
            if kw['released_tracker'].evidence() != old_trace['stages']['extraction']['attempts'][0]['initial_proximity']:
                raise ValueError('rebuilt tracker differs from original evidence')
            data.update(extraction=kw['extraction'], extraction_evidence=kw['trace']['stages']['extraction'],
                tracker=kw['released_tracker'].evidence(), attachment=dict(
                    tcp_from_box=kw['rigid'].tcp_from_box, half_extents=kw['rigid'].half_extents,
                    flange_from_virtual=c.flange_from_virtual_task_tcp,
                    flange_from_physical=c.flange_from_physical_contact),
                contact_q=kw['contact_q'], start_q=kw['extraction'][-1],
                placement=kw['placement'].as_dict(), release_height=kw['release_height'], seed=kw['seed'],
                remaining_scene=[dict(name=b.name, pose=b.world_from_local, half_extents=b.half_extents)
                                 for b in kw['payload_obstacles']],
                local_shared_remaining=c._local_transit_remaining,
                rrt_remaining=c._placement_remaining, rrt_candidate_allowance=c._placement_candidate_allowance)
            write(args.output.with_suffix('.frozen.json'), data)
            signal.setitimer(signal.ITIMER_REAL, args.connection_resource_seconds)
            return original_finish(**kw)
        c._finish_place_branch = finish
        original_connect = c._connect_pose
        def connection(pose, seeds, start, obstacles, **kw):
            nonlocal result
            data.update(preplace_virtual=pose, goal_seeds=seeds,
                        ik_seed=kw['ik_seed'], connection_seed=kw['connection_seed'])
            write(args.output.with_suffix('.frozen.json'), data)
            result = original_connect(pose, seeds, start, obstacles, **kw)
            raise Finished()
        c._connect_pose = connection
        try:
            c._plan_branch_search(target=target, face=case['face'],
                requested_virtual_contact=np.asarray(case['requested_virtual_contact']),
                grasp_q=np.asarray(case['grasp_q']), home_q=np.asarray(case['start_q']),
                all_obstacles=scene.all_obstacles,
                receiver=next(b for b in scene.all_obstacles if b.category == 'conveyor'),
                support_names=old_trace['stages']['support-release']['support_names'],
                suction=scene.policy.data['suction'], seed=old_trace['seed'])
        except Finished:
            q, path, failure, evidence = result
            data.update(status=failure_status(failure), failure=failure, path=path, evidence=evidence)
        except ResourceStop:
            data.update(status='INDETERMINATE', failure=dict(reason='OFFLINE_CONNECTION_RESOURCE_LIMIT'),
                        resource_stop_is_not_infeasibility=True)
        except KeyboardInterrupt:
            data.update(status='CANCELLED', failure=dict(reason='MANUAL_CANCELLED'))
        except BaseException as exc:
            data.update(status='INDETERMINATE', failure=dict(reason='PROBE_ERROR', detail=repr(exc)))
            raise
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            data.update(elapsed_seconds=perf_counter()-started, statistics=c._statistics,
                context_statistics=c._context_statistics(), local_shared_after=getattr(c,'_local_transit_remaining',None),
                connection_resource_seconds=args.connection_resource_seconds,
                source_unchanged=source==motion_implementation_identity(root),
                inputs_unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in inputs.items()))
            write(args.output, data)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('case','motion','approach','output'): p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--frozen', type=Path)
    p.add_argument('--connection-resource-seconds', type=float, default=300.)
    main(p.parse_args())
