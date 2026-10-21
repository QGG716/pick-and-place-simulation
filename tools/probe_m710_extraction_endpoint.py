"""Read-only endpoint diagnosis using the production Pinocchio FK and native KDL.

Reference numerical probes and out-of-bounds counterfactuals are diagnostics,
never motion candidates or changes to the production model/acceptance rules.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tarfile
from time import perf_counter


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--core-root', type=Path, required=True)
    p.add_argument('--previous-run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    sys.path[:0] = [str(args.core_root/'tools'), str(args.core_root/'src')]
    import numpy as np
    from unloading_sim.ik import solve_ik, pose_error
    from plan_m710_mount_comparison import make_context, FLAGS
    from plan_m710_top_layer_lifted import save
    from unloading_sim.moveit2_backend import ResidentMoveItClient
    _, connector, _ = make_context('xy_minus_100')
    robot = connector.robot
    assert type(robot).__name__ == 'PinocchioHppFclBackend'
    limits = robot.joint_limits.copy()
    args.output.mkdir(parents=True, exist_ok=False)
    summary = []
    for task, carton in (('task-012', 'carton_l05_c03'), ('task-016', 'carton_l04_c02')):
        current = args.previous_run/carton/'attempt-001/search-trace.json'
        if current.exists():
            trace = json.loads(current.read_text())
            source_trace = dict(path=str(current), sha256=hashlib.sha256(current.read_bytes()).hexdigest(), current_rerun=True)
        else:
            with tarfile.open(args.previous_run/(task+'.tar.gz')) as tar:
                raw = tar.extractfile(task+'/attempt-001/search-trace.json').read()
            trace = json.loads(raw)
            source_trace = dict(path=str(args.previous_run/(task+'.tar.gz')), member=task+'/attempt-001/search-trace.json',
                                sha256=hashlib.sha256(raw).hexdigest(), current_rerun=False)
        request = next(a['request'] for a in trace['attempts'] if a['request']['stage'] == 'extraction')
        target = np.array(request['goal_pose']); start = np.array(request['q_start'])
        target_id = request['process_policy']['target_id']
        client = ResidentMoveItClient('/bin/bash /worker.sh', log_path=str(args.output/(task+'.log')))
        native_records = []
        def native_ik(q):
            r = deepcopy(request)
            r.update(op='ik', q_start=q.tolist(), ik_mode='single_attempt', diagnostic_only=True)
            r.pop('q_goal', None)
            reply = client.request(r)
            native_records.append(dict(request=r, response=reply))
            return reply
        try:
            ready = client.request(trace['model'])
            assert ready['status'] == 'READY', ready
            f = client.request(dict(request, op='fk', links=['m710_virtual_task_tcp','flange','J2_link','J3_link']))
            mismatch = float(np.max(np.abs(np.asarray(f['frames']['m710_virtual_task_tcp'])-robot.fk(start))))
            assert mismatch < 1e-8, mismatch
            native_start = native_ik(start)
            def residual(q):
                actual = robot.fk(q)
                return pose_error(actual, target)[0]
            def measure(q):
                r = residual(q)
                singular_values = np.linalg.svd(robot.geometric_jacobian(q), compute_uv=False)
                return dict(q_rad=q.tolist(), position_error_m=float(np.linalg.norm(r[:3])),
                    orientation_error_rad=float(np.linalg.norm(r[3:])), within_official_limits=robot.within_limits(q),
                    margin_to_lower_rad=(q-limits[:,0]).tolist(), margin_to_upper_rad=(limits[:,1]-q).tolist(),
                    jacobian_singular_values=singular_values.tolist(),
                    fk_world=robot.fk(q).tolist())
            probes = []
            # Numerical initial guesses only; one identical endpoint, no face/pose scan.
            seeds = [('contact', start), ('limits_center', limits.mean(axis=1))]
            rng = np.random.default_rng(71070)
            seeds += [(f'diagnostic_seed_{i}', rng.uniform(limits[:,0],limits[:,1])) for i in range(8)]
            for label, seed in seeds:
                began = perf_counter()
                fit = solve_ik(robot, target, seed, max_iterations=800, damping=.02,
                               position_tolerance=1e-7, orientation_tolerance=1e-7,
                               position_weight=1., orientation_weight=1.)
                row = dict(seed_label=label, numerical_seed=seed.tolist(), bound_mode='OFFICIAL_LIMITS',
                    elapsed_s=perf_counter()-began, iterations=fit.iterations, optimizer_status=fit.message,
                    reference_solver='EXISTING_CORE_DLS_DIAGNOSTIC_ONLY', **measure(fit.q))
                if row['position_error_m']<1e-6 and row['orientation_error_rad']<1e-6:
                    row['native_kdl_from_reference_seed'] = native_ik(fit.q)
                probes.append(row)
                print(json.dumps(dict(target_id=target_id, seed=label, position_error_m=row['position_error_m'],
                      orientation_error_rad=row['orientation_error_rad'], q_rad=row['q_rad'])), flush=True)
            # Explicit counterfactual to identify a limit barrier. Never accepted or dispatched as motion.
            began = perf_counter()
            free_q=start.copy()
            for iteration in range(800):
                error=residual(free_q)
                if np.linalg.norm(error[:3])<1e-9 and np.linalg.norm(error[3:])<1e-9:break
                jacobian=robot.geometric_jacobian(free_q)
                step=jacobian.T@np.linalg.solve(jacobian@jacobian.T+.0001*np.eye(6),error)
                step*=min(1.,.15/max(float(np.linalg.norm(step)),1e-12))
                free_q+=step
            probes.append(dict(seed_label='contact', bound_mode='COUNTERFACTUAL_UNBOUNDED_NOT_ACCEPTABLE',
                elapsed_s=perf_counter()-began, iterations=iteration+1, **measure(free_q)))
            row = dict(target_id=target_id, source_task=task, source_trace=source_trace, source_request=request,
                official_limits_rad=limits.tolist(), native_pinocchio_fk_max_difference=mismatch,
                source_init=trace['model'], native_from_contact=native_start, probes=probes,
                native_records=native_records, start_measure=measure(start), **FLAGS)
            save(args.output/(task+'.json'), row)
            summary.append({k:row[k] for k in ('target_id','native_pinocchio_fk_max_difference','native_from_contact','probes')})
            save(args.output/'summary.json', dict(scope='HISTORICAL_ENDPOINT_DIAGNOSIS_NO_PLANNING_CREDIT',
                wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), outcomes=summary, **FLAGS))
        finally:
            client.close()


if __name__ == '__main__':
    main()
