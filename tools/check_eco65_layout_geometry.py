"""Directed layout-only mesh/installation checks; no native motion planning."""
import argparse
from pathlib import Path
import numpy as np

from unloading_sim.eco65.model import load_json, save_json, robot_for
from unloading_sim.eco65.coupled_scene import coupled_conveyor_state
from unloading_sim.eco65.native_io import verify_snapshot


def bounds(row):
    if 'convex_mesh' in row:
        v = np.asarray(row['convex_mesh']['vertices'])
    else:
        half = np.asarray(row['size']) / 2
        v = np.array([[x, y, z] for x in (-half[0], half[0])
                      for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    T = np.asarray(row['pose'])
    v = v @ T[:3, :3].T + T[:3, 3]
    return np.array([v.min(axis=0), v.max(axis=0)])


def check(snapshot, previous):
    scene, old = snapshot['scene'], previous['scene']
    spec = scene['coupled_conveyor_spec']
    rows, reference = [], None
    for height in (.18, .33, .43, .45):
        state = coupled_conveyor_state(spec, height)
        by_id = {row['id']: row for row in state['world_objects']}
        if reference is None:
            reference = by_id
        for name in state['mechanism']['fixed_structure_ids']:
            assert by_id[name] == reference[name], name
        belt_bounds, frame_bounds = {}, {}
        belt_ids = []
        for owner in ('conveyor_transverse', 'conveyor_longitudinal'):
            belts = [r for r in state['world_objects'] if r['owner'] == owner and r['role'] == 'conveyor_belt']
            # This particular frozen ECO65 asset has one contour per belt.
            assert len(belts) == 1, (owner, len(belts))
            belt_ids.append(belts[0]['id'])
            bb = bounds(belts[0])
            frame = next(r for r in state['world_objects'] if r['owner'] == owner and r['role'] == 'conveyor_frame')
            fb = bounds(frame)
            assert abs(bb[0, 1] + .3) < 1e-6, (owner, bb)
            assert np.max(abs(bb[:, 1] - fb[:, 1])) < 1e-6
            belt_bounds[owner], frame_bounds[owner] = bb.tolist(), fb.tolist()
        m = state['mechanism']
        length = float(np.linalg.norm(np.array(m['upper_endpoint_D_m']) - m['lower_endpoint_D_m']))
        assert abs(length - .4443197215708027) < 1e-12
        for stack in m['stacks']:
            for bar in stack['bars']:
                assert abs(np.linalg.norm(np.diff(bar['endpoints_m'], axis=0)) - bar['length_m']) < 1e-12
        rows.append(dict(height_m=height, tilt_rad=state['tilt_rad'], belt_ids=belt_ids,
                         actual_belt_bounds_D_m=belt_bounds, frame_bounds_D_m=frame_bounds,
                         seam=m['seam'], longitudinal_length_m=length,
                         fixed_structures_unchanged=True, scissor_lengths_unchanged=True))
    assert not any(r['id'].startswith('conveyor_') for r in scene.get('mesh_obstacles', []))
    assert len(scene['boxes']) == 12
    assert [(b['id'], b['size_m'], b['pose']) for b in scene['boxes']] == [
        (b['id'], b['size_m'], b['pose']) for b in old['boxes']]
    installation = []
    for q1 in (-2., 0., 1.7):
        q = np.array([q1, 0., 0., 0., 0., 0.])
        a = robot_for(previous['tool'], old).named_link_frames(q)
        b = robot_for(snapshot['tool'], scene).named_link_frames(q)
        A = a['link_1'] @ old['mast']['T_link1_mast']
        B = b['link_1'] @ scene['mast']['T_link1_mast']
        yaw_error = float(np.max(abs(B[:3, :3] @ A[:3, :3].T - np.diag([-1., -1., 1.]))))
        assert yaw_error < 1e-12
        ca = A @ old['permanent_camera']['T_mast_optical']
        cb = B @ scene['permanent_camera']['T_mast_optical']
        camera_error = float(np.max(abs(ca[:3, :3] - cb[:3, :3])))
        assert camera_error < 1e-12
        assert np.max(abs(b['base_link'] - a['base_link'])) < 1e-12
        assert np.max(abs(b['base_link'][:3, :3] @ [1., 0., 0.] - [-1., 0., 0.])) < 1e-12
        assert abs(B[2, 3] - A[2, 3]) < 1e-12
        installation.append(dict(raw_j1_rad=q1, mast_add180_matrix_error=yaw_error,
                                 camera_orientation_matrix_error=camera_error,
                                 mast_z_m=float(B[2, 3]), switch_normal_D=[-1., 0., 0.]))
    return dict(status='PASS', scope='LAYOUT_GEOMETRY_ONLY', planning_only=True,
                native_planning='NOT_RUN', physical_execution='NOT_RUN', execution_ready=False,
                snapshot_fingerprint=snapshot['fingerprint'], previous_snapshot_fingerprint=previous['fingerprint'],
                belt_geometry_revision=spec['belt_geometry_revision'],
                conveyor_assemblies=2, world_cartons=12, attached_cartons=0, completed_cartons=0,
                heights=rows, installation=installation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', required=True, type=Path)
    parser.add_argument('--previous', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = check(verify_snapshot(load_json(args.snapshot)), verify_snapshot(load_json(args.previous)))
    save_json(args.output, report)
    print(report['status'], report['scope'], args.output)


if __name__ == '__main__':
    main()
