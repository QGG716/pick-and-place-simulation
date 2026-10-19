"""Necessary reach bounds for the frozen ECO65 top-suction task, not a planner."""
import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from unloading_sim.eco65.asset_inventory import load_robot_config
from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot


def inspect(snapshot):
    verify_snapshot(snapshot)
    if snapshot['scene'].get('native_task',{}).get('cup_selection_mode') == 'SIMULATED_INDEPENDENT_CUPS':
        raise ValueError('ALL_CUP_TCP_HULL_BOUND_NOT_APPLICABLE_TO_INDEPENDENT_SELECTION')
    config = load_robot_config(); path = ROOT/config['urdf']
    joints = {j.get('name'): j for j in ET.parse(path).findall('joint')}
    def origin(index):
        joint = joints[f'joint_{index}']; node = joint.find('origin')
        if joint.get('type') != 'revolute' or not np.array_equal(
                np.fromstring(joint.find('axis').get('xyz'), sep=' '), [0, 0, 1]):
            raise ValueError('BOUND_REQUIRES_PINNED_ECO65_JOINT_AXES')
        return (np.fromstring(node.get('xyz'), sep=' '),
                Rotation.from_euler('xyz', np.fromstring(node.get('rpy'), sep=' ')).as_matrix())
    geometry = snapshot['native_model']['init']['process_geometry']
    F_C = np.asarray(geometry['flange_from_contact_frame'])
    F_TCP = np.asarray(snapshot['tool']['T_flange_tcp'])
    TCP_C = np.linalg.inv(F_TCP)@F_C
    points = np.concatenate([np.asarray(r['points_m']) for r in geometry['contours']])
    points = points@TCP_C[:3, :3].T+TCP_C[:3, 3]
    hull = ConvexHull(points[:, :2])
    if (len(geometry['contours']) != 12 or np.max(np.abs(points[:, 2])) > 1e-7
            or np.max(hull.equations[:, -1]) > -1e-8
            or not np.allclose(F_TCP[:3, 2], [0, 0, 1], atol=1e-12, rtol=0)):
        raise ValueError('TCP_INSIDE_TWELVE_SEAL_CONTOUR_HULL_REQUIRED')
    t1, R1 = origin(1); t2, _ = origin(2)
    if not np.allclose(R1, np.eye(3), atol=1e-12, rtol=0):
        raise ValueError('BOUND_REQUIRES_VERTICAL_J1')
    lengths = {f'joint_{i}': float(np.linalg.norm(origin(i)[0])) for i in (3, 4, 5)}
    maximum = sum(lengths.values())
    t6, R6 = origin(6); offset6 = R6.T@t6
    axial = float(F_TCP[2, 3]+offset6[2])
    # The rounded official rpy leaves a sub-micron transverse J6 offset.
    # Give that entire offset to reachability, regardless of the J6 angle.
    transverse = float(np.linalg.norm(offset6[:2])+np.linalg.norm(F_TCP[:2, 3]))
    if axial <= 0: raise ValueError('EXPECTED_FORWARD_TOOL_AXIS_REQUIRED')
    scene = snapshot['scene']; box = scene['box']
    center = np.asarray(box['pose']); size = np.asarray(box['size_m'])
    if center.shape != (3,): raise ValueError('BOUND_REQUIRES_CURRENT_AXIS_ALIGNED_BOX')
    base = np.asarray(scene['base_position_m']); axis = base+t1
    lo, hi = center-size/2, center+size/2
    angle = float(snapshot['tool']['contact']['normal_tolerance_rad'])
    tolerance = float(scene['contact_tolerance_m'])
    if not 0 <= angle < np.pi/2: raise ValueError('INVALID_CONTACT_NORMAL_TOLERANCE')
    planar_distance = float(np.linalg.norm(np.maximum(np.maximum(lo[:2]-axis[:2], axis[:2]-hi[:2]), 0)))
    horizontal_lower = max(0., planar_distance-np.linalg.norm(t2[:2])-axial*np.sin(angle)-transverse)
    shoulder_z = float(axis[2]+t2[2])
    wrist_z_lower = float(hi[2]-tolerance+axial*np.cos(angle)-transverse)
    vertical_lower = max(0., wrist_z_lower-shoulder_z)
    necessary = float(np.hypot(horizontal_lower, vertical_lower))
    # Minimum caliper width over every in-plane yaw of the actual 12-loop hull.
    polygon = points[hull.vertices, :2]; edges = np.roll(polygon, -1, axis=0)-polygon
    normals = np.column_stack([-edges[:, 1], edges[:, 0]])/np.linalg.norm(edges, axis=1)[:, None]
    minimum_width = float(np.min(np.ptp(polygon@normals.T, axis=0)))
    return dict(status='TOP_CONTACT_GEOMETRICALLY_UNREACHABLE' if necessary > maximum+1e-7 else 'BOUND_INCONCLUSIVE',
        snapshot_fingerprint=snapshot['fingerprint'], target_id=scene['target_box_id'],
        urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        joint2_to_joint5_segment_lengths_m=lengths, maximum_joint2_to_joint5_distance_m=maximum,
        necessary_joint2_to_joint5_distance_lower_bound_m=necessary, reach_deficit_lower_bound_m=necessary-maximum,
        horizontal_distance_lower_bound_m=horizontal_lower, vertical_distance_lower_bound_m=vertical_lower,
        shoulder_z_m=shoulder_z, wrist_z_lower_bound_m=wrist_z_lower,
        tcp_to_joint5_axial_length_m=axial, transverse_offset_allowance_m=transverse,
        contact_normal_tolerance_rad=angle, contact_position_tolerance_m=tolerance,
        tcp_inside_actual_contour_hull=True, contour_count=12,
        minimum_seal_hull_width_m=minimum_width, front_face_height_m=float(size[2]),
        front_face_possible_by_width=bool(minimum_width*np.cos(angle) <= size[2]-2*.003),
        proof='Triangle inequality for J2-to-J5; all J1 angles and all top-face TCP positions admitted. '
              'Uses the whole carton rectangle without the seal edge margin, and gives all transverse offsets '
              'and allowed normal tilt to reachability. Joint limits and obstacles are ignored, making the bound generous.',
        planning_status='PLANNING_ONLY_NOT_EXECUTABLE', dynamics='NOT_EVALUATED')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    report = inspect(load_json(args.snapshot)); save_json(args.output, report)
    print(report)
