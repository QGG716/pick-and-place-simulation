"""Synthetic display-only demo: no models, perception admission or execution.

Run in a separate ROS_DOMAIN_ID from a live system. Each three-second phase uses
WorldBridgeNode.publish_markers, including its actual publisher and lifecycle.
"""
from copy import deepcopy
import json

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from unloading_interfaces.msg import CargoObservation, PlanningWorldSnapshot, UnknownRegion

from .common import require_humble_python310
from .world_bridge_node import WorldBridgeNode


def demo_snapshot(phase):
    value = PlanningWorldSnapshot(source_epoch='SYNTHETIC-DISPLAY-ONLY', planning_admissible=phase == 0)
    value.source_capture_time.sec = 100
    cube = CargoObservation(object_id='synthetic-cube-A', source_instance_id='synthetic-A',
        has_pose=True, has_full_dimensions=True, pose_frame_id='world',
        full_dimensions_m=[.4, .5, .6], candidate_eligible=True, association_status='SYNTHETIC_IDENTITY')
    cube.pose.position.x, cube.pose.position.z = 1., .5
    cube.pose.orientation.z, cube.pose.orientation.w = .6, .8
    other = deepcopy(cube)
    other.object_id, other.source_instance_id = 'synthetic-cube-B', 'synthetic-B'
    other.pose.position.y = 1.
    surface = dict(schema_version='observed_surface_v1', module_id='synthetic-module',
        capture_id='synthetic-cap', source_instance_id='synthetic-instance', face_id='face:0',
        frame_id='world', capture_time=100., sensor_epoch=value.source_epoch,
        corners_3d_m=[[1., 0., 1.], [1.4, .1, 1.], [1.4, .5, 1.], [1., .4, 1.]],
        point_support_count=100, plane_normal=[0., 0., 1.], plane_offset_m=-1.,
        plane_residual_m=0., volume_status='UNKNOWN', clock_domain='ros_sim_time',
        calibration_identity='synthetic-calibration',
        T_W_C_at_capture=[[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
    patch = CargoObservation(source_instance_id='synthetic-fusion',
        observed_surfaces_json=json.dumps([surface]), association_status='UNRESOLVED', candidate_eligible=False)
    if phase == 0:
        value.obstacles = [cube, other]
    else:
        value.blocking_reasons = ['OBSERVED_SURFACES_WITH_UNKNOWN_VOLUME']
        value.unknown_regions = [UnknownRegion(region_id='synthetic-unknown', frame_id='world',
            reason='UNKNOWN_VOLUME_BEHIND_OBSERVED_PATCH')]
        value.obstacles = [other, patch] if phase == 1 else [patch] if phase < 4 else []
        if phase == 2:
            patch.association_status = 'CONFLICT_RETAINED_NO_AVERAGE'
        if phase == 3:
            value.blocking_reasons.append('OBSERVATION_STALE')
    return value


def main(args=None):
    require_humble_python310()
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WorldBridgeNode()
    phase = 0
    def show():
        nonlocal phase
        node.publish_markers(demo_snapshot(phase), 'world')
        node.get_logger().info(f'SYNTHETIC display-only phase {phase}: complete / patch / conflict / history / empty')
        phase = (phase + 1) % 5
    show()
    node.create_timer(3., show, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
