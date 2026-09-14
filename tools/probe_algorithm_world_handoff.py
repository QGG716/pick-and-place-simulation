"""Replay frozen algorithm observations through real ROS nodes, without actions."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from unloading_interfaces.msg import MechanismState,PerceptionObservation,PlanningWorldSnapshot
from unloading_contracts import loads,to_wire,canonical_fingerprint
from unloading_ros_bridge.mapping import observation_to_msg,observation_from_msg,snapshot_from_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode
from unloading_perception.isaac_validation import IsaacSceneManifest,build_feasibility_handoff


def stamp(value):
    ns=round(value*1e9); return Time(sec=ns//1000000000,nanosec=ns%1000000000)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--observation',type=Path,required=True); p.add_argument('--manifest',type=Path,required=True); p.add_argument('--output-directory',type=Path,required=True); args=p.parse_args()
    observation=loads(args.observation.read_text()); manifest=IsaacSceneManifest.from_dict(json.loads(args.manifest.read_text()))
    if observation.provider!='registered-rgbd-fused-algorithm' or observation.synthetic: raise ValueError('ALGORITHM_OBSERVATION_REQUIRED')
    message=observation_to_msg(observation)
    assert canonical_fingerprint(observation_from_msg(message))==canonical_fingerprint(observation)
    rclpy.init(); bridge=WorldBridgeNode(); bridge.set_parameters([Parameter('use_sim_time',value=True),Parameter('expected_joint_names',value=list(manifest.robot['joint_names']))])
    node=rclpy.create_node('frozen_algorithm_handoff_probe'); executor=SingleThreadedExecutor(); executor.add_node(node); executor.add_node(bridge)
    worlds=[]
    node.create_subscription(PlanningWorldSnapshot,'/unloading/world_snapshot',worlds.append,10)
    clocks=node.create_publisher(Clock,'/clock',10)
    joints=node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
    mechanisms=node.create_publisher(MechanismState,'/unloading/mechanism_state',10)
    observations=node.create_publisher(PerceptionObservation,'/unloading/perception',10)
    def wait(predicate,timeout=10):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        raise RuntimeError('ROS_HANDOFF_TIMEOUT')
    try:
        wait(lambda: all(p.get_subscription_count()>0 for p in (clocks,joints,mechanisms,observations)))
        # Keep actual capture/result times. Replay now is the recorded result
        # time, so old evidence stays stale; it is never refreshed to wall time.
        clocks.publish(Clock(clock=stamp(observation.processed_time)))
        wait(lambda: bridge.get_clock().now().nanoseconds>0)
        state=JointState(name=list(manifest.robot['joint_names']),position=list(manifest.robot['q_rad']),velocity=[0.]*6); state.header.stamp=stamp(observation.capture_time); joints.publish(state)
        tool=dict(manifest.mechanisms['tool_state']); tool.update(verified=tool.get('attached') is True,verification_source='ISAAC_MANIFEST_ATTACHED_STATE')
        base=dict(manifest.mechanisms['base_state']); base['position_m']=[row[3] for row in base['T_W_A'][:3]]
        mechanisms.publish(MechanismState(schema_version='1.1.0',source_epoch=observation.source_epoch,sequence=0,source_restart=True,observed_time=stamp(observation.capture_time),clock_domain='ros_sim_time',tool_state_identity=tool['identity'],payload_state_identity=manifest.mechanisms['payload_state']['identity'],base_state_identity=base['identity'],conveyor_state_identity=manifest.mechanisms['conveyor_state']['identity'],config_identity=manifest.manifest_fingerprint,robot_model_fingerprint=manifest.robot['robot_asset_hash'],world_model_fingerprint=manifest.world_fingerprint,tool_state_json=json.dumps(tool),payload_state_json=json.dumps(manifest.mechanisms['payload_state']),base_state_json=json.dumps(base),conveyor_state_json=json.dumps(manifest.mechanisms['conveyor_state'])))
        observations.publish(message); wait(lambda: bool(worlds))
        world=worlds[-1]; snapshot=snapshot_from_msg(world)
        expected=sum(len(c.observed_surfaces) for c in observation.cargo)
        actual=sum(len(o['observed_surfaces']) for o in snapshot.scene_snapshot['obstacles'])
        assert actual==expected
        assert all(o['full_dimensions_m'] is None for o in snapshot.scene_snapshot['obstacles'])
        assert not world.planning_admissible
        before=bridge.algorithm_observation_guard.sequence
        observations.publish(observation_to_msg(replace(observation,source_sequence=before-1)))
        end=time.monotonic()+.3
        while time.monotonic()<end: executor.spin_once(timeout_sec=.02)
        assert bridge.algorithm_observation_guard.sequence==before
        out=args.output_directory; out.mkdir(parents=True,exist_ok=False)
        (out/'perception_world_snapshot.json').write_text(json.dumps(to_wire(snapshot),indent=2))
        (out/'perception_feasibility_handoff.json').write_text(json.dumps(build_feasibility_handoff(snapshot,manifest,evidence_mode='VISION_ESTIMATE'),indent=2))
        (out/'ros_handoff_report.json').write_text(json.dumps({'status':'TRANSPORT_AND_SEMANTICS_PASS','planning_admissible':world.planning_admissible,'blocking_reasons':list(world.blocking_reasons),'surface_count':actual,'source_capture_time':observation.capture_time,'processed_time':observation.processed_time,'round_trip_fingerprint':canonical_fingerprint(observation),'stale_sequence_rejected':True,'execution_commands_generated':0,'source_kind':'ALGORITHM_FROM_ISAAC_RENDERED_RGBD'},indent=2))
        print((out/'ros_handoff_report.json').read_text())
    finally:
        executor.shutdown(); node.destroy_node(); bridge.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
