# Vision and ROS 2 integration

## Process split

1. The ROS process uses Ubuntu 22.04 system Python 3.10, `rclpy`, generated messages, tf2, and the lightweight contracts package.
2. The vision worker uses its own interpreter and the pinned visual SHA. It has no `rclpy` dependency. Requests and responses are bounded, newline-delimited schema-1.1 JSON with request/frame/epoch identity. Timeouts, crashes, bad schema, and late results fail closed.
3. A future planning worker may use another isolated interpreter. It consumes the same serialized domain contracts; it must not add multiple branch versions of `unloading_sim` to one process.

`CargoJsonReplayBackend` and `SimGroundTruthBackend` are runnable. `CargoPipelineBackend` implements process isolation, a bounded queue, timeouts, restart epochs and capability/error handling. It accepts only local input/output references under configured roots and verifies every returned SHA-256 before deserializing an observation. `tools/vision_worker_entry.py` binds the fixed upstream checkout to sequential SAM, 2-D geometry, MoGe, 3-D cuboid and result-assembly subprocesses. SAM and MoGe are rerun; the fixed proposal and person-mask inputs are disclosed by the Mode-B manifest, so the run is not labelled raw-image automatic.

## Evidence behavior

Replay preserves null detector score, SAM score, 2D reprojection certificate, plane residual, geometry status, completion mode, certificate status, face evidence, bags, unknown cargo, and rejected/missing geometry. A high SAM IoU or `accepted=true` never validates metric scale. Missing covariance stays absent.

World conversion uses explicit `T_parent_child` at capture time. The contract uses SI units, complete dimensions, xyzw quaternions, and row-recorded upstream axes. Missing TF never becomes identity. Unknown or 2D-only cargo remains a blocking region rather than disappearing.

## ROS graph

- `/unloading/perception`: reliable typed observation snapshots.
- `/joint_states`: sensor-data QoS actual robot/mock state.
- `/unloading/world_snapshot`: reliable complete typed snapshot.
- `/unloading/markers`: RViz conservative geometry.
- `/unloading/execution_authorization`: volatile reliable authorization; not transient-local, so restarts do not replay commands.
- `/mock_controller/follow_joint_trajectory`: Humble action used only for simulation.
- `/unloading/execution_cancel`: identity-bound cancellation requests. `CANCEL_ACCEPTED` is only an event, not proof of rest.
- `/unloading/controller_stop_facts`: controller/simulator measurements without planning identity.
- `/unloading/stop_acknowledgements`: execution-bridge output that binds a stationary controller fact to command, plan, epoch, and planning generation.

The nodes use bounded TF lookup, an async action client, a reentrant callback group, and a multithreaded executor. Long vision work stays in the external worker and the perception callback only polls bounded IPC state. Default `enable_hardware=false`; setting it true is rejected because no verified FANUC adapter is present.

For rosbag replay record RGB/depth/CameraInfo, `/tf`, `/tf_static`, `/joint_states`, perception/world topics, configuration identities, and the manifest. Replayed consumers must still check capture time, epoch, freshness, and calibration identity.
