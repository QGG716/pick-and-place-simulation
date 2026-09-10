"""Official-model gravity initialization diagnostic; never a pick execution.

Consumes its own diagnostic schema, not a motion bundle. The only robot
command is a constant finite-effort joint target. Cartons are never attached.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--usd-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()
    if not 0.5 <= args.seconds <= 30.0:
        raise ValueError("diagnostic duration must lie in [0.5, 30] seconds")
    root = args.project_root.resolve()
    sys.path.insert(0, str(root / "src"))
    from unloading_sim.m710_initialization_diagnostic import (
        SCOPE, combined_j6_inertial, generated_usd_tree_identity, summarize_initialization_motion,
        trailer_side_wall_transform, verify_initialization_diagnostic_contract,
    )
    from unloading_sim.asset_audit import audit_m710id70_official_model
    from unloading_sim.isaac_layout_replay import usd_safe_prim_segment
    from unloading_sim.isaac_collision_policy import apply_robot_only_srdf_filters
    contract = json.loads(args.contract.read_text(encoding="utf-8"))
    verify_initialization_diagnostic_contract(contract)
    official = audit_m710id70_official_model(root)
    if official.manifest_sha256 != contract["official_model_audit"]["manifest_sha256"]:
        raise ValueError("official model provenance changed since diagnostic freeze")
    for record in contract["assets"].values():
        path = (root / record["repository_path"]).resolve()
        path.relative_to(root)
        if hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
            raise ValueError(f"diagnostic asset hash mismatch: {path.name}")
    args.output.mkdir(parents=True, exist_ok=True)
    args.usd_directory.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = None
    writer = None
    states, frame_count = [], 0
    status = {"status": "INITIALIZING", "scope": SCOPE, "physical_pick_success": False, "attachment_count": 0,
              "initial_state_audit": contract["initial_state_audit"],
              "robot_pose_role": contract["robot_pose_role"],
              "contract_fingerprint": contract["contract_fingerprint"]}
    (args.output / "run_status.json").write_text(json.dumps(status, indent=2), encoding="utf-8")
    try:
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": 1920, "height": 1080})
        import cv2
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from isaacsim.core.api import World
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

        def pose(prim, transform):
            # USD row-vector matrices are the transpose of the core convention.
            matrix = Gf.Matrix4d(np.asarray(transform, dtype=float).T.tolist())
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(matrix)

        urdf_path = root / contract["robot"]["urdf"]["repository_path"]
        importer = URDFImporter(config=URDFImporterConfig(
            urdf_path=str(urdf_path), usd_path=str(args.usd_directory.resolve()),
            merge_fixed_joints=False, merge_mesh=False, collision_from_visuals=False,
            collision_type="Convex Decomposition", allow_self_collision=True, fix_base=True,
            joint_drive_type="force", joint_target_type="position",
            run_asset_transformer=True, run_multi_physics_conversion=True,
        ))
        usd_path = str(importer.import_urdf())
        status["generated_usd"] = generated_usd_tree_identity(usd_path, args.usd_directory)
        context = omni.usd.get_context()
        if not context.open_stage(usd_path):
            raise RuntimeError("could not open official imported USD")
        for _ in range(5):
            app.update()
        stage = context.get_stage()
        robot_root = stage.GetPrimAtPath(f"/{urdf_path.stem}")
        if not robot_root.IsValid():
            robot_root = stage.GetDefaultPrim()
        if not robot_root.IsValid():
            raise RuntimeError("official robot root is missing")
        variant = robot_root.GetVariantSet("Physics")
        if variant.IsValid():
            variant.SetVariantSelection("physx")
        robot_path = str(robot_root.GetPath())
        pose(robot_root, contract["robot"]["world_from_mount"])
        expected = list(contract["robot"]["joint_names"])
        q = np.asarray(contract["robot"]["q_rad"], dtype=np.float32)
        q_by_name = dict(zip(expected, q.tolist()))
        links = {p.GetName(): p for p in stage.Traverse()
                 if str(p.GetPath()).startswith(robot_path) and p.HasAPI(UsdPhysics.RigidBodyAPI)}
        status["srdf_collision_filters"] = apply_robot_only_srdf_filters(
            stage, robot_path, root / contract["assets"]["robot_srdf"]["repository_path"])
        dynamics = contract["dynamics"]
        for name, body in dynamics["robot_links"].items():
            if name not in links:
                raise RuntimeError(f"official physical link missing: {name}")
            mass, centre, tensor = body["mass_kg"], np.asarray(body["com_xyz_m"]), np.asarray(body["inertia_tensor_com_kg_m2"])
            if name == "J6_link":
                mass, centre, tensor = combined_j6_inertial(contract)
            moments, axes = np.linalg.eigh(tensor)
            if np.linalg.det(axes) < 0.0:
                axes[:, 0] *= -1.0
            quat = Gf.Matrix3d(axes.T.tolist()).ExtractRotation().GetQuat()
            mass_api = UsdPhysics.MassAPI.Apply(links[name])
            mass_api.CreateMassAttr(float(mass))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*centre.tolist()))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*moments.tolist()))
            mass_api.CreatePrincipalAxesAttr(Gf.Quatf(quat))
        authored_drives = set()
        for prim in stage.Traverse():
            if (str(prim.GetPath()).startswith(robot_path + "/")
                    and prim.GetName() in dynamics["joint_drives"] and prim.IsA(UsdPhysics.RevoluteJoint)):
                if prim.GetName() in authored_drives:
                    raise RuntimeError(f"ambiguous official drive joint: {prim.GetName()}")
                authored_drives.add(prim.GetName())
                drive = dynamics["joint_drives"][prim.GetName()]
                api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                api.CreateTypeAttr("force")
                api.CreateMaxForceAttr(drive["effort_limit_nm"])
                api.CreateStiffnessAttr(drive["stiffness_nm_rad"])
                api.CreateDampingAttr(drive["damping_nm_s_rad"])
                q_deg = float(np.degrees(q_by_name[prim.GetName()]))
                api.CreateTargetPositionAttr(q_deg)
                api.CreateTargetVelocityAttr(0.0)
                if prim.HasAPI(PhysxSchema.JointStateAPI, "angular"):
                    joint_state = PhysxSchema.JointStateAPI.Get(prim, "angular")
                else:
                    joint_state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
                joint_state.CreatePositionAttr(q_deg)
                joint_state.CreateVelocityAttr(0.0)
                PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(float(np.degrees(drive["velocity_limit_rad_s"])))
        if authored_drives != set(dynamics["joint_drives"]):
            raise RuntimeError("could not configure all six official finite-effort drives")

        def cube(path, transform, size, color, *, visible=True):
            obj = UsdGeom.Cube.Define(stage, path)
            obj.CreateSizeAttr(1.0)
            pose(obj.GetPrim(), transform)
            UsdGeom.Xformable(obj.GetPrim()).AddScaleOp().Set(Gf.Vec3f(*size))
            obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            if not visible:
                UsdGeom.Imageable(obj.GetPrim()).MakeInvisible()
            return obj.GetPrim()

        UsdGeom.Xform.Define(stage, "/Diagnostic")
        carton_paths, carton_names = [], []
        j6_from_world = np.linalg.inv(np.asarray(contract["robot"]["world_from_J6"]))
        for index, item in enumerate(contract["primitives"]):
            role = item["role"]
            transform = np.asarray(item["pose_world"])
            path = f"/Diagnostic/{usd_safe_prim_segment(index, item['name'])}"
            if role == "rigid_tool":
                path = f"{links['J6_link'].GetPath()}/DiagnosticRigidTool{index}"
                transform = j6_from_world @ transform
            prim = cube(path, transform, item["size_xyz_m"],
                        (0.62, 0.35, 0.13) if role == "dynamic_carton" else (0.19, 0.35, 0.43),
                        visible=role != "rigid_tool")
            if role == "dynamic_carton":
                UsdPhysics.RigidBodyAPI.Apply(prim)
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                mass_api.CreateMassAttr(42.5)
                mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0))
                mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(dynamics["cartons"]["inertia_tensor_com_kg_m2"]).tolist()))
                carton_paths.append(path)
                carton_names.append(item["name"])
            material_name = "carton" if role == "dynamic_carton" else "painted_steel"
            mat_path = f"/Diagnostic/Materials/{material_name}"
            material = UsdShade.Material.Define(stage, mat_path)
            api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            values = dynamics["contacts"][material_name]
            api.CreateStaticFrictionAttr(values["static_friction"])
            api.CreateDynamicFrictionAttr(values["dynamic_friction"])
            api.CreateRestitutionAttr(values["restitution"])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

        # Same CAD tessellation as the existing full replay. Flexible cup solids
        # are visible, while only the 58 rigid solids carry collision shapes.
        mesh_record = contract["tool_visual"]
        mesh_bytes = (root / mesh_record["mesh"]["repository_path"]).read_bytes()
        triangle_count = int.from_bytes(mesh_bytes[80:84], "little")
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        if len(mesh_bytes) != 84 + 50 * triangle_count:
            raise ValueError("tool visual requires the frozen binary CAD STL")
        vertices = np.frombuffer(mesh_bytes, dtype=dtype, count=triangle_count, offset=84)["vertices"].reshape(-1, 3)
        vertices = (vertices - np.asarray(mesh_record["flange_origin_step_mm"])) @ np.asarray(mesh_record["rotation_step_from_tool"]) * 0.001
        j6_from_flange = j6_from_world @ np.asarray(contract["robot"]["world_from_flange"])
        vertices = vertices @ j6_from_flange[:3, :3].T + j6_from_flange[:3, 3]
        mesh = UsdGeom.Mesh.Define(stage, f"{links['J6_link'].GetPath()}/DiagnosticWantaiVisual")
        mesh.CreatePointsAttr([Gf.Vec3f(*v.tolist()) for v in vertices])
        mesh.CreateFaceVertexCountsAttr([3] * triangle_count)
        mesh.CreateFaceVertexIndicesAttr(list(range(len(vertices))))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(0.48, 0.5, 0.52)])

        # The confirmed floor is a plane. Finite display extent is not a trailer
        # dimension or a collision wall; no invented trailer end wall is added.
        floor = UsdGeom.Plane.Define(stage, "/Diagnostic/Floor")
        floor.CreateAxisAttr("Z")
        floor.CreateWidthAttr(20.0)
        floor.CreateLengthAttr(20.0)
        floor_pose = np.eye(4)
        floor_pose[2, 3] = contract["world"]["floor_z_m"]
        pose(floor.GetPrim(), floor_pose)
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        floor.CreateDisplayColorAttr([Gf.Vec3f(0.21, 0.22, 0.23)])
        for name, y, inward_y in (
            ("RightWall", contract["trailer"]["right_wall_y_m"], 1.0),
            ("LeftWall", contract["trailer"]["left_wall_y_m"], -1.0),
        ):
            wall = UsdGeom.Plane.Define(stage, f"/Diagnostic/{name}")
            wall.CreateAxisAttr("Y")
            wall.CreateWidthAttr(20.0)
            wall.CreateLengthAttr(20.0)
            transform = trailer_side_wall_transform(y, inward_y)
            pose(wall.GetPrim(), transform)
            UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
            UsdGeom.Imageable(wall.GetPrim()).MakeInvisible()
        rep.create.light(light_type="distant", intensity=1000.0, rotation=(310.0, 0.0, 25.0))
        rep.create.light(light_type="sphere", position=(-1.0, -2.0, 4.0), intensity=5500.0, scale=0.8)
        camera = rep.create.camera(position=(-5.3, -4.1, 3.4), look_at=(-1.25, 0.0, 1.15), focal_length=24.0)
        product = rep.create.render_product(camera, (1920, 1080))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        dt = float(dynamics["simulation"]["physics_time_step_s"])
        # Bootstrap physics handles at zero time. World.reset() performs an
        # internal update, so using the run dt here would move bodies before
        # the authored-state acceptance check.
        world = World(stage_units_in_meters=1.0, physics_dt=0.0, rendering_dt=1.0 / 30.0)
        world.get_physics_context().set_gravity(-9.81)
        articulation = Articulation(robot_path)
        cartons = RigidPrim(carton_paths)
        names = list(articulation.dof_names)
        if set(names) != set(expected) or len(names) != 6:
            raise RuntimeError(f"official DOFs differ from contract: {names}")
        command = q[[expected.index(name) for name in names]][None, :]
        authored_positions = np.asarray([
            np.asarray(item["pose_world"], dtype=float)[:3, 3]
            for item in contract["primitives"] if item["role"] == "dynamic_carton"
        ])
        status["articulation_initialization"] = {
            "method": "usd_physx_joint_state_before_world_reset",
            "joint_state_position_units": "degrees",
            "joint_state_velocity_units": "degrees_per_second",
            "drive_target_position_units": "degrees",
            "per_frame_teleport": False,
        }
        world.reset()
        articulation.switch_dof_control_mode("position")
        effort_limits = np.asarray([dynamics["joint_drives"][name]["effort_limit_nm"] for name in names], dtype=np.float32)
        velocity_limits = np.asarray([dynamics["joint_drives"][name]["velocity_limit_rad_s"] for name in names], dtype=np.float32)
        stiffness = np.asarray([dynamics["joint_drives"][name]["stiffness_nm_rad"] for name in names], dtype=np.float32)
        damping = np.asarray([dynamics["joint_drives"][name]["damping_nm_s_rad"] for name in names], dtype=np.float32)
        articulation.set_dof_max_efforts(effort_limits[None, :])
        articulation.set_dof_max_velocities(velocity_limits[None, :])
        articulation.set_dof_gains(stiffness[None, :], damping[None, :])
        status["joint_drive_limits"] = {"joint_names": names, "effort_limit_nm": effort_limits.tolist(),
                                       "velocity_limit_rad_s": velocity_limits.tolist()}
        articulation.set_dof_position_targets(command)
        reset_q = np.asarray(articulation.get_dof_positions().numpy())[0]
        reset_q_error = float(np.max(np.abs(reset_q - command[0])))
        reset_carton_positions = np.asarray(cartons.get_world_poses()[0].numpy())
        reset_carton_error = float(np.max(np.linalg.norm(
            reset_carton_positions - authored_positions, axis=1)))
        status["articulation_initialization"].update({
            "bootstrap_physics_time_step_s": 0.0,
            "runtime_physics_time_step_s": dt,
            "commanded_q_rad": command[0].tolist(),
            "measured_q_after_reset_rad": reset_q.tolist(),
            "abs_q_error_after_reset_rad": np.abs(reset_q - command[0]).tolist(),
            "max_abs_q_error_after_reset_rad": reset_q_error,
            "acceptance_limit_rad": 1.0e-5,
            "max_authored_carton_position_error_after_reset_m": reset_carton_error,
        })
        if reset_q_error > 1.0e-5:
            raise RuntimeError(
                f"pre-reset authored articulation state mismatch: {reset_q_error:.9g} rad"
            )
        world.set_simulation_dt(physics_dt=dt, rendering_dt=1.0 / 30.0)
        # Synchronize the annotator to the verified state without advancing the
        # timeline, then explicitly discard the warm-up buffer.
        rep.orchestrator.step(
            rt_subframes=4, pause_timeline=False, delta_time=0.0, wait_for_render=True
        )
        rgb.get_data()
        status["render_synchronization"] = {
            "method": "replicator_orchestrator_zero_delta_prewarm",
            "delta_time_s": 0.0,
            "rt_subframes": 4,
            "discarded_annotator_buffers": 1,
        }
        writer = cv2.VideoWriter(str(args.output / "initialization.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (1920, 1080))
        if not writer.isOpened():
            raise RuntimeError("could not open continuous diagnostic MP4")
        first_scene_frame = second_scene_frame = final_scene_frame = None
        max_capture_q_delta = 0.0
        max_capture_carton_position_delta = 0.0
        render_every = int(round(1.0 / (30.0 * dt)))
        if not np.isclose(render_every * dt, 1.0 / 30.0):
            raise ValueError("physics time step must divide normal-time 30 FPS")
        steps = int(round(args.seconds / dt))
        for step in range(steps):
            render = (step + 1) % render_every == 0
            # Advance physics exactly once, synchronizing transforms to Fabric.
            # Rendering is a separate zero-time synchronous operation below.
            world.step(render=False, update_fabric=True)
            if not render:
                continue
            positions, orientations = cartons.get_world_poses()
            linear_velocities, angular_velocities = cartons.get_velocities()
            positions = np.asarray(positions.numpy()).copy()
            orientations = np.asarray(orientations.numpy()).copy()
            measured_q = np.asarray(articulation.get_dof_positions().numpy())[0].copy()
            # Replicator wait_for_render guarantees that the returned annotator
            # buffer is the transform state just pushed to Fabric. delta_time=0
            # must leave that physical state unchanged.
            rep.orchestrator.step(
                rt_subframes=1, pause_timeline=False, delta_time=0.0, wait_for_render=True
            )
            after_capture_q = np.asarray(articulation.get_dof_positions().numpy())[0]
            after_capture_positions = np.asarray(cartons.get_world_poses()[0].numpy())
            max_capture_q_delta = max(
                max_capture_q_delta, float(np.max(np.abs(after_capture_q - measured_q))))
            max_capture_carton_position_delta = max(
                max_capture_carton_position_delta,
                float(np.max(np.linalg.norm(after_capture_positions - positions, axis=1))))
            states.append({"time_s": (step + 1) * dt, "joint_names": names,
                           "q_rad": measured_q.tolist(),
                           "carton_names": carton_names, "carton_positions_m": positions.tolist(),
                           "carton_orientations_wxyz": orientations.tolist(),
                           "carton_linear_velocities_m_s": np.asarray(linear_velocities.numpy()).tolist(),
                           "carton_angular_velocities_rad_s": np.asarray(angular_velocities.numpy()).tolist()})
            rgba = np.asarray(rgb.get_data())
            if rgba.shape[:2] != (1080, 1920):
                raise RuntimeError(f"camera returned invalid diagnostic frame {rgba.shape}")
            bgr = cv2.cvtColor(rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
            scene_frame = bgr.copy()
            if frame_count == 0:
                first_scene_frame = scene_frame
            elif frame_count == 1:
                second_scene_frame = scene_frame
            final_scene_frame = scene_frame
            cv2.rectangle(bgr, (0, 0), (1920, 95), (18, 18, 18), -1)
            cv2.putText(bgr, SCOPE, (24, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 210, 255), 2)
            cv2.putText(bgr, f"40 dynamic cartons | gravity settling | initial clearance={contract['initial_state_audit']['status']} | t={(step + 1)*dt:.2f}s", (24, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (245, 245, 245), 2)
            writer.write(bgr)
            if frame_count == 0:
                if not cv2.imwrite(str(args.output / "initial.png"), bgr):
                    raise RuntimeError("could not write the initial diagnostic keyframe")
            elif frame_count == 1:
                if not cv2.imwrite(str(args.output / "second.png"), bgr):
                    raise RuntimeError("could not write the second diagnostic keyframe")
            frame_count += 1
        if not cv2.imwrite(str(args.output / "final.png"), bgr):
            raise RuntimeError("could not write the final diagnostic keyframe")
        writer.release()
        writer = None
        video_path = args.output / "initialization.mp4"
        if not video_path.is_file() or video_path.stat().st_size == 0:
            raise RuntimeError("continuous diagnostic MP4 was not written")
        (args.output / "actual_states.json").write_text(json.dumps(states), encoding="utf-8")
        criteria = dynamics["settling"]
        observations = summarize_initialization_motion(
            states, authored_positions, command[0], criteria
        )
        if first_scene_frame is None or second_scene_frame is None or final_scene_frame is None:
            raise RuntimeError("diagnostic did not capture first, second, and final scene frames")

        def image_delta(first, second):
            absolute = np.abs(first.astype(np.int16) - second.astype(np.int16))
            return {
                "mean_absolute_channel_difference": float(np.mean(absolute)),
                "p99_absolute_channel_difference": float(np.percentile(absolute, 99.0)),
                "max_absolute_channel_difference": int(np.max(absolute)),
                "fraction_channels_over_20": float(np.mean(absolute > 20)),
            }

        status["render_synchronization"].update({
            "formal_capture_method": "physics_only_update_fabric_then_zero_delta_replicator",
            "formal_capture_rt_subframes": 1,
            "max_capture_q_delta_rad": max_capture_q_delta,
            "max_capture_carton_position_delta_m": max_capture_carton_position_delta,
            "scene_rgb_differences_without_overlay": {
                "first_to_second": image_delta(first_scene_frame, second_scene_frame),
                "second_to_final": image_delta(second_scene_frame, final_scene_frame),
                "first_to_final": image_delta(first_scene_frame, final_scene_frame),
            },
        })
        status.update({"status": SCOPE, "backend_initialization_completed": True,
                       "carton_count": len(carton_names), "mass_kg_each": 42.5,
                       "robot_plus_tool_mass_kg": 600.347, "video_frame_count": frame_count,
                       "video_physical_time_scale": 1.0, "render_size": [1920, 1080],
                       "gravity_settling": "OBSERVED_NOT_FULL_PICK_VALIDATION",
                       "initialization_diagnostic_result": observations["classification"],
                       "settling_observations": observations,
                       "video": "initialization.mp4", "actual_states": "actual_states.json",
                       "keyframes": ["initial.png", "second.png", "final.png"],
                       "video_sha256": hashlib.sha256(video_path.read_bytes()).hexdigest(),
                       "adapter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    except Exception as exc:
        status.update({"status": "FAIL", "reason": str(exc), "traceback": traceback.format_exc()})
        raise
    finally:
        shutdown_exit_code = 1
        try:
            if writer is not None:
                writer.release()
            if states:
                (args.output / "actual_states.json").write_text(json.dumps(states), encoding="utf-8")
            status["recorded_state_count"] = len(states)
            status["video_frame_count"] = frame_count
            (args.output / "run_status.json").write_text(json.dumps(status, indent=2), encoding="utf-8")
            print(json.dumps(status, indent=2), flush=True)
            shutdown_exit_code = 0 if status.get("status") == SCOPE else 1
        finally:
            if app is not None:
                # Isaac 6 fast shutdown can otherwise terminate the process
                # with zero before Python propagates the active exception.
                app.close(exit_code=shutdown_exit_code)


if __name__ == "__main__":
    main()
