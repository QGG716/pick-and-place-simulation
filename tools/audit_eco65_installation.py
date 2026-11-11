"""Read-only installation audit: no hardware, planning, rendering or asset writes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from unloading_sim.robot import URDFRobot
from unloading_sim.moveit2_backend import digest
from unloading_sim.workbench.coordinates import Extrinsics, Installation, rigid
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.eco65.mount_alignment import check_alignment
from unloading_sim.eco65.native_contact import extract_cad_contours, validate_geometry
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.touch_calibration import validate_observations, validate_result, observation_binding
from build_desktop_suction_assembly import plane_loops

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def close(a, b, tolerance=1e-10):
    error = float(np.max(np.abs(np.asarray(a) - np.asarray(b))))
    if error > tolerance:
        raise ValueError(f"MATRIX_OR_GEOMETRY_MISMATCH: {error} > {tolerance}")
    return error


def delta(a, b):
    d = np.linalg.inv(rigid(a)) @ rigid(b)
    return dict(matrix=d.tolist(), translation_m=float(np.linalg.norm(d[:3, 3])),
                rotation_deg=float(np.degrees(Rotation.from_matrix(d[:3, :3]).magnitude())))


def direct_xml_fk(urdf, q):
    """Independent XML origin/axis composition, without URDFRobot FK helpers."""
    frames = {"base_link": np.eye(4)}
    angles = dict(zip((f"joint_{i}" for i in range(1, 7)), q))
    for j in ET.parse(urdf).getroot().findall("joint"):
        origin = j.find("origin")
        t = np.eye(4)
        t[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
        t[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).as_matrix()
        turn = np.eye(4)
        axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
        turn[:3, :3] = Rotation.from_rotvec(axis * angles[j.get("name")]).as_matrix()
        frames[j.find("child").get("link")] = frames[j.find("parent").get("link")] @ t @ turn
    return frames


def audit(root, snapshot_path, installation_path):
    def read(p):
        return json.loads((root / p).read_text(encoding="utf-8-sig"))

    tool_path = root / "configs/local/eco65_desktop/tool.json"
    cad_path = root / "assets/tools/desktop_suction/derived/manifests/cad.json"
    tool, cad = read(tool_path), read(cad_path)
    snapshot = read(snapshot_path)
    scene = snapshot["scene"]
    installation = Installation.load(installation_path)
    training_path = root / "configs/workbench/training_extrinsics.example.json"
    training = Extrinsics.load(training_path)
    installation.check_camera(training)
    step = resolve_asset(root, cad["source"])
    assert sha(step) == cad["source_sha256"] == tool["source_sha256"]
    assert sha(cad_path) == tool["cad_manifest_sha256"]
    assert cad["file_length_units"] == "millimetres" and cad["normalization_scale_to_m"] == .001
    assert tool["length_unit"] == "m" and tool["transform_convention"] == "T_A_B maps B to A"
    assert b"SI_UNIT ( .MILLI., .METRE. )" in step.read_bytes()
    assert len(cad["parts"]) == len(tool["visual_parts"]) == 43
    assert all(p["topology_valid"] for p in cad["parts"])
    assert len({p["id"] for p in tool["visual_parts"]}) == 43
    assert len(tool["collisions"]) == len({p["id"] for p in tool["collisions"]}) == 57
    for p in tool["visual_parts"]:
        assert sha(resolve_asset(root, p["visual"])) == p["visual_sha256"]
    for p in tool["collisions"]:
        assert sha(resolve_asset(root, p["mesh"])) == p["mesh_sha256"]
    geometry = validate_geometry(extract_cad_contours(tool, root))
    f_c, c_t, f_t = [rigid(tool[k]) for k in ("T_flange_tool_cad", "T_tool_cad_tcp", "T_flange_tcp")]
    composition_error = close(f_c @ c_t, f_t)
    configured_tool = dict(status="NO_TOOL_MATRIX_LOADED")
    if installation.tool is not None:
        configured_tool = dict(name=installation.tool.name, status=installation.tool.status,
                               difference_from_cad=delta(f_t, installation.tool.matrix))
        if "CAD_REFERENCE_ONLY" in installation.tool.status:
            close(installation.tool.matrix, f_t)
    centers = np.array([s["center_cad_m"] for s in tool["seals"]])
    center_error = close(centers.mean(axis=0), c_t[:3, 3])
    close(c_t[:3, :3], [[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    points_tcp = []
    for row in geometry["contours"]:
        t_c = np.linalg.inv(c_t)
        points_tcp.extend(np.asarray(row["points_m"]) @ t_c[:3, :3].T + t_c[:3, 3])
        close(t_c[:3, :3] @ row["normal"], [0, 0, 1])
    lip_plane_error = float(np.max(np.abs(np.asarray(points_tcp)[:, 2])))
    assert lip_plane_error < 1e-10

    # Re-read the official end-face mesh and CAD manifest, not stored hole residuals.
    flange, = [p for p in cad["parts"] if p["name"].startswith("L57-")]
    faces = flange["surface_features"]
    mounting = max((f for f in faces if f["type"] == "plane" and
                    np.allclose(f["outward_normal"], [0, 1, 0])), key=lambda f: f["point_m"][1])
    mount_y = mounting["point_m"][1]
    holes = [f for f in faces if f["type"] == "cylinder" and abs(f["axis"][1]) > .999
             and abs(f["bounds_m"][1][1] - mount_y) < 1e-8 and f["radius_m"] < .005]
    tool_xz = np.unique(np.round([np.asarray(f["point_m"])[[0, 2]] for f in holes], 10), axis=0)
    model_dir = root / "assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B"
    flange_mesh = model_dir / "meshes/link_6.STL"
    assert sha(flange_mesh) == tool["mounting_evidence"]["official_flange_mesh_sha256"]
    mesh = trimesh.load(flange_mesh, force="mesh", process=True)
    close(mesh.bounds[1, 2], 0., 1e-7)
    loops = plane_loops(mesh, np.array([0, 0, 1]), 0.)
    radii = [np.mean(np.linalg.norm(p[:, :2] - p[:, :2].mean(axis=0), axis=1)) for p in loops]
    robot_xy = np.array([p[:, :2].mean(axis=0) for p, radius in zip(loops, radii)
                         if abs(radius - min(radii)) < 1e-6])
    alignment = check_alignment(robot_xy, tool_xz, mount_y, f_c)
    assert alignment["status"] == "PASS"
    assert snapshot["tool"] == tool

    urdf = model_dir / "urdf/ECO65-B.urdf"
    robot = URDFRobot.from_urdf(urdf, active_joint_names=[f"joint_{i}" for i in range(1, 7)],
                               base_link="base_link", tip_link="link_6", tool_length=0.)
    d_b = np.eye(4)
    d_b[:3, :3] = Rotation.from_euler("xyz", scene["base_rpy_rad"]).as_matrix()
    d_b[:3, 3] = scene["base_position_m"]
    old_w_d = rigid(scene["T_W_D"])
    old_w_b = old_w_d @ d_b
    b_d = np.linalg.inv(old_w_b) @ old_w_d
    close(installation.world_from_base, np.eye(4))
    close(old_w_b[:3, 3], [-.12, -.1, 1.04])
    close(b_d[:3, 3], [-.44, 0, -.24])
    samples = []
    for degrees in ([0]*6, [90, 0, 0, 0, 0, 0], [20, -30, 45, 10, 25, -15], [-35, 15, -40, 20, -30, 45]):
        q = np.deg2rad(degrees)
        frames, direct = robot.named_link_frames(q), direct_xml_fk(urdf, q)
        fk_error = max(close(frames[k], direct[k]) for k in frames)
        b_cam = frames["link_1"] @ training.matrix
        b_tcp = frames["link_6"] @ f_t
        roundtrip = max(close(b_d @ d_b @ frames["link_1"] @ training.matrix, b_cam),
                        close(b_d @ d_b @ frames["link_6"] @ f_c @ c_t, b_tcp))
        samples.append(dict(q_degrees=degrees, independent_fk_max_error=fk_error,
                            scene_base_roundtrip_max_error=roundtrip,
                            T_base_camera_training=b_cam.tolist(), T_base_tcp_cad_reference=b_tcp.tolist()))
    unchanged = robot.named_link_frames(np.array([0,.2,-.3,.4,-.5,.6]))["link_1"]
    close(unchanged, robot.named_link_frames(np.zeros(6))["link_1"])

    nominal = rigid(scene["permanent_camera"]["T_link1_optical"])
    close(nominal, scene["camera_extrinsics"]["T_parent_child"])
    calibration = scene["perception_binding"]["calibration"]
    obs, result = calibration["tcp_observations"], calibration["tcp_solver_result"]
    validate_observations(obs)
    validate_result(result, obs)
    assert calibration["tcp_observation_binding"] == observation_binding(obs)
    assert calibration["tcp_heldout_validation"]["observation_binding"] == observation_binding(obs)
    close(calibration["T_link1_optical"], result["T_parent_child"])
    assert sha(urdf) == obs["kinematics"]["urdf_sha256"]
    snapshot_check = dict(snapshot); snapshot_check.pop("fingerprint")
    assert digest(snapshot_check) == snapshot["fingerprint"]
    file_failures = []
    counts = {}
    for group in ("asset_sha256", "config_sha256"):
        count = 0
        for name, expected in snapshot[group].items():
            p = resolve_asset(root, name)
            error = "MISSING" if not p.is_file() else "SHA256_MISMATCH" if sha(p) != expected else None
            if error:
                file_failures.append(dict(group=group, path=name, expected_sha256=expected, error=error))
            else:
                count += 1
        counts[group] = dict(verified=count, expected=len(snapshot[group]))
    try:
        verify_snapshot(snapshot, root)
        runtime = dict(status="PASS")
    except (ValueError, OSError) as exc:
        runtime = dict(status="BLOCKED", reason=str(exc))
    inventory = read("assets/ASSET_INVENTORY.json")["files"]
    image_hashes = {m["sha256"] for m in inventory.values()}
    archived_images = sum(p["sha256"] in image_hashes for p in obs["photos"])
    return dict(
        status="FRAME_CHECKS_PASS_WITH_OPEN_EVIDENCE_ITEMS", hardware_validation="NOT_RUN",
        scope="CAD-derived geometry and coordinate algebra; no CAD re-tessellation, image re-solving, motion or hardware IO",
        inputs={str(p.relative_to(root)): sha(p) for p in (step, cad_path, tool_path, snapshot_path, training_path, urdf, installation_path)},
        tool=dict(status="CAD_REFERENCE_VERIFIED_NOT_MEASURED_TCP", parts=43, collisions=57, cups=12,
                  configured_display_tool=configured_tool,
                  cup_ids=geometry["expected_contour_ids"], T_flange_tcp=f_t.tolist(),
                  translation_mm=(1000*f_t[:3,3]).tolist(), flange_z_rotation_deg=float(np.degrees(np.arctan2(f_t[1,0],f_t[0,0]))),
                  composition_error=composition_error, center_error_m=center_error, lip_plane_error_m=lip_plane_error,
                  recomputed_mount_alignment=alignment, assembly_status=tool["assembly_status"],
                  roll_candidates_rad=tool["mounting_evidence"]["candidate_rolls_rad"],
                  actual_tcp_calibration=tool["physics"]["actual_tcp_calibration"],
                  independent_control_verified=tool["contact"]["independent_control_verified"]),
        camera=dict(parent="link_1", optical_axes="x right, y down, z forward",
                    training=dict(status=training.status, usage=training.usage, session=training.session_status,
                                  T_link1_camera=training.matrix.tolist(), installation_identity="NOT_VERIFIED_ON_CURRENT_D435I"),
                    snapshot_nominal_status=scene["camera_extrinsics"]["status"],
                    simulation=dict(source=calibration["source"], archived_validation_status=calibration["status"],
                                    embedded_observation_binding="PASS", photos=len(obs["photos"]), touches=len(obs["touches"]),
                                    photo_hashes_present_in_asset_inventory=archived_images,
                                    independent_image_revalidation="NOT_RUN", T_link1_camera=calibration["T_link1_optical"],
                                    recorded_heldout_error_m=calibration["tcp_heldout_validation"]["held_out_max_3d_m"]),
                    nominal_to_training=delta(nominal, training.matrix),
                    simulation_to_training=delta(calibration["T_link1_optical"], training.matrix),
                    nominal_to_simulation=delta(nominal, calibration["T_link1_optical"])),
        world=dict(status="PASS_COORDINATE_DEFINITION", T_world_base=installation.world_from_base.tolist(),
                   historical_T_W_base=old_w_b.tolist(), T_base_D=b_d.tolist(), samples=samples),
        snapshot=dict(fingerprint="PASS", files=counts, failures=file_failures, runtime_validation=runtime),
        remaining=["Physical tool angular indexing and TCP not measured",
                   "Current D435i installation/extrinsics not independently validated",
                   "Historical snapshot dependencies must match before complete-scene acceptance"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, default=ROOT / "outputs/workbench/scene_snapshot.json")
    parser.add_argument("--installation", type=Path, default=ROOT / "outputs/workbench/installation.local.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(ROOT, args.snapshot.resolve(), args.installation.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(dict(status=report["status"], tool=report["tool"]["status"], world=report["world"]["status"],
                         snapshot=report["snapshot"]["runtime_validation"], hardware="NOT_RUN"), ensure_ascii=False))


if __name__ == "__main__":
    main()
