"""Numerical comparison against the actual authorized reference FK, not link names alone."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np

from unloading_sim.robot import URDFRobot
from unloading_sim.workbench.coordinates import rigid

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/urdf/ECO65-B.urdf"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--snapshot", type=Path)
    a = p.parse_args()
    reference = a.reference.resolve()
    sys.path.insert(0, str(reference))
    module = importlib.import_module("arith_dex_pkg.kinematics")
    ref = module.ECO65()
    sim = URDFRobot.from_urdf(URDF, active_joint_names=[f"joint_{i}" for i in range(1, 7)],
                              base_link="base_link", tip_link="link_6", tool_length=0.)
    samples = []
    for degrees in ([0, 0, 0, 0, 0, 0], [20, -30, 45, 10, 25, -15], [-35, 15, -40, 20, -30, 45]):
        q = np.deg2rad(degrees)
        rf, sf = ref.link_transforms(q), sim.named_link_frames(q)
        differences = {}
        for name in rf:
            delta = np.linalg.inv(rf[name]) @ sf[name]
            differences[name] = {"translation_m": float(np.linalg.norm(delta[:3, 3])),
                "rotation_matrix_max_abs": float(np.max(np.abs(delta[:3, :3]-np.eye(3)))),
                "reference_T_base_link": rf[name].tolist(), "main_T_base_link": sf[name].tolist()}
        samples.append({"source_joints_deg": degrees, "main_joints_rad": q.tolist(), "links": differences})
    max_error = max(max(r["translation_m"], r["rotation_matrix_max_abs"])
                    for s in samples for r in s["links"].values())
    if max_error > 1e-9:
        raise ValueError("FK mismatch; do not enable identity mapping")
    ref_joints = ET.parse(module.MODEL_PATH).getroot().findall("joint")
    model_joints = ET.parse(URDF).getroot().findall("joint")
    semantics = lambda joints: [{"name": j.get("name"), "type": j.get("type"),
        **{k: dict(j.find(k).attrib) for k in ("parent", "child", "origin", "axis") if j.find(k) is not None}}
        for j in joints]
    same_semantics = semantics(ref_joints) == semantics(model_joints)
    if not same_semantics:
        raise ValueError("URDF semantics mismatch")
    extrinsics = json.loads((reference / "active_extrinsics.json").read_text(encoding="utf-8"))
    legacy = json.loads((reference / "hand_eye.json").read_text(encoding="utf-8"))
    try:
        rigid(legacy["matrix"])
    except ValueError:
        invalid_rejected = True
    else:
        raise ValueError("Legacy nonrigid matrix unexpectedly accepted")
    rigid(extrinsics["matrix"])
    tool = json.loads((reference / "unloading/tool_tip.json").read_text(encoding="utf-8"))
    report = {"schema": "eco65_workbench_frame_audit_v1",
        "main_base_commit": "269ab67f8932771b6f1ad1f43ed73beaf6f90602",
        "reference_head": subprocess.check_output(["git", "-C", str(reference), "rev-parse", "HEAD"], text=True).strip(),
        "reference_urdf_sha256": sha(module.MODEL_PATH), "main_urdf_sha256": sha(URDF),
        "same_joint_origin_axis_semantics": same_semantics,
        "limit_differences": [{"joint": r.get("name"), "reference": dict(r.find("limit").attrib),
                               "main": dict(m.find("limit").attrib)} for r, m in zip(ref_joints, model_joints)
                              if r.find("limit").attrib != m.find("limit").attrib],
        "limits_policy": "Retain main model limits; measured samples are recorded but out-of-model coordinates are rejected",
        "joint_mapping": {"mapping_id": "ECO65_B_REFERENCE_F0353D7_IDENTITY_DEGREES_TO_RADIANS",
            "source_names": [f"J{i}" for i in range(1, 7)],
            "target_names": [f"joint_{i}" for i in range(1, 7)], "source_indices": list(range(6)),
            "signs": [1]*6, "zero_offsets_rad": [0.]*6, "source_unit": "degree",
            "evidence": "Identical joint origins/axes/order and all-link FK at three poses; limits differ and remain main-model values"},
        "samples": samples, "max_fk_component_error": max_error,
        "controller_mapping_basis": "reference robot_telemetry.py reads state['joint'] in degrees; reference GUI converts via deg2rad; no offset or sign remap",
        "hardware_joint_zero_validation": "NOT_RUN",
        "calibration": {"sha256": sha(reference / "active_extrinsics.json"),
            "status": extrinsics["status"], "usage": extrinsics["usage"],
            "model_sha256": extrinsics["model_sha256"],
            "matches_reference_model_hash": extrinsics["model_sha256"] == sha(module.MODEL_PATH),
            "raw_session": "MISSING_FROM_CHECKOUT", "legacy_hand_eye_rejected": invalid_rejected},
        "tool": tool, "implicit_180_degree_compensation": False,
        "hardware_validation": "NOT_RUN"}
    main_tool_path = ROOT / "configs/local/eco65_desktop/tool.json"
    if main_tool_path.exists():
        main_tool = json.loads(main_tool_path.read_text(encoding="utf-8"))
        actual_tcp = np.eye(4)
        actual_tcp[:3, 3] = tool["tool_tip_m"]
        report["tool"].update(main_simulation_T_flange_tcp=main_tool["T_flange_tcp"],
            T_simulation_tcp_reference_tool=(np.linalg.inv(rigid(main_tool["T_flange_tcp"])) @ actual_tcp).tolist(),
            reference_tool_orientation="IDENTITY_ASSUMPTION_REQUIRES_PHYSICAL_VALIDATION",
            collision_solids=len(main_tool["collisions"]), physical_cups=len(main_tool["seals"]))
    if a.snapshot:
        from unloading_sim.geometry import rotation_matrix_from_rpy
        scene = json.loads(a.snapshot.read_text(encoding="utf-8"))["scene"]
        d_base = np.eye(4)
        d_base[:3,:3] = rotation_matrix_from_rpy(*scene['base_rpy_rad'])
        d_base[:3,3] = scene['base_position_m']
        report['world_frames'] = {'viewer_frame':'D', 'world_frame':'W', 'T_D_base':d_base.tolist(),
                                  'T_W_D':scene['T_W_D'], 'T_W_base':(rigid(scene['T_W_D'])@d_base).tolist()}
        nominal = scene["permanent_camera"]["T_link1_optical"]
        report["camera_installation"] = {"snapshot_sha256": sha(a.snapshot),
            "T_simulated_optical_calibrated_optical": (np.linalg.inv(rigid(nominal)) @ rigid(extrinsics["matrix"])).tolist(),
            "action": "Show calibrated optical frame separately; retain original camera shell geometry and disclose disagreement"}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": "PASS", "samples": 3, "max_fk_component_error": max_error,
                      "legacy_hand_eye_rejected": invalid_rejected, "hardware_validation": "NOT_RUN"}))


if __name__ == "__main__":
    main()
