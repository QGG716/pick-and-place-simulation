"""Generate geometry-only receiving schemes. No native worker or motion requests."""
from pathlib import Path
import argparse
import json
import platform
import sys
from time import perf_counter
from types import SimpleNamespace
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.geometry import OBB
from unloading_sim.workcell_layout import _obb_record, canonical_digest, sha256_file
from unloading_sim.layout_single_carton import load_layout_motion_policy, _build_automatic_trajectory_connector
from unloading_sim.moveit2_backend import native_tool_collision_boxes, model_request
from unloading_sim.validation_physics import urdf_collision_shapes, world_link_boxes
from unloading_sim.independent_cups import (m710_cup_array_from_mapping,
    evaluate_independent_cup_geometry, select_ideal_independent_cups)
from unloading_sim.lifted_reception import (MARKERS, SCHEMES, LiftedConveyorScene, bounds,
    directional_contact, process_scheme)
from export_m710_search_replay import export_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/experiments/m710_lifted_reception_preview.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True, help="host checkout SHA (chroot need not contain git)")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    began = perf_counter()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if config["gap_m"] != .05 or config["external_clearance_m"] != .005:
        raise ValueError("PREVIEW_MUST_RETAIN_REQUESTED_GAP_AND_NORMAL_CLEARANCE")
    policy = load_layout_motion_policy(ROOT/config["motion_policy"])
    layout = policy.layout_validation.layout
    robot = layout.robot()
    # Construct existing read-only model/geometry providers. No plan, IK, worker,
    # candidate generation, strict full-path verification or execution is invoked.
    built = _build_automatic_trajectory_connector(policy, robot)
    if built.connector is None:
        raise RuntimeError(built.failure_reason)
    c = built.connector
    q = policy.layout_validation.initial_q
    original = layout.cartons()
    base_scene = LiftedConveyorScene.build(layout)
    static_tools, tool_binding = native_tool_collision_boxes(c, q)
    compliant = {b.name for b in c.robot_state_validator._compliant_boxes(q)}
    physical = c.physical_from_virtual(c.robot.fk(q))
    contact_from_world = np.linalg.inv(physical)
    local_tools = [b.transformed(contact_from_world) for b in static_tools]
    reference = world_link_boxes(robot, q, urdf_collision_shapes(robot)) + static_tools
    scene_adapter = SimpleNamespace(policy=policy, snapshot=dict(scene_fingerprint=base_scene.scene_fingerprint))
    model, _, _ = model_request(c, scene_adapter)
    render_model = export_model(model)
    top_z = max(bounds(b)[0][2] for b in original)
    row = sorted([b for b in original if abs(bounds(b)[0][2]-top_z)<1e-9], key=lambda b:b.center[1])
    right, middle, left = row[0], row[len(row)//2], row[-1]
    # Side cases are explicitly prepared geometry initial conditions, not new
    # unloading completions. Keep the opposite end and all lower cartons.
    removed = [b.name for b in row[1:-1]]
    opened = tuple(b for b in original if b.name not in removed)
    lower = max((b for b in original if bounds(b)[0][2]<top_z-1e-9 and abs(b.center[1]-middle.center[1])<1e-9),
                key=lambda b:b.center[2])
    taller = OBB(middle.center + [0,0,.05], middle.half_extents + [0,0,.05], middle.rotation,
                 middle.name, middle.category)  # 0.40 m tall; fixed original bottom, disclosed variant.
    varied = tuple(taller if b.name==middle.name else b for b in original)
    cases = [
        ("top_middle_front", "顶层中间 · 正面 / Hmax",middle,original,original,"middle_front",[]),
        ("top_left_front", "左端 · 正面 / Hmax",left,original,original,"left_front",[]),
        ("top_left_inner", "左端 · 内侧 / Hmin",left,original,opened,"left_inner",removed),
        ("top_right_inner", "右端 · 内侧 / Hmax",right,original,opened,"right_inner",removed),
        ("left_inner_blocked", "反例 · 中间箱未移除",left,original,original,"left_inner",[]),
        ("lower_middle_front", "较低层 · 上层保留",lower,original,original,"middle_front",[]),
        ("taller_middle_front", "变高箱 · h=0.40 m",taller,varied,varied,"middle_front",[]),
    ]
    rows=[]
    for key,label,target,provenance,remaining,scheme,removed_ids in cases:
        contact = directional_contact(target, SCHEMES[scheme][1])
        tools=[] if contact is None else [b.transformed(np.asarray(contact["contact_pose"])) for b in local_tools]
        row_result = process_scheme(layout,target,provenance,remaining,scheme,tool_in_contact=tools,
            compliant_ids=compliant, reference_robot=reference, edge_reserve_m=config["edge_reserve_m"],
            clearance_m=config["external_clearance_m"],old_maximum_extraction_m=c.budget.maximum_extraction_m,
            mechanical_surface_limits_m=config["mechanical_surface_limits_world_m"],
            ignore_flexible_neighbors=c.collision_policy.compliant_cup_neighbor_contact_mode=="ignore",
            planar_stack_contact_allowed=c.collision_policy.stack_contact_mode=="planner_relaxed_physics_checked",
            final_separation_m=c.collision_policy.free_space_clearance_m,
            maximum_initial_cup_penetration_m=c.collision_policy.maximum_compliant_cup_additional_compression_m)
        row_result.update(case_id=key,label=label,removed_ids=removed_ids,
            removed_reason="DECLARED_GEOMETRIC_INITIAL_CONDITION_MIDDLE_CHANNEL_OPEN" if removed_ids else None,
            newly_unloaded_count=0,original_target=_obb_record(target))
        if contact is not None:
            legacy={(0,-1):"front",(1,-1):"right",(1,1):"left"}[(contact["axis"],contact["sign"])]
            cups=evaluate_independent_cup_geometry(np.asarray(contact["contact_pose"]),target,legacy,
                m710_cup_array_from_mapping(policy.data["suction"]),max_attachment_gap_m=.002,
                maximum_penetration_m=c.contact_tolerance_m,max_normal_misalignment_rad=np.deg2rad(5),
                suction_edge_margin_m=policy.data["suction"]["suction_edge_margin_m"])
            selection=select_ideal_independent_cups(cups,pose_source="GEOMETRIC_TEMPLATE_ONLY_NO_IK")
            row_result["nominal_cup_geometry"]=dict(eligible_mask=list(selection.geometrically_eligible_mask),
                command_intent_mask=list(selection.commanded_active_mask),actual_contact_mask=None,
                physical_contact="NOT_MEASURED",attachment="NOT_PERFORMED")
            if not any(selection.commanded_active_mask):
                row_result["reasons"].append("NO_FULL_SEAL_RING_AT_TEMPLATE_POSE")
                row_result["applicability"]="TEMPLATE_NOT_APPLICABLE"
        rows.append(row_result)
        print(json.dumps(dict(case=key,receiving_region=row_result["receiving_region"]["exists"],
            applicability=row_result["applicability"],reasons=row_result["reasons"])),flush=True)
    source = args.source_commit
    if len(source)!=40 or any(ch not in "0123456789abcdef" for ch in source):
        raise ValueError("FULL_SOURCE_COMMIT_REQUIRED")
    result = dict(**MARKERS, schema="m710_lifted_reception_preview_v1", source_commit=source,
        command=[sys.executable,*sys.argv], config=config,config_sha256=sha256_file(args.config),
        environment=dict(python=sys.version,platform=platform.platform()),
        layout_fingerprint=layout.layout_fingerprint,model_identity=model["identity"],tool_binding=tool_binding,
        reference_q_rad=q.tolist(),reference_robot_motion="STATIC_REFERENCE_NOT_FOLLOWING_TOOL_NO_IK",
        normal_policy=c.collision_policy.to_mapping(), old_entries="CONTACT_EXTRACTION_PREPLACE_UNCHANGED_NOT_MIGRATED",
        native_worker_started=False,native_request_count=0,ik_search_count=0,motion_search_count=0,
        geometry_generation_s=perf_counter()-began,cases=rows)
    result["source_file_sha256"]={f:sha256_file(ROOT/f) for f in (
        "src/unloading_sim/lifted_reception.py","tools/preview_m710_lifted_reception.py",
        "tools/lifted_reception/viewer.html","configs/experiments/m710_lifted_reception_preview.yaml")}
    (args.output/"geometry.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    data=dict(result=result,model=render_model)
    template=(ROOT/"tools/lifted_reception/viewer.html").read_text(encoding="utf-8")
    vendor=ROOT/"tools/search_replay/vendor"
    scripts=(vendor/"three.min.js").read_text()+"\n"+(vendor/"OrbitControls.js").read_text()
    html=template.replace("/*__VENDOR__*/",scripts).replace("/*__DATA__*/",
        json.dumps(data,ensure_ascii=False,separators=(",",":"),allow_nan=False).replace("</","<\\/"))
    (args.output/"preview.html").write_text(html,encoding="utf-8")
    (args.output/"identity.json").write_text(json.dumps(dict(source_commit=source,
        geometry_sha256=sha256_file(args.output/"geometry.json"),html_sha256=sha256_file(args.output/"preview.html")),indent=2))


if __name__ == "__main__":
    main()
