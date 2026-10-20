"""Display saved failed goals and paired clearance evidence; never run IK."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.moveit2_backend import box_message
from unloading_sim.workcell_layout import load_workcell_layout


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(bundle, member):
    with zipfile.ZipFile(bundle) as archive:
        html = archive.read(member).decode("utf-8")
    return json.JSONDecoder().raw_decode(html.split("const DATA=", 1)[1].lstrip())[0]


def same_geometry(actual, expected):
    """Check actual recorded solids, allowing only floating-point roundoff."""
    a, b = ({v["id"]: v for v in x} for x in (actual, expected))
    assert a.keys() == b.keys(), "WORLD_IDENTITIES_DIFFER"
    for key in a:
        assert a[key].keys() == b[key].keys(), key
        for field in a[key]:
            if field in ("size", "pose"):
                np.testing.assert_allclose(a[key][field], b[key][field], atol=1e-12, rtol=0,
                                           err_msg=f"{key}/{field}")
            else:
                assert a[key][field] == b[key][field], (key, field)


def export(output):
    prior = ROOT/"docs/experiments/m710_junction_hmin_20261020"
    evidence = ROOT/"docs/experiments/m710_transverse_scissor_20261020"
    files = [prior/"offline-viewers.zip", evidence/"offline-viewers.zip",
             evidence/"paired-j2-probe.json", evidence/"frozen-paired-j2-probe-026.json"]
    for folder in (prior, evidence):
        manifest = json.loads((folder/"artifact-integrity.json").read_text(encoding="utf-8"))
        for file in files:
            if file.parent == folder:
                assert digest(file) == manifest[file.name]["sha256"], file
    old = replay(files[0], "width060/candidates.html")
    new = replay(files[1], "candidates.html")
    probe300, probe260 = [json.loads(p.read_text(encoding="utf-8")) for p in files[2:]]
    assert len(probe300["rows"]) == len(probe260["rows"]) == 144
    assert old["model"]["identity"]["model_tool_fingerprint"] == new["model"]["identity"]["model_tool_fingerprint"]
    assert old["model"]["links"] == new["model"]["links"]
    assert old["model"]["meshes"] == new["model"]["meshes"]
    requests = {a["request"]["stage_id"]: a["request"] for a in old["raw"]["attempts"]}
    layouts = [load_workcell_layout(ROOT/"configs/workcells"/f"m710id70_unloading_layout_v1{suffix}.yaml")
               for suffix in ("_longitudinal_060", "_transverse_scissor_030", "_transverse_scissor_026")]
    assert [layouts[0].layout_fingerprint, layouts[1].layout_fingerprint] == [
        item["fingerprint"] for item in probe300["layouts"]]
    assert [layouts[0].layout_fingerprint, layouts[2].layout_fingerprint] == [
        item["fingerprint"] for item in probe260["layouts"]]
    robot = layouts[0].robot()
    worlds, scenes, cases = {}, {}, []
    for number, (a, b) in enumerate(zip(probe300["rows"], probe260["rows"])):
        assert (a["stage_id"], a["q_rad"], a["height_m"], a["baseline"]) == (
            b["stage_id"], b["q_rad"], b["height_m"], b["baseline"])
        if a["baseline"]["passed"]:
            continue
        req = requests[a["stage_id"]]
        assert req["q_goal"] == a["q_rad"]
        height = a["height_m"]
        ids = {body["id"] for body in req["world"] if body["category"] == "carton"}
        cartons = tuple(c for c in layouts[0].cartons() if c.name in ids)
        assert len(cartons) == 37
        height_key = format(height, ".12f")
        if height_key not in worlds:
            states = [coupled_conveyor_state(layout, height, cartons) for layout in layouts]
            worlds[height_key] = [[box_message(box) for box in state.boxes] for state in states]
            scenes[height_key] = [state.scene_fingerprint for state in states]
        same_geometry(worlds[height_key][0], req["world"])
        # The 260 mm display also matches an actual new native planning world.
        recorded_new = next(t["request"]["world"] for t in new["raw"]["attempts"] if
            abs(next(x for x in t["request"]["world"] if x["id"] == "conveyor_transverse")["pose"][2][3]
                +.06-height) < 1e-12)
        same_geometry(worlds[height_key][2], recorded_new)
        frames = robot.named_link_frames(np.asarray(a["q_rad"]))
        variants = [a["baseline"], a["narrowed"], b["narrowed"]]
        pin = [next(p for p in v["pairs"] if p["pair"][1] == "lift_transverse_left_pin_1")
               for v in variants]
        cases.append(dict(source_row=number, attempt=a["attempt"], stage_id=a["stage_id"],
            q_rad=a["q_rad"], height_m=height, world_key=height_key, variants=variants,
            previous_native_failure=a["previous_native_failure"],
            new_pin_contact_at_300=pin[0]["passed"] and pin[1]["collision"],
            frame_witnesses={name:frames[name].tolist() for name in ("J2_link", "flange")}))
    assert len(cases) == 40
    default = next(i for i, row in enumerate(cases) if row["new_pin_contact_at_300"])
    data = dict(schema="m710_j2_scissor_saved_failed_goals_v1", cases=cases,
        default_case=default, worlds=worlds, scene_fingerprints=scenes, model=old["model"],
        widths_mm=[460, 300, 260], outer_widths_mm=[560, 400, 360], required_gap_m=.005,
        source_files={str(p.relative_to(ROOT)):digest(p) for p in files},
        layout_fingerprints=[layout.layout_fingerprint for layout in layouts],
        recorded_goal_source="3f177b11d3c17f9efee9d4b4e32fee7f09583879",
        diagnostic_sources=["501ee61ddd2a63b779187b901e5e2eb580f894d3",
                            "99569e7b7e4eaa2f499b3c1fde9fcaa0a606ca23"],
        scope="STATIC_RECORDED_FAILED_GOALS_NOT_A_TRAJECTORY",
        distances="SAVED_COAL_J2_OFFICIAL_MESH_VS_PROJECT_CONCEPT_SOLIDS",
        contact_witness_points_recorded=False, physical_execution="NOT_RUN",
        execution_ready=False, continuous_swept_proof=False)
    vendor = ROOT/"tools/search_replay/vendor"
    scripts = (vendor/"three.min.js").read_text(encoding="utf-8")+"\n"+(
        vendor/"OrbitControls.js").read_text(encoding="utf-8")
    html = (ROOT/"tools/search_replay/j2_scissor_viewer.html").read_text(encoding="utf-8")
    html = html.replace("/*__VENDOR__*/", scripts).replace("/*__DATA__*/",
        json.dumps(data, separators=(",", ":"), allow_nan=False).replace("</", "<\\/"))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8", newline="\n")
    checks = dict(status="PASS_SAVED_Q_GEOMETRY_AND_SOURCE_IDENTITY", cases=len(cases),
        new_pin_contact_at_300=sum(c["new_pin_contact_at_300"] for c in cases),
        heights=sorted({c["height_m"] for c in cases}), variants=3,
        source_files=data["source_files"], geometry_tolerance_m=1e-12,
        fk_witnesses="INDEPENDENT_CPU_URDF_LINK_FRAMES_FOR_EVERY_CASE",
        export_sha256=digest(output), physical_execution="NOT_RUN", execution_ready=False)
    output.with_suffix(".checks.json").write_text(json.dumps(checks, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(checks))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    export(parser.parse_args().output)
