"""Render archived measured states at native 720P; never execute a new physics trial.

Reuse the verified historical adapter's scene construction (before World creation)
so the official robot, CAD, materials, layout and camera remain identical. The
archived adapter and its bundle are read-only. Its original integrity/asset gates
run unchanged. This separate presentation entry renders saved q/box poses through
USD transforms with a stopped timeline; it does not run the execution loop.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def assignment(node, name):
    return isinstance(node, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == name for t in node.targets)


def initialize_scene(options):
    source = options.project_root / "scripts/isaacsim_fanuc_replay.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    # Explicit structural anchors: fail closed if the adapter layout changes.
    candidates = [(i, n) for i, n in enumerate(tree.body) if isinstance(n, ast.Try)
                  and any(assignment(s, "physics_dt") for s in n.body)]
    assert len(candidates) == 1, "historical scene-builder boundary changed"
    outer_index, outer = candidates[0]
    stop = next(i for i, n in enumerate(outer.body) if assignment(n, "physics_dt"))
    before = tree.body[:outer_index]
    app_index = next(i for i, n in enumerate(before) if assignment(n, "simulation_app"))
    sys.path.insert(0, str(options.project_root / "src"))
    sys.argv = [str(source), "--project-root", str(options.project_root),
        "--bundle", str(options.bundle), "--output", str(options.output / "scene_setup"),
        "--usd-directory", str(options.output / "usd_cache"),
        "--reuse-usd-entrypoint", str(options.usd),
        "--reuse-usd-run-evidence", str(options.usd_evidence),
        "--reuse-usd-source-contract", str(options.usd_contract)]
    namespace = dict(__name__="__recorded_state_scene__", __file__=str(source))
    def execute(nodes):
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    execute(before[:app_index])
    # The historical contract is verified above. Only this renderer's output
    # dimensions change; no execution bundle or source fingerprint is rewritten.
    namespace["args"].width, namespace["args"].height = 1280, 720
    execute(before[app_index:] + outer.body[:stop])
    return namespace, sha(source)


def render(options):
    options.output.mkdir(parents=True, exist_ok=False)
    manifest = read(options.montage_manifest)
    clips = manifest["clips"]
    assert len(clips) == 5 and manifest["same_world_continuous_trial"] is False
    inputs = []
    for clip in clips:
        directory = Path(clip["source"]).parent
        delivery_root = directory.parent.parent
        delivery = read(delivery_root / "delivery_manifest.json")
        state_path = directory / "actual_frame_states.json"
        relative = state_path.relative_to(delivery_root).as_posix()
        entry = next(f for f in delivery["files"] if f["path"] == relative)
        assert sha(state_path) == entry["sha256"]
        states = read(state_path)
        assert len(states["states"]) == clip["frames"]
        assert states["capture_max_joint_delta_rad"] == 0
        assert states["capture_max_carton_delta_m"] == 0
        initial = read(directory / "initial.png.json")
        assert initial["world_session_id"] == clip["world_session_id"]
        bound_files={}
        for filename in ("actual_frame_states.json","result.json","execution_events.json","actual_remaining_state.json"):
            path=directory/filename
            record=next(f for f in delivery["files"] if f["path"]==path.relative_to(delivery_root).as_posix())
            assert sha(path)==record["sha256"]
            bound_files[str(path)]=record["sha256"]
        inputs.append(dict(clip=clip, directory=directory, states=states,
                           state_sha256=entry["sha256"], result=read(directory / "result.json"),bound_files=bound_files))
    ns, source_hash = initialize_scene(options)
    np, rep = ns["np"], ns["rep"]
    Gf, UsdGeom, UsdShade = ns["Gf"], ns["UsdGeom"], ns["UsdShade"]
    from pxr import Usd
    import omni.timeline
    import cv2
    from unloading_sim.robot import URDFRobot
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    # Settle each stopped-time pose/UV update before capturing, without physics.
    if options.visual_asset_config:
        rep.settings.carb_settings('/rtx/post/motionblur/enabled', False)
    stage = ns["stage"]
    # All writes belong to an in-memory session layer, never the archived USD.
    stage.SetEditTarget(stage.GetSessionLayer())
    visuals = None
    schedules = None
    if options.visual_asset_config:
        from m710_visual_assets import apply_visual_assets
        from m710_belt_visual import build_schedules, distance_at
        visuals = apply_visual_assets(ns, options.visual_asset_config, options.visual_asset_cache)
        schedules = build_schedules(inputs)
        (options.output / "asset_manifest.json").write_text(json.dumps(visuals.report, indent=2))
        (options.output / "belt_schedule.json").write_text(json.dumps([
            {k:v for k,v in s.items() if k != "result"} for s in schedules], indent=2))
    # Check source cadence before choosing the existing 5 -> 80 fps mapping.
    for data in inputs:
        times = np.asarray([s["time_s"] for s in data["states"]["states"]])
        assert np.allclose(np.diff(times), 0.2, atol=1e-8)
    robot = URDFRobot.from_urdf(ns["urdf_path"], active_joint_names=inputs[0]["states"]["joint_names"],
        tip_link=ns["grasp_body_path"].rsplit("/", 1)[-1],
        base_position=ns["base_position"], base_rpy=ns["base_rpy"], tool_length=0.0)
    tip_from_tcp = np.eye(4)
    tip_from_tcp[:3, 3] = ns["flange_offset"]
    tip_from_tcp[0, 3] += float(ns["metadata"]["tool_length_m"])
    link_prims = ns["imported_link_prims"]
    initial_frames = robot.named_link_frames(inputs[0]["states"]["states"][0]["q_rad"])
    assert set(link_prims).issubset(initial_frames), (set(link_prims), set(initial_frames))
    carton_prims = {p.GetName(): p for p in stage.Traverse()
                    if str(p.GetPath()).startswith("/Validation/Scene/carton_")
                    and p.GetParent().GetPath().pathString == "/Validation/Scene"}
    assert len(carton_prims) == 40
    def matrix_op(prim):
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.SetResetXformStack(True)
        return xf.AddTransformOp(opSuffix="recordedState")
    link_ops = {name: matrix_op(prim) for name, prim in link_prims.items()}
    carton_ops = {name: matrix_op(prim) for name, prim in carton_prims.items()}
    # Use the current compact bottom-left drawing implementation, not old HUD.
    hud_tree = ast.parse(options.hud_source.read_text(encoding="utf-8"))
    hud_funcs = [n for n in ast.walk(hud_tree) if isinstance(n, ast.FunctionDef)
                 and n.name in ("_load_hud_fonts", "_draw_runtime_hud")]
    assert len(hud_funcs) == 2
    exec(compile(ast.Module(body=hud_funcs, type_ignores=[]), str(options.hud_source), "exec"), ns)
    ns["hud_fonts"] = ns["_load_hud_fonts"](720)
    movie = options.output / ("top_row_five_cartons_assets_native720p_16x.mp4" if visuals else "five_successful_pick_place_native720p_16x.mp4")
    encoder = subprocess.Popen([str(options.ffmpeg), "-nostdin", "-v", "error", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", "1280x720", "-framerate", "80", "-i", "pipe:0",
        "-an", "-c:v", "libx264", "-preset", "fast", "-threads", "2", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(movie)], stdin=subprocess.PIPE)
    audit = dict(schema="m710_native_recorded_state_render_v1", native_resolution=[1280,720],
        speed=16, fps=80, video_pixels_read_from_old_recording=False,
        visual_asset_swap=bool(visuals), source_worlds=manifest["world_session_ids"], new_physics_execution=False, same_world_continuous_trial=False,
        visual_history_reset=bool(visuals), visual_settle_subframes=8 if visuals else 2,
        source_adapter_sha256=source_hash, hud_source_sha256=sha(options.hud_source),
        original_montage_manifest_sha256=sha(options.montage_manifest),
        clips=[], frames=0, max_fk_tcp_position_error_m=0.0,
        minimum_frame_mean=255.,minimum_frame_std=255.,maximum_missing_material_pixel_fraction=0.,
        max_carton_usd_position_error_m=0.0, max_link_usd_position_error_m=0.0,
        note="Measured-state visualization; geometry reconstructed from official URDF and recorded q. No new qualification.")
    started = time.monotonic()
    try:
        for clip_index, data in enumerate(inputs, 1):
            clip, saved = data["clip"], data["states"]
            assert saved["joint_names"] == robot.active_joint_names
            world_index = manifest["world_session_ids"].index(clip["world_session_id"]) + 1
            mask = data["result"]["gripper_commanded_active_mask"]
            for cup_index, active in enumerate(mask):
                prim = stage.GetPrimAtPath(f"{ns['grasp_body_path']}/FG42CupVisual_{cup_index:02d}")
                assert prim.IsValid()
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    ns["active_rubber_material"] if active else ns["rubber_material"])
            clip_start = audit["frames"]
            saved_phases=set()
            states = saved["states"]
            if options.preview:
                states = [states[len(states)//2]]
            for frame_index, state in enumerate(states):
                assert not timeline.is_playing(), "renderer must never advance physics"
                frames = robot.named_link_frames(state["q_rad"])
                actual_tcp = np.asarray(state["tcp_actual_world"])
                predicted = frames[robot.tip_link] @ tip_from_tcp
                error = float(np.linalg.norm(predicted[:3,3] - actual_tcp[:3,3]))
                assert error < 0.0005, f"recorded q/FK mismatch: {error}"
                audit["max_fk_tcp_position_error_m"] = max(audit["max_fk_tcp_position_error_m"], error)
                # Measured gripper-body pose is authoritative for the tool.
                frames[robot.tip_link] = actual_tcp @ np.linalg.inv(tip_from_tcp)
                for name, op in link_ops.items():
                    op.Set(Gf.Matrix4d(frames[name].T.tolist()))
                by_name = {c["name"]: c for c in state["cartons"]}
                assert set(by_name).issubset(carton_prims)
                for name, prim in carton_prims.items():
                    imageable = UsdGeom.Imageable(prim)
                    if name not in by_name:
                        imageable.MakeInvisible()
                        continue
                    imageable.MakeVisible()
                    carton = by_name[name]
                    matrix = np.eye(4)
                    matrix[:3,:3] = ns["_rotation_matrix_from_quaternion_wxyz"](
                        carton["quaternion_wxyz"]) @ np.diag(carton["size_m"])
                    matrix[:3,3] = carton["center_m"]
                    carton_ops[name].Set(Gf.Matrix4d(matrix.T.tolist()))
                cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                for name, carton in by_name.items():
                    got = np.array(cache.GetLocalToWorldTransform(carton_prims[name]).ExtractTranslation())
                    error = float(np.linalg.norm(got - carton["center_m"]))
                    assert error < 1e-8
                    audit["max_carton_usd_position_error_m"] = max(audit["max_carton_usd_position_error_m"],error)
                for name, prim in link_prims.items():
                    got = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation())
                    error = float(np.linalg.norm(got-frames[name][:3,3]))
                    assert error < 1e-8
                    audit["max_link_usd_position_error_m"] = max(audit["max_link_usd_position_error_m"],error)
                if visuals:
                    schedule = schedules[clip_index-1]
                    phase = {name: initial + distance_at(data["result"], name, state["time_s"])
                             for name, initial in schedule["initial_phase_m"].items()}
                    visuals.update(phase)
                if visuals:
                    from m710_visual_review import render_updated_state
                    render_updated_state(rep)
                else:
                    rep.orchestrator.step(rt_subframes=2, pause_timeline=True, delta_time=0.0, wait_for_render=True)
                rgb = np.asarray(ns["rgb_annotator"].get_data())[:,:,:3].astype(np.uint8)
                assert rgb.shape == (720,1280,3)
                mean,std=float(rgb.mean()),float(rgb.std())
                pink=float(np.mean((rgb[:,:,0]>220)&(rgb[:,:,2]>220)&(rgb[:,:,1]<80)))
                assert mean>10 and std>10 and pink<.005, (mean,std,pink)
                audit['minimum_frame_mean']=min(audit['minimum_frame_mean'],mean)
                audit['minimum_frame_std']=min(audit['minimum_frame_std'],std)
                audit['maximum_missing_material_pixel_fraction']=max(audit['maximum_missing_material_pixel_fraction'],pink)
                attached = bool(state["attached"])
                lines = [f"{clip['target']} | {state['stage']} | t={state['time_s']:.1f}s",
                    f"{'Attached' if attached else 'Open / released'} | cups {sum(mask) if attached else 0}/72",
                    f"16x | clip {clip_index}/5 | world {world_index}/3"]
                rgb = ns["_draw_runtime_hud"](rgb,lines,
                    "Recorded states | ideal reception / outfeed", (72,232,150,255))
                encoder.stdin.write(rgb.tobytes())
                if frame_index == 0 or (clip_index == 1 and frame_index == 200):
                    ns["Image"].fromarray(rgb).save(options.output / f"clip_{clip_index:02d}_{frame_index:04d}.png")
                if state['stage'] not in saved_phases:
                    ns['Image'].fromarray(rgb).save(options.output/f"key_{clip_index:02d}_{state['stage']}.png")
                    saved_phases.add(state['stage'])
                audit["frames"] += 1
                if audit["frames"] % 50 == 0:
                    (options.output / "progress.json").write_text(json.dumps(dict(
                        frames=audit["frames"], total=3113, clip=clip_index,
                        elapsed_seconds=time.monotonic()-started)))
            audit["clips"].append(dict(target=clip["target"], world_session_id=clip["world_session_id"],
                states_sha256=data["state_sha256"], source_states=str(data["directory"] / "actual_frame_states.json"),
                frames=audit["frames"]-clip_start, start_frame=clip_start))
        if visuals and options.preview:
            from m710_visual_review import render_review
            render_review(ns, visuals, inputs, schedules, options)
        encoder.stdin.close()
        assert encoder.wait() == 0
        assert audit["frames"] == (5 if options.preview else 3113)
        cap = cv2.VideoCapture(str(movie))
        assert (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                cap.get(cv2.CAP_PROP_FPS)) == (1280,720,80)
        decoded = 0
        while cap.read()[0]: decoded += 1
        cap.release()
        assert decoded == audit["frames"]
        audit.update(status="PASS", decoded_frames=decoded, duration_seconds=decoded/80,
            video=movie.name, video_sha256=sha(movie), video_bytes=movie.stat().st_size,
            preview_only=options.preview, wall_seconds=time.monotonic()-started)
        for data in inputs:
            assert all(sha(path)==expected for path,expected in data['bound_files'].items())
        audit['immutable_record_files']={p:h for data in inputs for p,h in data['bound_files'].items()}
        (options.output / "manifest.json").write_text(json.dumps(audit,indent=2))
        print("RECORDED_RENDER_COMPLETE="+json.dumps(audit),flush=True)
    except BaseException:
        import traceback
        (options.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if not encoder.stdin.closed: encoder.stdin.close()
        encoder.wait()
        ns["simulation_app"].close()


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("project-root", "bundle", "usd", "usd-evidence", "usd-contract",
                 "montage-manifest", "hud-source", "output", "ffmpeg"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--preview",action="store_true")
    parser.add_argument("--visual-asset-config",type=Path)
    parser.add_argument("--visual-asset-cache",type=Path)
    options=parser.parse_args()
    render(options)
