"""Three sensor-only same-pose captures; no perception models or ROS execution."""
from copy import deepcopy
import json
from pathlib import Path


def add_inspection_light(stage, trailer):
    """Explicit inspection emitter aimed at the joint; existing lamps/exposure stay unchanged."""
    import math
    from pxr import Gf, UsdGeom, UsdLux

    # A narrow simulated inspection strip, not a claimed installed robot fixture.
    light = UsdLux.RectLight.Define(stage, '/PerceptionValidation/RoofRightWallInspectionLight')
    light.CreateWidthAttr(4.0)
    light.CreateHeightAttr(0.03)
    light.CreateNormalizeAttr(False)
    light.CreateIntensityAttr(5000.0)
    light.CreateExposureAttr(0.0)
    light.CreateColorAttr(Gf.Vec3f(1.0, 0.93, 0.82))
    light.CreateEnableColorTemperatureAttr(True)
    light.CreateColorTemperatureAttr(5000.0)
    api = UsdGeom.XformCommonAPI(light.GetPrim())
    api.SetTranslate(Gf.Vec3d(1.0, -float(trailer['inside_width_m']) / 2 + 0.35,
                              float(trailer['inside_height_m']) - 0.25))
    api.SetRotate(Gf.Vec3f(math.degrees(math.atan2(-0.35, -0.25)), 0, 0))


def capture_same_poses(args, stage, payload, handles, sensors, command, world,
                       render, capture, scene_record):
    import carb
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdShade
    from unloading_perception.finite_sequence import atomic_json, verify_capture
    from unloading_perception.isaac_validation import IsaacSceneManifest, canonical_digest
    from unloading_perception.video_demo import recording_frames

    _, frames = recording_frames(args.seam_check_recording)
    selected = [frames[i] for i in (0, len(frames) // 2, len(frames) - 1)]
    report = {'source_recording': str(args.seam_check_recording.resolve()),
              'planning_admissible': False, 'sam_run': False, 'frames': [],
              'inspection_lighting': bool(args.seam_inspection_light)}
    cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
    report['solids'] = {}
    for prim in stage.Traverse():
        if prim.GetName().endswith(('Roof', 'RightWall', 'LeftWall', 'Floor', 'RoofRim')):
            bounds = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            parents = []
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                parents.append({'path': str(ancestor.GetPath()),
                                'world_matrix_usd_rows': [list(r) for r in cache.GetLocalToWorldTransform(ancestor)]})
                ancestor = ancestor.GetParent()
            report['solids'][str(prim.GetPath())] = {
                'bounds_m': [list(bounds.GetMin()), list(bounds.GetMax())],
                'transforms': parents,
                'material': str(UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0].GetPath()),
                'visibility': str(UsdGeom.Imageable(prim).ComputeVisibility())}
    settings = carb.settings.get_settings()
    report['render_settings'] = {k: settings.get(k) for k in (
        '/rtx/rendermode', '/rtx/indirectDiffuse/enabled', '/rtx/ambientOcclusion/enabled',
        '/rtx/post/tonemap/op', '/rtx/post/tonemap/filmIso', '/rtx/post/tonemap/cameraShutter',
        '/rtx/post/tonemap/fNumber', '/rtx/post/histogram/enabled')}
    report['lights'] = {str(p.GetPath()): {a.GetName(): a.Get() for a in p.GetAttributes()
                      if a.GetName().startswith('inputs:')} for p in stage.Traverse()
                      if p.GetTypeName().endswith('Light')}
    # Usd values are converted to JSON below; original scene snapshot remains available.
    report['lights'] = json.loads(json.dumps(report['lights'], default=str))
    for row in selected:
        verify_capture(Path(row['capture']))
        original = json.loads((Path(row['capture']) / 'manifest.json').read_text(encoding='utf-8'))
        if (original['objects'] != payload['objects'] or original['robot'] != payload['robot']
                or original['layout'] != payload['layout']):
            raise ValueError('same-pose check requires identical objects, robot and effective layout')
        current = deepcopy(payload)
        current['cameras'] = original['cameras']
        camera_prims = []
        for camera in current['cameras']:
            prim = handles[camera['module_id']].get_output_prims()['prims'][0]
            if not hasattr(prim, 'GetPath'):
                prim = stage.GetPrimAtPath(str(prim))
            transform = np.asarray(camera['T_W_C']) @ np.diag([1., -1., -1., 1.])
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(transform.T.tolist()))
            render_cameras = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Camera)]
            if len(render_cameras) != 1:
                raise ValueError('expected one rendered camera below the Replicator handle')
            camera_prims.append((camera, render_cameras[0]))
        # Advance the new capture clock, never copy the old frame/time identity.
        world.step(render=False)
        for _ in range(32):
            render(command)
        actual_cameras = []
        camera_cache = UsdGeom.XformCache()
        for camera, prim in camera_prims:
            actual = np.asarray(camera_cache.GetLocalToWorldTransform(prim)).T @ np.diag([1., -1., -1., 1.])
            if not np.allclose(actual, camera['T_W_C'], rtol=0, atol=1e-10):
                raise ValueError('rendered camera pose differs from requested source pose')
            actual_cameras.append({'module_id': camera['module_id'], 'actual_T_W_C': actual.tolist(),
                                   'clipping_range_m': list(UsdGeom.Camera(prim).GetClippingRangeAttr().Get())})
        current['timing'].update(simulation_time=float(world.current_time),
                                 simulation_frame=int(world.current_time_step_index))
        current['provenance']['same_pose_source_manifest'] = original['manifest_fingerprint']
        current['dynamic_scene_fingerprint'] = canonical_digest({
            'objects': current['objects'], 'mechanisms': current['mechanisms'],
            'camera_calibration': tuple({k: c[k] for k in (
                'camera_id', 'frame_id', 'resolution', 'K', 'distortion_model', 'distortion',
                'T_W_C', 'near_clip_m', 'far_clip_m')} for c in current['cameras'])})
        current['world_fingerprint'] = canonical_digest({
            'layout_fingerprint': current['layout']['layout_fingerprint'],
            'dynamic_scene_fingerprint': current['dynamic_scene_fingerprint'], 'robot': current['robot']})
        current.pop('manifest_fingerprint')
        current['manifest_fingerprint'] = canonical_digest(current)
        manifest = IsaacSceneManifest.from_dict(current)
        folder = args.output / f"source-{row['frame_sequence']}"
        folder.mkdir(exist_ok=False)
        atomic_json(folder / 'manifest.json', current)
        for camera, (_, rgb, depth, instance) in zip(manifest.cameras, sensors):
            capture(folder / 'FULL_STACK_NOMINAL', 'FULL_STACK_NOMINAL', manifest,
                    camera, (rgb, depth, instance), scene_record)
        if not report['frames']:
            stage.Export(str(args.output / 'capture_stage.usda'))
        report['frames'].append({'source_frame': row['frame_sequence'], 'directory': folder.name,
                                 'new_capture_frame': current['timing']['simulation_frame'],
                                 'actual_cameras': actual_cameras})
        atomic_json(args.output / 'seam-check.json', report)
        print('SEAM_CHECK_CAPTURED', row['frame_sequence'], flush=True)
    atomic_json(args.output / 'run_status.json', {'status': 'SEAM_CHECK_COMPLETE',
                                                'planning_admissible': False, 'sam_run': False})
