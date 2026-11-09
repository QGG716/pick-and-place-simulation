"""Use the existing complete ECO65 scene renderer and FK; never step physics."""
import json
from copy import deepcopy
from pathlib import Path
import numpy as np


def load_snapshot(snapshot_path, input_root=None):
    """A historical snapshot may use archived configs; runtime meshes still must match."""
    from unloading_sim.eco65.model import ROOT
    from unloading_sim.eco65.native_io import verify_snapshot, file_hash
    from unloading_sim.eco65.asset_paths import resolve_asset
    snapshot = verify_snapshot(json.loads(Path(snapshot_path).read_text(encoding="utf-8")),
                               root=Path(input_root) if input_root else ROOT)
    if input_root and Path(input_root).resolve() != ROOT.resolve():
        # Shared renderer reads current-repository geometry, never current config files.
        for name, expected in snapshot['asset_sha256'].items():
            if file_hash(resolve_asset(ROOT, name)) != expected:
                raise ValueError('RUNTIME_SCENE_ASSET_CHANGED:'+name)
    return snapshot


def display_scene(scene):
    """Exact box vertices for the older shared convex renderer; never a planning input."""
    from itertools import product
    from unloading_sim.moveit2_backend import digest
    displayed = deepcopy(scene)
    converted = []
    for row in displayed['native_world_objects']:
        if 'convex_mesh' in row:
            continue
        size = np.asarray(row['size'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid source primitive size')
        row['convex_mesh'] = {'vertices':(np.asarray(list(product([-1.,1.], repeat=3)))*size/2).tolist()}
        converted.append(row['id'])
    displayed['workbench_display_derivation'] = {'source_scene_fingerprint':scene['scene_fingerprint'],
        'exact_box_vertex_conversion':converted, 'purpose':'DISPLAY_ONLY_NOT_PLANNING'}
    displayed.pop('scene_fingerprint')
    displayed['scene_fingerprint'] = digest(displayed)
    return displayed


class SceneRenderer:
    def __init__(self, snapshot_path, input_root=None):
        from unloading_sim.eco65.native_review_world import NativeSceneWorld
        self.snapshot = load_snapshot(snapshot_path, input_root)
        self.world = NativeSceneWorld(self.snapshot["tool"], display_scene(self.snapshot["scene"]))

    def render(self, q, optical_frame=None, tool_frame=None, azimuth=-30):
        from unloading_sim.eco65.visuals import current
        self.world.set_state(np.asarray(q), "initial")
        # mj_forward in the shared renderer updates geometry only. No mj_step or robot IO.
        pixels = current(self.world, [-.05, 0, .45], distance=2.1, azimuth=azimuth,
                         elevation=-25, axes=optical_frame, overlay=True)
        if tool_frame is not None:
            w = self.world
            for i, color in enumerate(([1,0,0,1], [0,1,0,1], [0,.4,1,1])):
                g = w.renderer.scene.geoms[w.renderer.scene.ngeom]
                w.mj.mjv_initGeom(g, w.mj.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).flatten(), np.array(color))
                w.mj.mjv_connector(g, w.mj.mjtGeom.mjGEOM_ARROW, .002, tool_frame[:3,3], tool_frame[:3,3]+.05*tool_frame[:3,i])
                w.renderer.scene.ngeom += 1
            pixels = w.renderer.render().copy()
        return pixels

    def close(self):
        renderer = getattr(self.world, "renderer", None)
        if renderer is not None:
            renderer.close()
