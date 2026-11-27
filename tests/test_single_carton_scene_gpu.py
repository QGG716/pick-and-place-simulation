"""GPU (MuJoCo + EGL) 3D environment view: display model, rendering, picking, labels (user feedback 2026-11-27)."""
import os
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.workbench.single_carton import scene_gpu as G


def layout():
    P = np.eye(4); P[:3, 3] = [-.27, -.22, .05]; Q = np.eye(4); Q[:3, 3] = [.6, 0, -.1]; R = np.eye(4); R[:3, 3] = [3, 3, 0]
    return dict(kept=[dict(id=G.HIGHLIGHT_PART, pose=P.tolist(), size=[.01, 1.5, 1.], role='FIXED', name='panel'),
                      dict(id='site_layout_layout_part_1', pose=Q.tolist(), size=[.2, .2, .2], role='FIXED')],
                pruned=[dict(id='site_layout_layout_part_2', pose=R.tolist(), size=[.1] * 3)], bounds=[[-1, -.8, -.15], [.8, .8, 1.3]],
                source_sha256='a4b6ed1298f7491b' + '0' * 48)   # panel 466 exists only in the first revision


def test_display_model_without_gpu():
    d = G.DisplayModel(layout(), None)
    assert not d.has_robot and d.ids(G.HIGHLIGHT_PART) and d.ids('site_layout_layout_part_2')
    g = d.ids('site_layout_layout_part_2')[0]
    assert d.model.geom_group[g] == G.GROUP_PRUNED and d.model.geom_rgba[g, 3] < .2
    P = np.eye(4); P[:3, 3] = [.3, 0, .2]
    d.set_task([('box', '实测纸箱', P, np.array([.1, .05, .05]), [1, .8, .4, .9])])
    t = d.geom_ids['task0']
    assert np.allclose(d.model.geom_pos[t], [.3, 0, .2]) and d.name_of(t) == ('box', '实测纸箱')
    assert d.model.geom_rgba[d.geom_ids['task1'], 3] == 0
    manager = SimpleNamespace(scene=dict(world=[dict(id='v', category='observed', pose=np.eye(4).tolist(), size=[.04] * 3),
        dict(id='h', category='neighbor_shadow', pose=np.eye(4).tolist(), convex_mesh=dict(vertices=[[0, 0, 0], [.1, 0, 0], [0, .2, 0], [0, 0, .3]])),
        dict(id='wall', category='workspace_boundary', pose=np.eye(4).tolist(), size=[1] * 3)]), boxes=[])
    objects = G.task_objects(manager)
    assert [o[0] for o in objects] == ['v', 'h'] and np.allclose(objects[1][3], [.05, .1, .15])


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Tk needs a display')
def test_gpu_view_renders_picks_and_projects():
    if not G.gpu_available(): pytest.skip('no EGL GPU context')
    import tkinter as tk
    root = tk.Tk(); root.geometry('640x480')
    try:
        v = G.SceneViewGPU(root, config={}, planner=SimpleNamespace(ready=False), model=None, manager=SimpleNamespace(),
                           backend=SimpleNamespace(joint=None))
        v.pack(fill='both', expand=True); root.update()
        v.display = G.DisplayModel(layout(), None); v.renderer = G.GpuRenderer(v.display.model); v.layout = layout()
        v.camera.lookat[:] = [-.27, -.22, .05]; v.camera.distance = 2.; v.camera.azimuth = 180; v.camera.elevation = 0
        root.update(); v._draw_now()
        w, h = v.canvas.winfo_width(), v.canvas.winfo_height()
        xy = v.project(np.array([-.27, -.22, .05]), w, h)[0]
        assert abs(xy[0] - w / 2) < 3 and abs(xy[1] - h / 2) < 3     # the look-at point is the image centre
        picked = v._pick(SimpleNamespace(x=w // 2, y=h // 2))
        assert picked is not None and picked[0] == G.HIGHLIGHT_PART
        v.offset.set('500'); v.apply_offset(draw=False)
        assert np.isclose(v.display.model.geom_pos[v.display.ids(G.HIGHLIGHT_PART)[0], 1], -.22 + .5)
        # Later revisions renumber part ids: the 466 display offset never moves a part there.
        v.offset.set('0'); v.apply_offset(draw=False); v.layout = dict(layout(), source_sha256='0b5be708' + '0' * 56)
        v.offset.set('500'); v.apply_offset(draw=False)
        assert np.isclose(v.display.model.geom_pos[v.display.ids(G.HIGHLIGHT_PART)[0], 1], -.22)
        assert v.photo is not None and v._frame_ms > 0
    finally:
        root.destroy()
