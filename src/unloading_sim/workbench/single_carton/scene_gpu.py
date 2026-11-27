"""GPU 3D view of the planning environment (user feedback 2026-11-27: the Tk
canvas view "太卡顿了，没有三维查看器丝滑，且UI也不好看").

The same content as ``scene_view.SceneView3D`` (STEP static layer exactly as
planning loads it, pruned parts, workspace bounds, latest task scene, robot with
tool and mast at the live joints, the 11-26 furthest -Y marker), rendered by
MuJoCo's OpenGL renderer in an off-screen EGL context on the GPU and shown as an
image on a Tk canvas. A separate display model is built here: nothing in it is a
planning or collision object, and nothing here changes a scene, a plan, a
device or a valve. When no EGL GPU context can be created the Tk canvas view is
used instead (``make_scene_view``).
"""
from __future__ import annotations

import threading
from functools import lru_cache
import time
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

from .scene_view import (FIELD_MIN_Y_REACHED_M, HIGHLIGHT_PART, highlight_part, legacy_markers, ROBOT_LINKS, ROLE_COLOURS, SCENE_COLOURS, SceneView3D,
                         _origin, _panel, display_joints, load_display_layout)

GROUP_STEP, GROUP_PRUNED, GROUP_TASK, GROUP_ROBOT, GROUP_AUX = 0, 1, 2, 3, 4
TASK_POOL = 1024
OFFSCREEN = (2560, 1600)
FONT_FILES = ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
              '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc')
DEFAULT_CAMERA = dict(lookat=(-.15, -.2, .3), distance=2.4, azimuth=120., elevation=-24.)


def _rgba(colour, alpha=1.):
    return [int(colour[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [alpha]


def _quat(R):
    import mujoco
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel()); return q


@lru_cache(maxsize=8)
def _font(size):
    for path in FONT_FILES:
        try: return ImageFont.truetype(path, size)
        except Exception: continue
    return ImageFont.load_default()


@lru_cache(maxsize=1024)
def _sprite(text, size, colour):
    """Rendered text as an RGBA image (laying out CJK text every frame costs tens of ms)."""
    font = _font(size); x0, y0, x1, y1 = font.getbbox(text)
    im = Image.new('RGBA', (max(1, x1 + 2), max(1, y1 + 3)), colour + (0,))
    ImageDraw.Draw(im).text((0, 0), text, font=font, fill=colour + (255,)); return im


def _put(image, text, size, colour, xy, anchor='la'):
    sp = _sprite(text, size, tuple(colour)); x, y = xy
    if anchor[0] == 'm': x -= sp.width / 2
    elif anchor[0] == 'r': x -= sp.width
    if anchor[1] == 'm': y -= sp.height / 2
    elif anchor[1] == 'd': y -= sp.height
    image.paste(sp, (int(x), int(y)), sp); return sp.width


def _mesh_file(filename):
    import trimesh
    m = trimesh.load(unquote(urlparse(filename).path), force='mesh', process=False)
    return np.asarray(m.vertices, float), np.asarray(m.faces, int)


def robot_parts(urdf):
    """[(robot link, key, display name, vertices in that link's frame, faces, kind)] for the
    robot links (official visual meshes) and every tool/mast collision mesh."""
    u = ET.fromstring(urdf)
    fixed = {j.find('child').get('link'): (j.find('parent').get('link'), _origin(j.find('origin')))
             for j in u.findall('joint') if j.get('type') == 'fixed'}

    def chain(name):
        T = np.eye(4)
        while name not in ROBOT_LINKS:
            if name not in fixed: return None, None
            parent, J = fixed[name]; T = J @ T; name = parent
        return name, T
    out = []; cache = {}
    for link in u.findall('link'):
        name = link.get('name')
        elements = link.findall('visual') if name in ROBOT_LINKS else link.findall('collision')
        root, T = chain(name)
        if root is None: continue
        for k, e in enumerate(elements):
            mesh = e.find('geometry').find('mesh')
            if mesh is None: continue
            key = (mesh.get('filename'), mesh.get('scale', '1 1 1'))
            if key not in cache:
                v, f = _mesh_file(key[0]); cache[key] = (v * np.fromstring(key[1], sep=' '), f)
            v, f = cache[key]; M = T @ _origin(e.find('origin')); v = v @ M[:3, :3].T + M[:3, 3]
            if name in ROBOT_LINKS: kind, label = 'robot', '机器人 ' + name
            elif name.startswith(('eco65_mast', 'eco65_camera')): kind, label = 'mast', '相机支架 ' + name
            elif name.endswith('_lip'): kind, label = 'lip', '吸盘唇 ' + name
            else: kind, label = 'tool', '吸具 ' + name
            out.append((root, 'robot:' + name, label, v, f, kind))
    return out


PART_COLOURS = dict(robot=(.93, .94, .95, 1), tool=(.62, .68, .74, 1), lip=(.16, .17, .19, 1), mast=(.85, .70, .42, 1))


class DisplayModel:
    """MuJoCo display model: static STEP boxes, a pool of task-scene boxes, one mocap
    body per robot link carrying its meshes, auxiliary lines; no physics is ever run."""

    def __init__(self, layout, urdf):
        import mujoco
        self.mj = mujoco
        spec = mujoco.MjSpec(); spec.modelname = 'eco65_display'
        spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
        g = spec.visual.global_; g.offwidth, g.offheight = OFFSCREEN; g.fovy = 40
        spec.visual.quality.shadowsize = 4096; spec.visual.quality.offsamples = 8
        spec.visual.map.znear = .005; spec.visual.map.zfar = 30
        spec.visual.headlight.ambient = [.32, .34, .38]; spec.visual.headlight.diffuse = [.45, .45, .45]
        spec.visual.headlight.specular = [.1, .1, .1]
        spec.add_texture(name='sky', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                         rgb1=[.17, .23, .30], rgb2=[.04, .06, .09], width=256, height=256)
        spec.add_texture(name='grid', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                         rgb1=[.12, .15, .19], rgb2=[.10, .12, .16], mark=mujoco.mjtMark.mjMARK_EDGE, markrgb=[.22, .28, .35],
                         width=512, height=512)
        mat = spec.add_material(name='floor', texrepeat=[12, 12], reflectance=.08)
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'grid'
        world = spec.worldbody; self.keys = {}; self.names = {}
        for pos, direction in (((2, -2, 4), (-.5, .5, -1)), ((-2, 2, 4), (.4, -.3, -1))):
            world.add_light(pos=list(pos), dir=list(direction), type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, castshadow=pos[0] > 0,
                            diffuse=[.45, .45, .45], specular=[.15, .15, .15])

        def add(body, name, key, label, group, **kw):
            geom = body.add_geom(name=name, group=group, contype=0, conaffinity=0, **kw)
            self.keys.setdefault(key, []).append(name); self.names[name] = (key, label); return geom
        bottoms = []; hl = highlight_part(layout)
        for kept, objects in ((True, (layout or {}).get('kept', [])), (False, (layout or {}).get('pruned', []))):
            for o in objects:
                P = np.asarray(o['pose'], float); size = np.asarray(o['size'], float)
                if kept: bottoms.append(P[2, 3] - (abs(P[2, :3]) @ size) / 2)
                if not kept: rgba = [.35, .45, .55, .10]
                elif o['id'] == hl: rgba = _rgba('#ff9f1c', .62)
                else: rgba = _rgba(ROLE_COLOURS.get(o.get('role', 'FIXED'), '#8d99ae'), .30 if _panel(size) else 1.)
                label = (f"STEP {o['id'].replace('site_layout_layout_', '')} {o.get('name', '')} · {o.get('role', 'FIXED')}"
                         + ('' if kept else '（工作空间外，不进入规划）'))
                add(world, 'step:' + o['id'], o['id'], label, GROUP_STEP if kept else GROUP_PRUNED,
                    type=mujoco.mjtGeom.mjGEOM_BOX, size=list(np.maximum(size / 2, 1e-4)), pos=list(P[:3, 3]),
                    quat=list(_quat(P[:3, :3])), rgba=rgba)
        self.floor_z = min(bottoms) if bottoms else -.7
        world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, .05], pos=[0, 0, self.floor_z - .002],
                       material='floor', group=GROUP_AUX, contype=0, conaffinity=0)
        # Workspace bounds and the 11-26 furthest -Y reached, as thin lines.
        def line(name, a, b, rgba, radius=.004):
            world.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=list(a) + list(b), size=[radius, 0, 0],
                           rgba=rgba, group=GROUP_AUX, contype=0, conaffinity=0)
        if (layout or {}).get('bounds') is not None:
            lo, hi = np.asarray(layout['bounds'], float); corners = lambda i: [(lo, hi)[(i >> k) & 1][k] for k in range(3)]
            for i in range(8):
                for k in range(3):
                    if not (i >> k) & 1: line(f'bounds{i}_{k}', corners(i), corners(i | 1 << k), [.25, .62, .85, .55], .003)
        if legacy_markers(layout):
            y = FIELD_MIN_Y_REACHED_M; ring = [(-.8, y, -.1), (-.8, y, .9), (.3, y, .9), (.3, y, -.1)]
            for i in range(4): line(f'reach{i}', ring[i], ring[(i + 1) % 4], [1, .3, .43, .9])
        for k, rgba in enumerate(([.95, .3, .3, 1], [.35, .9, .5, 1], [.3, .55, 1, 1])):
            line(f'axis{k}', (0, 0, 0), np.eye(3)[k] * .25, rgba, .006)
        # Task scene pool (boxes only; a convex hull is shown by its bounding box).
        for i in range(TASK_POOL):
            add(world, f'task{i}', f'task{i}', '', GROUP_TASK, type=mujoco.mjtGeom.mjGEOM_BOX, size=[.001] * 3,
                pos=[0, 0, -50], rgba=[0, 0, 0, 0])
        # Robot: one mocap body per link.
        self.links = {}
        parts = robot_parts(urdf) if urdf else []
        for link in ROBOT_LINKS:
            self.links[link] = world.add_body(name='body:' + link, mocap=True, mass=1, inertia=[1e-3] * 3)
        for n, (link, key, label, v, f, kind) in enumerate(parts):
            if len(v) < 4 or len(f) < 1: continue
            mesh = spec.add_mesh(name=f'mesh{n}', uservert=list(v.ravel()), userface=list(f.ravel()))
            mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
            add(self.links[link], f'part{n}', key, label, GROUP_ROBOT, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'mesh{n}',
                rgba=list(PART_COLOURS[kind]))
        self.model = spec.compile(); self.data = mujoco.MjData(self.model)
        m = self.model
        self.geom_ids = {name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name) for name in self.names}
        self.base_rgba = m.geom_rgba.copy(); self.base_pos = m.geom_pos.copy()
        self.mocap = {link: m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'body:' + link)] for link in ROBOT_LINKS}
        self.task_names = {}
        self.has_robot = bool(parts)

    def ids(self, key):
        return [self.geom_ids[n] for n in self.keys.get(key, [])]

    def set_robot(self, frames):
        d = self.data
        for link, k in self.mocap.items():
            T = frames.get(link) if frames else None
            if T is None: d.mocap_pos[k] = [0, 0, -50]; continue
            d.mocap_pos[k] = T[:3, 3]; d.mocap_quat[k] = _quat(T[:3, :3])

    def set_task(self, objects):
        """objects: [(key, label, pose 4x4, half extents, rgba)]; the rest of the pool is hidden."""
        m = self.model; self.task_names = {}
        for i in range(TASK_POOL):
            g = self.geom_ids[f'task{i}']
            if i < len(objects):
                key, label, P, half, rgba = objects[i]
                m.geom_pos[g] = P[:3, 3]; m.geom_quat[g] = _quat(P[:3, :3]); m.geom_size[g] = np.maximum(half, 1e-4)
                m.geom_rgba[g] = rgba; self.task_names[g] = (key, label)
            else:
                m.geom_pos[g] = [0, 0, -50]; m.geom_rgba[g] = [0, 0, 0, 0]
        self.base_rgba[[self.geom_ids[f'task{i}'] for i in range(TASK_POOL)]] = \
            m.geom_rgba[[self.geom_ids[f'task{i}'] for i in range(TASK_POOL)]]

    def name_of(self, geom):
        if geom in self.task_names: return self.task_names[geom]
        name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, geom)
        return self.names.get(name)


def task_objects(manager):
    out = []
    scene = getattr(manager, 'scene', None) or {}
    for o in scene.get('world', []):
        cat = o.get('category', '')
        if cat in ('site_layout_cad', 'workspace_boundary'): continue
        rgba = _rgba(SCENE_COLOURS.get(cat, '#55606e'), .85)
        P = np.asarray(o['pose'], float)
        if 'size' in o:
            out.append((o['id'], f"任务场景 {cat} · {o['id']}", P, np.asarray(o['size'], float) / 2, rgba))
        elif 'convex_mesh' in o:
            v = np.asarray(o['convex_mesh']['vertices'], float) @ P[:3, :3].T + P[:3, 3]
            lo, hi = v.min(0), v.max(0); B = np.eye(4); B[:3, 3] = (lo + hi) / 2
            out.append((o['id'], f"任务场景 {cat} · {o['id']}（外接盒显示）", B, (hi - lo) / 2, rgba))
    for b in getattr(manager, 'boxes', []) or []:
        if 'pose' in b and 'size_m' in b:
            out.append((b.get('id', 'box'), f"实测纸箱包络 {b.get('detection_index', '')} · {b.get('id', '')}",
                        np.asarray(b['pose'], float), np.asarray(b['size_m'], float) / 2, _rgba('#ffd166', .9)))
    return out[:TASK_POOL]


class GpuRenderer:
    """Off-screen EGL context + MuJoCo render context, used only from the Tk thread."""

    def __init__(self, model):
        import mujoco
        from mujoco.egl import GLContext
        self.mj = mujoco; self.gl = GLContext(*OFFSCREEN); self.gl.make_current()
        self.ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.ctx)
        self.scene = mujoco.MjvScene(model, maxgeom=20000); self.option = mujoco.MjvOption()
        self.option.flags[mujoco.mjtVisFlag.mjVIS_LIGHT] = 0; self.option.geomgroup[GROUP_AUX] = 1
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1; self.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 1

    def render(self, model, data, camera, w, h):
        mj = self.mj; self.gl.make_current()
        w, h = min(w, OFFSCREEN[0]), min(h, OFFSCREEN[1])
        mj.mjv_updateScene(model, data, self.option, None, camera, mj.mjtCatBit.mjCAT_ALL, self.scene)
        rect = mj.MjrRect(0, 0, w, h); mj.mjr_render(rect, self.scene, self.ctx)
        rgb = np.empty((h, w, 3), np.uint8); mj.mjr_readPixels(rgb, None, rect, self.ctx)
        return np.flipud(rgb)

    def close(self):
        try:
            self.gl.make_current(); self.ctx.free(); self.gl.free()
        except Exception: pass


class SceneViewGPU(ttk.Frame):
    """Orbit (left drag), pan (right drag), zoom (wheel), reset (double click), hover names,
    Ctrl+click see-through. Same constructor and load/refresh/joints as SceneView3D."""

    def __init__(self, parent, *, config, planner, model, manager, backend, log=lambda *_: None, standalone=False):
        super().__init__(parent)
        import mujoco
        self.mj = mujoco
        self._args = dict(config=config, planner=planner, model=model, manager=manager, backend=backend, log=log)
        self.config, self.planner, self.model, self.manager, self.backend = config, planner, model, manager, backend
        self.layout = None; self.layout_note = '正在加载 STEP 静态层与机器人模型…'; self.robot_error = None
        self.display = None; self.renderer = None; self._loading = False; self._pending = None; self._drag = None
        self._sig = None; self._task_sig = None; self._hover_at = 0.; self.hovered = None; self.wire = set()
        self.photo = None; self._frame_ms = 0.
        self.camera = mujoco.MjvCamera(); self.reset_view(draw=False)
        bar = ttk.Frame(self); bar.pack(side='top', fill='x', pady=(0, 4))
        self.vis = {}
        for text, group, on in (('STEP 零件', GROUP_STEP, True), ('空间外零件', GROUP_PRUNED, False),
                                ('任务场景', GROUP_TASK, True), ('机器人', GROUP_ROBOT, True)):
            var = tk.BooleanVar(value=on); self.vis[group] = var
            ttk.Checkbutton(bar, text=text, variable=var, command=self.draw).pack(side='left', padx=(0, 10))
        ttk.Label(bar, text='466 ΔY mm').pack(side='left', padx=(8, 3))
        self.offset = tk.StringVar(value='0'); entry = ttk.Entry(bar, textvariable=self.offset, width=6)
        entry.pack(side='left'); entry.bind('<Return>', lambda _: self.apply_offset())
        if not standalone: ttk.Button(bar, text='放大窗口', command=self.open_window).pack(side='right')
        ttk.Button(bar, text='复位视角', command=self.reset_view).pack(side='right', padx=6)
        self.canvas = tk.Canvas(self, bg='#0b121a', highlightthickness=0, width=640, height=300)
        self.canvas.pack(side='top', fill='both', expand=True)
        self._image_item = self.canvas.create_image(0, 0, anchor='nw')
        c = self.canvas
        c.bind('<Configure>', lambda _: self.draw())
        c.bind('<ButtonPress-1>', lambda e: self._start(e, mujoco.mjtMouse.mjMOUSE_ROTATE_V))
        c.bind('<ButtonPress-3>', lambda e: self._start(e, mujoco.mjtMouse.mjMOUSE_MOVE_V))
        c.bind('<Shift-ButtonPress-1>', lambda e: self._start(e, mujoco.mjtMouse.mjMOUSE_MOVE_V))
        c.bind('<B1-Motion>', self._move); c.bind('<B3-Motion>', self._move)
        c.bind('<ButtonRelease-1>', self._end); c.bind('<ButtonRelease-3>', self._end)
        c.bind('<Button-4>', lambda e: self._wheel(1)); c.bind('<Button-5>', lambda e: self._wheel(-1))
        c.bind('<MouseWheel>', lambda e: self._wheel(1 if e.delta > 0 else -1))
        c.bind('<Double-Button-1>', lambda _: self.reset_view())
        c.bind('<Control-Button-1>', self._toggle_wire); c.bind('<Motion>', self._hover)
        c.bind('<Leave>', lambda _: self._set_hover(None))
        self.bind('<Destroy>', self._destroyed, add='+')

    # -- data ------------------------------------------------------------------
    def load(self):
        if self._loading or self.display is not None: return
        self._loading = True

        def run():
            layout, note = load_display_layout(self.config); urdf = None
            try:
                if not getattr(self.planner, 'ready', False): self.planner.build()
                urdf = self.planner.init['parameters']['robot_description']
            except Exception as e:
                self.robot_error = f'{type(e).__name__}: {e}'
            try:
                display = DisplayModel(layout, urdf)
            except Exception as e:
                display = None; note += f' · 三维模型构建失败 {type(e).__name__}: {e}'
            self.layout, self.layout_note = layout, note
            self._built = display
            self._loading = False
        threading.Thread(target=run, daemon=True, name='scene-gpu-load').start()
        self._poll_loaded()

    def _poll_loaded(self):
        # Tk calls stay on the Tk thread: poll for the background build instead of after() from it.
        try:
            if self._loading: self._root().after(100, self._poll_loaded)
            else: self._attach()
        except tk.TclError: pass

    def _attach(self):
        self.display = getattr(self, '_built', None)
        if self.display is not None:
            try:
                self.renderer = GpuRenderer(self.display.model); self.apply_offset(draw=False)
            except Exception as e:
                self.display = None; self.layout_note += f' · GPU 渲染不可用 {type(e).__name__}: {e}'
        self._sig = None; self.refresh()

    def joints(self):
        return display_joints(self.backend, self.manager, self.config)

    def apply_offset(self, draw=True):
        d = self.display
        if d is None: return
        try: dy = float(self.offset.get() or 0) / 1000.
        except ValueError: dy = 0.
        for g in (d.ids(HIGHLIGHT_PART) if highlight_part(self.layout) else ()):
            d.model.geom_pos[g] = d.base_pos[g] + [0, dy, 0]
        if draw: self.draw()

    def refresh(self):
        """Periodic tick: redraw only when the joints, the task scene or a model changed."""
        if self._drag is not None: return
        q, _ = self.joints()
        task = (id(getattr(self.manager, 'scene', None)), id(getattr(self.manager, 'boxes', None)))
        sig = (None if q is None else tuple(np.round(np.rad2deg(q), 2)), task, self.display is not None, self.robot_error)
        if sig != self._sig:
            self._sig = sig; self.draw()

    def _update_model(self):
        d = self.display; q, source = self.joints()
        frames = None
        if q is not None and self.model is not None:
            try: frames = self.model.frames(q); frames.setdefault('base_link', np.eye(4))
            except Exception as e: self.robot_error = f'{type(e).__name__}: {e}'
        d.set_robot(frames)
        task = (id(getattr(self.manager, 'scene', None)), id(getattr(self.manager, 'boxes', None)))
        if task != self._task_sig:
            self._task_sig = task; d.set_task(task_objects(self.manager))
        for key in self.wire:
            for g in d.ids(key) or [g for g, kv in d.task_names.items() if kv[0] == key]:
                d.model.geom_rgba[g, 3] = min(d.base_rgba[g, 3], .12)
        self.mj.mj_kinematics(d.model, d.data); self.mj.mj_camlight(d.model, d.data)
        return q, source

    # -- drawing ---------------------------------------------------------------
    def draw(self):
        """Coalesce redraw requests into one render at idle time."""
        if self._pending is None:
            try: self._pending = self._root().after_idle(self._draw_now)
            except tk.TclError: pass

    def _draw_now(self):
        self._pending = None
        try:
            if not self.canvas.winfo_exists(): return
        except tk.TclError: return
        w, h = max(self.canvas.winfo_width(), 64), max(self.canvas.winfo_height(), 64)
        t0 = time.perf_counter()
        if self.display is None or self.renderer is None:
            image = Image.new('RGB', (w, h), (11, 18, 26)); q, source = self.joints()
        else:
            q, source = self._update_model()
            for group, var in self.vis.items(): self.renderer.option.geomgroup[group] = int(var.get())
            image = Image.fromarray(self.renderer.render(self.display.model, self.display.data, self.camera, w, h))
            if image.size != (w, h): image = image.resize((w, h))
        self._overlay(image, q, source)
        if self.photo is None or (self.photo.width(), self.photo.height()) != (w, h):
            self.photo = ImageTk.PhotoImage(image); self.canvas.itemconfigure(self._image_item, image=self.photo)
        else:
            self.photo.paste(image)
        self._frame_ms = (time.perf_counter() - t0) * 1000

    def project(self, points, w, h):
        """Screen pixels of base_link points with the camera of the last render (None behind it)."""
        cam = self.renderer.scene.camera[0]
        # camera[0]/[1] are the stereo eyes; the mono image uses their mean position.
        pos = (np.asarray(cam.pos) + np.asarray(self.renderer.scene.camera[1].pos)) / 2; fwd = np.asarray(cam.forward); up = np.asarray(cam.up); right = np.cross(fwd, up)
        top, bottom, near = cam.frustum_top, cam.frustum_bottom, cam.frustum_near
        half_w = cam.frustum_width or (top - bottom) / 2 * w / h; centre = cam.frustum_center
        out = []
        for p in np.atleast_2d(points):
            v = p - pos; z = v @ fwd
            if z <= 1e-3: out.append(None); continue
            x, y = (v @ right) * near / z, (v @ up) * near / z
            out.append(((x - (centre - half_w)) / (2 * half_w) * w, (top - y) / (top - bottom) * h))
        return out

    @staticmethod
    def _panel(image, box, alpha=.68, colour=(8, 14, 22)):
        """Darken only the rectangle under a text box (a full-image alpha composite costs ~45 ms)."""
        w, h = image.size; x0, y0, x1, y1 = (int(max(0, box[0])), int(max(0, box[1])), int(min(w, box[2])), int(min(h, box[3])))
        if x1 <= x0 or y1 <= y0: return
        region = image.crop((x0, y0, x1, y1))
        image.paste(Image.blend(region, Image.new('RGB', region.size, colour), alpha), (x0, y0))

    def _overlay(self, image, q, source):
        w, h = image.size; dr = ImageDraw.Draw(image)
        if self.display is not None and self.renderer is not None:
            labels = []
            if self.vis[GROUP_STEP].get():
                try: dy = float(self.offset.get() or 0)
                except ValueError: dy = 0.
                for g in (self.display.ids(HIGHLIGHT_PART) if highlight_part(self.layout) else ()):
                    labels.append((self.display.data.geom_xpos[g], '466 侧边1 外部挡板框架' + (f'（显示 ΔY {dy:+.0f} mm）' if dy else '（CAD 位置）'), (255, 196, 120)))
            if legacy_markers(self.layout): labels.append(((-.8, FIELD_MIN_Y_REACHED_M, .9), f'11-26 真机到达的最 −Y  y = {FIELD_MIN_Y_REACHED_M:.3f} m', (255, 110, 140)))
            for p, text, colour in labels:
                xy = self.project(np.asarray(p, float), w, h)[0]
                if xy is None: continue
                sp = _sprite(text, 15, colour)
                self._panel(image, (xy[0] - sp.width / 2 - 7, xy[1] - 13, xy[0] + sp.width / 2 + 7, xy[1] + 13), .6)
                _put(image, text, 15, colour, xy, 'mm')
        lines = [self.layout_note]
        joints = f"  [{', '.join(f'{x:.1f}' for x in np.rad2deg(q))}]°" if q is not None else ''
        robot = ('机器人模型不可用 ' + self.robot_error) if self.robot_error else (
            '' if self.display is None or self.display.has_robot else '机器人模型未加载')
        lines.append(f'机器人：{source}{joints}' + (f' · {robot}' if robot else ''))
        if self.layout:
            lines.append(f"STEP {len(self.layout['kept'])} 件进入规划 · 工作空间外 {len(self.layout['pruned'])} 件 · 蓝线 = 工作空间边界")
        if self.hovered: lines.append('▶ ' + self.hovered)
        lines = [x for t in lines for x in str(t).split('\n')]
        colours = [(255, 214, 140) if t.startswith('▶') else (226, 236, 244) for t in lines]
        pad = 8; widths = [_sprite(t, 13, c).width for t, c in zip(lines, colours)]
        self._panel(image, (8, 8, 8 + 2 * pad + max(widths), 8 + 2 * pad + 20 * len(lines) - 2))
        for i, (t, c) in enumerate(zip(lines, colours)): _put(image, t, 13, c, (8 + pad, 8 + pad + 20 * i))
        _put(image, '左键旋转 · 右键/Shift+左键平移 · 滚轮缩放 · 双击复位 · Ctrl+点击 零件半透明 · 仅显示，不改场景/计划',
             12, (160, 180, 198), (w - 10, h - 34), 'rd')
        legend = [('STEP 零件', '#8d99ae')] + ([('466', '#ff9f1c')] if highlight_part(self.layout) else []) + [('机器人', '#eef0f2'), ('吸具', '#9eadbd'), ('相机支架', '#d9b26b'),
                  ('RGB-D 障碍', SCENE_COLOURS['observed']), ('实测纸箱', '#ffd166')]
        x = 10
        for text, colour in legend:
            dr.rectangle((x, h - 26, x + 12, h - 14), fill=tuple(int(colour[i:i + 2], 16) for i in (1, 3, 5)))
            x += 26 + _put(image, text, 12, (210, 222, 232), (x + 16, h - 20), 'lm')

    # -- interaction -----------------------------------------------------------
    def reset_view(self, draw=True):
        c = self.camera; c.type = self.mj.mjtCamera.mjCAMERA_FREE
        c.lookat[:] = DEFAULT_CAMERA['lookat']; c.distance = DEFAULT_CAMERA['distance']
        c.azimuth = DEFAULT_CAMERA['azimuth']; c.elevation = DEFAULT_CAMERA['elevation']
        if draw: self.draw()

    def _start(self, e, action): self._drag = (action, e.x, e.y)

    def _end(self, _): self._drag = None

    def _move(self, e):
        if self._drag is None or self.display is None: return
        action, x0, y0 = self._drag; h = max(self.canvas.winfo_height(), 1)
        self.mj.mjv_moveCamera(self.display.model, action, (e.x - x0) / h, (e.y - y0) / h, self.camera)
        self._drag = (action, e.x, e.y); self.draw()

    def _wheel(self, direction):
        if self.display is None: return
        self.mj.mjv_moveCamera(self.display.model, self.mj.mjtMouse.mjMOUSE_ZOOM, 0, -.06 * direction, self.camera)
        self.draw()

    def _pick(self, e):
        if self.display is None or self.renderer is None: return None
        w, h = max(self.canvas.winfo_width(), 1), max(self.canvas.winfo_height(), 1)
        sel = np.zeros(3); geom = np.array([[-1]], np.int32); flex = np.array([[-1]], np.int32); skin = np.array([[-1]], np.int32)
        self.mj.mjv_select(self.display.model, self.display.data, self.renderer.option, w / h, e.x / w, 1 - e.y / h,
                           self.renderer.scene, sel, geom, flex, skin)
        return self.display.name_of(int(geom[0, 0])) if geom[0, 0] >= 0 else None

    def _hover(self, e):
        if self._drag is not None or time.monotonic() - self._hover_at < .06: return
        self._hover_at = time.monotonic(); picked = self._pick(e)
        self._set_hover(None if picked is None else picked[1] + ('（半透明）' if picked[0] in self.wire else ''))

    def _set_hover(self, text):
        if text != self.hovered: self.hovered = text; self.draw()

    def _toggle_wire(self, e):
        picked = self._pick(e)
        if picked is None: return
        key = picked[0]
        if key in self.wire:
            self.wire.discard(key)
            d = self.display
            for g in d.ids(key) or [g for g, kv in d.task_names.items() if kv[0] == key]: d.model.geom_rgba[g] = d.base_rgba[g]
        else:
            self.wire.add(key)
        self._drag = None; self.draw()

    def open_window(self):
        top = tk.Toplevel(self); top.title('三维仿真环境（只读）'); top.geometry('1400x900')
        view = SceneViewGPU(top, standalone=True, **self._args); view.pack(fill='both', expand=True, padx=6, pady=6)
        view.offset.set(self.offset.get()); view.wire = set(self.wire)
        for attr in ('lookat',): getattr(view.camera, attr)[:] = getattr(self.camera, attr)
        view.camera.distance, view.camera.azimuth, view.camera.elevation = self.camera.distance, self.camera.azimuth, self.camera.elevation
        view.load()

        def tick():
            try:
                if not top.winfo_exists(): return
                view.refresh(); top.after(300, tick)
            except tk.TclError: pass
        tick()

    def _destroyed(self, e):
        if e.widget is self and self.renderer is not None:
            self.renderer.close(); self.renderer = None


_GPU_OK = None


def gpu_available():
    """True when an EGL context on the GPU can be created in this process (checked once)."""
    global _GPU_OK
    if _GPU_OK is None:
        try:
            import mujoco
            from mujoco.egl import GLContext
            ctx = GLContext(64, 64); ctx.make_current()
            from OpenGL import GL
            vendor = GL.glGetString(GL.GL_RENDERER) or b''
            ctx.free(); _GPU_OK = b'llvmpipe' not in vendor.lower()
        except Exception:
            _GPU_OK = False
    return _GPU_OK


def make_scene_view(parent, **kwargs):
    """GPU view when available, else the Tk canvas view (same interface)."""
    return SceneViewGPU(parent, **kwargs) if gpu_available() else SceneView3D(parent, **kwargs)
