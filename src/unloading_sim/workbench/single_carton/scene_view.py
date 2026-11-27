"""Read-only 3D view of the planning environment on a Tk canvas (user request 2026-11-27:
"把你已经建好的仿真环境，在单箱视觉任务标签页里显示，要求能在3维空间内拖动着看不同的视角").

Shows, in base_link metres: the site STEP static layer exactly as planning loads it
(site_layout.load_layout, the process cache), parts pruned outside the workspace as
wireframes, the workspace bounds, the latest task scene (RGB-D obstacles, supports,
measured cartons) and the robot with tool and mast at the live joints. Display only:
nothing here changes a scene, a plan, a device or a valve. The optional display
offset of a part never enters planning.
"""
from __future__ import annotations

import json
import threading
import tkinter as tk
from tkinter import ttk
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np
from functools import lru_cache
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[4]
ROBOT_LINKS = ('base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6')
# Display fallback when the running configuration has no site_layout: the user's
# 2026-11-27 decisions (lift 300 mm, CAD design transform). Never used for planning.
# Layout revision "layout 20261127.stp" (user 2026-11-27: robot moved from left
# to right, physically moved after 2026-11-26).
DISPLAY_FALLBACK_LAYOUT = dict(
    manifest='assets/layout/derived/0b5be70827083977/site-layout.json',
    roles='configs/workcells/eco65_site_layout_roles_20261127.yaml', transform_source='CAD_DESIGN_USER_CONFIRMED',
    transform_basis='DISPLAY ONLY: user decision 2026-11-27 CAD design transform', excluded_part_ids=[],
    exclusion_basis='DISPLAY ONLY', lift=dict(h_m=-0.046, h_basis='DISPLAY ONLY: user decision 2026-11-27 lift 300 mm'))
# Furthest -Y reached by the robot/tool solids (AABBs) over 2446 executed field
# configurations of 2026-11-26 (evidence docs/evidence/20261127_step_static_layer).
FIELD_MIN_Y_REACHED_M = -0.711
HIGHLIGHT_PART = 'site_layout_layout_part_466'
# Panel 466 and the 11-26 reach marker describe the first STEP revision only;
# the robot was moved after 2026-11-26, so they are not drawn for later revisions.
LEGACY_LAYOUT_SHA_PREFIX = 'a4b6ed1298f7491b'


def legacy_markers(layout):
    return bool(layout) and str(layout.get('source_sha256') or '').startswith(LEGACY_LAYOUT_SHA_PREFIX)


def highlight_part(layout):
    """Id of panel 466 in the first revision; part ids are renumbered per revision."""
    return HIGHLIGHT_PART if legacy_markers(layout) else None

ROLE_COLOURS = dict(FIXED='#8d99ae', LIFT_TOP='#6c9a8b', BELT_INCLINED='#5b8fb9', BELT_SLIDER='#b08968',
                    LIFT_INTERIOR='#7f7f7f')
SCENE_COLOURS = dict(observed='#4f6d7a', confirmed_stack_support='#8a6d3b', observed_support='#8a6d3b',
                     neighbor_shadow='#5c4b6b', unresolved_carton_volume='#7a4b4b', adjacent_contact='#6b5b3b')


def load_display_layout(config):
    """(layout dict or None, note): the STEP layer exactly as planning loads it
    (process cache), plus the parts pruned outside the workspace, for display."""
    try:
        from .site_layout import load_layout
        spec = config.get('site_layout')
        cfg = dict(config) if spec is not None else dict(config, site_layout=DISPLAY_FALLBACK_LAYOUT)
        site = cfg.get('site') if isinstance(cfg.get('site'), dict) else {}
        full = load_layout(dict(cfg, site={}), ROOT)
        kept = load_layout(cfg, ROOT) if site.get('workspace_bounds_m') is not None else full
        kept_ids = {o['id'] for o in kept['world']}
        layout = dict(kept=kept['world'], pruned=[o for o in full['world'] if o['id'] not in kept_ids],
                      bounds=site.get('workspace_bounds_m'), source_sha256=kept['identity']['source_sha256'])
        return layout, ('STEP 静态层：与规划相同（当前配置）' if spec is not None else
                        'STEP 静态层：当前配置未启用，按 11-27 布局（layout 20261127，h 300 mm、CAD 设计变换）仅显示，不进入规划')
    except Exception as e:
        return None, f'STEP 静态层不可用：{type(e).__name__}: {e}'


def display_joints(backend, manager, config):
    """Live joints, else this task's q_start, else the configured observation start."""
    joint = getattr(backend, 'joint', None)
    if joint is not None and getattr(joint, 'q_rad', None) is not None: return np.asarray(joint.q_rad, float), '实测关节'
    obs = getattr(manager, 'observation', None)
    if obs is not None and getattr(obs, 'q_start', None) is not None: return np.asarray(obs.q_start, float), '本次 q_start'
    start = (config.get('observation_start') or {}).get('q_deg')
    if start: return np.deg2rad(start), '配置的观测起点'
    return None, '无关节数据'


def _origin(element):
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    if element is not None:
        T[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return T


def box_solid(pose, size):
    """(vertices 8x3, faces as index lists with outward order) of an oriented box."""
    P = np.asarray(pose, float); h = np.asarray(size, float) / 2
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    v = (signs * h) @ P[:3, :3].T + P[:3, 3]
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return v, faces


def hull_solid(points):
    points = np.asarray(points, float)
    try:
        hull = ConvexHull(points)
    except Exception:
        lo, hi = points.min(0), points.max(0); P = np.eye(4); P[:3, 3] = (lo + hi) / 2
        return (*box_solid(P, np.maximum(hi - lo, 1e-4)), None)
    return points, [list(s) for s in hull.simplices], hull.equations[:, :3]


class RobotGeometry:
    """Collision geometry of the planner URDF: robot links as convex hulls,
    tool/mast solids as their local boxes (enough to judge clearances by eye)."""

    def __init__(self, urdf, model):
        self.model = model; u = ET.fromstring(urdf); cache = {}
        self.fixed = {j.find('child').get('link'): (j.find('parent').get('link'), _origin(j.find('origin')))
                      for j in u.findall('joint') if j.get('type') == 'fixed'}
        self.links = {}; self.parts = []
        for link in u.findall('link'):
            name = link.get('name'); merged = []
            for c in link.findall('collision'):
                g = c.find('geometry'); box = g.find('box'); mesh = g.find('mesh'); T = _origin(c.find('origin'))
                if box is not None:
                    size = np.fromstring(box.get('size'), sep=' '); v, _ = box_solid(np.eye(4), size)
                elif mesh is not None:
                    key = (mesh.get('filename'), mesh.get('scale', '1 1 1'))
                    if key not in cache:
                        import trimesh
                        solid = trimesh.load(unquote(urlparse(key[0]).path), force='mesh', process=False)
                        verts = np.asarray(solid.vertices) * np.fromstring(key[1], sep=' ')
                        try: verts = verts[ConvexHull(verts).vertices]
                        except Exception: pass
                        cache[key] = verts
                    v = cache[key]
                else:
                    continue
                v = v @ T[:3, :3].T + T[:3, 3]
                if name in ROBOT_LINKS: merged.append(v)
                else:
                    lo, hi = v.min(0), v.max(0); P = np.eye(4); P[:3, 3] = (lo + hi) / 2
                    self.parts.append((name, P, np.maximum(hi - lo, 1e-4)))
            if merged:
                pts = np.vstack(merged)
                try: pts = pts[ConvexHull(pts).vertices]
                except Exception: pass
                self.links[name] = pts

    def solids(self, q):
        frames = {n: T for n, T in self.model.frames(q).items()}
        frames.setdefault('base_link', np.eye(4))

        def frame(name):
            if name not in frames:
                if name not in self.fixed: return None
                parent, T = self.fixed[name]; F = frame(parent)
                if F is None: return None
                frames[name] = F @ T
            return frames[name]
        out = []
        for name, pts in self.links.items():
            F = frame(name)
            if F is None: continue
            out.append(dict(kind='hull', points=pts @ F[:3, :3].T + F[:3, 3], colour='#e07a5f', label=None,
                            key='robot:' + name, name='机器人 ' + name))
        for name, P, size in self.parts:
            F = frame(name)
            if F is None: continue
            colour = '#f2cc8f' if name.startswith(('eco65_mast', 'eco65_camera')) else '#81b29a'
            out.append(dict(kind='box', pose=F @ P, size=size, colour=colour, label=None, key='robot:' + name,
                            name=('相机支架 ' if colour == '#f2cc8f' else '吸具 ') + name))
        return out


def _panel(size):
    """Thin large plates (side panels, top cover) are drawn see-through: they enclose the robot."""
    a = np.sort(np.asarray(size, float))
    return a[0] < .02 and a[1] * a[2] > .3


@lru_cache(maxsize=8192)
def _shade_q(colour, q):
    r, g, b = (int(colour[i:i + 2], 16) for i in (1, 3, 5)); f = q / 50.
    return '#%02x%02x%02x' % tuple(min(255, max(0, int(c * f))) for c in (r, g, b))


def _shade(colour, factor):
    return _shade_q(colour, int(round(min(1.2, max(0., float(factor))) * 50)))


class SceneView3D(ttk.Frame):
    """Canvas with orbit (left drag), pan (right drag), zoom (wheel), reset (double click)."""

    def __init__(self, parent, *, config, planner, model, manager, backend, log=lambda *_: None, standalone=False):
        super().__init__(parent)
        self._args = dict(config=config, planner=planner, model=model, manager=manager, backend=backend, log=log)
        self.config, self.planner, self.model, self.manager, self.backend, self.log = config, planner, model, manager, backend, log
        self.az, self.el, self.zoom, self.pan = np.deg2rad(-60.), np.deg2rad(25.), 1.6, np.zeros(2)
        self.layout = None; self.layout_note = '正在加载 STEP 静态层…'; self.robot = None; self.robot_error = None
        self._drag = None; self._loading = False; self.wire = set(); self._names = {}; self._sig = None
        bar = ttk.Frame(self); bar.pack(side='top', fill='x')
        self.show_step = tk.BooleanVar(value=True); self.show_pruned = tk.BooleanVar(value=True)
        self.show_task = tk.BooleanVar(value=True); self.show_robot = tk.BooleanVar(value=True)
        for text, var in (('STEP 零件', self.show_step), ('空间外零件', self.show_pruned),
                          ('最近任务场景', self.show_task), ('机器人', self.show_robot)):
            ttk.Checkbutton(bar, text=text, variable=var, command=self.render).pack(side='left', padx=4)
        ttk.Label(bar, text='466 ΔY mm（仅显示）').pack(side='left', padx=(12, 2))
        self.offset = tk.StringVar(value='0'); entry = ttk.Entry(bar, textvariable=self.offset, width=7)
        entry.pack(side='left'); entry.bind('<Return>', lambda _: self.render())
        ttk.Button(bar, text='复位', command=self.reset_view).pack(side='left', padx=6)
        if not standalone: ttk.Button(bar, text='放大窗口', command=self.open_window).pack(side='left', padx=6)
        self.hover = tk.StringVar(value='悬停查看零件名 · Ctrl+点击：该零件实体/线框切换')
        ttk.Label(self, textvariable=self.hover, width=1).pack(side='bottom', fill='x')
        self.canvas = tk.Canvas(self, bg='#142333', highlightthickness=0, width=650, height=220)
        self.canvas.pack(side='top', fill='both', expand=True)
        c = self.canvas
        c.bind('<Configure>', lambda _: self.render())
        c.bind('<ButtonPress-1>', lambda e: self._start(e, 'orbit')); c.bind('<B1-Motion>', self._move)
        c.bind('<ButtonPress-3>', lambda e: self._start(e, 'pan')); c.bind('<B3-Motion>', self._move)
        c.bind('<ButtonRelease-1>', self._end); c.bind('<ButtonRelease-3>', self._end)
        c.bind('<Button-4>', lambda e: self._wheel(1)); c.bind('<Button-5>', lambda e: self._wheel(-1))
        c.bind('<MouseWheel>', lambda e: self._wheel(1 if e.delta > 0 else -1))
        c.bind('<Double-Button-1>', lambda _: self.reset_view())
        c.bind('<Control-Button-1>', self._toggle_wire); c.bind('<Motion>', self._hover)

    def open_window(self):
        """The same read-only view in its own resizable window (refreshes once per second)."""
        top = tk.Toplevel(self); top.title('三维仿真环境（只读）'); top.geometry('1300x850')
        view = SceneView3D(top, standalone=True, **self._args); view.pack(fill='both', expand=True)
        view.az, view.el, view.zoom, view.wire = self.az, self.el, self.zoom, set(self.wire)
        view.offset.set(self.offset.get()); view.load()

        def tick():
            try:
                if not top.winfo_exists(): return
                view.refresh(); top.after(1000, tick)
            except tk.TclError: pass
        tick()

    def refresh(self):
        """Periodic tick: re-render only when something drawn changed (joints, task scene, models)."""
        if self._drag is not None: return
        q, _ = self.joints()
        sig = (None if q is None else tuple(np.round(np.rad2deg(q), 2)), id(getattr(self.manager, 'scene', None)),
               id(getattr(self.manager, 'boxes', None)), self.layout is not None, self.robot is not None, self.robot_error)
        if sig != self._sig:
            self._sig = sig; self.render()

    # -- data ---------------------------------------------------------------
    def load(self):
        """Load the STEP layer (process cache) and robot geometry in the background."""
        if self._loading: return
        self._loading = True

        def run():
            self.layout, self.layout_note = load_display_layout(self.config)
            try:
                if not getattr(self.planner, 'ready', False): self.planner.build()
                self.robot = RobotGeometry(self.planner.init['parameters']['robot_description'], self.model)
            except Exception as e:
                self.robot_error = f'{type(e).__name__}: {e}'
            self._loading = False
            try: self.after(0, self.render)
            except Exception: pass
        threading.Thread(target=run, daemon=True, name='scene-view-load').start()

    def joints(self):
        return display_joints(self.backend, self.manager, self.config)

    def items(self):
        """Solids: (kind, data, colour, filled, label)."""
        out = []
        try: dy = float(self.offset.get() or 0) / 1000.
        except ValueError: dy = 0.
        if self.layout:
            hl = highlight_part(self.layout)
            if self.show_step.get():
                for o in self.layout['kept']:
                    pose = np.asarray(o['pose'], float).copy(); label = None; colour = ROLE_COLOURS.get(o.get('role', 'FIXED'), '#8d99ae')
                    if o['id'] == hl:
                        pose[1, 3] += dy; colour = '#ff9f1c'
                        label = '466 侧边1 外部挡板框架' + ('（CAD 位置）' if dy == 0 else f'（显示位移 Y {dy * 1000:+.0f} mm）')
                    out.append(dict(kind='box', pose=pose, size=o['size'], colour=colour, label=label, filled=True, key=o['id'],
                                    stipple='gray50' if o['id'] == hl else ('gray25' if _panel(o['size']) else ''),
                                    name=f"STEP {o['id'].replace('site_layout_layout_', '')} {o.get('name', '')} · {o.get('role', 'FIXED')}"))
            if self.show_pruned.get():
                for o in self.layout['pruned']:
                    out.append(dict(kind='box', pose=o['pose'], size=o['size'], colour='#3d5266', label=None, filled=False))
            if self.layout.get('bounds') is not None:
                lo, hi = np.asarray(self.layout['bounds'], float); P = np.eye(4); P[:3, 3] = (lo + hi) / 2
                out.append(dict(kind='box', pose=P, size=hi - lo, colour='#2e86ab', label=None, filled=False, dash=True))
        if self.show_task.get():
            scene = getattr(self.manager, 'scene', None) or {}
            for o in scene.get('world', []):
                cat = o.get('category', '')
                if cat in ('site_layout_cad', 'workspace_boundary'): continue
                colour = SCENE_COLOURS.get(cat, '#55606e')
                if 'size' in o: out.append(dict(kind='box', pose=o['pose'], size=o['size'], colour=colour, label=None, filled=True,
                                                key=o['id'], name=f"任务场景 {cat} · {o['id']}"))
                elif 'convex_mesh' in o:
                    v = np.asarray(o['convex_mesh']['vertices'], float); P = np.asarray(o['pose'], float)
                    out.append(dict(kind='hull', points=v @ P[:3, :3].T + P[:3, 3], colour=colour, label=None, filled=True,
                                    key=o['id'], name=f"任务场景 {cat} · {o['id']}"))
            for b in getattr(self.manager, 'boxes', []) or []:
                if 'pose' in b and 'size_m' in b:
                    out.append(dict(kind='box', pose=b['pose'], size=b['size_m'], colour='#ffd166',
                                    label=f"箱 {b.get('detection_index', '')}", filled=True, key=b.get('id', 'box'),
                                    name=f"实测纸箱包络 {b.get('detection_index', '')} · {b.get('id', '')}"))
        if self.show_robot.get() and self.robot is not None:
            q, _ = self.joints()
            if q is not None:
                try:
                    for s in self.robot.solids(q): s['filled'] = True; out.append(s)
                except Exception as e:
                    self.robot_error = f'{type(e).__name__}: {e}'
        return out

    # -- view ---------------------------------------------------------------
    def basis(self):
        az, el = self.az, self.el
        eye = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        right = np.array([-np.sin(az), np.cos(az), 0.]); up = np.cross(eye, right)
        return eye, right, up

    def reset_view(self):
        self.az, self.el, self.zoom, self.pan = np.deg2rad(-60.), np.deg2rad(25.), 1.6, np.zeros(2); self.render()

    def _picked(self):
        for tag in self.canvas.gettags('current'):
            if tag.startswith('k') and tag[1:].isdigit(): return self._names.get(int(tag[1:]))
        return None

    def _hover(self, _):
        if self._drag is not None: return
        picked = self._picked()
        if picked: self.hover.set(picked[1] + ('（线框）' if picked[0] in self.wire else '') + ' · Ctrl+点击切换实体/线框')

    def _toggle_wire(self, _):
        picked = self._picked()
        if picked:
            self.wire ^= {picked[0]}; self._drag = None; self.render()

    def _start(self, e, mode): self._drag = (mode, e.x, e.y, self.az, self.el, self.pan.copy())

    def _end(self, _): self._drag = None; self.render()

    def _move(self, e):
        if self._drag is None: return
        mode, x0, y0, az, el, pan = self._drag
        if mode == 'orbit':
            self.az = az - (e.x - x0) * .01
            self.el = float(np.clip(el + (e.y - y0) * .01, -np.pi / 2 + .02, np.pi / 2 - .02))
        else:
            self.pan = pan + np.array([e.x - x0, e.y - y0], float)
        self.render(fast=True)

    def _wheel(self, direction):
        self.zoom = float(np.clip(self.zoom * (1.15 if direction > 0 else 1 / 1.15), .1, 40)); self.render()

    def render(self, fast=False):
        c = self.canvas
        try:
            if not c.winfo_exists(): return
        except tk.TclError: return
        c.delete('all'); w = max(c.winfo_width(), 300); h = max(c.winfo_height(), 200)
        eye, right, up = self.basis(); light = eye + .6 * up; light /= np.linalg.norm(light)
        scale = self.zoom * min(w, h) / 3.2   # about 3.2 m across at zoom 1
        centre = np.array([-.15, -.15, .35])   # robot working volume

        def screen(p):
            p = np.atleast_2d(p) - centre
            return np.c_[w / 2 + self.pan[0] + (p @ right) * scale, h / 2 + self.pan[1] - (p @ up) * scale]
        polys = []; lines = []; labels = []; self._names = {}
        for index, it in enumerate(self.items()):
            if it.get('key') is not None:
                self._names[index] = (it['key'], it.get('name', it['key']))
                if it['key'] in self.wire: it = dict(it, filled=False)
            if it['kind'] == 'box':
                v, faces = box_solid(it['pose'], it['size']); normals = None
            else:
                v, faces, normals = hull_solid(it['points'])
            if not it.get('filled', True):
                xy = screen(v)
                for f in faces:
                    lines.append((xy[f + [f[0]]], it['colour'], it.get('dash', False), index))
                continue
            xy = screen(v); fa = np.asarray(faces); V = v[fa]
            if normals is None:
                n = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]); norm = np.linalg.norm(n, axis=1)
                ok = norm > 1e-12; n[ok] /= norm[ok, None]
            else:
                n = np.asarray(normals, float); ok = np.ones(len(fa), bool)
            visible = np.nonzero(ok & (n @ eye > 0))[0]
            if not len(visible): continue
            depth = (V[visible] @ eye).mean(1); shade = .45 + .65 * np.maximum(0., n[visible] @ light); xyf = xy[fa[visible]]
            stipple = it.get('stipple', '')
            for k in range(len(visible)):
                polys.append((float(depth[k]), xyf[k], _shade(it['colour'], shade[k]), index, stipple))
            if it.get('label'): labels.append((screen(v.mean(0))[0], it['label'], it['colour']))
        for xy, colour, dash, index in lines:
            c.create_line(*xy.ravel(), fill=colour, dash=(4, 3) if dash else None, tags=(f'k{index}',))
        polys.sort(key=lambda p: p[0])
        outline = '' if fast else '#1d2b38'
        for _, xy, colour, index, stipple in polys:
            c.create_polygon(*xy.ravel(), fill=colour, outline=outline, stipple=stipple, tags=(f'k{index}',))
        # Base axes and the furthest -Y the real robot reached on 2026-11-26.
        o = screen(np.zeros(3))[0]
        for k, (name, colour) in enumerate((('X', '#ff5252'), ('Y', '#69f0ae'), ('Z', '#448aff'))):
            end = screen(np.eye(3)[k] * .3)[0]
            c.create_line(*o, *end, fill=colour, width=2, arrow='last'); c.create_text(*end, text=name, fill=colour, anchor='sw')
        if legacy_markers(self.layout):
            a = np.array([[-.8, FIELD_MIN_Y_REACHED_M, z] for z in (-.1, .9)] + [[.3, FIELD_MIN_Y_REACHED_M, z] for z in (.9, -.1)])
            c.create_line(*screen(np.vstack([a, a[:1]])).ravel(), fill='#ff4d6d', dash=(6, 4), width=2)
            c.create_text(*screen(a[1])[0], text=f'11-26 真机到达的最 −Y 处 y={FIELD_MIN_Y_REACHED_M:.3f} m', fill='#ff4d6d', anchor='sw')
        for xy, text, colour in labels:
            c.create_text(*xy, text=text, fill='#ffffff', font=('DejaVu Sans', 10, 'bold'))
        q, source = self.joints()
        info = [self.layout_note, f'机器人：{source}' + (f' {np.round(np.rad2deg(q), 1).tolist()}°' if q is not None else '') +
                (f' · 模型不可用 {self.robot_error}' if self.robot_error else ('' if self.robot else ' · 正在加载模型…'))]
        if self.layout:
            info.append(f"STEP 保留 {len(self.layout['kept'])} 件（实体）· 工作空间外 {len(self.layout['pruned'])} 件（线框）· 蓝虚线=工作空间边界")
        c.create_text(10, 8, anchor='nw', fill='#e2edf5', text='\n'.join(info))
        c.create_text(w - 10, h - 8, anchor='se', fill='#a3b6c6',
                      text=f'base_link（m）· 左键拖动旋转 · 右键平移 · 滚轮缩放 · 双击复位 · 方位 {np.rad2deg(self.az):.0f}° 俯仰 {np.rad2deg(self.el):.0f}° · 仅显示，不改场景/计划')
