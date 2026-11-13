"""Visualize obstacles near a target and its collision envelope from a saved task record.

Usage: visualize_single_carton_rest_check.py <task_id> [output.png]. Originally written
for TARGET_PAYLOAD_INTERSECTS_AT_REST (task 19803562); it uses the default (OBSTACLE)
target claims so every unexplained point near the box is shown.

Read-only diagnostic: reproduces the scene builder's target claims on the
record's own RGB-D and SAM mask, then shows (1) which measured points were left
as obstacles, (2) the obstacle boxes built from them, and (3) the few points
that actually lie inside the collision envelope. No device or planner code.
"""
import json, sys
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from scipy.spatial import ConvexHull
from unloading_sim.workbench.detection import metric_points
from unloading_sim.workbench.single_carton.geometry import transform, corners
from unloading_sim.workbench.single_carton import site_scene as ss

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
ROOT = Path(__file__).resolve().parents[1]
task = sys.argv[1] if len(sys.argv) > 1 else '1980356201804419a9e66e4063b9d732'
rec = ROOT/'outputs/single_carton'/task
out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT/'outputs/single_carton_development'/f'rest-check-{task[:8]}.png'

obs = json.loads((rec/'observation.json').read_text()); arr = np.load(rec/'observation.npz')
mask = np.load(rec/'segmentation.npz')['mask_0'].astype(bool)
box = json.loads((rec/'geometry.json').read_text())['boxes'][0]
scene = json.loads((rec/'site-scene.json').read_text())
found = next(rec.glob('*payload-rest-check.json'), None)
rest = json.loads(found.read_text()) if found else dict(intersections=[], supports=[])
frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=arr['rgb'], depth=arr['depth'],
                        intrinsics=obs['intrinsics'], depth_scale_m=obs['depth_scale_m'])
P = metric_points(frame); W = np.asarray(obs['T_world_camera'], float); C = np.linalg.inv(W)
depth = arr['depth'].astype(float)*obs['depth_scale_m']; valid = np.isfinite(P).all(-1) & (depth > .1) & (depth < 3.)
world = transform(W, P.reshape(-1, 3)).reshape(P.shape)
# Same target claims as the scene builder: mask, own plane / side planes, flying pixels.
z = np.where(valid, depth, 0.).astype(np.float32); sq = np.ones((3, 3), np.uint8)
jump = (cv2.dilate(z, sq)-cv2.erode(z, sq)) > ss.FLYING_DEPTH_JUMP_M
owned, flying, _ = ss._target_claims(box, mask, valid, world, jump); flying &= ~owned
obstacle_points = valid & ~owned & ~flying

T = np.asarray(box['pose'], float); half = np.asarray(box['size_m'], float)/2
face = box['faces'][0]; prior = np.asarray(box['nominal_size_m']); tol = box['uncertainty']['dimension_tolerance_m']
local = transform(np.linalg.inv(T), world.reshape(-1, 3)).reshape(P.shape)
vertical = int(np.argmax(abs(T[2, :3]))); lateral = [i for i in (0, 1) if i != vertical][0]
objects = {o['id']: o for o in scene['world']}
support = objects[rest['supports'][0]] if rest.get('supports') else None
support_top = float(np.asarray(box['corners_m'])[:, 2].min()) if support is None else \
    float(corners(support['pose'], support['size'])[:, 2].max())
height = (world[..., 2]-support_top)*1000                     # mm above the measured tray top
inside = obstacle_points & np.all(abs(local) <= half, axis=-1)  # obstacle points inside the envelope
colors = ['#e6194b', '#f58231', '#911eb4', '#0082c8']
hits = [dict(h, label='ABCD'[k], color=colors[k % 4]) for k, h in enumerate(rest['intersections'])]
for h in hits:
    o = objects[h['id']]; Po = np.asarray(o['pose'], float); so = np.asarray(o['size'], float)/2
    h['corners_world'] = corners(Po, o['size'])
    h['source'] = obstacle_points & np.all(abs(transform(np.linalg.inv(Po), world.reshape(-1, 3)).reshape(P.shape)) <= so+1e-6, axis=-1)
    h['inside'] = h['source'] & inside


def project(points_world):
    return ss._project(transform(C, np.asarray(points_world, float)), obs['intrinsics'])


EDGES = [(a, b) for a in range(8) for b in range(a+1, 8) if bin(a ^ b).count('1') == 1]


def wire(ax, corners_world, color, lw, ls='-'):
    uv = project(corners_world)
    for a, b in EDGES: ax.plot(*uv[[a, b]].T, color=color, lw=lw, ls=ls)


n, u, v = (np.asarray(face[k]) for k in ('normal', 'u', 'v')); c = np.asarray(face['center_m']); s = np.asarray(face['size_m'])/2
quad = [c+su*s[0]*u+sv*s[1]*v for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
rgb = arr['rgb'].astype(float)/255
tint = rgb.copy(); tint[mask] = .6*tint[mask]+.4*np.array([.25, .85, .25])
for h in hits: tint[h['source']] = .35*tint[h['source']]+.65*np.array(matplotlib.colors.to_rgb(h['color']))

fig = plt.figure(figsize=(20, 11.5), dpi=105)
g = fig.add_gridspec(3, 4, width_ratios=[1.6, .8, 1.05, 1.05], height_ratios=[1, 1, 1], wspace=.22, hspace=.42)

# 1. Image overview.
ax = fig.add_subplot(g[:2, 0]); ax.imshow(tint)
ax.add_patch(Polygon(project(quad), fill=False, ec='#00ff00', lw=2.5))
wire(ax, box['corners_m'], '#ffe119', 2, '--')
for h in hits:
    wire(ax, h['corners_world'], h['color'], 1.8)
    uvh = project(h['corners_world']); top = uvh[np.argmin(uvh[:, 1])]
    ax.text(*top, f" {h['label']} {h['overlap_depth_m']*1000:.1f} mm ", color='white', fontsize=11, weight='bold',
            bbox=dict(fc=h['color'], ec='none', alpha=.9, pad=1.2), va='bottom')
vi, ui = np.nonzero(inside); ax.scatter(ui, vi, s=14, facecolors='none', edgecolors='#ff00ff', lw=1.2)
ax.set_xlim(0, rgb.shape[1]); ax.set_ylim(rgb.shape[0], 0); ax.axis('off')
ax.set_title('相机原图　绿＝SAM 掩码　亮绿框＝实测可见面　黄虚线＝完整碰撞包络\n'
             '彩色区＝生成障碍 A～D 的实测点及其障碍方块　洋红圈＝真正落在包络内的点', fontsize=11)

# 2. Zoomed corners (where the in-envelope points are).
def zoom(ax, center, title, r=38):
    cu, cv = map(int, center); u0, u1 = max(0, cu-r), min(rgb.shape[1], cu+r); v0, v1 = max(0, cv-r), min(rgb.shape[0], cv+r)
    ax.imshow(tint[v0:v1, u0:u1], extent=(u0, u1, v1, v0), interpolation='nearest')
    edge = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)).astype(bool)
    ev, eu = np.nonzero(edge[v0:v1, u0:u1]); ax.scatter(eu+u0, ev+v0, s=2, c='#00ff00')
    sel = inside[v0:v1, u0:u1]; iv, iu = np.nonzero(sel)
    sc = ax.scatter(iu+u0, iv+v0, s=40, c=height[v0:v1, u0:u1][sel], cmap='magma', vmin=-5, vmax=20, edgecolors='white', lw=.6)
    ax.set_xlim(u0, u1); ax.set_ylim(v1, v0); ax.set_title(title, fontsize=10); ax.set_xticks([]); ax.set_yticks([])
    return sc
left = np.argwhere(inside[:, :rgb.shape[1]//2]); right = np.argwhere(inside[:, rgb.shape[1]//2:])
sc = zoom(fig.add_subplot(g[0, 1]), (left[:, 1].mean(), left[:, 0].mean()) if len(left) else (130, 320),
          f'放大：左下角　包络内 {len(left)} 个点')
zoom(fig.add_subplot(g[1, 1]), ((right[:, 1]+rgb.shape[1]//2).mean(), right[:, 0].mean()) if len(right) else (400, 330),
     f'放大：右侧　包络内 {len(right)} 个点')
cb = fig.colorbar(sc, ax=fig.axes[-2:], fraction=.05, pad=.02); cb.set_label('高出托盘面 / mm')


def panel(ax, horizontal, title, xlabel):
    """Box-frame lateral/depth vs. a second axis; height is mm above the tray top."""
    hval = lambda m: local[m][:, horizontal]*1000
    yval = lambda m: height[m] if vertical_axis else local[m][:, 2]*1000
    near = valid & np.all(abs(local) <= half+.07, axis=-1)
    ax.scatter(hval(near & ~mask), yval(near & ~mask), s=1, c='#b0b0b0', alpha=.35)
    ax.scatter(hval(near & mask), yval(near & mask), s=1, c='#3cb44b', alpha=.25)
    env_lo = np.r_[-half[horizontal], (np.asarray(box['corners_m'])[:, 2].min()-support_top) if vertical_axis else -half[2]]*np.r_[1000, 1000 if vertical_axis else 1000]
    env_size = np.r_[2*half[horizontal], (np.ptp(np.asarray(box['corners_m'])[:, 2])) if vertical_axis else 2*half[2]]*1000
    ax.add_patch(Rectangle(env_lo, *env_size, fill=False, ec='#c9a400', lw=2.2, ls='--'))
    for h in hits:
        lc = transform(np.linalg.inv(T), h['corners_world'])
        xy = np.c_[lc[:, horizontal]*1000, (h['corners_world'][:, 2]-support_top)*1000 if vertical_axis else lc[:, 2]*1000]
        ax.add_patch(Polygon(xy[ConvexHull(xy).vertices], fc=h['color'], ec=h['color'], alpha=.28, lw=1.3))
        ax.scatter(hval(h['source']), yval(h['source']), s=3, c=h['color'])
        ax.text(*xy.mean(0), h['label'], fontsize=12, weight='bold', ha='center', va='center')
    ax.scatter(hval(inside), yval(inside), s=26, facecolors='none', edgecolors='#ff00ff', lw=1.1)
    if vertical_axis: ax.axhline(0, color='#4363d8', lw=1.2, ls=':')
    ax.set_title(title, fontsize=10.5); ax.set_xlabel(xlabel); ax.grid(alpha=.3); ax.set_aspect('equal')
    ax.set_xlim(-half[horizontal]*1000-60, half[horizontal]*1000+60)


vertical_axis = True
ax = fig.add_subplot(g[0, 2]); panel(ax, lateral, '正视（从相机方向看）', '横向 / mm'); ax.set_ylabel('高出托盘面 / mm'); ax.set_ylim(-60, 190)
ax = fig.add_subplot(g[0, 3]); panel(ax, 2, '侧视', '深度 / mm（右＝朝相机，可见面在 +55）'); ax.set_ylim(-60, 190)
vertical_axis = False
ax = fig.add_subplot(g[1, 2:]); panel(ax, lateral, '俯视（只看包络高度范围内的点）', '横向 / mm'); ax.set_ylabel('深度 / mm（上＝朝相机）')

# 3. Explanation.
ax = fig.add_subplot(g[2, :]); ax.axis('off')
seen = np.asarray(face['size_m'])*1000; env = np.asarray(box['size_m'])*1000
over = (env[lateral]-seen.max())/2
rows = [f'任务 {task}　检测 {box["detection"]["label"]} {box["detection"]["score"]:.3f}；实测可见面 {seen.max():.0f} × {seen.min():.0f} mm；'
        f'箱型先验 {prior[0]*1000:.0f}×{prior[1]*1000:.0f}×{prior[2]*1000:.0f} mm ±{tol*1000:.0f} mm；'
        f'完整碰撞包络 横 {env[lateral]:.0f} × 高 {env[vertical]:.0f} × 深 {env[2]:.0f} mm（两侧各比可见面宽约 {over:.0f} mm，后方厚度看不到）',
        '包络 = 所有"尺寸在误差内、朝向在 ±2.5° 内、且包含可见面"的箱子的并集。带箱脱离时整个包络随吸盘移动，所以它在原位就不能与任何实测物体相交。', '']
for h in hits:
    hv = height[h['source']]; hin = height[h['inside']]
    rows.append(f'{h["label"]}　{h["id"]}　相交 {h["overlap_depth_m"]*1000:.1f} mm：由 {int(h["source"].sum())} 个实测点生成（高出托盘面 {hv.min():.0f}～{hv.max():.0f} mm），'
                f'其中真正落在包络内的只有 {int(h["inside"].sum())} 个' + (f'（{hin.min():.0f}～{hin.max():.0f} mm）' if len(hin) else ''))
rows += ['', f'结论：全图约 {int(valid.sum()/1000)}k 个有效深度点里，真正落在包络内的障碍点只有 {int(inside.sum())} 个，都在箱体左下角、右侧边缘，'
         '离 SAM 掩码边界只有几个像素、高出托盘面 0～10 mm（右上另有几个约 80 mm 高的点）。'
         '它们很可能是箱子自身底边 / 侧边边缘没被掩码覆盖的点，或紧贴箱底的泡沫边。',
         '场景会把每个 4 cm 网格里的实测点合成一个轴对齐方块；这几个边缘点和下方托盘格子里的点合在一起，方块就向上伸进了包络，相交深度被放大到 12～19 mm。']
ax.text(0, 1, '\n'.join(rows), va='top', fontsize=10.5, transform=ax.transAxes)
fig.suptitle('TARGET_PAYLOAD_INTERSECTS_AT_REST：箱体原位的完整碰撞包络与实测障碍相交', fontsize=15, weight='bold')
fig.savefig(out, bbox_inches='tight'); print(out)
print(json.dumps(dict(inside_points=int(inside.sum()), per_hit={h['label']: [int(h['source'].sum()), int(h['inside'].sum())] for h in hits}), ensure_ascii=False))
