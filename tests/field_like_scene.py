"""Field-like synthetic RGB-D scenes for ECO65 carton geometry regression (no device IO).

Unlike the isolated-box fixtures, these scenes reproduce the field conditions that
broke geometry on 2026-11-19: real camera pose (link_1 extrinsics at the field J1),
stacked cartons with coplanar seams, touching neighbours, D435-like stereo occlusion
bands, depth noise, and SAM-like low-resolution mask edges.

World frame = base_link. Camera points are returned in the optical frame (z-depth),
which is what the production geometry functions consume together with T_base_cam.
Numbers are synthetic; use them for relative behaviour, not field accuracy claims.
"""
import json
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXTRINSICS = ROOT/'configs/workbench/reference/active_extrinsics.json'
FIELD_J1 = 1.4160903418981192          # q_start J1 of tasks d6d985f1 / 045e08d1
H, W = 480, 640
FX = FY = 607.0; CX, CY = 320.0, 240.0  # approx. D435 colour intrinsics at 640x480
UPPER = (.170, .110, .111)              # width along face, height (z), depth behind face
UPPER_FRONT = (-.53, -.03, .375)        # front-face centre in base_link
YAW_DEG = -20.0                         # face normal (cos,sin,0); field faces ~ -20 deg


def T_base_cam(j1=FIELD_J1):
    E = np.array(json.loads(EXTRINSICS.read_text())['matrix'])
    c, s = np.cos(j1), np.sin(j1); T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; T[2, 3] = .1625   # ECO65-B joint_1 origin
    return T@E


def box_pose(front_center, size, yaw_deg=YAW_DEG):
    """Box frame: x along the face (width), y = world z (height), z = outward face normal."""
    psi = np.radians(yaw_deg); n = np.array([np.cos(psi), np.sin(psi), 0.]); t = np.cross([0, 0, 1.], n)
    T = np.eye(4); T[:3, :3] = np.column_stack((t, [0, 0, 1.], n))
    T[:3, 3] = np.asarray(front_center, float)-n*size[2]/2
    return T


def render(objects, wall_x=-1.25, floor_z=.20, cam=None):
    """objects: [(pose, size)] -> (camera points HxWx3, id map: i>=0 box, -1 wall, -2 floor, cam)."""
    cam = T_base_cam() if cam is None else cam
    v, u = np.indices((H, W)); rc = np.stack(((u-CX)/FX, (v-CY)/FY, np.ones((H, W))), -1)
    rw = rc@cam[:3, :3].T; o = cam[:3, 3]
    best = np.full((H, W), np.inf); ident = np.full((H, W), -1)
    for i, (T, size) in enumerate(objects):
        R = T[:3, :3]; half = np.asarray(size)/2; ol = (o-T[:3, 3])@R; dl = rw@R
        with np.errstate(divide='ignore', invalid='ignore'):
            a = (-half-ol)/dl; b = (half-ol)/dl
            near = np.minimum(a, b).max(-1); far = np.maximum(a, b).min(-1)
        hit = (near < far) & (near > .05) & (near < best); best[hit] = near[hit]; ident[hit] = i
    with np.errstate(divide='ignore', invalid='ignore'):
        tw = (wall_x-o[0])/rw[..., 0]; tf = (floor_z-o[2])/rw[..., 2] if floor_z is not None else None
    hit = (tw > 0) & (tw < best); best[hit] = tw[hit]; ident[hit] = -1
    if tf is not None:
        hit = (tf > 0) & (tf < best); best[hit] = tf[hit]; ident[hit] = -2
    return rc*best[..., None], ident, cam


def add_noise(points, sigma=.0007, seed=1):
    rng = np.random.default_rng(seed); z = points[..., 2]
    return points/z[..., None]*(z+rng.normal(0, sigma, z.shape))[..., None]


def stereo_shadow(points, baseline=.05, f=FX):
    """D4xx-like occlusion band: background left of each foreground step loses depth,
    width f*B*(1/z_near - 1/z_far) pixels."""
    q = points.copy(); z = points[..., 2]
    step = (z[:, :-1]-z[:, 1:]) > .01           # pixel u is farther than u+1
    for row, u in zip(*np.nonzero(step)):
        w = int(f*baseline*(1/z[row, u+1]-1/z[row, u]))
        q[row, max(0, u-w+1):u+1] = np.nan
    return q


def samlike(mask, threshold=.5):
    """SAM decodes on a 256x256 grid of the 1024-px resized image, then upsamples."""
    h, w = mask.shape; s = 1024/max(h, w); h2, w2 = int(round(h*s)), int(round(w*s))
    pad = np.zeros((1024, 1024), np.float32)
    pad[:h2, :w2] = cv2.resize(mask.astype(np.float32), (w2, h2), interpolation=cv2.INTER_LINEAR)
    low = cv2.resize(pad, (256, 256), interpolation=cv2.INTER_AREA)
    up = cv2.resize(low, (1024, 1024), interpolation=cv2.INTER_LINEAR)[:h2, :w2]
    return cv2.resize(up, (w, h), interpolation=cv2.INTER_LINEAR) > threshold


def dilate(mask, px):
    k = np.ones((2*abs(px)+1,)*2, np.uint8); m = mask.astype(np.uint8)
    return (cv2.dilate(m, k) if px > 0 else cv2.erode(m, k)).astype(bool) if px else mask.copy()


def gray(ident):
    """Distinct flat shading per object, so carton seams have an RGB edge."""
    img = np.full(ident.shape, 60, np.uint8)
    for i in np.unique(ident):
        if i >= 0: img[ident == i] = 110+35*(i % 4)
    return np.repeat(img[..., None], 3, -1)


def upper_box(yaw=YAW_DEG):
    return (box_pose(UPPER_FRONT, UPPER, yaw), UPPER)


def lower_box(recess=0., width=.20, height=.12, depth=.22, yaw=YAW_DEG):
    """Box under the upper one; recess > 0 moves its front face behind the upper front plane."""
    up = upper_box(yaw)[0]; n = up[:3, 2]; t = up[:3, 0]
    fc = np.array([UPPER_FRONT[0], UPPER_FRONT[1], .32-height/2])-n*recess+t*.005
    return (box_pose(fc, (width, height, depth), yaw), (width, height, depth))


def side_neighbour(side=-1, gap=0., recess=0., height=.13, yaw=YAW_DEG):
    """Neighbour along the face direction (side=-1/+1), same front plane unless recessed."""
    up = upper_box(yaw)[0]; n = up[:3, 2]; t = up[:3, 0]
    fc = np.asarray(UPPER_FRONT)+side*t*(UPPER[0]/2+gap+.10)-n*recess
    return (box_pose(fc, (.20, height, .20), yaw), (.20, height, .20))


def truth_corners(pose, size):
    from itertools import product
    local = np.asarray(list(product((-1., 1.), repeat=3)))*np.asarray(size)/2
    return local@pose[:3, :3].T+pose[:3, 3]


def scene(name):
    """Named field-like cases used by the regression and the before/after table."""
    up = upper_box()
    cases = {
        'ideal_stack':            dict(objects=[up, lower_box(.015)], noise=False, shadow=False),
        'noisy_visible_support':  dict(objects=[up, lower_box(.015)]),
        'flush_stack':            dict(objects=[up, lower_box(0.)]),
        'flush_stack_mask_+1px':  dict(objects=[up, lower_box(0.)], mask_px=1),
        'stack_mask_+1px':        dict(objects=[up, lower_box(.015)], mask_px=1),
        'stack_mask_+3px':        dict(objects=[up, lower_box(.015)], mask_px=3),
        'stack_mask_-2px':        dict(objects=[up, lower_box(.015)], mask_px=-2),
        'stack_samlike_0.35':     dict(objects=[up, lower_box(.015)], samlike=.35),
        'frontal_yaw_0':          dict(objects=[upper_box(0.), lower_box(.015, yaw=0.)]),
        'yaw_-12':                dict(objects=[upper_box(-12.), lower_box(.015, yaw=-12.)]),
        'yaw_+20':                dict(objects=[upper_box(20.), lower_box(.015, yaw=20.)]),
        'coplanar_touching_left': dict(objects=[up, side_neighbour(-1, 0., 0.)]),
        'left_gap_30mm':          dict(objects=[up, side_neighbour(-1, .03, 0.)]),
        'parent_mask_with_child': dict(objects=[up, lower_box(.015)], parent=True),
        'packed_wall':            dict(objects=[up, lower_box(0., width=.45), side_neighbour(-1), side_neighbour(+1)]),
    }
    return cases[name]


def frame(name):
    """-> points, masks (target first), rgb, T_base_cam, truth (pose,size) of the upper box."""
    c = scene(name); objects = c['objects']
    p, ident, cam = render(objects)
    if c.get('noise', True): p = add_noise(p)
    if c.get('shadow', True): p = stereo_shadow(p)
    target = ident == 0
    if c.get('samlike'): target = samlike(target, c['samlike'])
    if c.get('mask_px'): target = dilate(target, c['mask_px'])
    masks = [(ident == 0) | (ident == 1), ident == 1] if c.get('parent') else [target]
    for i in range(1, len(objects)):
        if not c.get('parent'): masks.append(ident == i)
    return p, masks, gray(ident), cam, objects[0]


if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(ROOT/'src'))
    from unloading_sim.workbench.single_carton.instance_fit import measured_instance
    names = ['ideal_stack', 'noisy_visible_support', 'flush_stack', 'flush_stack_mask_+1px', 'stack_mask_+1px',
             'stack_mask_+3px', 'stack_mask_-2px', 'stack_samlike_0.35', 'frontal_yaw_0', 'yaw_-12', 'yaw_+20',
             'coplanar_touching_left', 'left_gap_30mm', 'parent_mask_with_child', 'packed_wall']
    for name in names:
        p, masks, rgb, cam, (pose, size) = frame(name)
        try:
            b = measured_instance(p, masks, 0, cam, rgb=rgb, maximum=.4)
            T = np.asarray(b['pose']); inside = np.max(abs((truth_corners(pose, size)-T[:3, 3])@T[:3, :3])-np.asarray(b['size_m'])/2)
            sides = {('wht'[i['axis']]+('-+'[i['side'] > 0])): i['method'] for i in b['uncertainty']['intervals']}
            print(f"{name:24s} OK measured={np.round(np.array(b['measured_size_m'])*1e3,1).tolist()} "
                  f"envelope={np.round(np.array(b['size_m'])*1e3).tolist()} truth_outside_mm={max(inside,0)*1e3:.1f} {sides}")
        except ValueError as e:
            print(f'{name:24s} FAIL {e}')
