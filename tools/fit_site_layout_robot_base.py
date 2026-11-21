"""Fit the official ECO65-B base_link mesh to the CAD robot base body.

Gives T_base_cad (CAD metres -> base_link) from the user's site layout STEP:
CAD +Y is mapped to base +Z, every yaw seed (2 deg steps) is refined by ICP
and the lowest RMS wins. This is CAD design geometry, not a field measurement;
the real site still requires frozen RGB-D registration.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
MESH = ROOT / 'assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/meshes/base_link.STL'


def fit(manifest_path, body_name='11', samples=20000, seed=0):
    manifest = json.loads(Path(manifest_path).read_text(encoding='utf-8'))
    rng = np.random.default_rng(seed)
    base = trimesh.load(MESH, force='mesh')
    rows = [p for p in manifest['parts'] if p.get('body_name') == body_name and p.get('body_kind') == 'SOLID']
    if len(rows) != 1:
        raise ValueError('CAD_ROBOT_BASE_BODY_NOT_UNIQUE')
    cad = trimesh.load(ROOT / rows[0]['visual'], force='mesh')
    src = trimesh.sample.sample_surface(cad, samples, seed=int(rng.integers(1 << 31)))[0]
    dst = trimesh.sample.sample_surface(base, samples, seed=int(rng.integers(1 << 31)))[0]
    tree = cKDTree(dst)
    R0 = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])  # CAD +Y -> base +Z
    best = None
    for yaw in np.radians(np.arange(0., 360., 2.)):
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ R0
        p = src @ R.T
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.r_[base.bounds.mean(0)[:2] - (p.min(0)[:2] + p.max(0)[:2]) / 2, base.bounds[0][2] - p.min(0)[2]]
        T = trimesh.registration.icp(src, dst, initial=T, max_iterations=30, reflection=False, scale=False)[0]
        d = tree.query(trimesh.transform_points(src, T))[0]
        rms = float(np.sqrt(np.mean(d ** 2)))
        if best is None or rms < best[0]:
            best = (rms, T, float(np.degrees(yaw)), d)
    rms, T, yaw, d = best
    back = cKDTree(trimesh.transform_points(src, T)).query(dst)[0]
    return dict(T_base_cad=T.tolist(), cad_body=rows[0]['id'], cad_body_name=body_name, seed_yaw_deg=yaw,
                rms_m=rms, p95_m=float(np.percentile(d, 95)), max_m=float(d.max()),
                reverse_rms_m=float(np.sqrt(np.mean(back ** 2))), reverse_p95_m=float(np.percentile(back, 95)),
                det=float(np.linalg.det(T[:3, :3])), samples=samples, seed=seed,
                basis='ICP of official base_link.STL to CAD body; CAD design geometry, not field measurement')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest', type=Path)
    print(json.dumps(fit(p.parse_args().manifest), indent=2))
