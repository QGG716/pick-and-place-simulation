"""Part roles and the one-DOF lift mechanism of the user's site layout STEP.

Pure geometry on the converted manifest (``site-layout.json``) in the CAD
frame (metres, CAD +Y up). Shared by the simulation scene and the real-site
static layer; neither may substitute its own guesses. The lift height ``h``
is the top-plate displacement along CAD +Y from the CAD pose. Its range and
its runtime value are field inputs with a recorded basis, never inferred.
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

import numpy as np
import yaml

MOVING_ROLES = ('LIFT_TOP', 'BELT_INCLINED', 'BELT_SLIDER')
# Range comparisons absorb the analytic BRep bound tolerance (about 0.1 um in
# the site STEP plate bounds); 1 um is far below every physical lift input.
RANGE_TOLERANCE_M = 1e-6
FIXED = 'FIXED'
INTERIOR = 'LIFT_INTERIOR'


def load_roles(path):
    path = Path(path)
    roles = yaml.safe_load(path.read_text(encoding='utf-8'))
    if roles.get('schema') != 'eco65_site_layout_roles_v1':
        raise ValueError('SITE_ROLES_SCHEMA_INVALID')
    for rule in roles['rules']:
        if not isinstance(rule.get('basis'), str) or not rule['basis'].strip():
            raise ValueError('SITE_ROLES_RULE_BASIS_REQUIRED')
        if ('name' in rule) == ('region_mm' in rule):
            raise ValueError('SITE_ROLES_RULE_NEEDS_NAME_XOR_REGION')
    for role, basis in roles['excluded_roles'].items():
        if not isinstance(basis, str) or not basis.strip():
            raise ValueError('SITE_ROLES_EXCLUSION_BASIS_REQUIRED:' + role)
    roles['_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return roles


def classify(manifest, roles):
    """part id -> {role, rule, basis}. Unmatched parts stay FIXED obstacles."""
    if roles['source_sha256'] != manifest['source_sha256']:
        raise ValueError('SITE_ROLES_SOURCE_MISMATCH')
    result = {}
    for part in manifest['parts']:
        lo, hi = np.asarray(part['collision_bounds_m'], float) * 1000.
        name = part.get('body_name') or part.get('name') or ''
        for index, rule in enumerate(roles['rules']):
            if 'name' in rule:
                hit = re.search(rule['name'], name) is not None
            else:
                rlo, rhi = np.asarray(rule['region_mm'], float)
                hit = bool(np.all(lo >= rlo) and np.all(hi <= rhi))
            if hit:
                result[part['id']] = dict(role=rule['role'], rule=index, basis=rule['basis'])
                break
        else:
            result[part['id']] = dict(role=FIXED, rule=None, basis='NO_RULE_MATCHED_KEPT_AS_FIXED_OBSTACLE')
    return result


def _bounds(manifest, classification, pattern=None, role=None):
    rows = [p for p in manifest['parts']
            if (pattern is None or re.search(pattern, p.get('body_name') or ''))
            and (role is None or classification[p['id']]['role'] == role)]
    if not rows:
        raise ValueError('SITE_MECHANISM_PART_MISSING:' + str(pattern or role))
    b = np.array([p['collision_bounds_m'] for p in rows], float)
    return rows, b[:, 0].min(0), b[:, 1].max(0)


def _hinge(manifest, classification, pattern):
    rows, _, _ = _bounds(manifest, classification, pattern)
    centres = np.array([np.mean(p['collision_bounds_m'], axis=0) for p in rows])
    if len(rows) != 2 or np.ptp(centres[:, 1]) > 1e-4 or np.ptp(centres[:, 2]) > 1e-4:
        raise ValueError('SITE_MECHANISM_HINGE_NOT_COAXIAL_ALONG_CAD_X:' + pattern)
    return centres.mean(0)


def part_box(part):
    """(pose in CAD, size, kind): the analytic oriented box when it is smaller
    than the analytic axis-aligned box; both are conservative BRep bounds."""
    lo, hi = np.asarray(part['collision_bounds_m'], float)
    pose, size, kind = translation((lo + hi) / 2), hi - lo, 'ANALYTIC_AABB'
    obb = part.get('collision_obb')
    if obb is not None and np.prod(obb['size_m']) < np.prod(size):
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = np.asarray(obb['rotation'], float), obb['center_m']
        size, kind = np.asarray(obb['size_m'], float), 'ANALYTIC_OBB'
    return pose, size, kind


def translation(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


class LiftMechanism:
    """Closed chain: top plate +h along CAD Y; rigid inclined belt between the
    lower hinge (on the top plate) and the upper hinge (slider on CAD-Z rails)."""

    def __init__(self, manifest, roles, classification=None):
        self.manifest, self.roles = manifest, roles
        self.classification = classification or classify(manifest, roles)
        m = roles['mechanism']
        if m.get('lift_axis_cad') != [0, 1, 0]:
            raise ValueError('SITE_MECHANISM_ONLY_CAD_Y_LIFT_SUPPORTED')
        self.lower = _hinge(manifest, self.classification, m['lower_hinge'])
        self.upper = _hinge(manifest, self.classification, m['upper_hinge'])
        self.dy0, self.dz0 = self.upper[1] - self.lower[1], self.upper[2] - self.lower[2]
        if self.dz0 <= 0:
            raise ValueError('SITE_MECHANISM_UPPER_HINGE_NOT_TOWARDS_CAD_PLUS_Z')
        self.length_m = float(np.hypot(self.dy0, self.dz0))
        self.angle0_rad = float(np.arctan2(self.dy0, self.dz0))
        _, base_lo, base_hi = _bounds(manifest, self.classification, m['base_plate'])
        _, top_lo, top_hi = _bounds(manifest, self.classification, m['top_plate'])
        self.base_top_y, self.top_under_y = float(base_hi[1]), float(top_lo[1])
        self.overall_height_cad_m = float(top_hi[1] - base_lo[1])
        self.interior_parts, ilo, ihi = _bounds(manifest, self.classification, role=INTERIOR)
        self.interior_xz = (ilo, ihi)
        _, rlo, rhi = _bounds(manifest, self.classification, m['rail'])
        _, slo, shi = _bounds(manifest, self.classification, role='BELT_SLIDER')
        self.rail_z, self.slider_z = (float(rlo[2]), float(rhi[2])), (float(slo[2]), float(shi[2]))
        r = m.get('lift_range')
        if (not isinstance(r, dict) or r.get('measure') != 'OVERALL_HEIGHT_BASE_PLATE_BOTTOM_TO_TOP_PLATE_TOP'
                or not isinstance(r.get('basis'), str) or not r['basis'].strip()):
            raise ValueError('SITE_MECHANISM_LIFT_RANGE_REQUIRED')
        lo, hi = (float(v) / 1000. for v in r['overall_height_mm'])
        if not lo < hi:
            raise ValueError('SITE_MECHANISM_LIFT_RANGE_INVALID')
        self.range_h_m = [lo - self.overall_height_cad_m, hi - self.overall_height_cad_m]
        self.range_basis = r['basis']
        for h in self.range_h_m:
            self.state(h)

    def state(self, h):
        h = float(h)
        dy = self.dy0 - h
        if not np.isfinite(h) or abs(dy) >= self.length_m:
            raise ValueError('SITE_MECHANISM_LIFT_OUT_OF_CHAIN_RANGE')
        dz = float(np.sqrt(self.length_m ** 2 - dy ** 2))
        angle = float(np.arctan2(dy, dz))
        gap = self.top_under_y + h - self.base_top_y
        if gap <= 0:
            raise ValueError('SITE_MECHANISM_LIFT_PLATES_OVERLAP')
        shift = dz - self.dz0
        return dict(h_m=h, belt_angle_rad=angle, belt_angle_deg=float(np.degrees(angle)), slider_shift_m=shift,
                    plate_gap_m=gap, slider_z_m=[self.slider_z[0] + shift, self.slider_z[1] + shift],
                    slider_overhang_past_rail_end_m=max(0., self.slider_z[1] + shift - self.rail_z[1]),
                    slider_short_of_rail_start_m=max(0., self.rail_z[0] - (self.slider_z[0] + shift)))

    def transform(self, role, h):
        """CAD-frame rigid motion of a role group at lift height h."""
        if role in (FIXED, INTERIOR):
            return np.eye(4)
        if role == 'LIFT_TOP':
            return translation([0., h, 0.])
        s = self.state(h)
        if role == 'BELT_SLIDER':
            return translation([0., 0., s['slider_shift_m']])
        if role == 'BELT_INCLINED':
            # Rotation about CAD +X maps (y,z) polar angle atan2(z,y); the belt
            # direction angle from +Z towards +Y is its complement.
            d = -(s['belt_angle_rad'] - self.angle0_rad)
            c, n = np.cos(d), np.sin(d)
            R = np.eye(4)
            R[1:3, 1:3] = [[c, -n], [n, c]]
            return translation(self.lower + [0., h, 0.]) @ R @ translation(-self.lower)
        raise ValueError('SITE_MECHANISM_ROLE_NOT_MOVABLE:' + role)

    def boxes(self, h, h_high=None, pad_target_m=1e-4):
        """Collision boxes in the CAD frame: every retained part as its oriented
        analytic AABB moved with its group; interior parts as one envelope.

        With ``h_high`` the boxes conservatively cover every lift height in
        [h, h_high] (sensor uncertainty): translations are swept exactly; the
        rotating belt is sampled densely and padded by an exact speed bound
        so no swept point lies more than ``pad_target_m`` outside a sample box.
        """
        lo_h = float(h)
        hi_h = lo_h if h_high is None else float(h_high)
        if not hi_h >= lo_h:
            raise ValueError('SITE_MECHANISM_H_INTERVAL_INVALID')
        mid = (lo_h + hi_h) / 2
        a, b = self.state(lo_h), self.state(hi_h)
        span = hi_h - lo_h
        excluded = set(self.roles['excluded_roles'])
        out = []
        for part in self.manifest['parts']:
            role = self.classification[part['id']]['role']
            if role in excluded or role == INTERIOR:
                continue
            pose0, size, kind = part_box(part)
            R0 = pose0[:3, :3]
            if span == 0 or role == FIXED:
                pose, grown = self.transform(role, mid) @ pose0, size
            elif role == 'LIFT_TOP':
                # Box translated along CAD Y over the span: grow by the segment's
                # extent along each box axis (exact for a translation sweep).
                pose, grown = translation([0., mid, 0.]) @ pose0, size + np.abs(R0.T @ [0., span, 0.])
            elif role == 'BELT_SLIDER':
                shift = (a['slider_shift_m'] + b['slider_shift_m']) / 2
                pose = translation([0., 0., shift]) @ pose0
                grown = size + np.abs(R0.T @ [0., 0., b['slider_shift_m'] - a['slider_shift_m']])
            else:
                pose, grown = self._swept_belt_box(pose0, size, lo_h, hi_h, pad_target_m)
            out.append(dict(id=part['id'], role=role, name=part.get('body_name', ''), pose=pose.tolist(),
                            size=np.asarray(grown).tolist(), part_ids=[part['id']], box_kind=kind))
        lo, hi = self.interior_xz
        top = self.top_under_y + hi_h
        elo = np.array([lo[0], self.base_top_y, lo[2]])
        ehi = np.array([hi[0], top, hi[2]])
        out.append(dict(id='lift_interior_envelope', role=INTERIOR, name='scissor interior envelope',
                        pose=translation((elo + ehi) / 2).tolist(), size=(ehi - elo).tolist(),
                        part_ids=[p['id'] for p in self.interior_parts],
                        basis='Interior parts stay between the plates for every h in the physical lift range'))
        return out

    def _swept_belt_box(self, pose0, size, lo_h, hi_h, pad_target_m):
        """Box in the belt frame at the mid height containing the part box at
        every h in [lo_h, hi_h]. A point moves with speed |dp/dh| <= 1 + r/dz
        (translation plus rotation, d(angle)/dh = -1/dz), so samples spaced
        delta apart leave at most (1 + r_max/dz_min) * delta / 2 unsampled."""
        unit = np.array([[i, j, k] for i in (-.5, .5) for j in (-.5, .5) for k in (-.5, .5)]) * size
        corners = (pose0 @ np.c_[unit, np.ones(8)].T).T[:, :3]
        r_max = float(np.max(np.linalg.norm((corners - self.lower)[:, 1:], axis=1)))
        # dz(h) = sqrt(L^2 - (dy0 - h)^2) is concave in h: its minimum is at an end.
        dz_min = min(np.sqrt(self.length_m ** 2 - (self.dy0 - h) ** 2) for h in (lo_h, hi_h))
        speed = 1. + r_max / dz_min
        steps = max(1, int(np.ceil((hi_h - lo_h) * speed / (2 * pad_target_m))))
        pad = speed * (hi_h - lo_h) / steps / 2
        mid = (lo_h + hi_h) / 2
        frame = self.transform('BELT_INCLINED', mid) @ pose0
        inv = np.linalg.inv(frame)
        local = []
        for h in np.linspace(lo_h, hi_h, steps + 1):
            T = self.transform('BELT_INCLINED', h)
            local.append((inv @ T @ np.c_[corners, np.ones(8)].T).T[:, :3])
        local = np.concatenate(local)
        lo, hi = local.min(0) - pad, local.max(0) + pad
        return frame @ translation((lo + hi) / 2), hi - lo

    def summary(self):
        return dict(lower_hinge_m=self.lower.tolist(), upper_hinge_m=self.upper.tolist(), hinge_length_m=self.length_m,
                    belt_angle0_deg=float(np.degrees(self.angle0_rad)), base_plate_top_y_m=self.base_top_y,
                    top_plate_underside_y_m=self.top_under_y, rail_z_m=list(self.rail_z), slider_z_m=list(self.slider_z),
                    interior_parts=len(self.interior_parts), overall_height_cad_m=self.overall_height_cad_m,
                    machine_range_h_m=list(self.range_h_m), machine_range_basis=self.range_basis)


def lift_state(config, mechanism):
    """Validated field lift input. h needs a basis; the machine range comes from
    the roles file and a field range may only tighten it. No defaults."""
    c = config.get('lift') if isinstance(config, dict) else None
    if not isinstance(c, dict):
        raise ValueError('SITE_LIFT_STATE_REQUIRED')
    if not isinstance(c.get('h_basis'), str) or not c['h_basis'].strip():
        raise ValueError('SITE_LIFT_BASIS_REQUIRED:h_basis')
    machine = list(mechanism.range_h_m)
    r, basis = machine, mechanism.range_basis
    if 'range_m' in c:
        r = c['range_m']
        if not isinstance(c.get('range_basis'), str) or not c['range_basis'].strip():
            raise ValueError('SITE_LIFT_BASIS_REQUIRED:range_basis')
        if (not isinstance(r, list) or len(r) != 2 or any(type(v) not in (int, float) for v in r)
                or not np.all(np.isfinite(r)) or not r[0] < r[1]):
            raise ValueError('SITE_LIFT_RANGE_INVALID')
        if r[0] < machine[0] - RANGE_TOLERANCE_M or r[1] > machine[1] + RANGE_TOLERANCE_M:
            raise ValueError('SITE_LIFT_RANGE_WIDENS_MACHINE_RANGE')
        basis = c['range_basis']
    h = c.get('h_m')
    if type(h) not in (int, float) or not np.isfinite(h) or not r[0] - RANGE_TOLERANCE_M <= h <= r[1] + RANGE_TOLERANCE_M:
        raise ValueError('SITE_LIFT_H_OUTSIDE_CONFIGURED_RANGE')
    return dict(h_m=float(h), range_m=[float(r[0]), float(r[1])], h_basis=c['h_basis'], range_basis=basis,
                machine_range_m=machine, machine_range_basis=mechanism.range_basis)


def check_range(mechanism, lift):
    """Both range ends must be geometrically valid chain states."""
    return dict(low=mechanism.state(lift['range_m'][0]), high=mechanism.state(lift['range_m'][1]))


def load_manifest(path):
    path = Path(path)
    return json.loads(path.read_text(encoding='utf-8'))


def cad_robot_base(roles):
    """T_base_cad from the roles file, re-orthonormalised (rounding <= 1e-5)."""
    c = roles.get('cad_robot_base')
    if not isinstance(c, dict) or not isinstance(c.get('basis'), str) or not c['basis'].strip():
        raise ValueError('SITE_CAD_ROBOT_BASE_REQUIRED')
    T = np.asarray(c['T_base_cad'], float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError('SITE_CAD_ROBOT_BASE_INVALID')
    u, _, vt = np.linalg.svd(T[:3, :3])
    R = u @ vt
    if np.linalg.det(R) <= 0 or np.abs(R - T[:3, :3]).max() > 1e-5:
        raise ValueError('SITE_CAD_ROBOT_BASE_NOT_A_ROTATION')
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, T[:3, 3]
    return out
