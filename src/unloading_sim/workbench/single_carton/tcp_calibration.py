"""TCP-based recalibration of the link_1 camera (user request 2026-11-27:
"重做一次标定 ... 写好基于TCP的标定算法").

The camera sits on link_1, so the unknown is ``Y = T_link1_camera_optical``
(``T_D_optical(q) = T_D_link1(q) @ Y``). Two acquisition modes, both using the
robot's own FK as the reference; neither commands motion - the operator jogs
the robot with the teach pendant and the UI only reads joints and frames:

``TIP_TOUCH`` (recommended; the field method)
    The board is fixed in the scene; a tip rigidly attached to the tool
    touches named board corners with q recorded at each touch. The tip offset
    on link_6 is either given or solved (the four-point TCP method: one corner
    touched in several clearly different tool orientations). The touches
    register the board in base_link; photos at any q - typically J1 turned so
    that the camera faces the board - give ``Y = inv(T_base_link1(q))
    T_base_board inv(B)``. The board may be moved (placements); one joint
    Huber fit refines Y, every board pose and the tip on all corner
    reprojections (px) and all touch residuals (mm). Field 2026-11-27: the
    camera looks about 90 deg away from the arm plane and turns with J1, so a
    board carried by the tool is never in view.

``BOARD_ON_TOOL`` (hand-eye; only for a camera that can see the tool)
    A tag board is fixed rigidly to the tool. At each stationary pose i the
    robot gives ``A_i = T_link1_tcp(q_i)`` and the camera gives the board pose
    ``B_i = T_camera_board`` (planar PnP), so ``A_i X = Y B_i`` with the
    unknown board mounting ``X = T_tcp_board``. Closed-form start (pairwise
    ``AX = XB``, Park-Martin rotation, least-squares translation, SO(3)
    chordal mean for Y), then a joint refinement of Y and X on every detected
    tag corner's reprojection error (Huber, 1 px).

Checks (implementer's values, recorded with the result; the user decides):
reprojection RMS, per-sample RMS, leave-one-out prediction, per-sample
consistency of Y, rotation diversity of the poses, estimated standard
deviation of Y, and a depth diagnostic (D435 depth at the tags against the
geometric prediction). A result that fails a check is written as
``extrinsics.rejected.json`` and cannot be used.
"""
from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SCHEMA = 'eco65_tcp_camera_extrinsics_v1'
MODES = ('BOARD_ON_TOOL', 'TIP_TOUCH')
STATUS_PASS = 'SOLVED_REQUIRES_VALIDATION'
STATUS_FAIL = 'REJECTED_BY_CALIBRATION_CHECKS'
USAGE = 'FIELD_RECALIBRATION_2026_11_27_OPERATOR_REVIEW_REQUIRED'

# Implementer's thresholds (2026-11-27); the 1 deg consistency and the 3 mm
# tip-touch tolerance are those of the earlier field calibration settings.
THRESHOLDS = dict(
    min_samples=dict(BOARD_ON_TOOL=8, TIP_TOUCH=3),   # TIP_TOUCH: photos
    min_touches=6,                      # per board placement, pivot touches included
    min_corners_per_sample=16,
    rms_px=0.8,
    sample_rms_px=1.5,
    leave_one_out_px=2.5,
    consistency_m=0.02,                 # one-image PnP depth noise at ~1 m (synthetic: 7-14 mm); the joint solve is gated by LOO and std
    consistency_deg=1.0,
    touch_rms_m=0.003,
    min_rotation_spread_deg=15.0,       # BOARD_ON_TOOL: 2nd singular value of relative-rotation vectors
    max_translation_std_m=0.002,
    max_rotation_std_deg=0.2,
    min_pivot_spread_deg=10.0,          # TIP_TOUCH with the tip solved: 2nd singular value of the pivot rotations
    max_tip_std_m=0.001,
    stationary_deg=0.02,                # joints before/after the frame (start tolerance value)
)


# --------------------------------------------------------------- rigid helpers
def rigid(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def inv(T):
    T = np.asarray(T, float)
    return rigid(T[:3, :3].T, -T[:3, :3].T@T[:3, 3])


def rotvec(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R, float)).as_rotvec()


def from_rotvec(v):
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(np.asarray(v, float)).as_matrix()


def angle_deg(R):
    return float(np.degrees(np.linalg.norm(rotvec(R))))


def kabsch(P, Q):
    """R, t minimising sum |R p + t - q|^2 (rows of P, Q)."""
    P = np.asarray(P, float); Q = np.asarray(Q, float)
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P-cp).T@(Q-cq))
    D = np.diag([1., 1., np.sign(np.linalg.det(Vt.T@U.T))])
    R = Vt.T@D@U.T
    return R, cq-R@cp


def mean_rotation(Rs):
    """Chordal L2 mean on SO(3) (never an Euler average)."""
    U, _, Vt = np.linalg.svd(np.sum(Rs, axis=0))
    D = np.diag([1., 1., np.sign(np.linalg.det(U@Vt))])
    return U@D@Vt


def difference(T_ref, T_new):
    """Translation (m) and rotation (deg) of T_new against T_ref, plus the
    small-angle split about the reference optical axes (x right, y down, z forward)."""
    D = inv(T_ref)@np.asarray(T_new, float)
    v = np.degrees(rotvec(D[:3, :3]))
    return dict(translation_m=float(np.linalg.norm(T_new[:3, 3]-np.asarray(T_ref)[:3, 3])),
                translation_link1_m=(np.asarray(T_new)[:3, 3]-np.asarray(T_ref)[:3, 3]).tolist(),
                rotation_deg=float(np.linalg.norm(v)),
                about_optical_x_deg=float(v[0]), about_optical_y_deg=float(v[1]), about_optical_z_deg=float(v[2]),
                point_at_1m_moves_m=float(np.linalg.norm((D@[0, 0, 1., 1])[:3]-[0, 0, 1.])))


# ------------------------------------------------------------------------ board
DICTIONARIES = ('DICT_APRILTAG_36h11', 'DICT_APRILTAG_25h9', 'DICT_4X4_50', 'DICT_5X5_100')


@dataclass
class Board:
    """Tag board: per-id corner coordinates (m, board frame, z = 0) in the
    detector's corner order (0 = the marker image's top-left, clockwise)."""
    name: str
    dictionary: str
    border_bits: int
    corners: dict
    source: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.dictionary not in DICTIONARIES or self.border_bits not in (1, 2):
            raise ValueError('CALIBRATION_BOARD_DICTIONARY_INVALID')
        self.corners = {int(k): np.asarray(v, float).reshape(4, 3) for k, v in self.corners.items()}
        if len(self.corners) < 4 or any(abs(c[:, 2]).max() > 1e-9 for c in self.corners.values()):
            raise ValueError('CALIBRATION_BOARD_NEEDS_FOUR_PLANAR_TAGS')

    @classmethod
    def from_extrinsics_settings(cls, settings, name='现场 AprilGrid（上次现场标定所用）'):
        layout = settings['marker_layout']; corners = settings['marker_corners_m']
        if settings.get('board_type') != 'aprilgrid' or len(layout) != len(corners):
            raise ValueError('CALIBRATION_BOARD_SETTINGS_INVALID')
        return cls(name, settings['dictionary'], int(settings.get('marker_border_bits', 1)),
                   {int(item[0]): c for item, c in zip(layout, corners)},
                   dict(kind='EXTRINSICS_SETTINGS', marker_m=settings.get('marker_m'), gap_m=settings.get('marker_gap_m')))

    @classmethod
    def grid(cls, rows, cols, marker_m, gap_m, dictionary='DICT_APRILTAG_36h11', first_id=0, border_bits=1, name=None):
        """Marker (r, c) has its lower-left corner at (c*pitch, r*pitch); printed
        upright (board +y = up on the print, +z out of the print)."""
        if not (1 <= rows <= 12 and 1 <= cols <= 12 and rows*cols >= 4 and .01 <= marker_m <= .3 and 0 <= gap_m <= .2):
            raise ValueError('CALIBRATION_BOARD_GRID_INVALID')
        pitch = marker_m+gap_m; corners = {}
        for r in range(rows):
            for c in range(cols):
                x0, y0 = c*pitch, r*pitch
                corners[first_id+r*cols+c] = [[x0, y0+marker_m, 0], [x0+marker_m, y0+marker_m, 0],
                                              [x0+marker_m, y0, 0], [x0, y0, 0]]
        return cls(name or f'{rows}×{cols} {dictionary} {marker_m*1000:.1f} mm', dictionary, border_bits, corners,
                   dict(kind='GRID', rows=rows, cols=cols, marker_m=marker_m, gap_m=gap_m, first_id=first_id))

    @classmethod
    def from_json(cls, path):
        raw = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if raw.get('board_type') == 'aprilgrid' and 'marker_corners_m' in raw:
            return cls.from_extrinsics_settings(raw, raw.get('name') or Path(path).stem)
        if raw.get('board_type') != 'grid':
            raise ValueError('CALIBRATION_BOARD_JSON_TYPE')
        return cls.grid(int(raw['rows']), int(raw['cols']), float(raw['marker_m']), float(raw['gap_m']),
                        raw.get('dictionary', 'DICT_APRILTAG_36h11'), int(raw.get('first_id', 0)),
                        int(raw.get('marker_border_bits', 1)), raw.get('name'))

    def to_dict(self):
        return dict(name=self.name, dictionary=self.dictionary, border_bits=self.border_bits, source=self.source,
                    corners={str(k): v.tolist() for k, v in sorted(self.corners.items())})

    @classmethod
    def from_dict(cls, d):
        return cls(d['name'], d['dictionary'], int(d['border_bits']), d['corners'], d.get('source', {}))

    @property
    def front_z(self):
        """+1 when the printed face looks along board +z, -1 along -z: the
        detector order (TL, TR, BR, BL) is clockwise seen from the front."""
        C = next(iter(self.corners.values()))[:, :2]
        area = sum(C[k, 0]*C[(k+1) % 4, 1]-C[(k+1) % 4, 0]*C[k, 1] for k in range(4))
        return 1 if area < 0 else -1

    def corner_names(self):
        return [f'tag{k}_c{j}' for k in sorted(self.corners) for j in range(4)]

    def corner(self, name):
        tag, c = name[3:].split('_c')
        return self.corners[int(tag)][int(c)]

    def extent(self):
        P = np.concatenate(list(self.corners.values()))
        return P.min(0), P.max(0)

    def cv_dictionary(self):
        import cv2
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))

    def marker_image(self, tag, pixels):
        import cv2
        return cv2.aruco.generateImageMarker(self.cv_dictionary(), int(tag), int(pixels), borderBits=self.border_bits)

    def printable(self, dpi=300, margin_m=.02):
        """Grey image of the board at exact scale (1 px = 25.4/dpi mm) with a 100 mm scale bar."""
        ppm = dpi/.0254; lo, hi = self.extent()
        W = int(round((hi[0]-lo[0]+2*margin_m)*ppm)); H = int(round((hi[1]-lo[1]+2*margin_m+.03)*ppm))
        img = np.full((H, W), 255, np.uint8)
        to_px = lambda p: self.print_px(p, dpi, margin_m)
        for tag, C in self.corners.items():
            self._paint(img, tag, np.array([to_px(p) for p in C]))
        y = int(round((hi[1]-lo[1]+2*margin_m+.015)*ppm)); x0 = int(round(margin_m*ppm))
        img[y:y+max(2, int(.002*ppm)), x0:x0+int(round(.1*ppm))] = 0
        return img

    def print_px(self, p, dpi=300, margin_m=.02):
        """Pixel of board point p on ``printable`` (seen from the printed face:
        x to the right for +z boards, mirrored for -z boards)."""
        ppm = dpi/.0254; lo, hi = self.extent()
        x = (p[0]-lo[0]+margin_m) if self.front_z > 0 else (hi[0]-p[0]+margin_m)
        return np.array([x*ppm, (hi[1]-p[1]+margin_m)*ppm])

    def _paint(self, img, tag, dst):
        """Warp the marker so that its image corner k lands on dst[k] (pixel-centre convention)."""
        import cv2
        n = 400; marker = self.marker_image(tag, n)
        src = np.array([[-.5, -.5], [n-.5, -.5], [n-.5, n-.5], [-.5, n-.5]], np.float32)
        H = cv2.getPerspectiveTransform(src, np.asarray(dst, np.float32))
        warped = cv2.warpPerspective(marker, H, (img.shape[1], img.shape[0]), flags=cv2.INTER_AREA,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        cover = cv2.warpPerspective(np.full_like(marker, 255), H, (img.shape[1], img.shape[0]), flags=cv2.INTER_NEAREST)
        img[cover > 0] = warped[cover > 0]

    def render(self, T_cam_board, K, size, background=90):
        """Synthetic camera image (tests): white board face, tags, grey surroundings."""
        W, H = size; img = np.full((H, W), background, np.uint8)
        lo, hi = self.extent(); m = .02
        outline = np.array([[lo[0]-m, lo[1]-m, 0], [hi[0]+m, lo[1]-m, 0], [hi[0]+m, hi[1]+m, 0], [lo[0]-m, hi[1]+m, 0]])
        import cv2
        cv2.fillConvexPoly(img, np.round(project(K, T_cam_board, outline)*16).astype(np.int32), 255, shift=4)
        for tag, C in self.corners.items():
            self._paint(img, tag, project(K, T_cam_board, C))
        return np.repeat(img[:, :, None], 3, axis=2)


def project(K, T_cam_x, P):
    P = np.asarray(P, float)@np.asarray(T_cam_x)[:3, :3].T+np.asarray(T_cam_x)[:3, 3]
    return (P[:, :2]/P[:, 2:3])@np.asarray(K)[:2, :2].T+np.asarray(K)[:2, 2]


def camera_matrix(intrinsics):
    k = intrinsics
    return np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1.]])


def distortion(intrinsics):
    """RealSense colour coefficients; D435 colour streams report zeros. Non-zero
    Brown-Conrady coefficients are passed to OpenCV in its order (k1,k2,p1,p2,k3)."""
    coeffs = [float(x) for x in (intrinsics.get('color_coeffs') or intrinsics.get('coeffs') or [0]*5)][:5]
    return np.array(coeffs+[0.]*(5-len(coeffs)))


# --------------------------------------------------------------------- detection
CORNER_ORDERS = [(s, rev) for rev in (False, True) for s in range(4)]


def detect(rgb, board):
    """Detected tags of this board: {tag: (4, 2) pixel corners in detector order}."""
    import cv2
    gray = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2GRAY)
    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = board.border_bits
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 4
    corners, ids, _ = cv2.aruco.ArucoDetector(board.cv_dictionary(), params).detectMarkers(gray)
    found = {}
    for c, i in zip(corners or (), (ids.ravel() if ids is not None else ())):
        if int(i) in board.corners and int(i) not in found:
            found[int(i)] = np.asarray(c, float).reshape(4, 2)
    return found


def correspondences(found, board, order=(0, False)):
    """Object (N,3) and image (N,2) points; ``order`` maps detector corner k to
    board corner (s+k)%4, reversed when the board file lists corners counter-clockwise."""
    s, rev = order; obj, img, names = [], [], []
    for tag in sorted(found):
        for k in range(4):
            j = (s+(-k if rev else k)) % 4
            obj.append(board.corners[tag][j]); img.append(found[tag][k]); names.append(f'tag{tag}_c{j}')
    return np.asarray(obj, float).reshape(-1, 3), np.asarray(img, float).reshape(-1, 2), names


def pnp(obj, img, K, dist):
    """Planar PnP: IPPE candidates refined by LM; (T_cam_board, rms px, ambiguity ratio)."""
    import cv2
    if len(obj) < 8:
        raise ValueError('CALIBRATION_TOO_FEW_CORNERS')
    obj32, img32 = obj.astype(np.float64), img.astype(np.float64)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj32, img32, K, dist, flags=cv2.SOLVEPNP_IPPE)
    best = []
    for r, t in zip(rvecs, tvecs):
        r, t = cv2.solvePnPRefineLM(obj32, img32, K, dist, r.copy(), t.copy())
        proj, _ = cv2.projectPoints(obj32, r, t, K, dist)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2)-img32)**2, 1))))
        best.append((rms, r.ravel(), t.ravel()))
    best.sort(key=lambda x: x[0])
    rms, r, t = best[0]
    if t[2] <= 0:
        raise ValueError('CALIBRATION_BOARD_BEHIND_CAMERA')
    ratio = best[1][0]/max(rms, 1e-9) if len(best) > 1 else math.inf
    return rigid(from_rotvec(r), t), rms, ratio


def choose_corner_order(samples, board, K, dist):
    """The board file's corner convention, decided once per session on the
    images: the order with the smallest median PnP error must be clearly best."""
    scores = []
    for order in CORNER_ORDERS:
        errs = []
        for s in samples:
            obj, img, _ = correspondences(s['found'], board, order)
            try:
                errs.append(pnp(obj, img, K, dist)[1])
            except Exception:
                errs.append(1e9)
        scores.append((float(np.median(errs)), order))
    scores.sort(key=lambda x: x[0])
    return scores[0][1], dict(best_median_px=scores[0][0], second_median_px=scores[1][0],
                              order=dict(shift=scores[0][1][0], reversed=scores[0][1][1]))


# ------------------------------------------------------------------- hand-eye
def axxb_pairs(A, B, min_angle_deg=3.0):
    pairs = []
    for i in range(len(A)):
        for j in range(i+1, len(A)):
            Ap = inv(A[j])@A[i]; Bp = inv(B[j])@B[i]
            if angle_deg(Ap[:3, :3]) >= min_angle_deg:
                pairs.append((Ap, Bp))
    return pairs


def solve_axxb(pairs):
    """Park-Martin: R_X from the rotation vectors, t_X by least squares."""
    if len(pairs) < 2:
        raise ValueError('CALIBRATION_POSES_NOT_DIVERSE')
    alpha = np.array([rotvec(a[:3, :3]) for a, _ in pairs]); beta = np.array([rotvec(b[:3, :3]) for _, b in pairs])
    R, _ = kabsch_origin(beta, alpha)
    M = np.concatenate([a[:3, :3]-np.eye(3) for a, _ in pairs])
    v = np.concatenate([R@b[:3, 3]-a[:3, 3] for a, b in pairs])
    t = np.linalg.lstsq(M, v, rcond=None)[0]
    return rigid(R, t)


def kabsch_origin(P, Q):
    U, _, Vt = np.linalg.svd(np.asarray(P).T@np.asarray(Q))
    D = np.diag([1., 1., np.sign(np.linalg.det(Vt.T@U.T))])
    return Vt.T@D@U.T, None


def rotation_spread_deg(A):
    """Second singular value (deg) of the relative-rotation vectors: both
    rotation axes of AX=XB need motion about at least two different axes."""
    vs = [rotvec((inv(A[j])@A[i])[:3, :3]) for i in range(len(A)) for j in range(i+1, len(A))]
    if len(vs) < 2:
        return 0., 0.
    s = np.linalg.svd(np.array(vs), compute_uv=False)/math.sqrt(len(vs))
    return float(np.degrees(s[0])), float(np.degrees(s[1]))


def _pack(*Ts):
    return np.concatenate([np.r_[rotvec(T[:3, :3]), T[:3, 3]] for T in Ts])


def _unpack(x, n):
    return [rigid(from_rotvec(x[6*k:6*k+3]), x[6*k+3:6*k+6]) for k in range(n)]


def _residuals(Y, X, A, data, K, dist):
    import cv2
    out = []
    for Ai, (obj, img) in zip(A, data):
        T = inv(Y)@Ai@X
        proj, _ = cv2.projectPoints(obj, rotvec(T[:3, :3]), T[:3, 3], K, dist)
        out.append((proj.reshape(-1, 2)-img).ravel())
    return np.concatenate(out)


def refine(Y0, X0, A, data, K, dist, fixed_X=False):
    """Joint Huber refinement on corner reprojection; returns Y, X, covariance of Y (6x6)."""
    from scipy.optimize import least_squares
    if fixed_X:
        f = lambda x: _residuals(_unpack(x, 1)[0], X0, A, data, K, dist)
        x0 = _pack(Y0)
    else:
        f = lambda x: _residuals(*_unpack(x, 2), A, data, K, dist)
        x0 = _pack(Y0, X0)
    sol = least_squares(f, x0, loss='huber', f_scale=1.0, x_scale='jac', max_nfev=200)
    Ts = _unpack(sol.x, 1 if fixed_X else 2)
    Y = Ts[0]; X = X0 if fixed_X else Ts[1]
    r = sol.fun; dof = max(1, len(r)-len(sol.x))
    sigma2 = float(r@r)/dof
    try:
        cov = np.linalg.inv(sol.jac.T@sol.jac)*sigma2
    except np.linalg.LinAlgError:
        cov = np.full((len(sol.x),)*2, np.inf)
    return Y, X, cov[:6, :6], sol


def pivot_tip(groups):
    """Tip on link_6 from touches of one fixed point in several orientations
    (the usual four-point TCP method): R_i t + p_i = c_g for every group g."""
    rows, rhs, n = [], [], len(groups)
    for g, Ts in enumerate(groups):
        for T in Ts:
            M = np.zeros((3, 3+3*n)); M[:, :3] = T[:3, :3]; M[:, 3+3*g:6+3*g] = -np.eye(3)
            rows.append(M); rhs.append(-T[:3, 3])
    sol = np.linalg.lstsq(np.concatenate(rows), np.concatenate(rhs), rcond=None)[0]
    return sol[:3]


def _touch_solve(usable, data, B, board, touches, tip_link6_m, base_link1, base_link6, K, dist):
    """TIP_TOUCH: board pose per placement from the tip touches, the tip itself
    when not given (pivot touches: one corner touched in several tool
    orientations), then one joint Huber fit of Y, every board pose and the tip
    on all corner reprojections (px) and all touch residuals (mm)."""
    from scipy.optimize import least_squares
    th = THRESHOLDS
    touches = [(t[0], np.asarray(t[1], float), int(t[2]) if len(t) > 2 else 1) for t in (touches or [])]
    places = sorted({p for _, _, p in touches})
    photo_place = [int(s.get('placement', 1)) for s in usable]
    if not touches or not places:
        raise ValueError('CALIBRATION_TOO_FEW_TOUCHES')
    if set(photo_place)-set(places):
        raise ValueError('CALIBRATION_PHOTO_PLACEMENT_WITHOUT_TOUCHES')
    for p in places:
        if sum(1 for _, _, q in touches if q == p) < 3:
            raise ValueError('CALIBRATION_TOO_FEW_TOUCHES')
    F6 = [np.asarray(base_link6(q), float) for _, q, _ in touches]
    auto = tip_link6_m is None
    spread = None
    if auto:
        groups = {}
        for (name, _, p), T in zip(touches, F6):
            groups.setdefault((p, name), []).append(T)
        pivots = [Ts for Ts in groups.values() if len(Ts) >= 3]
        if not pivots:
            raise ValueError('CALIBRATION_TIP_NEEDS_PIVOT_TOUCHES')
        tip0 = pivot_tip(pivots)
        spread = max(rotation_spread_deg(Ts)[1] for Ts in pivots)
    else:
        tip0 = np.asarray(tip_link6_m, float)
    boards0 = {}
    for p in places:
        idx = [i for i, t in enumerate(touches) if t[2] == p]
        Pb = np.array([board.corner(touches[i][0]) for i in idx])
        if np.linalg.matrix_rank(Pb-Pb.mean(0), tol=.02) < 2:
            raise ValueError('CALIBRATION_TOUCHES_COLLINEAR')
        R, t = kabsch(Pb, np.array([(F6[i]@np.r_[tip0, 1])[:3] for i in idx]))
        boards0[p] = rigid(R, t)
    L1 = [np.asarray(base_link1(s['q']), float) for s in usable]
    Ys = [inv(l)@boards0[p]@inv(b) for l, p, b in zip(L1, photo_place, B)]
    Y0 = rigid(mean_rotation([y[:3, :3] for y in Ys]), np.mean([y[:3, 3] for y in Ys], axis=0))
    corner_pts = np.array([board.corner(n) for n, _, _ in touches])
    pi = {p: k for k, p in enumerate(places)}

    def unpack(x):
        Y = _unpack(x[:6], 1)[0]
        boards = {p: _unpack(x[6+6*k:12+6*k], 1)[0] for p, k in pi.items()}
        tip = x[6+6*len(places):] if auto else tip0
        return Y, boards, tip

    def residuals(x, keep):
        Y, boards, tip = unpack(x)
        A = [inv(L1[i])@boards[photo_place[i]] for i in keep]
        r = [_residuals(Y, np.eye(4), A, [data[i] for i in keep], K, dist)] if keep else []
        for (name, _, p), T, c in zip(touches, F6, corner_pts):
            r.append(1000*((T@np.r_[tip, 1])[:3]-(boards[p]@np.r_[c, 1])[:3]))
        return np.concatenate(r)

    x0 = np.concatenate([_pack(Y0)]+[_pack(boards0[p]) for p in places]+([tip0] if auto else []))

    def fit(keep, start):
        return least_squares(lambda x: residuals(x, keep), start, loss='huber', f_scale=1.0, x_scale='jac', max_nfev=400)
    sol = fit(list(range(len(usable))), x0)
    Y, boards, tip = unpack(sol.x)
    r = sol.fun; sigma2 = float(r@r)/max(1, len(r)-len(sol.x))
    try:
        cov = np.linalg.inv(sol.jac.T@sol.jac)*sigma2
    except np.linalg.LinAlgError:
        cov = np.full((len(sol.x),)*2, np.inf)
    res = np.array([np.linalg.norm((T@np.r_[tip, 1])[:3]-(boards[p]@np.r_[c, 1])[:3]) for (n, _, p), T, c in zip(touches, F6, corner_pts)])
    A = [inv(l)@boards[p] for l, p in zip(L1, photo_place)]
    tip_std = float(np.sqrt(np.clip(np.trace(cov[-3:, -3:]), 0, None))) if auto else 0.
    extra = dict(
        tip=dict(mode='AUTO' if auto else 'GIVEN', link6_m=np.asarray(tip, float).tolist(), initial_link6_m=np.asarray(tip0).tolist(),
                 std_m=tip_std, pivot_rotation_spread_deg=spread),
        touch_registration=dict(rms_m=float(np.sqrt(np.mean(res**2))), max_m=float(res.max()), count=len(touches)),
        placements=[dict(placement=p, board_pose_base=boards[p].tolist(), touches=sum(1 for t in touches if t[2] == p),
                         distinct_corners=len({t[0] for t in touches if t[2] == p}), photos=photo_place.count(p))
                    for p in places],
        touch_residuals_m=[dict(corner=n, placement=p, error_m=float(e)) for (n, _, p), e in zip(touches, res)])

    def loo(k):
        keep = [i for i in range(len(usable)) if i != k]
        Yk, bk, _ = unpack(fit(keep, sol.x).x)
        return per_sample_rms(Yk, np.eye(4), [inv(L1[k])@bk[photo_place[k]]], [data[k]], K, dist)[0], Yk
    return Y, A, cov[:6, :6], sol, extra, loo


def per_sample_rms(Y, X, A, data, K, dist):
    out = []
    for Ai, d in zip(A, data):
        r = _residuals(Y, X, [Ai], [d], K, dist).reshape(-1, 2)
        out.append(float(np.sqrt(np.mean(np.sum(r**2, 1)))))
    return out


# --------------------------------------------------------------- depth check
def depth_check(sample, T_cam_board, board):
    """D435 depth at the tag centres against the geometric prediction of the
    solved camera; a diagnostic (the solve never uses depth)."""
    depth = sample.get('depth'); scale = sample.get('depth_scale_m'); K = sample['K']
    if depth is None or not scale:
        return None
    rows = []
    for tag in sample['found']:
        c = board.corners[tag].mean(0)
        p = T_cam_board[:3, :3]@c+T_cam_board[:3, 3]
        u, v = (K[:2, :2]@(p[:2]/p[2])+K[:2, 2])
        ui, vi = int(round(u)), int(round(v))
        if not (3 <= ui < depth.shape[1]-3 and 3 <= vi < depth.shape[0]-3):
            continue
        patch = depth[vi-3:vi+4, ui-3:ui+4].astype(float).ravel()
        patch = patch[patch > 0]
        if len(patch) >= 10:
            rows.append((float(p[2]), float(np.median(patch))*scale))
    if not rows:
        return None
    z = np.array(rows)
    return dict(tags=len(rows), predicted_z_m=float(z[:, 0].mean()), median_error_m=float(np.median(z[:, 1]-z[:, 0])))


# --------------------------------------------------------------------- session
class Session:
    """One calibration session on disk: session.json + samples/<n>.{png,npy,json}."""

    def __init__(self, folder, data):
        self.folder = Path(folder); self.data = data

    @classmethod
    def create(cls, root, mode, board, tool_matrix, model_sha256, tip_link6_m=None, basis=''):
        if mode not in MODES:
            raise ValueError('CALIBRATION_MODE_INVALID')
        if mode == 'TIP_TOUCH' and tip_link6_m is not None:      # None: solved from pivot touches
            tip = np.asarray(tip_link6_m, float)
            if tip.shape != (3,) or not np.all(np.isfinite(tip)) or np.linalg.norm(tip) > .6:
                raise ValueError('CALIBRATION_TIP_OFFSET_INVALID')
        ident = time.strftime('%Y%m%d-%H%M%S')+'-'+hashlib.sha256(str(time.time_ns()).encode()).hexdigest()[:6]
        folder = Path(root)/'calibration'/ident
        (folder/'samples').mkdir(parents=True)
        data = dict(schema='eco65_tcp_calibration_session_v1', id=ident, mode=mode, board=board.to_dict(),
                    tool_matrix_link6_tcp=None if tool_matrix is None else np.asarray(tool_matrix, float).tolist(),
                    tip_link6_m=None if tip_link6_m is None else [float(x) for x in tip_link6_m],
                    model_sha256=model_sha256, created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                    hardware_motion_commanded=False, basis=basis, photos=[], touches=[], placement=1)
        s = cls(folder, data); s.save()
        return s

    @classmethod
    def load(cls, folder):
        return cls(folder, json.loads((Path(folder)/'session.json').read_text()))

    def save(self):
        tmp = self.folder/'session.json.tmp'
        tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=1)); tmp.replace(self.folder/'session.json')

    @property
    def board(self):
        return Board.from_dict(self.data['board'])

    def add_photo(self, rgb, depth, depth_scale_m, intrinsics, q_rad, evidence):
        import cv2
        n = len(self.data['photos'])+1; name = f'{n:03d}'
        found = detect(rgb, self.board)
        cv2.imwrite(str(self.folder/'samples'/f'{name}.png'), np.asarray(rgb)[:, :, ::-1])
        if depth is not None:
            np.save(self.folder/'samples'/f'{name}.depth.npy', np.asarray(depth))
        item = dict(name=name, q_rad=[float(x) for x in q_rad], intrinsics=intrinsics, depth_scale_m=depth_scale_m,
                    tags=sorted(found), corners=int(4*len(found)), evidence=evidence, active=True,
                    placement=self.placement)
        self.data['photos'].append(item); self.save()
        return item

    def add_touch(self, corner_name, q_rad, evidence):
        self.board.corner(corner_name)              # validates the name
        item = dict(corner=corner_name, q_rad=[float(x) for x in q_rad], evidence=evidence, active=True,
                    placement=self.placement)
        self.data['touches'].append(item); self.save()
        return item

    @property
    def placement(self):
        return int(self.data.get('placement', 1))

    def next_placement(self):
        """The board was moved: later touches and photos belong to a new board pose."""
        if not any(t.get('placement', 1) == self.placement for t in self.data['touches']):
            raise ValueError('CALIBRATION_PLACEMENT_HAS_NO_TOUCHES')
        self.data['placement'] = self.placement+1; self.save()
        return self.placement

    def drop_last(self, kind='photos'):
        items = self.data[kind]
        if items:
            items.pop(); self.save()

    def photo_samples(self):
        import cv2
        out = []
        for p in self.data['photos']:
            if not p.get('active', True):
                continue
            rgb = cv2.imread(str(self.folder/'samples'/f"{p['name']}.png"))[:, :, ::-1]
            dpath = self.folder/'samples'/f"{p['name']}.depth.npy"
            out.append(dict(name=p['name'], q=np.asarray(p['q_rad'], float), K=camera_matrix(p['intrinsics']),
                            dist=distortion(p['intrinsics']), found=detect(rgb, self.board),
                            depth=np.load(dpath) if dpath.is_file() else None, depth_scale_m=p.get('depth_scale_m'),
                            placement=int(p.get('placement', 1))))
        return out

    def fingerprint(self):
        h = hashlib.sha256(json.dumps(self.data, sort_keys=True).encode())
        for f in sorted((self.folder/'samples').glob('*')):
            h.update(f.name.encode()); h.update(hashlib.sha256(f.read_bytes()).digest())
        return h.hexdigest()


# ----------------------------------------------------------------------- solve
def solve(samples, board, mode, link1_tcp=None, base_link1=None, touches=None, tip_link6_m=None, base_link6=None):
    """Core solve on prepared samples (each: q, K, dist, found[, depth]).

    BOARD_ON_TOOL needs ``link1_tcp(q) -> T_link1_tcp``; TIP_TOUCH needs
    ``base_link1(q)``, ``base_link6(q)``, touches [(corner_name, q)] and the tip.
    Returns (Y, report) where report['checks'] holds every check and 'passed'.
    """
    th = THRESHOLDS
    if not samples:
        raise ValueError('CALIBRATION_NO_SAMPLES')
    K, dist = samples[0]['K'], samples[0]['dist']
    if any(not np.allclose(s['K'], K) for s in samples):
        raise ValueError('CALIBRATION_INTRINSICS_CHANGED_DURING_SESSION')
    usable = [s for s in samples if 4*len(s['found']) >= th['min_corners_per_sample']]
    skipped = [dict(name=s['name'], corners=4*len(s['found'])) for s in samples if s not in usable]
    if len(usable) < 2:
        raise ValueError('CALIBRATION_TOO_FEW_USABLE_SAMPLES')
    order, order_info = choose_corner_order(usable, board, K, dist)
    data, B, pnp_info = [], [], []
    for s in usable:
        obj, img, _ = correspondences(s['found'], board, order)
        T, rms, ratio = pnp(obj, img, K, dist)
        data.append((obj, img)); B.append(T); pnp_info.append(dict(name=s['name'], rms_px=rms, ambiguity_ratio=ratio,
                                                                     distance_m=float(np.linalg.norm(T[:3, 3]))))
    report = dict(mode=mode, samples=len(samples), usable=len(usable), skipped=skipped, corner_order=order_info,
                  pnp=pnp_info, K=K.tolist(), distortion=dist.tolist())
    if mode == 'BOARD_ON_TOOL':
        A = [np.asarray(link1_tcp(s['q']), float) for s in usable]
        pairs = axxb_pairs(A, B)
        X0 = solve_axxb(pairs)
        Y0 = rigid(mean_rotation([(a@X0@inv(b))[:3, :3] for a, b in zip(A, B)]),
                   np.mean([(a@X0@inv(b))[:3, 3] for a, b in zip(A, B)], axis=0))
        Y, X, covY, sol = refine(Y0, X0, A, data, K, dist)
        spread = rotation_spread_deg(A)
        report.update(X_tcp_board=X.tolist(), rotation_spread_deg=list(spread), pairs=len(pairs))

        def loo(k):
            keep = [i for i in range(len(A)) if i != k]
            Yk, Xk, _, _ = refine(Y, X, [A[i] for i in keep], [data[i] for i in keep], K, dist)
            return per_sample_rms(Yk, Xk, [A[k]], [data[k]], K, dist)[0], Yk
    else:
        Y, A, covY, sol, extra, loo = _touch_solve(usable, data, B, board, touches, tip_link6_m, base_link1, base_link6, K, dist)
        X = np.eye(4)
        report.update(extra)
    rms = per_sample_rms(Y, X, A, data, K, dist)
    report['_overlay'] = [(s['name'], img, img+_residuals(Y, X, [a], [(obj, img)], K, dist).reshape(-1, 2))
                          for s, a, (obj, img) in zip(usable, A, data)]
    allr = _residuals(Y, X, A, data, K, dist).reshape(-1, 2)
    overall = float(np.sqrt(np.mean(np.sum(allr**2, 1))))
    consistency = [difference(Y, a@X@inv(b)) for a, b in zip(A, B)]
    loos = [loo(k) for k in range(len(A))] if len(A) >= 3 else []
    std = np.sqrt(np.clip(np.diag(covY), 0, None))
    depth = [depth_check(s, inv(Y)@a@X, board) for s, a in zip(usable, A)]
    for info, r, c, d, l in zip(pnp_info, rms, consistency, depth, loos or [None]*len(A)):
        info.update(model_rms_px=r, consistency_m=c['translation_m'], consistency_deg=c['rotation_deg'], depth=d,
                    leave_one_out_px=None if l is None else l[0])
    checks = dict(
        samples=dict(value=len(usable), limit=th['min_samples'][mode], ok=len(usable) >= th['min_samples'][mode]),
        rms_px=dict(value=overall, limit=th['rms_px'], ok=overall <= th['rms_px']),
        sample_rms_px=dict(value=max(rms), limit=th['sample_rms_px'], ok=max(rms) <= th['sample_rms_px']),
        consistency_m=dict(value=max(c['translation_m'] for c in consistency), limit=th['consistency_m'],
                           ok=max(c['translation_m'] for c in consistency) <= th['consistency_m']),
        consistency_deg=dict(value=max(c['rotation_deg'] for c in consistency), limit=th['consistency_deg'],
                             ok=max(c['rotation_deg'] for c in consistency) <= th['consistency_deg']),
        leave_one_out_px=dict(value=max((l[0] for l in loos), default=None), limit=th['leave_one_out_px'],
                              ok=bool(loos) and max(l[0] for l in loos) <= th['leave_one_out_px']),
        translation_std_m=dict(value=float(np.linalg.norm(std[3:6])), limit=th['max_translation_std_m'],
                               ok=float(np.linalg.norm(std[3:6])) <= th['max_translation_std_m']),
        rotation_std_deg=dict(value=float(np.degrees(np.linalg.norm(std[:3]))), limit=th['max_rotation_std_deg'],
                              ok=float(np.degrees(np.linalg.norm(std[:3]))) <= th['max_rotation_std_deg']),
        corner_order=dict(value=order_info['best_median_px'], limit=2.0,
                          ok=order_info['best_median_px'] <= 2.0 and order_info['second_median_px'] >= 3*max(order_info['best_median_px'], .2)))
    if mode == 'BOARD_ON_TOOL':
        checks['rotation_spread_deg'] = dict(value=report['rotation_spread_deg'][1], limit=th['min_rotation_spread_deg'],
                                             ok=report['rotation_spread_deg'][1] >= th['min_rotation_spread_deg'])
    else:
        tr = report['touch_registration']
        checks['touch_rms_m'] = dict(value=tr['rms_m'], limit=th['touch_rms_m'], ok=tr['rms_m'] <= th['touch_rms_m'])
        per = [p['touches'] for p in report['placements']]; corners = [p['distinct_corners'] for p in report['placements']]
        checks['touches_per_placement'] = dict(value=min(per), limit=th['min_touches'], ok=min(per) >= th['min_touches'])
        checks['distinct_corners_per_placement'] = dict(value=min(corners), limit=4, ok=min(corners) >= 4)
        if report['tip']['mode'] == 'AUTO':
            checks['pivot_rotation_spread_deg'] = dict(value=report['tip']['pivot_rotation_spread_deg'],
                                                       limit=th['min_pivot_spread_deg'],
                                                       ok=report['tip']['pivot_rotation_spread_deg'] >= th['min_pivot_spread_deg'])
            checks['tip_std_m'] = dict(value=report['tip']['std_m'], limit=th['max_tip_std_m'],
                                       ok=report['tip']['std_m'] <= th['max_tip_std_m'])
    report.update(Y_link1_camera=Y.tolist(), reprojection_rms_px=overall, std_rotvec_deg=np.degrees(std[:3]).tolist(),
                  std_translation_m=std[3:6].tolist(), optimizer=dict(status=int(sol.status), nfev=int(sol.nfev)),
                  leave_one_out_spread=[difference(Y, l[1]) for l in loos], checks=checks,
                  passed=all(c['ok'] for c in checks.values()), thresholds=th,
                  threshold_basis='IMPLEMENTER_VALUES_2026_11_27_CONSISTENCY_AND_TOUCH_FROM_PREVIOUS_FIELD_SETTINGS')
    return Y, report


def solve_session(session, frames, reference=None):
    """Solve a stored session; ``frames(q)`` is CoordinateModel.frames (base_link
    frames). Writes extrinsics(.rejected).json + report.json; returns (path, report)."""
    d = session.data; board = session.board
    if d['mode'] == 'BOARD_ON_TOOL':
        tool = np.asarray(d['tool_matrix_link6_tcp'] if d.get('tool_matrix_link6_tcp') is not None else np.eye(4), float)
        f = lambda q: inv(frames(q)['link_1'])@frames(q)['link_6']@tool
        Y, report = solve(session.photo_samples(), board, 'BOARD_ON_TOOL', link1_tcp=f)
    else:
        touches = [(t['corner'], np.asarray(t['q_rad'], float), int(t.get('placement', 1)))
                   for t in d['touches'] if t.get('active', True)]
        Y, report = solve(session.photo_samples(), board, 'TIP_TOUCH', base_link1=lambda q: frames(q)['link_1'],
                          base_link6=lambda q: frames(q)['link_6'], touches=touches, tip_link6_m=d['tip_link6_m'])
    if reference is not None:
        report['against_reference'] = dict(path=str(reference.get('path')), sha256=reference.get('sha256'),
                                           **difference(np.asarray(reference['matrix'], float), Y))
    out = dict(schema=SCHEMA, status=STATUS_PASS if report['passed'] else STATUS_FAIL, usage=USAGE,
               parent_frame='link_1', child_frame='camera_optical', frame_from='camera', frame_to='link_1',
               direction='T_parent_child maps camera_optical into parent', unit='m', matrix=Y.tolist(),
               model_sha256=d['model_sha256'], method=d['mode'], session_file='session.json',
               observation_fingerprint=session.fingerprint(), created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
               hardware_execution=False, hardware_motion_commanded=False,
               settings=dict(board=board.to_dict(), K=report['K'], distortion=report['distortion'],
                             tip_link6_m=d.get('tip_link6_m'), tip=report.get('tip'), placements=report.get('placements'), tool_matrix_link6_tcp=d.get('tool_matrix_link6_tcp')),
               training={k: report[k] for k in ('reprojection_rms_px', 'checks', 'pnp', 'corner_order', 'std_rotvec_deg',
                                                 'std_translation_m', 'touch_registration') if k in report},
               against_reference=report.get('against_reference'))
    overlay = report.pop('_overlay', [])
    write_overlays(session.folder, overlay)
    name = 'extrinsics.json' if report['passed'] else 'extrinsics.rejected.json'
    for stale in ('extrinsics.json', 'extrinsics.rejected.json'):
        (session.folder/stale).unlink(missing_ok=True)
    path = session.folder/name
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    (session.folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float))
    return path, report


def write_overlays(folder, overlay, scale=8.0):
    """overlays/<name>.png: detected corners (green) and the solved model's
    projections (red), residual arrows magnified ``scale`` times."""
    import cv2
    out = Path(folder)/'overlays'; out.mkdir(exist_ok=True)
    for name, det, proj in overlay:
        img = cv2.imread(str(Path(folder)/'samples'/f'{name}.png'))
        if img is None:
            continue
        for a, b in zip(det, proj):
            pa = tuple(np.round(a).astype(int)); pb = tuple(np.round(a+(b-a)*scale).astype(int))
            cv2.circle(img, pa, 3, (60, 220, 60), 1); cv2.line(img, pa, pb, (40, 40, 230), 1)
        r = float(np.sqrt(np.mean(np.sum((proj-det)**2, 1))))
        cv2.putText(img, f'{name}  RMS {r:.2f} px  (red = residual x{scale:g})', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
        cv2.imwrite(str(out/f'{name}.png'), img)


def field_config_with(config_path, extrinsics_path, out_path):
    """A copy of the field configuration that uses the new extrinsics (the
    running application keeps its file until it is restarted with the copy)."""
    raw = json.loads(Path(config_path).read_text(encoding='utf-8-sig'))
    ext = json.loads(Path(extrinsics_path).read_text())
    if ext.get('status') != STATUS_PASS or ext.get('schema') != SCHEMA:
        raise ValueError('CALIBRATION_RESULT_NOT_USABLE')
    raw['camera_extrinsics'] = str(Path(extrinsics_path).resolve())
    raw['camera_extrinsics_basis'] = (f'2026-11-27 TCP recalibration ({ext["method"]}), fingerprint '
                                      f'{ext["observation_fingerprint"][:16]}, operator review required; '
                                      f'previous file {json.loads(Path(config_path).read_text(encoding="utf-8-sig")).get("camera_extrinsics")}')
    Path(out_path).write_text(json.dumps(raw, ensure_ascii=False, indent=1))
    return out_path
