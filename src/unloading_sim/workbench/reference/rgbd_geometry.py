"""Selected RGBD validation from the pinned reference."""
from dataclasses import dataclass
import numpy as np
@dataclass
class RGBDInput:
    rgb: np.ndarray
    points_m: np.ndarray
    base_from_camera: np.ndarray
    source: str
    virtual_pose: bool = True


def validate_input(rgb, points_m, transform, source, virtual_pose=True):
    rgb, points = np.asarray(rgb), np.asarray(points_m, np.float32)
    transform = np.asarray(transform, float)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('彩色图须为 H×W×3 的 uint8 RGB 图像')
    if points.shape != rgb.shape or max(rgb.shape[:2]) > 4096 or min(rgb.shape[:2]) < 16:
        raise ValueError('彩色图和三维数据须同分辨率，边长为 16～4096 像素')
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-5)):
        raise ValueError('T_base_camera 必须是有效的刚体变换矩阵')
    valid = np.isfinite(points).all(axis=-1) & (points[..., 2] > 0)
    if valid.sum() < 12 or np.median(points[..., 2][valid]) > 10:
        raise ValueError('缺少有效度量深度；points_m/depth_m 必须以米为单位')
    points = points.copy()
    points[~valid] = 0.
    return RGBDInput(rgb.copy(), points, transform.copy(), source, virtual_pose)
