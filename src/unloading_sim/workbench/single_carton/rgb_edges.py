"""One RGB edge detector for coplanar-boundary checks and the seam policy.

RGB edges never become depth evidence. They are used only (1) to accept a
mask boundary that cuts a continuing plane when a real colour/brightness
break is there, and (2) by the explicit, user-selected coplanar seam policy.
"""
import cv2
import numpy as np

CANNY_LOW = 30
CANNY_HIGH = 80
BAND_PX = 3


def edge_map(rgb):
    """Canny edges and Sobel gradient magnitude of the grey image."""
    gray = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH) > 0
    g = gray.astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    return edges, np.hypot(gx, gy), gx, gy


def distance_to_edges(edges):
    return cv2.distanceTransform((~edges).astype(np.uint8), cv2.DIST_L2, 3)


def mask_boundary(mask):
    m = np.asarray(mask, bool)
    return m & ~cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8), borderType=cv2.BORDER_CONSTANT,
                          borderValue=0).astype(bool)


def boundary_edge_coverage(rgb, mask, region, band_px=BAND_PX):
    """Share of mask-boundary pixels next to ``region`` with an RGB edge within band_px.

    The band lies on both sides of the boundary, so a mask that is one or two
    pixels off the true break still finds it; a cut through a uniform surface
    has no edge anywhere in the band.
    """
    edges, *_ = edge_map(rgb)
    near = cv2.dilate(np.asarray(region, bool).astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    boundary = mask_boundary(mask) & near
    if not boundary.any():
        return dict(boundary_pixels=0, covered_pixels=0, fraction=0., band_px=band_px)
    distance = distance_to_edges(edges)
    covered = int((distance[boundary] <= band_px).sum())
    return dict(boundary_pixels=int(boundary.sum()), covered_pixels=covered,
                fraction=covered/float(boundary.sum()), band_px=band_px,
                detector=dict(canny=[CANNY_LOW, CANNY_HIGH]))
