"""
Geometry helpers — the only file that imports shapely.

Keeps geometric truth in one place (the grant's 'single rule-evaluation code
path' invariant, scaled down).
"""
from __future__ import annotations

from shapely.geometry import Point, Polygon

from .models import PolygonFence


def fence_polygon(fence: PolygonFence) -> Polygon:
    return Polygon([(v.x, v.y) for v in fence.vertices])


def point_in_fence(x: float, y: float, up: float, fence: PolygonFence,
                   poly: Polygon | None = None) -> bool:
    """True when the point violates the fence: inside the (margin-buffered)
    polygon AND within its altitude band."""
    if not (fence.altitude_floor_m <= up <= fence.altitude_ceiling_m):
        return False
    poly = poly if poly is not None else fence_polygon(fence)
    return poly.buffer(fence.margin_m).contains(Point(x, y))


def push_out_direction(x: float, y: float, poly: Polygon) -> tuple[float, float]:
    """Unit vector pointing from (x, y) toward the nearest way OUT of poly.
    Used by the repair operator to know which velocity component to remove."""
    p = Point(x, y)
    nearest = poly.exterior.interpolate(poly.exterior.project(p))
    dx, dy = x - nearest.x, y - nearest.y          # boundary -> point vector
    norm = (dx * dx + dy * dy) ** 0.5

    if norm < 1e-9:
        # Dead on the boundary: fall back to "away from centroid".
        c = poly.centroid
        dx, dy = x - c.x, y - c.y
        norm = (dx * dx + dy * dy) ** 0.5 or 1.0
        return dx / norm, dy / norm

    if poly.contains(p):
        # Inside: the way out is TOWARD the nearest boundary point.
        return -dx / norm, -dy / norm
    # Outside: away from the polygon.
    return dx / norm, dy / norm
