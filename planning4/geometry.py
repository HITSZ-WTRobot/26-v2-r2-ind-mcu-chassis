"""Shared field geometry, hard safe corridors, and footprint checks.

The planner intentionally routes the inflated chassis through a small set of
certified convex corridors.  Each corridor has an enclosing free rectangle that
contains the entire inflated footprint for every center/yaw state in that
corridor.  Because the state interpolation used for exported trajectories is
linear, a segment whose endpoints stay in one corridor is also in that corridor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from config import (
    HALF_L_EXPANDED,
    HALF_W_EXPANDED,
    RECT_OBSTACLES,
    rect_obstacles_for_trajectory,
    THIN_OBS_X_MAX,
    THIN_OBS_X_MIN,
    THIN_OBS_Y_MAX,
    THIN_OBS_Y_MIN,
    ZONE1,
    ZONE2,
    ZONE3,
    ZONE4,
)


EPS_CLEARANCE = 1e-4


@dataclass(frozen=True)
class Rect:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains_open(self, x: float, y: float, eps: float = EPS_CLEARANCE) -> bool:
        return (
            self.x_min + eps < x < self.x_max - eps
            and self.y_min + eps < y < self.y_max - eps
        )


@dataclass(frozen=True)
class SafeCorridor:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    yaw_min_deg: float
    yaw_max_deg: float
    certificate: Rect
    requires_h_03: bool = False

    def contains_state(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        eps: float = EPS_CLEARANCE,
    ) -> bool:
        return (
            self.x_min - eps <= x <= self.x_max + eps
            and self.y_min - eps <= y <= self.y_max + eps
            and self.yaw_min_deg - eps <= yaw_deg <= self.yaw_max_deg + eps
        )


FREE_CERT_LEFT_TOP = Rect("left_top_clear", 5.60, 9.30, 4.63, 6.00)
FREE_CERT_ZONE2_TOP = Rect("zone2_top_clear", 8.00, 12.00, 4.63, 6.00)
FREE_CERT_RIGHT_UPPER = Rect("right_upper_clear", 10.80, 12.00, 3.40, 6.00)
FREE_CERT_RIGHT_NO_OBS2 = Rect("right_no_obstacle2_clear", 10.80, 12.00, 2.70, 6.00)
FREE_CERT_UPPER_MID = Rect("zone3_upper_mid_clear", 9.50, 12.00, 3.40, 4.47)
FREE_CERT_MID_NO_OBS2 = Rect("zone3_no_obstacle2_mid_clear", 9.50, 12.00, 2.70, 4.47)
FREE_CERT_MID_CLEAR = Rect("zone3_mid_clear", 9.50, 11.65, 2.70, 4.47)
FREE_CERT_LOW_LEFT = Rect("zone3_low_left_clear", 9.50, 11.65, 0.20, 4.47)


CORRIDORS: dict[str, SafeCorridor] = {
    "zone4_approach": SafeCorridor(
        "zone4_approach",
        x_min=6.40,
        x_max=8.50,
        y_min=5.35,
        y_max=5.45,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_LEFT_TOP,
    ),
    "zone2_top0": SafeCorridor(
        "zone2_top0",
        x_min=8.50,
        x_max=11.43,
        y_min=5.35,
        y_max=5.45,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_ZONE2_TOP,
        requires_h_03=True,
    ),
    "right_descend": SafeCorridor(
        "right_descend",
        x_min=11.24,
        x_max=11.55,
        y_min=3.90,
        y_max=5.45,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_RIGHT_UPPER,
        requires_h_03=True,
    ),
    "finish": SafeCorridor(
        "finish",
        x_min=10.95,
        x_max=11.20,
        y_min=3.227,
        y_max=3.95,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_MID_CLEAR,
    ),
    "mid_left": SafeCorridor(
        "mid_left",
        x_min=10.95,
        x_max=11.30,
        y_min=3.90,
        y_max=3.95,
        yaw_min_deg=0.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_UPPER_MID,
    ),
    "right_down0": SafeCorridor(
        "right_down0",
        x_min=11.34,
        x_max=11.43,
        y_min=3.235,
        y_max=5.45,
        yaw_min_deg=-90.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_RIGHT_NO_OBS2,
    ),
    "upper_turn_left": SafeCorridor(
        "upper_turn_left",
        x_min=10.75,
        x_max=11.43,
        y_min=3.235,
        y_max=3.238,
        yaw_min_deg=-90.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_MID_NO_OBS2,
    ),
    "lower_left_down": SafeCorridor(
        "lower_left_down",
        x_min=10.55,
        x_max=11.10,
        y_min=2.36,
        y_max=3.238,
        yaw_min_deg=-90.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_LOW_LEFT,
    ),
    "lower_finish": SafeCorridor(
        "lower_finish",
        x_min=10.55,
        x_max=11.10,
        y_min=2.00,
        y_max=2.40,
        yaw_min_deg=-90.0,
        yaw_max_deg=0.0,
        certificate=FREE_CERT_LOW_LEFT,
    ),
}


PHASE_ORDER = [
    "zone4_approach",
    "zone2_top0",
    "right_descend",
    "mid_left",
    "finish",
    "right_down0",
    "upper_turn_left",
    "lower_left_down",
    "lower_finish",
]


def footprint_polygon(x: float, y: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    body = np.array(
        [
            [HALF_L_EXPANDED, HALF_W_EXPANDED],
            [HALF_L_EXPANDED, -HALF_W_EXPANDED],
            [-HALF_L_EXPANDED, -HALF_W_EXPANDED],
            [-HALF_L_EXPANDED, HALF_W_EXPANDED],
        ],
        dtype=float,
    )
    world = np.empty_like(body)
    world[:, 0] = x + body[:, 0] * c - body[:, 1] * s
    world[:, 1] = y + body[:, 0] * s + body[:, 1] * c
    return world


def footprint_sample_points(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """Return corners, edge midpoints, and center of the inflated footprint."""
    poly = footprint_polygon(x, y, yaw_deg)
    mids = 0.5 * (poly + np.roll(poly, -1, axis=0))
    return np.vstack([poly, mids, np.array([[x, y]], dtype=float)])


def point_in_free_space_strict(
    x: float,
    y: float,
    eps: float = EPS_CLEARANCE,
    rect_obstacles=None,
) -> bool:
    if rect_obstacles is None:
        rect_obstacles = RECT_OBSTACLES
    offsets = (0.0,) if eps <= 0.0 else (-eps, 0.0, eps)
    for dx in offsets:
        for dy in offsets:
            px = x + dx
            py = y + dy
            if not _point_in_any_zone_closed(px, py):
                return False
            if _point_in_any_rect_obstacle_closed(px, py, rect_obstacles):
                return False
    return True


def footprint_intersects_zone2(x: float, y: float, yaw_deg: float) -> bool:
    """Exact inflated-footprint/Zone2 intersection predicate for h/yaw rules."""
    points = footprint_sample_points(x, y, yaw_deg)
    if any(ZONE2.contains(float(px), float(py)) for px, py in points):
        return True

    poly = footprint_polygon(x, y, yaw_deg)
    zone2_poly = np.array(
        [
            [ZONE2.x_min, ZONE2.y_min],
            [ZONE2.x_max, ZONE2.y_min],
            [ZONE2.x_max, ZONE2.y_max],
            [ZONE2.x_min, ZONE2.y_max],
        ],
        dtype=float,
    )
    if any(_point_in_convex_polygon(corner, poly, include_boundary=True) for corner in zone2_poly):
        return True

    return any(
        _segments_intersect(foot_a, foot_b, zone_a, zone_b)
        for foot_a, foot_b in _polygon_edges(poly)
        for zone_a, zone_b in _polygon_edges(zone2_poly)
    )


def footprint_collision_free(
    x: float,
    y: float,
    yaw_deg: float,
    trajectory_idx: int | None = None,
) -> bool:
    """General strict collision check against outside-zone obstacles and the strip."""
    rect_obstacles = rect_obstacles_for_trajectory(trajectory_idx)
    poly = footprint_polygon(x, y, yaw_deg)
    if not all(
        point_in_free_space_strict(float(px), float(py), rect_obstacles=rect_obstacles)
        for px, py in poly
    ):
        return False

    boundary_segments = _free_obstacle_boundary_segments(rect_obstacles)
    footprint_edges = list(_polygon_edges(poly))
    for edge in footprint_edges:
        for boundary in boundary_segments:
            if _segments_intersect(edge[0], edge[1], boundary[0], boundary[1]):
                return False

    for a, b in boundary_segments:
        if _point_in_convex_polygon(a, poly, include_boundary=True):
            return False
        if _point_in_convex_polygon(b, poly, include_boundary=True):
            return False

    return True


def certify_corridors() -> list[str]:
    """Return human-readable corridor clearance certificates.

    The check is conservative: it bounds every inflated-footprint vertex over the
    full corridor x/y/yaw box and requires that bounding box to fit strictly
    inside the corridor's free-space certificate rectangle.
    """
    messages: list[str] = []
    for name in PHASE_ORDER:
        corridor = CORRIDORS[name]
        x_low, x_high, y_low, y_high = footprint_aabb_over_corridor(corridor)
        cert = corridor.certificate
        margins = (
            x_low - cert.x_min,
            cert.x_max - x_high,
            y_low - cert.y_min,
            cert.y_max - y_high,
        )
        min_margin = min(margins)
        if min_margin <= EPS_CLEARANCE:
            raise ValueError(
                f"Corridor {name} is not certified: footprint AABB "
                f"[{x_low:.6f},{x_high:.6f}]x[{y_low:.6f},{y_high:.6f}] "
                f"does not fit inside {cert}"
            )
        messages.append(
            f"{name}: footprint inside {cert.name}, min clearance {min_margin:.3f} m"
        )
    return messages


def footprint_aabb_over_corridor(corridor: SafeCorridor) -> tuple[float, float, float, float]:
    yaw_min = math.radians(corridor.yaw_min_deg)
    yaw_max = math.radians(corridor.yaw_max_deg)
    xs: list[float] = []
    ys: list[float] = []
    for dx in (-HALF_L_EXPANDED, HALF_L_EXPANDED):
        for dy in (-HALF_W_EXPANDED, HALF_W_EXPANDED):
            fx_min, fx_max = _trig_linear_range(dx, -dy, yaw_min, yaw_max)
            fy_min, fy_max = _trig_linear_range(dy, dx, yaw_min, yaw_max)
            xs.extend([corridor.x_min + fx_min, corridor.x_max + fx_max])
            ys.extend([corridor.y_min + fy_min, corridor.y_max + fy_max])
    return min(xs), max(xs), min(ys), max(ys)


def _trig_linear_range(a: float, b: float, lo: float, hi: float) -> tuple[float, float]:
    """Range of a*cos(theta) + b*sin(theta) on [lo, hi]."""
    candidates = [lo, hi]
    phase = math.atan2(b, a)
    for base in (phase, phase + math.pi):
        k_min = math.floor((lo - base) / (2.0 * math.pi)) - 1
        k_max = math.ceil((hi - base) / (2.0 * math.pi)) + 1
        for k in range(k_min, k_max + 1):
            theta = base + 2.0 * math.pi * k
            if lo <= theta <= hi:
                candidates.append(theta)
    values = [a * math.cos(theta) + b * math.sin(theta) for theta in candidates]
    return min(values), max(values)


def _inside_rect_open(
    x: float,
    y: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    eps: float,
) -> bool:
    return x_min + eps < x < x_max - eps and y_min + eps < y < y_max - eps


def _inside_rect_closed(
    x: float,
    y: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    eps: float,
) -> bool:
    return x_min - eps <= x <= x_max + eps and y_min - eps <= y <= y_max + eps


def _point_in_any_zone_closed(x: float, y: float) -> bool:
    return (
        _inside_rect_closed(x, y, ZONE1.x_min, ZONE1.x_max, ZONE1.y_min, ZONE1.y_max, 0.0)
        or _inside_rect_closed(x, y, ZONE2.x_min, ZONE2.x_max, ZONE2.y_min, ZONE2.y_max, 0.0)
        or _inside_rect_closed(x, y, ZONE3.x_min, ZONE3.x_max, ZONE3.y_min, ZONE3.y_max, 0.0)
        or _inside_rect_closed(x, y, ZONE4.x_min, ZONE4.x_max, ZONE4.y_min, ZONE4.y_max, 0.0)
    )


def _point_in_any_rect_obstacle_closed(x: float, y: float, rect_obstacles) -> bool:
    return any(
        _inside_rect_closed(x, y, x_min, x_max, y_min, y_max, 0.0)
        for x_min, x_max, y_min, y_max in rect_obstacles
    )


def _point_in_convex_polygon(point: Iterable[float], poly: np.ndarray, include_boundary: bool) -> bool:
    px, py = point
    sign = 0
    for a, b in _polygon_edges(poly):
        cross = (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])
        if abs(cross) < 1e-10:
            continue
        cur = 1 if cross > 0 else -1
        if sign == 0:
            sign = cur
        elif cur != sign:
            return False
    if include_boundary:
        return True
    return not any(_point_on_segment(np.array([px, py]), a, b) for a, b in _polygon_edges(poly))


def _polygon_edges(poly: np.ndarray):
    for i in range(len(poly)):
        yield poly[i], poly[(i + 1) % len(poly)]


def _free_obstacle_boundary_segments(rect_obstacles) -> list[tuple[np.ndarray, np.ndarray]]:
    xs = [5.10, 5.60, 8.00, 9.30, 9.44, 9.50, 10.80, 11.65, 12.00, 12.50]
    ys = [-0.3, 0.20, 1.23, 1.65, 2.00, 2.35, 2.70, 3.05, 3.40, 4.47, 4.50, 4.63, 4.80, 6.00, 6.50]
    segments: list[tuple[np.ndarray, np.ndarray]] = []

    for x in xs[1:-1]:
        for y0, y1 in zip(ys[:-1], ys[1:]):
            ym = 0.5 * (y0 + y1)
            left_free = point_in_free_space_strict(
                x - 1e-5,
                ym,
                eps=0.0,
                rect_obstacles=rect_obstacles,
            )
            right_free = point_in_free_space_strict(
                x + 1e-5,
                ym,
                eps=0.0,
                rect_obstacles=rect_obstacles,
            )
            if left_free != right_free:
                segments.append((np.array([x, y0]), np.array([x, y1])))

    for y in ys[1:-1]:
        for x0, x1 in zip(xs[:-1], xs[1:]):
            xm = 0.5 * (x0 + x1)
            below_free = point_in_free_space_strict(
                xm,
                y - 1e-5,
                eps=0.0,
                rect_obstacles=rect_obstacles,
            )
            above_free = point_in_free_space_strict(
                xm,
                y + 1e-5,
                eps=0.0,
                rect_obstacles=rect_obstacles,
            )
            if below_free != above_free:
                segments.append((np.array([x0, y]), np.array([x1, y])))
    return segments


def _segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    def orient(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if (
        (o1 > 1e-10 and o2 < -1e-10 or o1 < -1e-10 and o2 > 1e-10)
        and (o3 > 1e-10 and o4 < -1e-10 or o3 < -1e-10 and o4 > 1e-10)
    ):
        return True

    return (
        _point_on_segment(c, a, b)
        or _point_on_segment(d, a, b)
        or _point_on_segment(a, c, d)
        or _point_on_segment(b, c, d)
    )


def _point_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> bool:
    cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    if abs(cross) > 1e-10:
        return False
    dot = (p[0] - a[0]) * (p[0] - b[0]) + (p[1] - a[1]) * (p[1] - b[1])
    return dot <= 1e-10
