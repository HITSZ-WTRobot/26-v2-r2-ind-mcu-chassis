"""Mecanum4 wheel model matching master/path_mecanum4."""

from __future__ import annotations

import numpy as np

from config import LXY_M, MAX_WHEEL_ACCEL, MAX_WHEEL_SPEED, WHEEL_RADIUS_M


def wheel_speeds_np(yaw_deg, vx, vy, wz_deg):
    """Return wheel speeds [w1..w4] in rad/s.

    This mirrors `/home/syhanjin/workspace/robocon2026/master/path_mecanum4`:

        phi = theta + pi/4
        vw1 = sqrt(2)*cos(phi)*vx + sqrt(2)*sin(phi)*vy
        vw2 = sqrt(2)*sin(phi)*vx - sqrt(2)*cos(phi)*vy
        w1 = (vw1 + LXY*wz)/radius
        w2 = (vw2 - LXY*wz)/radius
        w3 = (vw1 - LXY*wz)/radius
        w4 = (vw2 + LXY*wz)/radius
    """
    theta = np.deg2rad(yaw_deg)
    wz = np.deg2rad(wz_deg)
    phi = theta + np.pi / 4.0
    vw1 = np.sqrt(2.0) * np.cos(phi) * vx + np.sqrt(2.0) * np.sin(phi) * vy
    vw2 = np.sqrt(2.0) * np.sin(phi) * vx - np.sqrt(2.0) * np.cos(phi) * vy
    return np.vstack(
        [
            (vw1 + LXY_M * wz) / WHEEL_RADIUS_M,
            (vw2 - LXY_M * wz) / WHEEL_RADIUS_M,
            (vw1 - LXY_M * wz) / WHEEL_RADIUS_M,
            (vw2 + LXY_M * wz) / WHEEL_RADIUS_M,
        ]
    ).T


def wheel_limit_report(s_array: np.ndarray, t_array: np.ndarray) -> dict[str, float]:
    wheels = wheel_speeds_np(s_array[:, 2], s_array[:, 4], s_array[:, 5], s_array[:, 6])
    max_abs_wheel_speed = float(np.max(np.abs(wheels))) if len(wheels) else 0.0
    speed_limited_time = 0.0
    accel_limited_time = 0.0
    combined_limited_time = 0.0
    total_interval_time = float(t_array[-1] - t_array[0]) if len(t_array) >= 2 else 0.0

    if len(wheels) >= 2:
        dt = np.diff(t_array)
        wheel_acc = np.diff(wheels, axis=0) / dt[:, None]
        max_abs_wheel_accel = float(np.max(np.abs(wheel_acc)))
        speed_limited = np.max(np.abs(wheels[:-1]), axis=1) >= 0.95 * MAX_WHEEL_SPEED
        accel_limited = np.max(np.abs(wheel_acc), axis=1) >= 0.95 * MAX_WHEEL_ACCEL
        speed_limited_time = float(np.sum(dt[speed_limited]))
        accel_limited_time = float(np.sum(dt[accel_limited]))
        combined_limited_time = float(np.sum(dt[np.logical_or(speed_limited, accel_limited)]))
    else:
        max_abs_wheel_accel = 0.0

    speed_ratio = max_abs_wheel_speed / MAX_WHEEL_SPEED if MAX_WHEEL_SPEED else 0.0
    accel_ratio = max_abs_wheel_accel / MAX_WHEEL_ACCEL if MAX_WHEEL_ACCEL else 0.0
    speed_limited_ratio = speed_limited_time / total_interval_time if total_interval_time > 0 else 0.0
    accel_limited_ratio = accel_limited_time / total_interval_time if total_interval_time > 0 else 0.0
    combined_limited_ratio = combined_limited_time / total_interval_time if total_interval_time > 0 else 0.0
    return {
        "max_abs_wheel_speed": max_abs_wheel_speed,
        "max_abs_wheel_accel": max_abs_wheel_accel,
        "speed_ratio": speed_ratio,
        "accel_ratio": accel_ratio,
        "limit_ratio": max(speed_ratio, accel_ratio),
        "speed_limited_ratio": speed_limited_ratio,
        "accel_limited_ratio": accel_limited_ratio,
        "combined_limited_ratio": combined_limited_ratio,
        "speed_limited_time": speed_limited_time,
        "accel_limited_time": accel_limited_time,
        "combined_limited_time": combined_limited_time,
    }
