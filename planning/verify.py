"""Strict verification for generated trajectories.

This verifier checks the exported 500 Hz samples and the swept inflated
footprint between adjacent samples.  It is intentionally stricter than the
optimizer: any uncertain geometry is treated as a failure.
"""

from __future__ import annotations

import math

import numpy as np

from config import (
    ENTRY_POINTS,
    EXIT_POINT,
    H_MAX,
    H_MIN,
    LIMITS,
    MAX_WHEEL_ACCEL,
    MAX_WHEEL_SPEED,
    SAMPLE_DT,
    all_trajectory_indices,
    trajectory_name,
)
from branch_specs import BRANCHES, certify_start_corridors
from geometry import (
    footprint_collision_free,
    footprint_intersects_zone2,
)
from mecanum import wheel_limit_report, wheel_speeds_np


MIN_COMBINED_LIMITED_RATIO = 0.75


def verify_all(results) -> bool:
    print("=" * 60)
    print("轨迹严格验证")
    print("=" * 60)

    all_errors: list[str] = []
    all_errors += _check_corridor_certificates()
    all_errors += _check_boundary(results)
    all_errors += _check_velocity_continuity(results)
    all_errors += _check_state_velocity_integration(results)
    all_errors += _check_height_limits(results)
    all_errors += _check_wheel_constraints(results)
    all_errors += _check_limit_saturation(results)
    all_errors += _check_zone2(results)
    all_errors += _check_swept_collision(results)

    print("\n" + "=" * 60)
    failures = [e for e in all_errors if "[FAIL]" in e]
    warnings = [e for e in all_errors if "[WARN]" in e]
    print(f"结果: {len(failures)} 失败, {len(warnings)} 警告")
    for error in failures[:20]:
        print(error)
    if len(failures) > 20:
        print(f"  ... {len(failures) - 20} more failures")
    print("=" * 60)
    return len(failures) == 0


def _check_corridor_certificates() -> list[str]:
    errors: list[str] = []
    try:
        for idx in all_trajectory_indices():
            for branch in BRANCHES:
                messages = certify_start_corridors(idx, branch)
                for message in messages:
                    print(f"  [OK] {message}")
    except ValueError as exc:
        errors.append(f"  [FAIL] corridor certificate: {exc}")
    status = "OK" if not errors else "FAILURES"
    print(f"走廊解析证书: {status}")
    return errors


def _check_boundary(results) -> list[str]:
    errors: list[str] = []
    for s_idx in all_trajectory_indices():
        name = trajectory_name(s_idx)
        r = results.get(name)
        if not r or not r.success:
            errors.append(f"  [FAIL] {name}: missing trajectory")
            continue
        s_arr = r.s_array
        for j, label in enumerate(["x", "y", "yaw", "h"]):
            if abs(s_arr[0, j] - ENTRY_POINTS[s_idx][j]) > 2e-3:
                errors.append(f"  [FAIL] {name} start {label}: {s_arr[0, j]:.6f}")
            if abs(s_arr[-1, j] - EXIT_POINT[j]) > 2e-3:
                errors.append(f"  [FAIL] {name} end {label}: {s_arr[-1, j]:.6f}")
        for j, label in enumerate(["dx", "dy", "dyaw", "dh"]):
            if abs(s_arr[0, 4 + j]) > 2e-3:
                errors.append(f"  [FAIL] {name} start {label} not zero: {s_arr[0, 4 + j]:.6f}")
            if abs(s_arr[-1, 4 + j]) > 2e-3:
                errors.append(f"  [FAIL] {name} end {label} not zero: {s_arr[-1, 4 + j]:.6f}")
    _print_status("边界条件", errors)
    return errors


def _check_velocity_continuity(results) -> list[str]:
    errors: list[str] = []
    for name, r in results.items():
        if not r.success or len(r.s_array) < 3:
            continue
        dv = np.abs(np.diff(r.s_array[:, 4:], axis=0))
        max_step = np.max(dv, axis=0)
        if max_step[3] > LIMITS.h.a_max * SAMPLE_DT + 0.08:
            errors.append(f"  [FAIL] {name}: dh discontinuity step {max_step[3]:.6f}")
    _print_status("速度连续性", errors)
    return errors


def _check_state_velocity_integration(results) -> list[str]:
    errors: list[str] = []
    labels = ("x", "y", "yaw", "h")
    step_tolerances = np.array([8e-5, 8e-5, 3e-3, 8e-5], dtype=float)

    for name, r in results.items():
        if not r.success or len(r.s_array) < 2:
            continue

        dt = np.diff(r.t_array)
        state_delta = np.diff(r.s_array[:, :4], axis=0)
        velocity_trapezoid = 0.5 * (r.s_array[:-1, 4:] + r.s_array[1:, 4:]) * dt[:, None]
        step_error = np.abs(state_delta - velocity_trapezoid)
        max_step_error = np.max(step_error, axis=0)

        print(
            f"  [INFO] {name}: integral step error "
            f"x/y/yaw/h = {max_step_error[0]:.6g}/{max_step_error[1]:.6g}/"
            f"{max_step_error[2]:.6g}/{max_step_error[3]:.6g}"
        )
        for j, label in enumerate(labels):
            if max_step_error[j] > step_tolerances[j]:
                errors.append(
                    f"  [FAIL] {name}: {label} integral step error "
                    f"{max_step_error[j]:.6g} > {step_tolerances[j]:.6g}"
                )

    _print_status("位置/速度积分关系", errors)
    return errors


def _check_height_limits(results) -> list[str]:
    errors: list[str] = []
    for name, r in results.items():
        if not r.success:
            continue
        min_h = float(np.min(r.s_array[:, 3]))
        max_h = float(np.max(r.s_array[:, 3]))
        mean_h = float(np.mean(r.s_array[:, 3]))
        print(f"  [INFO] {name}: h min/mean/max = {min_h:.6f}/{mean_h:.6f}/{max_h:.6f}")
        if min_h < H_MIN - 2e-3:
            errors.append(f"  [FAIL] {name}: h min {min_h:.6f} < {H_MIN}")
        if max_h > H_MAX + 2e-3:
            errors.append(f"  [FAIL] {name}: h max {max_h:.6f} > {H_MAX}")
        max_dh = float(np.max(np.abs(r.s_array[:, 7])))
        if max_dh > LIMITS.h.v_max + 1e-3:
            errors.append(f"  [FAIL] {name}: dh max {max_dh:.6f} > {LIMITS.h.v_max}")
        dt = _nominal_dt(r)
        if len(r.s_array) >= 2:
            ah = np.diff(r.s_array[:, 7]) / dt
            max_ah = float(np.max(np.abs(ah)))
            max_down_ah = float(np.max(np.maximum(-ah, 0.0)))
            print(f"  [INFO] {name}: max downward ah = {max_down_ah:.6f}")
            if max_ah > LIMITS.h.a_max + 0.12:
                errors.append(f"  [FAIL] {name}: ah max {max_ah:.6f} > {LIMITS.h.a_max}")
    _print_status("h 轴速度/加速度限制", errors)
    return errors


def _check_wheel_constraints(results) -> list[str]:
    errors: list[str] = []
    for name, r in results.items():
        if not r.success:
            continue
        wheels = wheel_speeds_np(r.s_array[:, 2], r.s_array[:, 4], r.s_array[:, 5], r.s_array[:, 6])
        max_speed = float(np.max(np.abs(wheels))) if len(wheels) else 0.0
        if max_speed > MAX_WHEEL_SPEED + 1e-3:
            errors.append(f"  [FAIL] {name}: wheel speed max {max_speed:.6f} > {MAX_WHEEL_SPEED}")
        if len(wheels) >= 2:
            dt = np.diff(r.t_array)
            wheel_acc = np.diff(wheels, axis=0) / dt[:, None]
            max_acc = float(np.max(np.abs(wheel_acc)))
            if max_acc > MAX_WHEEL_ACCEL + 0.5:
                errors.append(f"  [FAIL] {name}: wheel accel max {max_acc:.6f} > {MAX_WHEEL_ACCEL}")
    _print_status("path_mecanum4 轮速/轮加速度约束", errors)
    return errors


def _check_limit_saturation(results) -> list[str]:
    errors: list[str] = []
    print("时间最优限制贴近度评估:")
    for name, r in results.items():
        if not r.success:
            continue
        report = wheel_limit_report(r.s_array, r.t_array)
        print(
            f"  [INFO] {name}: max wheel speed {report['max_abs_wheel_speed']:.2f}/{MAX_WHEEL_SPEED:.2f} "
            f"({100*report['speed_ratio']:.0f}%), max wheel accel "
            f"{report['max_abs_wheel_accel']:.2f}/{MAX_WHEEL_ACCEL:.2f} "
            f"({100*report['accel_ratio']:.0f}%)"
        )
        print(
            f"  [INFO] {name}: >=95% speed-limit time "
            f"{report['speed_limited_time']:.2f}s ({100*report['speed_limited_ratio']:.0f}%), "
            f">=95% accel-limit time {report['accel_limited_time']:.2f}s "
            f"({100*report['accel_limited_ratio']:.0f}%), combined "
            f"{report['combined_limited_time']:.2f}s ({100*report['combined_limited_ratio']:.0f}%)"
        )
        if report["combined_limited_ratio"] < MIN_COMBINED_LIMITED_RATIO:
            errors.append(
                f"  [FAIL] {name}: only {100*report['combined_limited_ratio']:.1f}% of time near "
                f"wheel speed/accel limits, expected >= {100*MIN_COMBINED_LIMITED_RATIO:.0f}%"
            )
    _print_status("时间最优限幅持续时间", errors)
    return errors


def _check_zone2(results) -> list[str]:
    errors: list[str] = []
    for name, r in results.items():
        if not r.success:
            continue
        in_zone2_count = 0
        for state in _sample_states_for_swept_check(r.s_array):
            x, y, yaw, h = map(float, state[:4])
            if footprint_intersects_zone2(x, y, yaw):
                in_zone2_count += 1
                if abs(yaw) > 1e-3:
                    errors.append(f"  [FAIL] {name}: Zone2 yaw {yaw:.6f} at x={x:.3f}, y={y:.3f}")
                    break
                if abs(h - 0.3) > 2e-3:
                    errors.append(f"  [FAIL] {name}: Zone2 h {h:.6f} at x={x:.3f}, y={y:.3f}")
                    break
        print(f"  [OK] {name}: Zone2-contact samples checked = {in_zone2_count}")
    _print_status("Zone2 yaw=0/h 约束", errors)
    return errors


def _check_swept_collision(results) -> list[str]:
    errors: list[str] = []
    for name, r in results.items():
        if not r.success:
            continue
        checked = 0
        for state in _sample_states_for_swept_check(r.s_array):
            x, y, yaw = map(float, state[:3])
            if not footprint_collision_free(x, y, yaw):
                errors.append(f"  [FAIL] {name}: inflated footprint collision at x={x:.4f}, y={y:.4f}, yaw={yaw:.2f}")
                break
            checked += 1
        if not any(name in error for error in errors):
            print(f"  [OK] {name}: swept inflated footprint samples={checked}")
    _print_status("膨胀底盘 swept 碰撞", errors)
    return errors


def _sample_states_for_swept_check(s_arr: np.ndarray):
    """Yield states at samples and between samples.

    Each 500 Hz interval is subdivided so the verifier observes the swept
    inflated footprint, not only exported samples.
    """
    if len(s_arr) == 0:
        return
    yield s_arr[0]
    for i in range(len(s_arr) - 1):
        a = s_arr[i]
        b = s_arr[i + 1]
        dist = math.hypot(float(b[0] - a[0]), float(b[1] - a[1]))
        dyaw = abs(float(b[2] - a[2]))
        n = max(2, int(math.ceil(dist / 0.005)), int(math.ceil(dyaw / 0.25)))
        for j in range(1, n + 1):
            alpha = j / n
            yield (1.0 - alpha) * a + alpha * b


def _nominal_dt(result) -> float:
    if len(result.t_array) < 2:
        return SAMPLE_DT
    return float(np.median(np.diff(result.t_array)))


def _print_status(label: str, errors: list[str]) -> None:
    status = "OK" if not errors else "FAILURES"
    print(f"{label}: {status}")
    if not errors:
        print("  [OK] all passed")
