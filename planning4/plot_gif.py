"""Generate synchronized trajectory GIFs with chassis, height, axis, and wheel states."""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon, Rectangle
from PIL import Image

from branch_specs import initial_guess_points_for_branch
from config import (
    HALF_L,
    HALF_W,
    LIMITS,
    MAX_WHEEL_ACCEL,
    MAX_WHEEL_SPEED,
    RECT_OBSTACLES,
    ZONE1,
    ZONE2,
    ZONE3,
    ZONE4,
)
from mecanum import wheel_speeds_np

FIGURES_DIR = os.path.join(os.path.dirname(__file__), "figures")
MAX_GIF_FRAMES = 240
GIF_DURATION_QUANTUM_MS = 10
MIN_GIF_FRAME_DURATION_MS = 20


def _make_rect_polygon(cx, cy, yaw_deg):
    yaw = np.deg2rad(yaw_deg)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    corners = np.array(
        [
            [HALF_L, HALF_W],
            [HALF_L, -HALF_W],
            [-HALF_L, -HALF_W],
            [-HALF_L, HALF_W],
        ]
    )
    pts = []
    for dx, dy in corners:
        pts.append([cx + dx * cos_y - dy * sin_y, cy + dx * sin_y + dy * cos_y])
    return np.array(pts)


def _series(result):
    s = result.s_array
    t = result.t_array
    dt = np.diff(t)
    safe_dt = np.where(dt > 1e-9, dt, 1.0)

    velocity = s[:, 4:7]
    acceleration = np.zeros_like(velocity)
    if len(s) >= 2:
        acceleration[1:, :] = np.diff(velocity, axis=0) / safe_dt[:, None]
    height_velocity = s[:, 7]
    height_acceleration = np.zeros_like(height_velocity)
    if len(s) >= 2:
        height_acceleration[1:] = np.diff(height_velocity) / safe_dt

    wheels = wheel_speeds_np(s[:, 2], s[:, 4], s[:, 5], s[:, 6])
    wheel_acc = np.zeros_like(wheels)
    if len(wheels) >= 2:
        wheel_acc[1:, :] = np.diff(wheels, axis=0) / safe_dt[:, None]

    return {
        "t": t,
        "s": s,
        "velocity": velocity,
        "acceleration": acceleration,
        "height_velocity": height_velocity,
        "height_acceleration": height_acceleration,
        "wheels": wheels,
        "wheel_acc": wheel_acc,
    }


def _axis_limit(values, fallback):
    max_v = float(np.nanmax(np.abs(values))) if values.size else fallback
    return max(fallback, max_v * 1.15)


def generate_gif(result, save_path: str, step: int = 5):
    """Generate one trajectory GIF with synchronized chassis and state charts."""
    name = result.name
    data = _series(result)
    s = data["s"]
    t = data["t"]
    velocity = data["velocity"]
    acceleration = data["acceleration"]
    height_velocity = data["height_velocity"]
    height_acceleration = data["height_acceleration"]
    wheels = data["wheels"]
    wheel_acc = data["wheel_acc"]
    n = len(s)

    total_ms = max(1, int(round(float(t[-1] - t[0]) * 1000.0)))
    max_frames_for_duration = max(2, total_ms // MIN_GIF_FRAME_DURATION_MS)
    max_frames = max(2, min(MAX_GIF_FRAMES, max_frames_for_duration))
    step = max(step, int(np.ceil(n / max_frames)))
    frame_idx = list(range(0, n, step))
    if frame_idx[-1] != n - 1:
        frame_idx.append(n - 1)
    frame_times = t[frame_idx]
    frame_durations_ms = _gif_frame_durations_ms(frame_times)

    fig = plt.figure(figsize=(18, 12))
    grid = GridSpec(4, 2, width_ratios=[1.05, 1.0], height_ratios=[0.85, 1.0, 0.9, 1.0], figure=fig)
    ax_top = fig.add_subplot(grid[:, 0])
    ax_h = fig.add_subplot(grid[0, 1])
    ax_xy = fig.add_subplot(grid[1, 1])
    ax_yaw = fig.add_subplot(grid[2, 1])
    ax_wheel = fig.add_subplot(grid[3, 1])

    _setup_field(ax_top, result, frame_idx)
    height_line = ax_h.plot(t, s[:, 3], color="tab:blue", linewidth=1.2, label="h")[0]
    ax_h_v = ax_h.twinx()
    ax_h_v.plot(t, height_velocity, color="tab:green", linewidth=1.0, label="vh")
    ax_h_v.plot(t, height_acceleration, color="tab:red", linestyle="--", linewidth=0.9, label="ah")
    ax_h_v.axhline(y=LIMITS.h.a_max, color="tab:red", linewidth=0.7, alpha=0.3)
    ax_h_v.axhline(y=-LIMITS.h.a_max, color="tab:red", linewidth=0.7, alpha=0.3)
    ax_h.axhline(y=0.3, color="tab:orange", linestyle="--", linewidth=0.8, label="Zone2 h")
    ax_h.set_xlim(0.0, t[-1] * 1.02)
    ax_h.set_ylim(0.25, 0.45)
    h_dyn_ylim = _axis_limit(np.column_stack([height_velocity, height_acceleration]), LIMITS.h.a_max)
    ax_h_v.set_ylim(-h_dyn_ylim, h_dyn_ylim)
    ax_h.set_ylabel("h [m]")
    ax_h_v.set_ylabel("vh / ah")
    ax_h.set_title("Height / Height Velocity / Height Acceleration")
    ax_h.grid(True, alpha=0.3)
    h_handles, h_labels = ax_h.get_legend_handles_labels()
    hv_handles, hv_labels = ax_h_v.get_legend_handles_labels()
    ax_h.legend(h_handles + hv_handles, h_labels + hv_labels, fontsize=8, loc="upper right")

    for col, label, color in [(0, "x", "tab:blue"), (1, "y", "tab:green")]:
        ax_xy.plot(t, velocity[:, col], color=color, linewidth=1.0, label=f"v_{label}")
        ax_xy.plot(t, acceleration[:, col], color=color, linestyle="--", linewidth=0.9, label=f"a_{label}")
    xy_ylim = _axis_limit(np.column_stack([velocity[:, :2], acceleration[:, :2]]), 3.0)
    ax_xy.set_xlim(0.0, t[-1] * 1.02)
    ax_xy.set_ylim(-xy_ylim, xy_ylim)
    ax_xy.set_ylabel("x/y")
    ax_xy.set_title("X/Y Velocity / Acceleration")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8, ncol=2, loc="upper right")

    ax_yaw.plot(t, velocity[:, 2], color="tab:red", linewidth=1.0, label="v_yaw")
    ax_yaw.plot(t, acceleration[:, 2], color="tab:red", linestyle="--", linewidth=0.9, label="a_yaw")
    yaw_ylim = _axis_limit(np.column_stack([velocity[:, 2], acceleration[:, 2]]), 180.0)
    ax_yaw.set_xlim(0.0, t[-1] * 1.02)
    ax_yaw.set_ylim(-yaw_ylim, yaw_ylim)
    ax_yaw.set_ylabel("yaw")
    ax_yaw.set_title("Yaw Velocity / Acceleration")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend(fontsize=8, loc="upper right")

    wheel_colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    for col, color in zip(range(4), wheel_colors):
        ax_wheel.plot(t, wheels[:, col], color=color, linewidth=1.0, label=f"w{col + 1}")
        ax_wheel.plot(t, wheel_acc[:, col], color=color, linestyle="--", linewidth=0.8, label=f"a{col + 1}")
    ax_wheel.axhline(y=MAX_WHEEL_SPEED, color="black", linewidth=0.8, alpha=0.45)
    ax_wheel.axhline(y=-MAX_WHEEL_SPEED, color="black", linewidth=0.8, alpha=0.45)
    ax_wheel.axhline(y=MAX_WHEEL_ACCEL, color="gray", linestyle=":", linewidth=0.8, alpha=0.6)
    ax_wheel.axhline(y=-MAX_WHEEL_ACCEL, color="gray", linestyle=":", linewidth=0.8, alpha=0.6)
    wheel_ylim = _axis_limit(np.column_stack([wheels, wheel_acc]), MAX_WHEEL_ACCEL)
    ax_wheel.set_xlim(0.0, t[-1] * 1.02)
    ax_wheel.set_ylim(-wheel_ylim, wheel_ylim)
    ax_wheel.set_xlabel("Time [s]")
    ax_wheel.set_ylabel("wheel")
    ax_wheel.set_title("Wheel Speed / Acceleration")
    ax_wheel.grid(True, alpha=0.3)
    ax_wheel.legend(fontsize=7, ncol=4, loc="upper right")

    chassis_poly = ax_top.add_patch(
        Polygon(_make_rect_polygon(s[0, 0], s[0, 1], s[0, 2]), closed=True, facecolor="red",
                edgecolor="darkred", alpha=0.7, linewidth=1.5)
    )
    heading_line = ax_top.plot([], [], color="black", linewidth=1.2)[0]
    h_cursor = ax_h.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    h_v_cursor = ax_h_v.axvline(x=0.0, color="red", alpha=0.25, linewidth=1.0)
    xy_cursor = ax_xy.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    yaw_cursor = ax_yaw.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    wheel_cursor = ax_wheel.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    h_point = ax_h.plot([], [], "ro", markersize=5)[0]
    h_v_point = ax_h_v.plot([], [], marker="o", color="tab:green", markersize=4, linestyle="None")[0]
    h_a_point = ax_h_v.plot([], [], marker="o", color="tab:red", markersize=4, linestyle="None")[0]
    h_text = ax_h.text(
        0.01,
        0.05,
        "",
        transform=ax_h.transAxes,
        fontsize=8,
        va="bottom",
        fontfamily="monospace",
        bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "none"},
    )

    info_text = ax_top.text(
        0.02,
        0.98,
        "",
        transform=ax_top.transAxes,
        fontsize=9,
        va="top",
        fontfamily="monospace",
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
    )
    xy_text = ax_xy.text(
        0.01,
        0.02,
        "",
        transform=ax_xy.transAxes,
        fontsize=8,
        va="bottom",
        fontfamily="monospace",
        bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "none"},
    )
    yaw_text = ax_yaw.text(
        0.01,
        0.02,
        "",
        transform=ax_yaw.transAxes,
        fontsize=8,
        va="bottom",
        fontfamily="monospace",
        bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "none"},
    )
    wheel_text = ax_wheel.text(
        0.01,
        0.02,
        "",
        transform=ax_wheel.transAxes,
        fontsize=8,
        va="bottom",
        fontfamily="monospace",
        bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "none"},
    )

    fig.suptitle(f"{name} trajectory state replay", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    def animate(frame):
        idx = frame_idx[frame]
        t_i = float(t[idx])
        x_i, y_i, yaw_i, h_i = map(float, s[idx, :4])
        vx_i, vy_i, wz_i = velocity[idx]
        ax_i, ay_i, aw_i = acceleration[idx]
        vh_i = float(height_velocity[idx])
        ah_i = float(height_acceleration[idx])
        wheel_i = wheels[idx]
        wheel_acc_i = wheel_acc[idx]

        chassis_poly.set_xy(_make_rect_polygon(x_i, y_i, yaw_i))
        yaw_rad = np.deg2rad(yaw_i)
        nose = np.array([x_i + HALF_L * np.cos(yaw_rad), y_i + HALF_L * np.sin(yaw_rad)])
        heading_line.set_data([x_i, nose[0]], [y_i, nose[1]])

        for cursor in (h_cursor, h_v_cursor, xy_cursor, yaw_cursor, wheel_cursor):
            cursor.set_xdata([t_i, t_i])
        h_point.set_data([t_i], [h_i])
        h_v_point.set_data([t_i], [vh_i])
        h_a_point.set_data([t_i], [ah_i])
        h_text.set_text(f"h={h_i:6.3f} m\nvh={vh_i:6.3f} m/s\nah={ah_i:6.3f} m/s2")

        info_text.set_text(
            f"t={t_i:5.2f}s\nx={x_i:6.3f} y={y_i:6.3f}\nyaw={yaw_i:6.2f}deg h={h_i:5.3f}m"
        )
        xy_text.set_text(
            f"v:  x={vx_i:6.2f} m/s     y={vy_i:6.2f} m/s\n"
            f"a:  x={ax_i:6.2f} m/s2    y={ay_i:6.2f} m/s2"
        )
        yaw_text.set_text(
            f"v_yaw={wz_i:7.2f} deg/s\n"
            f"a_yaw={aw_i:7.2f} deg/s2"
        )
        wheel_text.set_text(
            "w:  "
            + " ".join(f"{v:6.2f}" for v in wheel_i)
            + " rad/s\naw: "
            + " ".join(f"{v:6.2f}" for v in wheel_acc_i)
            + " rad/s2"
        )
        return (
            chassis_poly,
            heading_line,
            h_cursor,
            h_v_cursor,
            xy_cursor,
            yaw_cursor,
            wheel_cursor,
            h_point,
            h_v_point,
            h_a_point,
            info_text,
            h_text,
            xy_text,
            yaw_text,
            wheel_text,
        )

    frames = []
    for frame in range(len(frame_idx)):
        animate(frame)
        fig.canvas.draw()
        width, height = fig.canvas.get_width_height()
        frame_image = Image.frombuffer(
            "RGBA",
            (width, height),
            fig.canvas.buffer_rgba(),
            "raw",
            "RGBA",
            0,
            1,
        ).copy()
        frames.append(frame_image)

    frames[0].save(
        save_path,
        save_all=True,
        append_images=frames[1:],
        duration=frame_durations_ms,
        loop=0,
        disposal=2,
    )
    plt.close(fig)
    print(f"  GIF saved: {save_path}")


def _setup_field(ax, result, frame_idx):
    s = result.s_array
    ax.set_xlim(5.0, 12.5)
    ax.set_ylim(-0.3, 7.0)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    for z, color, label in [
        (ZONE1, "lightgreen", "Z1"),
        (ZONE2, "orange", "Z2"),
        (ZONE3, "lightblue", "Z3"),
        (ZONE4, "lightgreen", "Z4"),
    ]:
        ax.add_patch(
            Rectangle(
                (z.x_min, z.y_min),
                z.x_max - z.x_min,
                z.y_max - z.y_min,
                facecolor=color,
                alpha=0.25,
                edgecolor="gray",
                linewidth=0.5,
            )
        )
        ax.text((z.x_min + z.x_max) / 2, z.y_max - 0.1, label, fontsize=8, ha="center")
    _plot_obstacles(ax)
    ax.plot(s[:, 0], s[:, 1], color="tab:red", linewidth=1.0, alpha=0.45, label="path")
    _plot_initial_guess_path(ax, result)
    ax.plot(s[frame_idx, 0], s[frame_idx, 1], "k.", markersize=1.5, alpha=0.25)
    ax.plot(s[0, 0], s[0, 1], "go", markersize=8, label="start")
    ax.plot(s[-1, 0], s[-1, 1], "r*", markersize=12, label="end")
    _plot_initial_guess_points(ax, result)
    ax.set_title("Chassis Rectangle Replay")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")


def _plot_obstacles(ax):
    for i, (x_min, x_max, y_min, y_max) in enumerate(RECT_OBSTACLES):
        ax.add_patch(
            Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                facecolor="tab:red",
                edgecolor="darkred",
                linewidth=1.0,
                alpha=0.45,
                label="obstacle" if i == 0 else None,
            )
        )


def _gif_frame_durations_ms(frame_times: np.ndarray) -> list[int]:
    """Return per-frame GIF durations in milliseconds.

    The durations are derived from the actual trajectory timestamps so the replay
    length tracks the true execution time instead of a fixed display rate.  GIF
    stores delays in 10 ms units, so durations are quantized before saving
    instead of relying on Pillow's truncation behavior.
    """
    if len(frame_times) <= 1:
        return [MIN_GIF_FRAME_DURATION_MS]

    motion_times_ms = np.diff(frame_times) * 1000.0
    total_cs = max(1, int(round(float(frame_times[-1] - frame_times[0]) * 1000.0 / GIF_DURATION_QUANTUM_MS)))
    min_frame_cs = max(1, MIN_GIF_FRAME_DURATION_MS // GIF_DURATION_QUANTUM_MS)
    interval_count = len(motion_times_ms)
    final_hold_cs = min_frame_cs
    minimum_total_cs = min_frame_cs * interval_count + final_hold_cs
    if total_cs < minimum_total_cs:
        total_cs = minimum_total_cs

    motion_budget_cs = total_cs - final_hold_cs
    if float(np.sum(motion_times_ms)) > 0.0:
        raw_durations = motion_times_ms / float(np.sum(motion_times_ms)) * motion_budget_cs
    else:
        raw_durations = np.full(interval_count, motion_budget_cs / interval_count)

    durations = np.floor(raw_durations).astype(int)
    durations = np.maximum(durations, min_frame_cs)

    remainder = motion_budget_cs - int(np.sum(durations))
    if remainder != 0:
        fractional = raw_durations - np.floor(raw_durations)
        if remainder > 0:
            order = np.argsort(-fractional)
            while remainder > 0:
                for idx in order:
                    durations[idx] += 1
                    remainder -= 1
                    if remainder == 0:
                        break
        else:
            order = np.argsort(fractional)
            while remainder < 0:
                progressed = False
                for idx in order:
                    if durations[idx] > min_frame_cs:
                        durations[idx] -= 1
                        remainder += 1
                        progressed = True
                        if remainder == 0:
                            break
                if not progressed:
                    break

    durations_list = (durations * GIF_DURATION_QUANTUM_MS).astype(int).tolist()
    durations_list.append(final_hold_cs * GIF_DURATION_QUANTUM_MS)
    return durations_list


def _plot_initial_guess_points(ax, result):
    branch = getattr(result, "branch", "")
    if not branch:
        return
    try:
        points = initial_guess_points_for_branch(branch)
    except KeyError:
        return
    for i, (label, x, y, _yaw, _h) in enumerate(points):
        ax.plot(
            x,
            y,
            marker="o",
            color="purple",
            markerfacecolor="none",
            markersize=6,
            linestyle="None",
            alpha=0.7,
            label="initial guess point" if i == 0 else None,
        )
        ax.text(x + 0.025, y - 0.045, label, fontsize=7, color="purple", alpha=0.8)


def _plot_initial_guess_path(ax, result):
    branch = getattr(result, "branch", "")
    if not branch:
        return
    try:
        points = initial_guess_points_for_branch(branch)
    except KeyError:
        return
    if not points:
        return
    xs = [x for _, x, *_ in points]
    ys = [y for _, _, y, *_ in points]
    ax.plot(
        xs,
        ys,
        color="purple",
        linestyle="--",
        linewidth=1.0,
        alpha=0.45,
        label="initial guess path",
    )


def generate_all_gifs(results: dict):
    os.makedirs(FIGURES_DIR, exist_ok=True)
    for name, r in results.items():
        if not r.success:
            continue
        path = os.path.join(FIGURES_DIR, f"trajectory_{name}.gif")
        generate_gif(r, path)
