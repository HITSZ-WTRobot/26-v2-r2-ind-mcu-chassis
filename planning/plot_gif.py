"""Generate synchronized trajectory GIFs with chassis, height, axis, and wheel states."""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon, Rectangle

from branch_specs import anchor_points_for_branch, initial_guess_points_for_branch
from config import HALF_L, HALF_W, MAX_WHEEL_ACCEL, MAX_WHEEL_SPEED, ZONE1, ZONE2, ZONE3
from mecanum import wheel_speeds_np

FIGURES_DIR = os.path.join(os.path.dirname(__file__), "figures")


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

    wheels = wheel_speeds_np(s[:, 2], s[:, 4], s[:, 5], s[:, 6])
    wheel_acc = np.zeros_like(wheels)
    if len(wheels) >= 2:
        wheel_acc[1:, :] = np.diff(wheels, axis=0) / safe_dt[:, None]

    return {
        "t": t,
        "s": s,
        "velocity": velocity,
        "acceleration": acceleration,
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
    wheels = data["wheels"]
    wheel_acc = data["wheel_acc"]
    n = len(s)

    frame_idx = list(range(0, n, step))
    if frame_idx[-1] != n - 1:
        frame_idx.append(n - 1)

    fig = plt.figure(figsize=(18, 12))
    grid = GridSpec(4, 2, width_ratios=[1.05, 1.0], height_ratios=[0.85, 1.0, 0.9, 1.0], figure=fig)
    ax_top = fig.add_subplot(grid[:, 0])
    ax_h = fig.add_subplot(grid[0, 1])
    ax_xy = fig.add_subplot(grid[1, 1])
    ax_yaw = fig.add_subplot(grid[2, 1])
    ax_wheel = fig.add_subplot(grid[3, 1])

    _setup_field(ax_top, result, frame_idx)
    height_line = ax_h.plot(t, s[:, 3], color="tab:blue", linewidth=1.2, label="h")[0]
    ax_h.axhline(y=0.3, color="tab:orange", linestyle="--", linewidth=0.8, label="Zone2 h")
    ax_h.set_xlim(0.0, t[-1] * 1.02)
    ax_h.set_ylim(0.25, 0.45)
    ax_h.set_ylabel("h [m]")
    ax_h.set_title("Height")
    ax_h.grid(True, alpha=0.3)
    ax_h.legend(fontsize=8, loc="upper right")

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
    xy_cursor = ax_xy.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    yaw_cursor = ax_yaw.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    wheel_cursor = ax_wheel.axvline(x=0.0, color="red", alpha=0.35, linewidth=1.0)
    h_point = ax_h.plot([], [], "ro", markersize=5)[0]

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
        wheel_i = wheels[idx]
        wheel_acc_i = wheel_acc[idx]

        chassis_poly.set_xy(_make_rect_polygon(x_i, y_i, yaw_i))
        yaw_rad = np.deg2rad(yaw_i)
        nose = np.array([x_i + HALF_L * np.cos(yaw_rad), y_i + HALF_L * np.sin(yaw_rad)])
        heading_line.set_data([x_i, nose[0]], [y_i, nose[1]])

        for cursor in (h_cursor, xy_cursor, yaw_cursor, wheel_cursor):
            cursor.set_xdata([t_i, t_i])
        h_point.set_data([t_i], [h_i])

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
            xy_cursor,
            yaw_cursor,
            wheel_cursor,
            h_point,
            info_text,
            xy_text,
            yaw_text,
            wheel_text,
        )

    ani = FuncAnimation(fig, animate, frames=len(frame_idx), interval=100, blit=False)
    writer = PillowWriter(fps=10)
    ani.save(save_path, writer=writer)
    plt.close(fig)
    print(f"  GIF saved: {save_path}")


def _setup_field(ax, result, frame_idx):
    s = result.s_array
    ax.set_xlim(7.5, 12.5)
    ax.set_ylim(-0.3, 7.0)
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    for z, color, label in [
        (ZONE1, "lightgreen", "Z1"),
        (ZONE2, "orange", "Z2"),
        (ZONE3, "lightblue", "Z3"),
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
    ax.plot(s[:, 0], s[:, 1], color="tab:red", linewidth=1.0, alpha=0.45, label="path")
    _plot_initial_guess_path(ax, result)
    ax.plot(s[frame_idx, 0], s[frame_idx, 1], "k.", markersize=1.5, alpha=0.25)
    ax.plot(s[0, 0], s[0, 1], "go", markersize=8, label="start")
    ax.plot(s[-1, 0], s[-1, 1], "r*", markersize=12, label="end")
    _plot_forced_anchors(ax, result)
    _plot_initial_guess_points(ax, result)
    ax.set_title("Chassis Rectangle Replay")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")


def _plot_forced_anchors(ax, result):
    branch = getattr(result, "branch", "")
    if not branch:
        return
    try:
        anchors = anchor_points_for_branch(branch)
    except KeyError:
        return
    for i, (label, x, y, _yaw, _h) in enumerate(anchors):
        ax.plot(
            x,
            y,
            marker="x",
            color="purple",
            markersize=8,
            markeredgewidth=1.6,
            linestyle="None",
            label="forced anchor" if i == 0 else None,
        )
        ax.text(x + 0.025, y + 0.025, label, fontsize=7, color="purple")


def _plot_initial_guess_points(ax, result):
    branch = getattr(result, "branch", "")
    if not branch:
        return
    try:
        points = initial_guess_points_for_branch(branch)
    except KeyError:
        return
    hard_labels = {label for label, *_ in anchor_points_for_branch(branch)}
    soft_points = [(label, x, y, yaw, h) for label, x, y, yaw, h in points if label not in hard_labels]
    for i, (label, x, y, _yaw, _h) in enumerate(soft_points):
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
