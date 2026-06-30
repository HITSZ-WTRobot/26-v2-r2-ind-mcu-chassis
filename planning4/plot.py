"""轨迹可视化 v3：2D 场地图 + A* 路径 + 平滑路径 + 速度曲线"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from branch_specs import initial_guess_points_for_branch
from config import ENTRY_POINTS, EXIT_POINT, RECT_OBSTACLES, ZONE1, ZONE2, ZONE3, ZONE4


def plot_all_v3(results: dict, save_path: str = "trajectory_plot_v3.png"):
    fig, axes = plt.subplots(2, 3, figsize=(22, 14))

    _plot_field(axes[0, 0], results)
    _plot_astar_smooth(axes[0, 1], results)
    _plot_speed_profile(axes[0, 2], results)
    _plot_height_profile(axes[1, 0], results)
    _plot_curvature(axes[1, 1], results)
    _plot_accel(axes[1, 2], results)

    fig.suptitle("v3 Trajectory Pipeline (OCP Optimization)", fontsize=14)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"  图表已保存: {save_path}")
    plt.close(fig)


def _plot_field(ax, results):
    ax.add_patch(Rectangle((ZONE1.x_min, ZONE1.y_min), ZONE1.x_max - ZONE1.x_min,
        ZONE1.y_max - ZONE1.y_min, facecolor="lightgreen", alpha=0.3, label="Z1"))
    ax.add_patch(Rectangle((ZONE2.x_min, ZONE2.y_min), ZONE2.x_max - ZONE2.x_min,
        ZONE2.y_max - ZONE2.y_min, facecolor="orange", alpha=0.4, label="Z2"))
    ax.add_patch(Rectangle((ZONE3.x_min, ZONE3.y_min), ZONE3.x_max - ZONE3.x_min,
        ZONE3.y_max - ZONE3.y_min, facecolor="lightblue", alpha=0.3, label="Z3"))
    ax.add_patch(Rectangle((ZONE4.x_min, ZONE4.y_min), ZONE4.x_max - ZONE4.x_min,
        ZONE4.y_max - ZONE4.y_min, facecolor="lightgreen", alpha=0.2, label="Z4"))
    _plot_obstacles(ax)
    for name, r in results.items():
        if not r.success:
            continue
        ax.plot(r.s_array[:, 0], r.s_array[:, 1], linewidth=0.8, alpha=0.6, label=name)
        _plot_initial_guess_path(ax, r)
        _plot_initial_guess_points(ax, r)
    for ep in ENTRY_POINTS:
        ax.plot(ep[0], ep[1], "ko", markersize=6)
    ax.plot(EXIT_POINT[0], EXIT_POINT[1], "k*", markersize=10)
    ax.set_xlim(5.0, 12.5)
    ax.set_ylim(-0.5, 7.0)
    ax.set_aspect("equal")
    ax.set_title("Smoothed Trajectories")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="lower right")


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
            markersize=5,
            linestyle="None",
            alpha=0.65,
            label="initial guess point" if i == 0 else None,
        )
        ax.text(x + 0.025, y - 0.045, label, fontsize=6, color="purple", alpha=0.8)


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
        alpha=0.55,
        label="initial guess path",
    )


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


def _plot_astar_smooth(ax, results):
    demo = None
    for r in results.values():
        if r.success and hasattr(r, 'a_star_path') and r.a_star_path is not None:
            demo = r
            break
    if demo is None:
        ax.set_title("A* → Smooth (not available)")
        return
    ax.add_patch(Rectangle((ZONE1.x_min, ZONE1.y_min), ZONE1.x_max - ZONE1.x_min,
        ZONE1.y_max - ZONE1.y_min, facecolor="lightgreen", alpha=0.2))
    ax.add_patch(Rectangle((ZONE2.x_min, ZONE2.y_min), ZONE2.x_max - ZONE2.x_min,
        ZONE2.y_max - ZONE2.y_min, facecolor="orange", alpha=0.3))
    ax.add_patch(Rectangle((ZONE3.x_min, ZONE3.y_min), ZONE3.x_max - ZONE3.x_min,
        ZONE3.y_max - ZONE3.y_min, facecolor="lightblue", alpha=0.2))
    ax.add_patch(Rectangle((ZONE4.x_min, ZONE4.y_min), ZONE4.x_max - ZONE4.x_min,
        ZONE4.y_max - ZONE4.y_min, facecolor="lightgreen", alpha=0.15))
    _plot_obstacles(ax)
    if demo.a_star_path is not None:
        ax.plot(demo.a_star_path[:, 0], demo.a_star_path[:, 1], "c-", linewidth=1, alpha=0.7, label="A* raw")
    if hasattr(demo, 'smoothed_path') and demo.smoothed_path is not None:
        ax.plot(demo.smoothed_path[:, 0], demo.smoothed_path[:, 1], "r-", linewidth=1.5, label="Smoothed")
    ax.plot(demo.s_array[:, 0], demo.s_array[:, 1], "b-", linewidth=0.5, alpha=0.4, label="Sampled")
    ax.legend(fontsize=7)
    ax.set_aspect("equal")
    ax.set_title(f"A* → Smooth → Sample ({demo.name})")
    ax.set_xlim(5.0, 12.5)
    ax.set_ylim(-0.5, 7.0)
    ax.grid(True, alpha=0.3)


def _plot_speed_profile(ax, results):
    for name, r in results.items():
        if not r.success:
            continue
        s = r.s_array
        v = np.sqrt(s[:, 4] ** 2 + s[:, 5] ** 2)
        ax.plot(r.t_array, v, linewidth=0.7, alpha=0.5, label=name)
    ax.axhline(y=8.0, color="r", linestyle="--", alpha=0.5, label="v_max=8.0")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.set_title("Velocity Profile")
    ax.legend(fontsize=5, ncol=2)
    ax.grid(True, alpha=0.3)


def _plot_height_profile(ax, results):
    for name, r in results.items():
        if not r.success:
            continue
        ax.plot(r.t_array, r.s_array[:, 3], linewidth=0.7, alpha=0.5, label=name)
    ax.axhline(y=0.3, color="orange", linestyle="--", alpha=0.7, label="h=0.3 (Zone2)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("h [m]")
    ax.set_title("Height Profile")
    ax.legend(fontsize=5, ncol=2)
    ax.grid(True, alpha=0.3)


def _plot_curvature(ax, results):
    for name, r in results.items():
        if not r.success:
            continue
        s = r.s_array
        dx = np.diff(s[:, 0])
        dy = np.diff(s[:, 1])
        ddx = np.diff(dx)
        ddy = np.diff(dy)
        ds = np.sqrt(dx[1:] ** 2 + dy[1:] ** 2) + 1e-6
        k = np.abs(ddx * dy[1:] - ddy * dx[1:]) / (ds ** 3 + 1e-6)
        ax.plot(r.t_array[2:], k, linewidth=0.5, alpha=0.4, label=name)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Curvature [1/m]")
    ax.set_title("Path Curvature")
    ax.grid(True, alpha=0.3)


def _plot_accel(ax, results):
    for name, r in results.items():
        if not r.success:
            continue
        s = r.s_array
        dt = r.t_array[1] - r.t_array[0] if len(r.t_array) > 1 else 0.01
        a = np.sqrt(np.diff(s[:, 4]) ** 2 + np.diff(s[:, 5]) ** 2) / dt
        ax.plot(r.t_array[1:], a, linewidth=0.5, alpha=0.4, label=name)
    ax.axhline(y=3.0, color="r", linestyle="--", alpha=0.5, label="a_max=3.0")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Accel [m/s²]")
    ax.set_title("Acceleration Magnitude")
    ax.legend(fontsize=5)
    ax.grid(True, alpha=0.3)
