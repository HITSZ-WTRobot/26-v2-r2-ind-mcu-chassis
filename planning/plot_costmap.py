"""Costmap 可视化诊断模块"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from costmap import build_costmap
from config import (
    ZONE1, ZONE2, ZONE3, ENTRY_POINTS, EXIT_POINT,
    THIN_OBS_X_MIN, THIN_OBS_X_MAX, THIN_OBS_Y_MIN, THIN_OBS_Y_MAX,
)


def plot_costmap(save_path: str = "costmap_debug.png"):
    """绘制 costmap 诊断图"""
    cm = build_costmap()

    fig, axes = plt.subplots(1, 3, figsize=(22, 7))

    # 网格坐标
    wx_edges = np.linspace(cm["x_min"], cm["x_min"] + cm["width"] * cm["resolution"],
                           cm["width"] + 1)
    wy_edges = np.linspace(cm["y_min"], cm["y_min"] + cm["height"] * cm["resolution"],
                           cm["height"] + 1)
    extent = [cm["x_min"], cm["x_min"] + cm["width"] * cm["resolution"],
              cm["y_min"], cm["y_min"] + cm["height"] * cm["resolution"]]

    free = np.sum(cm["search_grid"] < 50)
    total = cm["width"] * cm["height"]

    # ---- 1. Search Grid (A*) ----
    ax = axes[0]
    obstacle_mask = cm["search_grid"] >= 50
    rgba = np.zeros((cm["height"], cm["width"], 4), dtype=np.float64)
    rgba[~obstacle_mask] = [0.85, 1.0, 0.85, 0.6]
    rgba[obstacle_mask] = [0.0, 0.0, 0.0, 1.0]
    ax.imshow(rgba, origin="lower", extent=extent, interpolation="nearest")
    ax.set_title(f"Search Grid (A*) — black=obstacle, green=free ({100*free/total:.0f}%)")
    _draw_zone_outlines(ax)
    _draw_waypoints(ax)
    _draw_thin_obstacle(ax)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

    # ---- 2. Distance Field (Smoother) ----
    ax = axes[1]
    im = ax.imshow(cm["dist_field"], origin="lower", extent=extent,
                   cmap="RdYlGn_r", interpolation="nearest",
                   vmin=0, vmax=max(1.0, np.max(cm["dist_field"])))
    ax.set_title("Distance Field (Smoother) — red=near obstacle, green=clear")
    plt.colorbar(im, ax=ax, label="Clearance [m]", shrink=0.8)
    _draw_zone_outlines(ax)
    _draw_thin_obstacle(ax)

    # ---- 3. Obstacle Map (black filled) ----
    ax = axes[2]
    ax.pcolormesh(wx_edges, wy_edges, obstacle_mask,
                  cmap="Greys", vmin=0, vmax=1, edgecolors="none", alpha=0.9)
    ax.set_title("Obstacle Map — black=obstacle, white=free")
    _draw_zone_outlines(ax)
    _draw_waypoints(ax)
    _draw_thin_obstacle(ax)
    ax.set_xlabel("X [m]")

    for ax_i in axes:
        ax_i.set_xlim(7.5, 12.5)
        ax_i.set_ylim(-0.3, 7.0)
        ax_i.set_aspect("equal")
        ax_i.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  costmap 诊断图已保存: {save_path}")
    print(f"  search_grid: {free}/{total} free ({100*free/total:.0f}%)")


def _draw_zone_outlines(ax):
    """绘制区域虚线边界"""
    for z, c, lbl in [(ZONE1, "blue", "Z1"), (ZONE2, "orange", "Z2"), (ZONE3, "cyan", "Z3")]:
        ax.add_patch(Rectangle((z.x_min, z.y_min), z.x_max - z.x_min, z.y_max - z.y_min,
                     fill=False, edgecolor=c, linewidth=1.5, linestyle="--"))
        ax.text((z.x_min + z.x_max) / 2, z.y_min - 0.12, lbl, color=c, fontsize=10,
                fontweight="bold", ha="center")


def _draw_waypoints(ax):
    """标注起止点"""
    for ep in ENTRY_POINTS:
        ax.plot(ep[0], ep[1], "go", markersize=8, markeredgecolor="black")
    ax.plot(EXIT_POINT[0], EXIT_POINT[1], "m*", markersize=14, markeredgecolor="black")


def _draw_thin_obstacle(ax):
    """绘制薄障碍物"""
    ax.add_patch(Rectangle(
        (THIN_OBS_X_MIN, THIN_OBS_Y_MIN),
        THIN_OBS_X_MAX - THIN_OBS_X_MIN,
        THIN_OBS_Y_MAX - THIN_OBS_Y_MIN,
        facecolor="red", alpha=0.5, edgecolor="darkred", linewidth=1.0,
        label="thin obstacle"
    ))
