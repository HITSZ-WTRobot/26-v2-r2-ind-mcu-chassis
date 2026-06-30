"""Costmap 构建与障碍距离场"""

import numpy as np
from scipy.ndimage import distance_transform_edt
from config import (
    ALL_ZONES, RECT_OBSTACLES,
    ZONE1, ZONE2, ZONE3, ZONE4,
    CHASSIS_LENGTH, CHASSIS_WIDTH, COLLISION_EXPANSION,
)


def _chassis_inflation_radius() -> float:
    half_diag = np.sqrt((CHASSIS_LENGTH / 2) ** 2 + (CHASSIS_WIDTH / 2) ** 2)
    return half_diag + COLLISION_EXPANSION


def build_costmap(resolution: float = 0.05) -> dict:
    """构建 occupancy grid 和距离场（向量化）"""
    margin = 0.5
    x_min = min(z.x_min for z in ALL_ZONES) - margin
    x_max = max(z.x_max for z in ALL_ZONES) + margin
    y_min = min(z.y_min for z in ALL_ZONES) - margin
    y_max = max(z.y_max for z in ALL_ZONES) + margin

    W = int((x_max - x_min) / resolution) + 1
    H = int((y_max - y_min) / resolution) + 1

    # 网格坐标（向量化）
    ix = np.arange(W)
    iy = np.arange(H)
    wx = x_min + ix * resolution + resolution / 2
    wy = y_min + iy * resolution + resolution / 2
    WX, WY = np.meshgrid(wx, wy)  # (H, W)

    # 检查每个格是否在至少一个区域内
    in_z1 = (WX >= ZONE1.x_min) & (WX <= ZONE1.x_max) & (WY >= ZONE1.y_min) & (WY <= ZONE1.y_max)
    in_z2 = (WX >= ZONE2.x_min) & (WX <= ZONE2.x_max) & (WY >= ZONE2.y_min) & (WY <= ZONE2.y_max)
    in_z3 = (WX >= ZONE3.x_min) & (WX <= ZONE3.x_max) & (WY >= ZONE3.y_min) & (WY <= ZONE3.y_max)
    in_z4 = (WX >= ZONE4.x_min) & (WX <= ZONE4.x_max) & (WY >= ZONE4.y_min) & (WY <= ZONE4.y_max)
    in_any = in_z1 | in_z2 | in_z3 | in_z4

    in_rect_obs = np.zeros_like(in_any)
    for x0, x1, y0, y1 in RECT_OBSTACLES:
        in_rect_obs |= (WX >= x0) & (WX <= x1) & (WY >= y0) & (WY <= y1)

    search_grid = np.where(in_any & ~in_rect_obs, 0, 100).astype(np.uint8)

    # 膨化距离场
    search_obstacles = (search_grid >= 50).astype(np.uint8)
    dist_raw = distance_transform_edt(1 - search_obstacles) * resolution
    inflation_r = _chassis_inflation_radius()
    inflated_obstacles = (dist_raw < inflation_r).astype(np.uint8)
    dist_field = distance_transform_edt(1 - inflated_obstacles) * resolution

    free_count = np.sum(search_grid < 50)
    print(f"    costmap: {W}x{H}, free: {free_count}/{W*H} cells ({100*free_count/(W*H):.0f}%)")

    return {
        "search_grid": search_grid,
        "dist_field": dist_field,
        "x_min": x_min,
        "y_min": y_min,
        "resolution": resolution,
        "width": W,
        "height": H,
    }
