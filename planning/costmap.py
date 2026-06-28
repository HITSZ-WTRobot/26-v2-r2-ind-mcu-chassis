"""Costmap 构建与障碍距离场"""

import numpy as np
from scipy.ndimage import distance_transform_edt
from config import (
    ZONE1, ZONE2, ZONE3,
    CHASSIS_LENGTH, CHASSIS_WIDTH, COLLISION_EXPANSION,
    THIN_OBS_X_MIN, THIN_OBS_X_MAX, THIN_OBS_Y_MIN, THIN_OBS_Y_MAX,
)


def _chassis_inflation_radius() -> float:
    half_diag = np.sqrt((CHASSIS_LENGTH / 2) ** 2 + (CHASSIS_WIDTH / 2) ** 2)
    return half_diag + COLLISION_EXPANSION


def build_costmap(resolution: float = 0.05) -> dict:
    """构建 occupancy grid 和距离场（向量化）"""
    margin = 0.5
    x_min = min(ZONE1.x_min, ZONE2.x_min, ZONE3.x_min) - margin
    x_max = max(ZONE1.x_max, ZONE2.x_max, ZONE3.x_max) + margin
    y_min = min(ZONE1.y_min, ZONE2.y_min, ZONE3.y_min) - margin
    y_max = max(ZONE1.y_max, ZONE2.y_max, ZONE3.y_max) + margin

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
    in_any = in_z1 | in_z2 | in_z3

    # Zone1 右边界 + Zone1-Zone3 间隙（不被 Zone2 打通）
    in_gap = (((WX >= ZONE1.x_max - 0.05) & (WX <= ZONE3.x_min + 0.05)) &
              (WY < ZONE2.y_min - 0.02))

    # Zone2 底部阻挡（与 ObstacleMap 统一）：
    #   仅阻挡 x∈[9.30, 9.50] 的坡道底部边缘（防止绕过坡道）
    #   x>9.50 为 Zone3 合法空间，不阻挡
    in_z2_below = ((WX >= ZONE2.x_min - 0.02) & (WX <= ZONE3.x_min + 0.02) &
                   (WY < ZONE2.y_min))

    # 薄障碍物：Zone2 底部的水平条带
    in_thin_obs = ((WX >= THIN_OBS_X_MIN) & (WX <= THIN_OBS_X_MAX) &
                   (WY >= THIN_OBS_Y_MIN) & (WY <= THIN_OBS_Y_MAX))

    search_grid = np.where(in_any & ~in_gap & ~in_z2_below & ~in_thin_obs, 0, 100).astype(np.uint8)

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
