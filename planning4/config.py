"""Offline trajectory planning config for curves 4 and 5."""

from dataclasses import dataclass


# ============================================================
# 场地区域定义（场地物理边界，底盘整体不能超出）
# ============================================================


@dataclass
class Zone:
    """矩形区域"""
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


ZONE1 = Zone("Zone1_Flat", 8.00, 9.44, 1.23, 6.00)
ZONE2 = Zone("Zone2_Ramp", 9.30, 10.80, 4.50, 6.00)
ZONE3 = Zone("Zone3_Flat", 9.50, 12.00, 0.20, 6.00)
ZONE4 = Zone("Zone4_Start", 5.60, 8.00, 4.80, 6.00)

ALL_ZONES = [ZONE1, ZONE2, ZONE3, ZONE4]

# ============================================================
# 路径点（世界坐标系，单位 m / deg）
# ============================================================

ENTRY_POINTS = [
    [6.40, 5.40, 0.0, 0.215],
]

TARGET_POINTS = [
    [11.05, 3.227, 0.0, 0.215],
    [10.75, 2.0, -90.0, 0.44],
]

# Compatibility name for the first curve-4 target.
EXIT_POINT = TARGET_POINTS[0]


def trajectory_name(trajectory_idx: int) -> str:
    """Trajectory naming follows the firmware sequence: traj_4, traj_5."""
    if not 0 <= trajectory_idx < len(TARGET_POINTS):
        raise IndexError(f"unknown trajectory index {trajectory_idx}")
    return f"traj_{trajectory_idx + 4}"


def start_point_for_trajectory(_trajectory_idx: int) -> list[float]:
    """All CLAUDE.md targets share the same start pose."""
    return ENTRY_POINTS[0]


def end_point_for_trajectory(trajectory_idx: int) -> list[float]:
    return TARGET_POINTS[trajectory_idx]


def all_trajectory_indices():
    return list(range(len(TARGET_POINTS)))


# ============================================================
# 运动限制（来自 UserCode/chassis/Config.hpp MaxTrajectoryLimit）
# ============================================================


@dataclass
class Limit:
    """单轴运动限制"""
    v_max: float
    a_max: float
    j_max: float  # 用于 cost 正则


@dataclass
class Limits:
    """四轴运动限制"""
    x: Limit
    y: Limit
    yaw: Limit
    h: Limit


LIMITS = Limits(
    x=Limit(v_max=8.0, a_max=3.0, j_max=150.0),
    y=Limit(v_max=8.0, a_max=3.0, j_max=150.0),
    yaw=Limit(v_max=460.0, a_max=360.0, j_max=8500.0),
    h=Limit(v_max=1.178, a_max=3.0, j_max=275.0),
)

# h 轴高度范围与下行加速度峰值优化偏好。
# Zone2 高度另由 ZONE2_REQUIRED_H 独立固定为 0.3m。
H_MIN = 0.215
H_MAX = 0.44
ZONE2_REQUIRED_H = 0.3
H_DOWN_ACCEL_PEAK_WEIGHT = 1e-3

# ============================================================
# 底盘尺寸（来自 Config.hpp ChassisInfo）
# ============================================================

CHASSIS_LENGTH = 0.760
CHASSIS_WIDTH = 0.520
HALF_L = CHASSIS_LENGTH / 2.0  # 0.38 m
HALF_W = CHASSIS_WIDTH / 2.0  # 0.26 m

# ============================================================
# 麦轮底盘参数（Mecanum4，来自 path_mecanum4 验证公式）
# ============================================================

WHEEL_RADIUS_MM = 77.0
DISTANCE_X_MM = 500.00  # 半前后轮距
DISTANCE_Y_MM = 748.60  # 半左右轮距
MAX_WHEEL_SPEED = 160.0  # rad/s  最大轮速
MAX_WHEEL_ACCEL = 60.0  # rad/s² 最大轮加速度

WHEEL_RADIUS_M = WHEEL_RADIUS_MM / 1000.0
LX_M = DISTANCE_X_MM / 1000.0
LY_M = DISTANCE_Y_MM / 1000.0
LXY_M = (LX_M + LY_M) / 2.0

# 碰撞硬约束膨胀量
COLLISION_EXPANSION = 0.05  # 5cm

# 膨胀后的 footprint 半尺寸（用于 Zone 硬约束）
HALF_L_EXPANDED = HALF_L + COLLISION_EXPANSION
HALF_W_EXPANDED = HALF_W + COLLISION_EXPANSION

# ============================================================
# 障碍物
# ============================================================

# Zone 1,2,3,4 以外区域为障碍物（在 costmap 中处理）

# 薄障碍物：Zone2 底部的水平条带
THIN_OBS_X_MIN, THIN_OBS_X_MAX = 9.30, 10.80
THIN_OBS_Y_MIN, THIN_OBS_Y_MAX = 4.47, 4.63

# Zone3 右侧的三个矩形障碍物
OBSTACLE_STRIP = (THIN_OBS_X_MIN, THIN_OBS_X_MAX, THIN_OBS_Y_MIN, THIN_OBS_Y_MAX)
OBSTACLE_2 = (11.65, 12.00, 3.05, 3.40)
OBSTACLE_3 = (11.65, 12.00, 2.35, 2.70)
OBSTACLE_4 = (11.65, 12.00, 1.65, 2.00)

COMMON_RECT_OBSTACLES = [
    OBSTACLE_STRIP,
    OBSTACLE_3,
    OBSTACLE_4,
]

TRAJECTORY_ONLY_RECT_OBSTACLES = {
    0: [OBSTACLE_2],
    1: [],
}

# Full obstacle list retained for aggregate diagnostics and legacy imports.
RECT_OBSTACLES = [
    OBSTACLE_STRIP,
    OBSTACLE_2,
    OBSTACLE_3,
    OBSTACLE_4,
]


def rect_obstacles_for_trajectory(trajectory_idx: int | None = None):
    """Return rectangular obstacles for a target curve.

    CLAUDE.md defines obstacle 2 as absent for target/end point 2, i.e. traj_5.
    """
    if trajectory_idx is None:
        return RECT_OBSTACLES
    return COMMON_RECT_OBSTACLES + TRAJECTORY_ONLY_RECT_OBSTACLES.get(trajectory_idx, [])

# ============================================================
# 优化参数
# ============================================================

SAMPLE_DT = 0.002  # 输出采样周期 (500 Hz)
