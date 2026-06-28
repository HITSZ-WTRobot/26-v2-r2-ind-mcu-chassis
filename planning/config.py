"""离线轨迹规划 v2 - 场地 / 路径点 / 运动限制 / 可调参数"""

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

ALL_ZONES = [ZONE1, ZONE2, ZONE3]

# ============================================================
# 路径点（世界坐标系，单位 m / deg）
# ============================================================

# 三个起点
ENTRY_POINTS = [
    [8.43, 1.80, 0.0, 0.412],  # 起点1
    [8.43, 3.00, 0.0, 0.412],  # 起点2
    [8.43, 4.20, 0.0, 0.412],  # 起点3
]

# 唯一终点
EXIT_POINT = [11.25, 2.5, -90.0, 0.412]


def trajectory_name(start_idx: int) -> str:
    """轨迹命名: traj_1, traj_2, traj_3"""
    return f"traj_{start_idx + 1}"


def all_trajectory_indices():
    """生成 3 组 start_idx"""
    return list(range(3))


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
COLLISION_EXPANSION = 0.03  # 3cm

# 膨胀后的 footprint 半尺寸（用于 Zone 硬约束）
HALF_L_EXPANDED = HALF_L + COLLISION_EXPANSION
HALF_W_EXPANDED = HALF_W + COLLISION_EXPANSION

# ============================================================
# 障碍物
# ============================================================

# Zone 1,2,3 以外区域为障碍物（在 costmap 中处理）

# 薄障碍物：Zone2 底部的水平条带
THIN_OBS_X_MIN, THIN_OBS_X_MAX = 9.30, 10.80
THIN_OBS_Y_MIN, THIN_OBS_Y_MAX = 4.47, 4.53

# ============================================================
# 优化参数
# ============================================================

SAMPLE_DT = 0.01  # 输出采样周期 (100 Hz)
