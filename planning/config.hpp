#pragma once
#include <array>
#include <string>

namespace planning
{

// ============================================================
// 场地区域（场地物理边界）
// ============================================================
struct Zone
{
    const char*    name;
    double         x_min, x_max, y_min, y_max;
    constexpr bool contains(double x, double y) const
    {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

inline constexpr Zone ZONE1{ "Zone1_Flat", 8.00, 9.44, 1.23, 6.00 };
inline constexpr Zone ZONE2{ "Zone2_Ramp", 9.30, 10.80, 4.50, 6.00 };
inline constexpr Zone ZONE3{ "Zone3_Flat", 9.50, 12.00, 0.20, 6.00 };

// ============================================================
// 路径点
// ============================================================
struct Waypoint
{
    double x, y, yaw_deg, h;
};

inline constexpr std::array<Waypoint, 3> ENTRY_POINTS{ {
        { 8.43, 1.80, 0.0, 0.412 },
        { 8.43, 3.00, 0.0, 0.412 },
        { 8.43, 4.20, 0.0, 0.412 },
} };

inline constexpr Waypoint EXIT_POINT{ 11.25, 2.5, -90.0, 0.412 };

static inline std::string trajectory_name(int start_idx)
{
    return "traj_" + std::to_string(start_idx + 1);
}

// ============================================================
// 底盘尺寸
// ============================================================
inline constexpr double CHASSIS_LENGTH = 0.760;
inline constexpr double CHASSIS_WIDTH  = 0.520;
inline constexpr double HALF_L         = CHASSIS_LENGTH / 2.0;
inline constexpr double HALF_W         = CHASSIS_WIDTH / 2.0;

// ============================================================
// 麦轮底盘参数（Mecanum4，来自 path_mecanum4 验证公式）
// ============================================================
inline constexpr double WHEEL_RADIUS_MM = 77.0;
inline constexpr double DISTANCE_X_MM   = 500.00; // 半前后轮距
inline constexpr double DISTANCE_Y_MM   = 748.60; // 半左右轮距
inline constexpr double MAX_WHEEL_SPEED = 160.0;  // rad/s  最大轮速
inline constexpr double MAX_WHEEL_ACCEL = 60.0;   // rad/s² 最大轮加速度

inline constexpr double WHEEL_RADIUS_M = WHEEL_RADIUS_MM / 1000.0;
inline constexpr double LX_M           = DISTANCE_X_MM / 1000.0;
inline constexpr double LY_M           = DISTANCE_Y_MM / 1000.0;
inline constexpr double LXY_M          = (LX_M + LY_M) / 2.0;

// 碰撞硬约束膨胀量
inline constexpr double COLLISION_EXPANSION = 0.03; // 3cm

// 膨胀后的 footprint 半尺寸（用于 Zone 硬约束）
inline constexpr double HALF_L_EXPANDED = HALF_L + COLLISION_EXPANSION;
inline constexpr double HALF_W_EXPANDED = HALF_W + COLLISION_EXPANSION;

// ============================================================
// 障碍物
// ============================================================

// 薄障碍物：Zone2 底部的水平条带
inline constexpr double THIN_OBS_X_MIN = 9.30, THIN_OBS_X_MAX = 10.80;
inline constexpr double THIN_OBS_Y_MIN = 4.47, THIN_OBS_Y_MAX = 4.65;

// ============================================================
// 运动限制（MaxTrajectoryLimit）
// ============================================================
inline constexpr double V_MAX_X   = 8.0;
inline constexpr double A_MAX_X   = 3.0;
inline constexpr double V_MAX_Y   = 8.0;
inline constexpr double A_MAX_Y   = 3.0;
inline constexpr double V_MAX_YAW = 460.0; // deg/s
inline constexpr double A_MAX_YAW = 360.0; // deg/s²
inline constexpr double V_MAX_H   = 1.178;
inline constexpr double A_MAX_H   = 3.0;

// h 轴高度范围与下行加速度峰值优化偏好
inline constexpr double H_MIN                    = 0.3;
inline constexpr double H_MAX                    = 0.412;
inline constexpr double H_DOWN_ACCEL_PEAK_WEIGHT = 1e-3;

// ============================================================
// 规划参数
// ============================================================
inline constexpr double SAMPLE_DT = 0.01; // 输出采样周期 (100Hz)

// ============================================================
// 矩形 footprint 采样点（车身系）
// ============================================================
inline constexpr std::array<std::array<double, 2>, 9> FOOTPRINT_POINTS{ {
        { HALF_L, HALF_W },
        { HALF_L, -HALF_W },
        { -HALF_L, HALF_W },
        { -HALF_L, -HALF_W },
        { HALF_L, 0.0 },
        { -HALF_L, 0.0 },
        { 0.0, HALF_W },
        { 0.0, -HALF_W },
        { 0.0, 0.0 },
} };

} // namespace planning
