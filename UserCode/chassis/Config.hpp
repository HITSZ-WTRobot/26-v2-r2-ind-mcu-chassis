/**
 * @file    Config.hpp
 * @author  syhanjin
 * @date    2026-03-26
 *
 * 底盘参数
 */
#pragma once
#include "Master.hpp"
#include "homing_motor_trajectory.hpp"
#include "pid_motor.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "tests/tests.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846f
#endif

namespace Chassis::Config
{
using Limit = velocity_profile::SCurveProfile::Config;

/**
 * 抬升配置
 */
namespace Lift
{
constexpr PIDMotor::Config PIDCfg{
    .Kp = 450.0f, .Ki = 1.0f, .Kd = 0.00f, .abs_output_max = 16384 * 0.75
};

constexpr PD::Config PDErrorCfg{ .Kp = 5, .Kd = 25, .abs_output_max = 60 };

constexpr float MinToLimitMM = 1.25f;  /// 辅助轮接地时离下限位距离
constexpr float RangeMM      = 410.0f; ///  到上限位时离下限位

constexpr float LiftMaxMM    = RangeMM - MinToLimitMM; // 最高抬升位置 unit mm
constexpr float LiftMinMM    = 0;                      // 最低抬升位置 unit mm
constexpr float LiftOffsetMM = MinToLimitMM;           // 辅助轮接地时到机械限位的偏移 (>0) unit mm
constexpr float GearRadiusMM = 25;                     // 抬升齿轮半径 mm
constexpr float GroundingChassisHeightMM = 195.0f;     // 辅助轮接地时底盘离地高度 unit mm

constexpr float LiftMax                = LiftMaxMM * 1e-3f;
constexpr float LiftMin                = LiftMinMM * 1e-3f;
constexpr float LiftOffset             = LiftOffsetMM * 1e-3f;
constexpr float GearRadius             = GearRadiusMM * 1e-3f;
constexpr float GroundingChassisHeight = GroundingChassisHeightMM * 1e-3f;

constexpr float chassisHeightToLiftPosition(const float chassis_height)
{
    return chassis_height - GroundingChassisHeight;
}

// TODO: 将回程差纳入考虑

#if TEST_ENABLE_AUTO_MAPPING
constexpr float MaxSpeed       = 0.18f; // unit: m/s
constexpr float MaxOnloadAccel = 0.35f; // unit: m/s^2
constexpr float MaxNoloadAccel = 0.60f; // unit: m/s^2
#else
constexpr float MaxSpeed       = 1.178; // unit: m/s
constexpr float MaxOnloadAccel = 3.0;   // unit: m/s^2， 对车先好一点
// constexpr float MaxOnloadAccel = 5.5;   // unit: m/s^2
constexpr float MaxNoloadAccel = 100; // unit: m/s^2
#endif

constexpr Limit OnloadLimit{ MaxSpeed, MaxOnloadAccel, MaxOnloadAccel * 50 };

constexpr Limit NoloadLimit{ MaxSpeed, MaxNoloadAccel, MaxNoloadAccel * 50 };

constexpr Limit DefaultLimit = OnloadLimit;

constexpr float CalibrationSpeed = 0.05f; // 校准归零速度 m/s, 该速度无问题，无需增大
constexpr float CalibrationRpm   = -CalibrationSpeed / GearRadius * 60.0f / (2.0f * M_PI);

constexpr float    CalibrationMaxCurrent = 4000.0f; // 约 2 * 1.22 Nm
constexpr uint32_t CalibrationMinTicks   = 800;     // 堵转最小保持时间 (ms)
constexpr float    CalibrationDeadAngle  = 0.1f;    // 堵转检测
// 允许的角度误差 (deg)

/**
 * 使用到的点位
 *
 * 以下高度为抬升高度（在本机抬升高度表示辅助轮离地高度）
 */
namespace Position
{
using Lift::LiftMin;

constexpr float Normal         = 0.02f;   // 行进默认保持高度 unit m
constexpr float StepTransition = 0.01f;   // 上下台阶过程中的过渡高度 unit m
constexpr float UpStep         = 0.22f;   // 比台阶略高 unit m
constexpr float UpR1           = LiftMax; // 比 R1 的台阶高，在最后阶段 unit m
} // namespace Position

constexpr float CalibrationOffsetAngle = LiftOffset / GearRadius / M_PI * 180.0f;
constexpr float PositionNormalAngle    = Position::Normal / GearRadius / M_PI * 180.0f;

constexpr trajectory::HomingMotorTrajectory<2>::CalibrationConfig CalibrationCfg{
    .speed               = CalibrationRpm,
    .max_current         = CalibrationMaxCurrent,
    .min_ticks           = CalibrationMinTicks,
    .offset              = CalibrationOffsetAngle,
    .target_after_homing = PositionNormalAngle,
    .dead_angle          = CalibrationDeadAngle,
};
} // namespace Lift

/**
 * 底盘解算配置
 */
namespace Motion
{
constexpr PIDMotor::Config MotorWheelVelPIDCfg = { //
    .Kp             = 450.0f,
    .Ki             = 1.0f,
    .Kd             = 0.00f,
    .abs_output_max = 8000.0f
};

constexpr float WheelRadiusMM    = 50.0f;   // 轮子半径 unit mm
constexpr float WheelDistanceXMM = 458.1f;  // 前后轮距 unit mm
constexpr float WheelDistanceYMM = 401.08f; // 左右轮距 unit mm
} // namespace Motion

/**
 * 底盘在 X-Y 方向上的位置信息
 */
namespace ChassisInfo
{
constexpr float AuxiliaryWheelRadiusMM  = 20.0f;  // 辅助轮半径
constexpr float AuxOuterWheelDistanceMM = 760.0f; // 外侧两个辅助轮中心距
constexpr float AuxInnerWheelDistanceMM = 160.0f; // 内侧两个辅助轮中心距
constexpr float ChassisDistanceXMM      = 800.0f; // 车体长度

// 换算后用于使用
constexpr float AuxWheelRadius = 1e-3f * AuxiliaryWheelRadiusMM; // 辅助轮半径

// 外侧辅助轮中心相对于车体中心
constexpr float AbsAuxOuterWheelX = AuxOuterWheelDistanceMM * 0.5f * 1e-3f;
// 内侧辅助轮中心相对于车体中心
constexpr float AbsAuxInnerWheelX = AuxInnerWheelDistanceMM * 0.5f * 1e-3f;

constexpr float AbsWheelX   = Motion::WheelDistanceXMM * 0.5f * 1e-3f; // 主动轮距中心的距离
constexpr float WheelRadius = Motion::WheelRadiusMM * 1e-3f;           // 主动轮半径

constexpr float AbsWheelOuterEdgeX = AbsWheelX + WheelRadius; // 主动轮外边缘位置
constexpr float AbsWheelInnerEdgeX = AbsWheelX - WheelRadius; // 主动轮内边缘位置

constexpr float ChassisDistanceX     = ChassisDistanceXMM * 1e-3f;
constexpr float HalfChassisDistanceX = ChassisDistanceX / 2.0f;

constexpr float SafeDistance = 0.01; // 1cm 安全距离（底盘方向）
} // namespace ChassisInfo

/**
 * 控制器参数
 */
namespace Control
{
#if TEST_ENABLE_AUTO_MAPPING
constexpr chassis::controller::Master::TrajectoryLimit DefaultTrajectoryLimit = {
    .x   = { .max_spd = 0.60f, .max_acc = 0.60f, .max_jerk = 12.0f },
    .y   = { .max_spd = 0.60f, .max_acc = 0.60f, .max_jerk = 12.0f },
    .yaw = { .max_spd = 45.0f, .max_acc = 60.0f, .max_jerk = 1200.0f }
};
#else
constexpr chassis::controller::Master::TrajectoryLimit DefaultTrajectoryLimit = {
    // .x = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
    // .y   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
    // .yaw = { .max_spd = 90, .max_acc = 45, .max_jerk = 90 }
    .x   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .y   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .yaw = { .max_spd = 460, .max_acc = 170, .max_jerk = 170 * 50 }
};
#endif

constexpr chassis::controller::Master::Config masterCfg = {
    .posture_error_pd_cfg = {
        .vx = { .Kp = 3.0, .Kd = 0.3f, .abs_output_max = 0.6f },
        .vy = { .Kp = 3.0, .Kd = 0.3f, .abs_output_max = 0.6f },
        .wz = { .Kp = 3.0, .Kd = 0.3f, .abs_output_max = 135.0f },
    },
    .limit = DefaultTrajectoryLimit,
};

} // namespace Control
} // namespace Chassis::Config
