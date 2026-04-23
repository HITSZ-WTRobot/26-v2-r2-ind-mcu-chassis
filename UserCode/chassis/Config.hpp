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
    .Kp = 500.0f, .Ki = 5.0f, .Kd = 0.00f, .abs_output_max = 16384 * 0.75
};

constexpr PD::Config PDErrorCfg{ .Kp = 5, .Kd = 3, .abs_output_max = 60 };

/**
 * 辅助轮接地时离下限位距离 1.9mm
 * 到上限位时离下限位 227.74mm
 */
constexpr float MinToLimitMM = 1.9f;
constexpr float RangeMM      = 227.74;

constexpr float LiftMaxMM    = RangeMM - MinToLimitMM; // 最高抬升位置 unit mm
constexpr float LiftMinMM    = 0;                      // 最低抬升位置 unit mm
constexpr float LiftOffsetMM = MinToLimitMM;           // 辅助轮接地时到机械限位的偏移 (>0) unit mm
constexpr float GearRadiusMM = 25;                     // 抬升齿轮半径 mm

constexpr float LiftMax    = LiftMaxMM * 1e-3f;
constexpr float LiftMin    = LiftMinMM * 1e-3f;
constexpr float LiftOffset = LiftOffsetMM * 1e-3f;
constexpr float GearRadius = GearRadiusMM * 1e-3f;

// TODO: 将回程差纳入考虑

constexpr float MaxSpeed       = 1.178; // unit: m/s
constexpr float MaxOnloadAccel = 3.0;   // unit: m/s^2， 对车先好一点
// constexpr float MaxOnloadAccel = 5.5;   // unit: m/s^2
constexpr float MaxNoloadAccel = 100; // unit: m/s^2

constexpr Limit OnloadLimit{ MaxSpeed, MaxOnloadAccel, MaxOnloadAccel * 50 };

constexpr Limit NoloadLimit{ MaxSpeed, MaxNoloadAccel, MaxNoloadAccel * 50 };

constexpr Limit DefaultLimit = OnloadLimit;

constexpr float CalibrationSpeed = 0.05f; // 校准归零速度 m/s, 该速度无问题，无需增大
constexpr float CalibrationRpm   = -CalibrationSpeed / GearRadius * 60.0f / (2.0f * M_PI);

constexpr float    CalibrationMaxCurrent = 2000.0f; // 约 2 * 0.61 Nm
constexpr uint32_t CalibrationMinTicks   = 500;     // 堵转最小保持时间 (ms)
constexpr float    CalibrationDeadAngle  = 0.1f;    // 堵转检测
// 允许的角度误差 (deg)

/**
 * 使用到的点位
 */
namespace Position
{
using Lift::LiftMin;

constexpr float Normal = 0.010f;  // 行进默认保持高度 unit m
constexpr float UpStep = 0.22f;   // 比台阶略高 unit m
constexpr float UpR1   = LiftMax; // 比 R1 的台阶高，在最后阶段 unit m
} // namespace Position

constexpr float CalibrationOffsetAngle = LiftOffset / GearRadius / M_PI * 180.0f;

constexpr trajectory::HomingMotorTrajectory<2>::CalibrationConfig CalibrationCfg{
    .speed       = CalibrationRpm,
    .max_current = CalibrationMaxCurrent,
    .min_ticks   = CalibrationMinTicks,
    .offset      = CalibrationOffsetAngle,
    .dead_angle  = CalibrationDeadAngle,
};
} // namespace Lift

/**
 * 底盘解算配置
 */
namespace Motion
{
constexpr PIDMotor::Config MotorWheelVelPIDCfg = { //
    .Kp             = 500.0f,
    .Ki             = 5.0f,
    .Kd             = 0.00f,
    .abs_output_max = 8000.0f
};

constexpr float WheelRadiusMM    = 50.0f;   // 轮子半径 unit mm
constexpr float WheelDistanceXMM = 454.9f;  // 前后轮距 unit mm
constexpr float WheelDistanceYMM = 406.78f; // 左右轮距 unit mm
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
constexpr chassis::controller::Master::Config masterCfg = {
    .posture_error_pd_cfg = {
        .vx = { .Kp = 0.3, .Kd = 0.2f, .abs_output_max = 0.2f },
        .vy = { .Kp = 0.3, .Kd = 0.2f, .abs_output_max = 0.2f },
        .wz = { .Kp = 0.3, .Kd = 0.2f, .abs_output_max = 45.0f },
    },
    .limit = {
        // .x = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
        // .y   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
        // .yaw = { .max_spd = 90, .max_acc = 45, .max_jerk = 90 }
        .x = { .max_spd = 8.0f, .max_acc = 1.5f, .max_jerk = 80.0f },
        .y   = { .max_spd = 8.0f, .max_acc = 1.5f, .max_jerk = 80.0f },
        .yaw = { .max_spd = 1080, .max_acc = 360, .max_jerk = 1080 }
    }
};

}
} // namespace Chassis::Config
