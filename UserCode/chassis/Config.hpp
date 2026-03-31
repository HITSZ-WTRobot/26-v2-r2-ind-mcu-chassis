/**
 * @file    Config.hpp
 * @author  syhanjin
 * @date    2026-03-26
 *
 * 底盘参数
 */
#pragma once
#include "Master.hpp"
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
    .Kp             = 0.09f,
    .Ki             = 0.001f,
    .Kd             = 0.0f,
    .abs_output_max = 11,
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
constexpr float GearRadiusMM = 20;                     // 抬升齿轮半径 mm

constexpr float LiftMax    = LiftMaxMM * 1e-3f;
constexpr float LiftMin    = LiftMinMM * 1e-3f;
constexpr float LiftOffset = LiftOffsetMM * 1e-3f;
constexpr float GearRadius = GearRadiusMM * 1e-3f;

constexpr float MaxSpeed       = 0.418; // unit: m/s
constexpr float MaxOnloadAccel = 5.5;   // unit: m/s^2
constexpr float MaxNoloadAccel = 50;    // unit: m/s^2

constexpr Limit DefaultLimit = { .max_spd  = MaxSpeed,
                                 .max_acc  = MaxOnloadAccel,
                                 .max_jerk = MaxOnloadAccel * 50 };

constexpr float CalibrationSpeed = 0.02f; // 校准归零速度 m/s

constexpr float    StalledTorqueMax = 1.5f;                    // 校准时最大扭矩
constexpr float    StalledTorqueMin = 0.95 * StalledTorqueMax; // 堵转时最小扭矩
constexpr float    StalledSpeedMax  = 0.1f * CalibrationSpeed; // 堵转时最大速度
constexpr uint32_t StalledTicks     = 500;                     // 堵转最小保持时间 (ms)

/**
 * 使用到的点位
 */
namespace Position
{
using Lift::LiftMin;

constexpr float Normal = 0.02f;   // 行进默认保持高度 unit m
constexpr float UpStep = 0.22f;   // 比台阶略高 unit m
constexpr float UpR1   = LiftMax; // 比 R1 的台阶高，在最后阶段 unit m
} // namespace Position
} // namespace Lift

/**
 * 底盘解算配置
 */
namespace Motion
{
constexpr PIDMotor::Config MotorWheelVelPIDCfg = { //
    .Kp             = 45.0f,
    .Ki             = 0.15f,
    .Kd             = 0.00f,
    .abs_output_max = 8000.0f
};

constexpr float wheelRadius    = 50.0f;   // 轮子半径 unit mm
constexpr float wheelDistanceX = 454.9f;  // 前后轮距 unit mm
constexpr float wheelDistanceY = 406.78f; // 左右轮距 unit mm
} // namespace Motion

/**
 * 控制器参数
 */
namespace Control
{
constexpr chassis::controller::Master::Config masterCfg = {
    .posture_error_pd_cfg = {
        .vx = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
        .vy = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
        .wz = { .Kp = 30, .Kd = 4.0f, .abs_output_max = 25.0f },
    },
    .limit = {
        .x = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
        .y   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
        .yaw = { .max_spd = 90, .max_acc = 45, .max_jerk = 90 }
    }
};

}
} // namespace Chassis::Config