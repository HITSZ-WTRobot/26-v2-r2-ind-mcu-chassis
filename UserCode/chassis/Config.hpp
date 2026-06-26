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
    .Kp = 200.0f, .Ki = 3.0f, .Kd = 0.00f, .abs_output_max = 16384 * 0.75
};

constexpr PD::Config PDErrorCfg{ .Kp = 5, .Kd = 25, .abs_output_max = 60 };

constexpr float OuterAuxWheelToChassisMM = 193.0f;
constexpr float MinWheelToChassisMM      = 192.5f;
constexpr float InnerAuxWheelToChassisMM = 207.0f;

constexpr float ReturnTripDifferenceMM = 5.0f; /// 测量估计的回程差

/// 内侧辅助轮接地时离下限位距离
constexpr float MinToLimitMM = InnerAuxWheelToChassisMM - MinWheelToChassisMM +
                               ReturnTripDifferenceMM; /// 将该距离加上回程差

constexpr float RangeMM = 442.7f; ///  到上限位时离下限位

constexpr float LiftMaxMM    = RangeMM - MinToLimitMM; // 最高抬升位置 unit mm
constexpr float LiftMinMM    = -MinToLimitMM;          // 最低抬升位置 unit mm
constexpr float LiftOffsetMM = MinToLimitMM;           // 辅助轮接地时到机械限位的偏移 (>0) unit mm
constexpr float GearRadiusMM = 25;                     // 抬升齿轮半径 mm
constexpr float GroundingChassisHeightMM = InnerAuxWheelToChassisMM; // 辅助轮接地时底盘离地高度
                                                                     // unit mm

constexpr float LiftMax                = LiftMaxMM * 1e-3f;
constexpr float LiftMin                = LiftMinMM * 1e-3f;
constexpr float LiftOffset             = LiftOffsetMM * 1e-3f;
constexpr float GearRadius             = GearRadiusMM * 1e-3f;
constexpr float GroundingChassisHeight = GroundingChassisHeightMM * 1e-3f;

constexpr float chassisHeightToLiftPosition(const float chassis_height)
{
    return chassis_height - GroundingChassisHeight;
}

constexpr float MaxSpeed = 1.178; // unit: m/s
// constexpr float MaxOnloadAccel = 2.0;   // unit: m/s^2， 对车先好一点
constexpr float MaxOnloadAccel = 5.5; // unit: m/s^2
constexpr float MaxNoloadAccel = 100; // unit: m/s^2

constexpr Limit OnloadLimit{ MaxSpeed, MaxOnloadAccel, MaxOnloadAccel * 50 };

constexpr Limit NoloadLimit{ MaxSpeed, MaxNoloadAccel, MaxNoloadAccel * 50 };

constexpr Limit DefaultLimit = OnloadLimit;

constexpr float CalibrationSpeed = 0.05f; // 校准归零速度 m/s, 该速度无问题，无需增大
constexpr float CalibrationRpm   = -CalibrationSpeed / GearRadius * 60.0f / (2.0f * M_PI);

constexpr float    CalibrationMaxCurrent = 4000.0f; // 约 2 * 1.22 Nm
constexpr uint32_t CalibrationMinTicks   = 800;     // 堵转最小保持时间 (ms)
constexpr float    CalibrationDeadAngle  = 0.1f;    // 堵转检测

constexpr float LiftFinishThresholdMM    = 10.0f;
constexpr float LiftFinishThresholdAngle = LiftFinishThresholdMM / GearRadiusMM / M_PI * 180.0f;
// 允许的角度误差 (deg)

/**
 * 使用到的点位
 *
 * 以下高度为抬升高度（在本机抬升高度表示辅助轮离地高度）
 */
namespace Position
{
using Lift::LiftMin;

constexpr float Normal         = 0.008f;    // 行进默认保持高度 unit m
constexpr float StepTransition = 0.004f;    // 上下台阶过程中的过渡高度 unit m
constexpr float StepUp200      = 0.205f;    // 比 200mm 台阶略高 unit m
constexpr float StepUp400      = 0.405f;    // 比 400mm 台阶略高 unit m
constexpr float StepFinalLow   = Normal;    // 0x50..0x5F 台阶动作结束低底盘高度 unit m
constexpr float StepFinalHigh  = StepUp200; // 0x50..0x5F 台阶动作结束高底盘高度 unit m
constexpr float UpR1           = LiftMax;   // 比 R1 的台阶高，在最后阶段 unit m
constexpr float UpR1EndHeight  = 0.1f;      // TODO: 填入实际值，R1 动作结束后 lift 恢复高度 unit m
} // namespace Position

/// 上R1台阶终点相对于 stepTargetPos 的位姿偏移
/// 即登上 R1 后车体中心在台阶参考系下的位置
constexpr chassis::Posture UpR1EndRelativePos = { .x = 0.45f, .y = 0.0f, .yaw = 0.0f };

constexpr float CalibrationOffsetAngle = LiftOffset / GearRadius / M_PI * 180.0f;

constexpr trajectory::HomingMotorTrajectory<2>::CalibrationConfig CalibrationCfg{
    .speed               = CalibrationRpm,
    .max_current         = CalibrationMaxCurrent,
    .min_ticks           = CalibrationMinTicks,
    .offset              = CalibrationOffsetAngle,
    .target_after_homing = -0.005 / GearRadius / M_PI * 180.0f,
    .dead_angle          = CalibrationDeadAngle,
};
} // namespace Lift

/**
 * 底盘解算配置
 */
namespace Motion
{
constexpr PIDMotor::Config MotorWheelVelPIDCfg = { //
    .Kp             = 200.0f,
    .Ki             = 3.0f,
    .Kd             = 0.0f,
    .abs_output_max = 8000.0f
};

constexpr float WheelRadiusMM    = 50.0f;   // 轮子半径 unit mm
constexpr float WheelDistanceXMM = 458.1f;  // 前后轮距 unit mm
constexpr float WheelDistanceYMM = 401.08f; // 左右轮距 unit mm
} // namespace Motion

namespace BlockInfo
{
constexpr float BlockSize = 1.2f;
}

/**
 * 底盘在 X-Y 方向上的位置信息
 */
namespace ChassisInfo
{
constexpr float AuxiliaryWheelRadiusMM  = 20.0f;  // 辅助轮半径
constexpr float AuxOuterWheelDistanceMM = 720.0f; // 外侧两个辅助轮中心距
constexpr float AuxInnerWheelDistanceMM = 150.0f; // 内侧两个辅助轮中心距
constexpr float ChassisDistanceXMM      = 760.0f; // 车体长度
constexpr float ChassisDistanceYMM      = 510.0f; // 车体宽度

constexpr float constexpr_sqrt_impl(const float x, const float curr, const float prev)
{
    return curr == prev ? curr : constexpr_sqrt_impl(x, 0.5f * (curr + x / curr), curr);
}

constexpr float constexpr_sqrt(const float x)
{
    return x > 0.0f ? constexpr_sqrt_impl(x, x, 0.0f) : 0.0f;
}

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
constexpr float ChassisDistanceY     = ChassisDistanceYMM * 1e-3f;
constexpr float HalfChassisDistanceY = ChassisDistanceY / 2.0f;
constexpr float HalfWheelDistanceY   = Motion::WheelDistanceYMM * 0.5f * 1e-3f;

constexpr float HalfChassisDiagonal = constexpr_sqrt(HalfChassisDistanceX * HalfChassisDistanceX +
                                                     HalfChassisDistanceY * HalfChassisDistanceY);
constexpr float HalfWheelDiagonal   = constexpr_sqrt(AbsWheelX * AbsWheelX +
                                                   HalfWheelDistanceY * HalfWheelDistanceY);

constexpr float SafeDistance = 0.01; // 1cm 安全距离（底盘方向）

constexpr float StepPrepareYThreshold = 0.5f * BlockInfo::BlockSize - HalfChassisDistanceY -
                                        3 * SafeDistance; // 上台阶预备阶段 y 方向允许误差 unit m
constexpr float StepPrepareYawThreshold = 1.0f;           // 台阶动作预备阶段 yaw 允许误差 unit deg

constexpr float StepPrepareDownYThreshold = 0.5f * BlockInfo::BlockSize - HalfChassisDiagonal;
} // namespace ChassisInfo

/**
 * 控制器参数
 */
namespace Control
{
using TrajectoryLimit             = chassis::controller::Master::TrajectoryLimit;
using TrajectoryTrackingThreshold = chassis::controller::Master::TrajectoryTrackingThreshold;
using MasterConfig                = chassis::controller::Master::Config;

#if TEST_ENABLE_AUTO_MAPPING
constexpr TrajectoryLimit DefaultTrajectoryLimit = {
    .x   = { .max_spd = 0.60f, .max_acc = 0.60f, .max_jerk = 12.0f },
    .y   = { .max_spd = 0.60f, .max_acc = 0.60f, .max_jerk = 12.0f },
    .yaw = { .max_spd = 45.0f, .max_acc = 60.0f, .max_jerk = 1200.0f }
};
#else
// max!!
constexpr TrajectoryLimit MaxTrajectoryLimit = {
    .x   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .y   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 150.0f },
    .yaw = { .max_spd = 460, .max_acc = 170, .max_jerk = 170 * 50 }
};

// 下调限速
constexpr float TrajectoryLimitRatio = 0.5;

constexpr TrajectoryLimit DefaultTrajectoryLimit = MaxTrajectoryLimit * TrajectoryLimitRatio;

#endif

/// 上 R1 动作专用底盘轨迹限制，初始与默认限制一致，便于单独调参。
constexpr TrajectoryLimit UpR1TrajectoryLimit = MaxTrajectoryLimit * 0.2;

constexpr TrajectoryTrackingThreshold DefaultTrajectoryTrackingThreshold{
    .x   = 0.01f,
    .y   = 0.01f,
    .yaw = 1.0f,
};

constexpr MasterConfig masterCfg = {
    .posture_error_pd_cfg = {
        .vx = { .Kp = 5.0, .Kd = 25.0f, .abs_output_max = 0.6f },
        .vy = { .Kp = 5.0, .Kd = 25.0f, .abs_output_max = 0.6f },
        .wz = { .Kp = 5.0, .Kd = 25.0f, .abs_output_max = 135.0f },
    },
    .limit              = DefaultTrajectoryLimit,
    .tracking_threshold = DefaultTrajectoryTrackingThreshold,
};

} // namespace Control
} // namespace Chassis::Config
