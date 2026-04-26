/**
 * @file    Config.hpp
 * @brief   夹取机构参数
 */
#pragma once

#include "can.h"
#include "dji.hpp"
#include "homing_motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"

namespace Grip::Config
{

namespace Position
{
// 之后可以改为相对电机绝对零点的角度，引入堵转检测的偏移setoff
constexpr float ArmNowork = 130.0f;
constexpr float ArmReady  = 68.0f;
constexpr float ArmOut    = 100.0f;
constexpr float ArmStore  = 90.0f;      //  吸取卷轴机械臂位置

constexpr float TurnGrip    = 235.0f;
constexpr float TurnDocking = 145.0f;

constexpr float LiftGrab = 0.18f; // 矛头夹取高度 unit m

// 下面的距离参数均为：从当前位姿到目标位姿的相对距离，正值表示车向前，负值表示车向后。
constexpr float gotoReadypos_x  = -10.0f;
constexpr float gotoReadypos_y   = 20.0f;
constexpr float gotoGrabpos_x   = -25.0f;
constexpr float grabBackoff_x   = 25.0f;
} // namespace Position

namespace Motor
{
/// 大臂速度环参数
constexpr controllers::MotorVelController::Config ArmVelControllerCfg{
    .pid = { .Kp = 370.0f, .Ki = 5.0f, .Kd = 0.0f, .abs_output_max = 12000.0f },
};

/// 转向电机速度环参数
constexpr controllers::MotorVelController::Config TurnVelControllerCfg{
    .pid = { .Kp = 400.0f, .Ki = 5.0f, .Kd = 0.0f, .abs_output_max = 2000.0f },
};

} // namespace Motor

namespace Calibration
{
constexpr float ArmLockCurrent  = 2000.0f;
constexpr float TurnLockCurrent = 2000.0f;

constexpr float    deadAngle   = 0.1f;
constexpr uint32_t lockedTicks = 1000;

constexpr float ArmCalibVel  = -30.0f;
constexpr float TurnCalibVel = -30.0f;

constexpr trajectory::HomingMotorTrajectory<1>::CalibrationConfig ArmCalibCfg = { //
    .speed       = ArmCalibVel,
    .max_current = ArmLockCurrent,
    .min_ticks   = lockedTicks,
    .offset      = 0.0f,
    .dead_angle  = deadAngle
};
constexpr trajectory::HomingMotorTrajectory<1>::CalibrationConfig TurnCalibCfg = { //
    .speed       = TurnCalibVel,
    .max_current = TurnLockCurrent,
    .min_ticks   = lockedTicks,
    .offset      = 0.0f,
    .dead_angle  = deadAngle
};

} // namespace Calibration

namespace Trajectory
{
constexpr velocity_profile::SCurveProfile::Config ArmCfg{
    .max_spd  = 360.0f,
    .max_acc  = 720.0f,
    .max_jerk = 1440.0f,
};

constexpr PD::Config ArmPDCfg{
    .Kp = 50.0f,
    .Kd = 5.0f,
};

constexpr velocity_profile::SCurveProfile::Config TurnCfg{
    .max_spd  = 360.0f,
    .max_acc  = 720.0f,
    .max_jerk = 1440.0f,
};

constexpr PD::Config TurnPDCfg{
    .Kp = 50.0f,
    .Kd = 1.0f,
};
} // namespace Trajectory

} // namespace Grip::Config
