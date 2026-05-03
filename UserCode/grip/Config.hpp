/**
 * @file    Config.hpp
 * @brief   夹取机构参数
 */
#pragma once

#include "IChassisDef.hpp"
#include "can.h"
#include "dji.hpp"
#include "gpio_driver.h"
#include "homing_motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "suction/SuctionCup.hpp"

#include <cstdint>

namespace Grip::Config
{

struct JointPose
{
    /// 大臂目标角度，单位 deg，基于 grip 自身校准零点。
    float arm_pos;
    /// 转向目标角度，单位 deg，基于 grip 自身校准零点。
    float turn_pos;
};

namespace Poses
{
/// 待机姿态：系统空闲、KFS 释放完成后都应回到这里。
constexpr JointPose Standby{ 90.0f, 0.0f };
/// 准备夹取姿态：夹爪张开，等待底盘靠近目标。
constexpr JointPose PrepareGrab{ 0.0f, 0.0f };
/// 夹取执行姿态：夹爪闭合，并把大臂推出完成矛头夹取。
constexpr JointPose Grab{ 45.0f, 0.0f };
/// 对接姿态：夹取完成后转向对接角度，等待底盘移动到最终位置。
constexpr JointPose Docking{ 0.0f, -91.0f };
/// KFS 拾取姿态：吸盘对准取料位置。
constexpr JointPose KfsPickup{ 90.0f, 0.0f };
/// KFS 暂存姿态：吸住卷轴后转到暂存朝向。
constexpr JointPose KfsStore{ -80.0f, 0.0f };
/// KFS 释放姿态：释放吸住的卷轴
inline constexpr const JointPose& KfsRelease = KfsPickup;
} // namespace Poses

namespace KfsStore
{
inline constexpr uint32_t SuctionPressureUpdatePhaseMs = 5U;
/// 无气压计时：到达取料位后，等待负压建立再认为已吸住。
inline constexpr uint32_t AttachConfirmDelayMs = 150U;
/// 无气压计时：到达释放位并关闭气泵后，等待卷轴脱离再认为已放下。
inline constexpr uint32_t ReleaseConfirmDelayMs = 100U;

inline const Suction::SuctionCup::Config SuctionCupConfig{
    .pump_gpio                     = { GRIP_SUCTION_GPIO_Port, GRIP_SUCTION_Pin },
    .pressure_stale_ms             = 120U,
    .object_detect_on_pressure_pa  = Suction::Config::DetectOnPressurePa,
    .object_detect_off_pressure_pa = Suction::Config::DetectOffPressurePa,
};
} // namespace KfsStore

namespace SpearGrab
{
// TODO: 用实测世界系位姿替换下面 6 个占位矛位。
constexpr chassis::Posture TargetPoses[] = {
    { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
};

constexpr uint16_t TargetPosCount = std::size(TargetPoses);

constexpr float SafeDistance = 0.20f; // 夹取后沿目标 x 方向先撤离的安全距离 unit m
constexpr float LiftExecute  = 0.18f; // 矛头夹取执行高度 unit m
constexpr float LiftDocking  = 0.01f; // 夹取完成后的对接高度 unit m

constexpr float PrepareYThreshold   = 0.005f; // prepare 阶段允许的侧向误差 unit m
constexpr float PrepareYawThreshold = 0.5f;   // prepare 阶段允许的偏航误差 unit deg
} // namespace SpearGrab

namespace Motor
{
/// 大臂速度环参数
constexpr controllers::MotorVelController::Config ArmVelControllerCfg{
    .pid = { .Kp = 370.0f, .Ki = 5.0f, .Kd = 0.0f, .abs_output_max = 12000.0f },
};

/// 转向电机速度环参数
constexpr controllers::MotorVelController::Config TurnVelControllerCfg{
    .pid = { .Kp = 500.0f, .Ki = 5.0f, .Kd = 0.0f, .abs_output_max = 4000.0f },
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
    .speed               = ArmCalibVel,                                           //
    .max_current         = ArmLockCurrent,                                        //
    .min_ticks           = lockedTicks,                                           //
    .offset              = -100.0f,
    .target_after_homing = 90.0f,
    .dead_angle          = deadAngle
};
constexpr trajectory::HomingMotorTrajectory<1>::CalibrationConfig TurnCalibCfg = { //
    .speed               = TurnCalibVel,
    .max_current         = TurnLockCurrent, //
    .min_ticks           = lockedTicks,
    .offset              = 200.0f, //
    .target_after_homing = 0.0f,
    .dead_angle          = deadAngle
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
