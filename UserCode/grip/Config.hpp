/**
 * @file    Config.hpp
 * @brief   夹取机构参数
 */
#pragma once

#include "IChassisDef.hpp"
#include "can.h"
#include "dm.hpp"
#include "gpio_driver.h"
#include "homing_motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "chassis/Config.hpp"

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
constexpr JointPose Standby{ 60.0f, 0.0f };
/// 准备夹取姿态：夹爪张开，等待底盘靠近目标。
constexpr JointPose PrepareGrab{ 0.0f, 0.0f };
/// 夹取执行姿态：夹爪闭合，并把大臂推出完成矛头夹取。
constexpr JointPose Grab{ 45.0f, 0.0f };
/// 对接姿态：夹取完成后转向对接角度，等待底盘移动到最终位置。
constexpr JointPose Docking{ 80.0f, -90.0f };
/// KFS 拾取姿态：吸盘对准取料位置。
constexpr JointPose KfsPickup{ 90.0f, 0.0f };
/// KFS 暂存姿态：吸住卷轴后转到暂存朝向。
constexpr JointPose KfsStore{ -80.0f, 0.0f };
/// KFS 释放姿态：释放吸住的卷轴
inline constexpr const JointPose& KfsRelease = KfsPickup;
} // namespace Poses

namespace InfraredDocking
{
/// 红外对接完成并开爪后，等待物体脱离再回收至该姿态。
constexpr JointPose ReleaseRetractPose{ 60.0f, 0.0f };
/// 红外对接完成开爪后，到回收姿态前的等待时间。
constexpr uint32_t ReleaseRetractDelayMs = 5000U;
} // namespace InfraredDocking

namespace KfsStore
{
inline constexpr uint32_t SuctionPressureUpdatePhaseMs = 5U;
/// 无气压计时：到达取料位后，等待负压建立再认为已吸住。
inline constexpr uint32_t AttachConfirmDelayMs = 150U;
/// 无气压计时：到达释放位并关闭气泵后，等待卷轴脱离再认为已放下。
inline constexpr uint32_t ReleaseConfirmDelayMs = 100U;

} // namespace KfsStore

namespace SpearGrab
{
// TODO: 用实测世界系位姿替换下面 6 个占位矛位。
constexpr chassis::Posture TargetPoses[] = {
    { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
};

using TrajectoryLimit = chassis::controller::Master::TrajectoryLimit;

constexpr TrajectoryLimit PrepareGrabLimit = {
    { 3, 1.2, 30 },
    { 3, 1.2, 30 },
    { 360, 180, 2500 },
};

constexpr TrajectoryLimit GrabLimit = {
    { 0.8, 0.6, 30 },
    { 0.8, 0.6, 30 },
    { 100, 50, 2500 },
};

constexpr uint16_t TargetPosCount = std::size(TargetPoses);

constexpr float SafeDistance = 0.25f; // 夹取后沿目标 x 方向先撤离的安全距离 unit m

// 矛头夹取执行高度 unit m
constexpr float LiftExecute = Chassis::Config::Lift::chassisHeightToLiftPosition(0.455);

constexpr uint32_t ClawCloseDelayMs  = 100U;  // 目标位合爪后的保持等待时间 unit ms
constexpr float    PostGrabLiftRaise = 0.06f; // 合爪后 lift 继续抬高的距离 unit m

// 夹取完成后的对接高度 unit m
constexpr float LiftDocking = Chassis::Config::Lift::chassisHeightToLiftPosition(0.265);

constexpr float PrepareYThreshold     = 0.003f; // prepare 阶段允许的侧向误差 unit m
constexpr float PrepareYawThreshold   = 0.5f;   // prepare 阶段允许的偏航误差 unit deg
constexpr float PrepareLiftZThreshold = 0.002f; // prepare 阶段允许的 lift 高度误差 unit m
} // namespace SpearGrab

namespace Motor
{

constexpr float ArmAngleZeroDeg = 0;

/// 大臂速度环参数
constexpr controllers::MotorVelController::Config ArmVelControllerCfg{
    .ctrl_mode          = controllers::ControlMode::InternalVel,
    .internal_set_ratio = 10,
};

/// 转向电机速度环参数，输出单位为 DM MIT 力矩 Nm。
constexpr controllers::MotorVelController::Config TurnVelControllerCfg{
    .pid       = { .Kp = 0.09f, .Ki = 0.0025f, .Kd = 0.0f, .abs_output_max = 4 },
    .ctrl_mode = controllers::ControlMode::ExternalPID,
};

} // namespace Motor

namespace Calibration
{
constexpr float TurnLockTorque = 0.5f;

constexpr float    deadAngle   = 0.1f;
constexpr uint32_t lockedTicks = 500;

constexpr float TurnCalibVel = -30.0f; // TODO: 调参

constexpr trajectory::HomingMotorTrajectory<1>::CalibrationConfig TurnCalibCfg = { //
    .speed               = TurnCalibVel,
    .max_current         = TurnLockTorque, //
    .min_ticks           = lockedTicks,
    .offset              = 0.0f, // TODO: 调参
    .target_after_homing = Poses::Standby.turn_pos,
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

constexpr PD::Config ArmPDCfg{ .Kp = 5, .Kd = 25, .abs_output_max = 60 };

constexpr velocity_profile::SCurveProfile::Config TurnCfg{
    .max_spd  = 360.0f,
    .max_acc  = 720.0f,
    .max_jerk = 1440.0f,
};

constexpr PD::Config TurnPDCfg{ .Kp = 5, .Kd = 25, .abs_output_max = 60 };

} // namespace Trajectory

} // namespace Grip::Config
