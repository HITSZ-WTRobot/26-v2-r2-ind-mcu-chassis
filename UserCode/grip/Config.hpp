/**
 * @file    Config.hpp
 * @brief   夹取机构参数
 */
#pragma once

// 这一层集中放 grip 机构的语义姿态、轨迹限幅和校准参数。
// 上层动作模块只引用这里的命名常量，不在动作代码里散落角度或电流魔法数。

#include "IChassisDef.hpp"
#include "can.h"
#include "dji.hpp"
#include "gpio_driver.h"
#include "homing_motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "chassis/Config.hpp"
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
// 这些姿态是“机械臂语义姿态”，不是底盘世界系位姿。
/// 待机姿态：系统空闲、KFS 释放完成后都应回到这里。
constexpr JointPose Standby{ 90.0f, 0.0f };
/// 准备夹取姿态：夹爪张开，等待底盘靠近目标。
constexpr JointPose PrepareGrab{ 0.0f, 0.0f };
/// 夹取执行姿态：夹爪闭合，并把大臂推出完成矛头夹取。
constexpr JointPose Grab{ 45.0f, 0.0f };
/// 对接姿态：夹取完成后转向对接角度，等待底盘移动到最终位置。
constexpr JointPose Docking{ 60.0f, -100.0f };
/// KFS 拾取姿态：吸盘对准取料位置。
constexpr JointPose KfsPickup{ 90.0f, 0.0f };
/// KFS 暂存姿态：吸住卷轴后转到暂存朝向。
constexpr JointPose KfsStore{ -80.0f, 0.0f };
/// KFS 释放姿态：释放吸住的卷轴
inline constexpr const JointPose& KfsRelease = KfsPickup;
} // namespace Poses

namespace KfsStore
{
// KFS 动作的 owner-side 时序参数都放在这里，动作逻辑只引用这些常量。
inline constexpr uint32_t SuctionPressureUpdatePhaseMs = 5U;
/// 无气压计时：到达取料位后，等待负压建立再认为已吸住。
inline constexpr uint32_t AttachConfirmDelayMs = 150U;
/// 无气压计时：到达释放位并关闭气泵后，等待卷轴脱离再认为已放下。
inline constexpr uint32_t ReleaseConfirmDelayMs = 100U;

inline const Suction::SuctionCup::Config SuctionCupConfig{
    // 吸盘组件本身不知道自己属于谁，这里只把 owner 的 GPIO 和阈值装配进去。
    .pump_gpio                     = { GRIP_SUCTION_GPIO_Port, GRIP_SUCTION_Pin },
    .pressure_stale_ms             = 120U,
    .object_detect_on_pressure_pa  = Suction::Config::DetectOnPressurePa,
    .object_detect_off_pressure_pa = Suction::Config::DetectOffPressurePa,
};
} // namespace KfsStore

namespace SpearGrab
{
// TODO: 用实测世界系位姿替换下面 6 个占位矛位。
// 这个表是固定矛位索引的唯一来源，上位机 TakeSpearById 会直接引用它。
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

// 目标矛位个数，便于上位机与本地动作层做边界检查。
constexpr uint16_t TargetPosCount = std::size(TargetPoses);

constexpr float SafeDistance = 0.25f; // 夹取后沿目标 x 方向先撤离的安全距离 unit m

// 矛头夹取执行高度 unit m
constexpr float LiftExecute = Chassis::Config::Lift::chassisHeightToLiftPosition(0.475);

constexpr uint32_t ClawCloseDelayMs  = 100U;  // 目标位合爪后的保持等待时间 unit ms
constexpr float    PostGrabLiftRaise = 0.06f; // 合爪后 lift 继续抬高的距离 unit m

// 夹取完成后的对接高度 unit m
constexpr float LiftDocking = Chassis::Config::Lift::chassisHeightToLiftPosition(0.300);

constexpr float PrepareYThreshold     = 0.003f; // prepare 阶段允许的侧向误差 unit m
constexpr float PrepareYawThreshold   = 0.5f;   // prepare 阶段允许的偏航误差 unit deg
constexpr float PrepareLiftZThreshold = 0.002f; // prepare 阶段允许的 lift 高度误差 unit m
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
// 校准相关参数只在回零流程里使用，和正常轨迹参数分开保存。
constexpr float ArmLockCurrent  = 8000.0f;
constexpr float TurnLockCurrent = 2000.0f;

constexpr float    deadAngle   = 0.1f;
constexpr uint32_t lockedTicks = 1000;

constexpr float ArmCalibVel  = 15.0f;
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
    .offset              = 205.0f, //
    .target_after_homing = 0.0f,
    .dead_angle          = deadAngle
};

} // namespace Calibration

namespace Trajectory
{
// 轨迹层只关心限速、限加速度和 PD 补偿参数。
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
