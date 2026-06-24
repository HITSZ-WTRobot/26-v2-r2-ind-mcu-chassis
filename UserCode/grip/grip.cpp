/**
 * @file    grip.cpp
 * @brief   夹取机构入口
 */
#include "grip.hpp"

#include "cmsis_os2.h"
#include "device.hpp"

namespace Grip
{

Grip::Grip() :
    arm_vel_controller_(Device::Motor::grip_arm, Config::Motor::ArmVelControllerCfg),
    turn_vel_controller_(Device::Motor::grip_turn, Config::Motor::TurnVelControllerCfg),
    arm_trajectory_(&arm_vel_controller_, Config::Trajectory::ArmCfg, Config::Trajectory::ArmPDCfg),
    turn_trajectory_(trajectory::MotorTrajectory<1>(&turn_vel_controller_,
                                                    Config::Trajectory::TurnCfg,
                                                    Config::Trajectory::TurnPDCfg),
                     Config::Calibration::TurnCalibCfg),
    claw_{ RELAY3_GPIO_Port, RELAY3_Pin }
{
    // 默认上电保持张开，避免在校准前误夹持机构或工件。
    openClaw();
}

void init()
{
    // grip 采用单例式入口，重复初始化时直接复用已有实例。
    if (grip != nullptr)
        return;

    grip = new Grip();
}

bool Grip::enable()
{
    // 固定零点初始化完成前禁止使能轨迹控制，避免直接用未定义零点规划姿态。
    if (!isCalibrated())
        return false;

    bool ok = true;
    ok &= arm_trajectory_.enable();
    ok &= turn_trajectory_.enable();

    // 任一轴使能失败就整体回退，保持双轴状态一致。
    if (!ok)
        disable();

    enabled_ = ok;
    return enabled_;
}

void Grip::disable()
{
    // disable() 内部会先 stop，再关闭速度环输出。
    arm_trajectory_.disable();
    turn_trajectory_.disable();
    enabled_ = false;
}

void Grip::update_500Hz_2()
{
    // 速度环是最快路径，放在 1 kHz 定时回调中。
    // arm_trajectory_.controllerUpdate(); 内部速度环，不需要主动维持 controllerUpdate()
    turn_trajectory_.controllerUpdate();
}

void Grip::update_500Hz_1()
{
    // 误差补偿频率低于速度环，但仍需比轨迹推进更快。
    arm_trajectory_.errorUpdate();
    turn_trajectory_.errorUpdate();
}

void Grip::update_100Hz()
{
    // 双轴轨迹都按同一个固定 dt 推进，确保姿态组合时序一致。
    arm_trajectory_.profileUpdate(0.01);
    turn_trajectory_.profileUpdate(0.01);
}

void Grip::startCalibration()
{
    if (!turn_trajectory_.enable() || !arm_trajectory_.enable())
    {
        turn_trajectory_.disable();
        arm_trajectory_.disable();
        return;
    }

    // turn 轴回零，由轨迹对象内部维护校准状态机。
    turn_trajectory_.startCalibration();
    // arm 轴直接移动到 StandBy 位置
    arm_trajectory_.setTarget(Config::Poses::Standby.arm_pos);
}

bool Grip::planPose(const Config::JointPose& pose)
{
    // 每次新规划前清空旧失败信息，避免上层把上一轮失败误报到本轮动作。
    last_plan_failure_mask_ = 0;
    last_plan_failure_      = {};

    // 先规划大臂，再规划转向；任一失败都必须把两轴都停住。
    if (!arm_trajectory_.setTarget(pose.arm_pos))
    {
        // bit0 固定表示 arm 轴，供动作诊断按 axis0 展开。
        last_plan_failure_mask_ = 1U << 0U;
        last_plan_failure_[0]   = arm_trajectory_.lastFailureInfo();
        arm_trajectory_.stop();
        turn_trajectory_.stop();
        return false;
    }

    if (!turn_trajectory_.setTarget(pose.turn_pos))
    {
        // bit1 固定表示 turn 轴，供动作诊断按 axis1 展开。
        last_plan_failure_mask_ = 1U << 1U;
        last_plan_failure_[1]   = turn_trajectory_.lastFailureInfo();
        arm_trajectory_.stop();
        turn_trajectory_.stop();
        return false;
    }

    return true;
}

bool Grip::toStandbyPose()
{
    // 待机时必须保证夹爪张开，避免对接或收纳阶段带着夹持力。
    openClaw();
    return planPose(Config::Poses::Standby);
}

bool Grip::toPrepareGrabPose()
{
    // 准备夹取同样要求夹爪先张开，留给底盘切入和最终夹持动作余量。
    openClaw();
    return planPose(Config::Poses::PrepareGrab2);
}

bool Grip::toGrabPose()
{
    // 夹取执行前先闭爪，再推进到最终夹取姿态。
    closeClaw();
    return planPose(Config::Poses::Grab);
}

bool Grip::toDockingPose()
{
    return planPose(Config::Poses::Docking);
}

bool Grip::toKfsPickupPose()
{
    return planPose(Config::Poses::KfsPickup);
}

bool Grip::toKfsStorePose()
{
    return planPose(Config::Poses::KfsStore);
}

bool Grip::toKfsReleasePose()
{
    return planPose(Config::Poses::KfsRelease);
}

bool Grip::toJointPose(const Config::JointPose& pose)
{
    return planPose(pose);
}

float Grip::armPosition() const
{
    return arm_vel_controller_.getMotor()->getAngle();
}

void Grip::openClaw()
{
    GPIO_ResetPin(&claw_);
}

void Grip::closeClaw()
{
    GPIO_SetPin(&claw_);
}

void Grip::stop()
{
    arm_trajectory_.stop();
    turn_trajectory_.stop();
}

bool Grip::isFinished() const
{
    // 只有校准完成且两条轨迹都跑完，外层动作组才可继续推进。
    return isCalibrated() && arm_trajectory_.isFinished() && turn_trajectory_.isFinished();
}

void Grip::waitForFinish() const
{
    // Grip 没有独立事件通知，当前仍使用简单轮询等待。
    while (!isFinished())
        osDelay(10);
}

} // namespace Grip
