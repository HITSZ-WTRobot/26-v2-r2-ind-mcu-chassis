/**
 * @file    grip.cpp
 * @brief   夹取机构入口
 */
#include "grip.hpp"

#include "cmsis_os2.h"
#include "device.hpp"

#include <cmath>

namespace Grip
{

Grip::Grip() :
    arm_vel_controller_(Device::Motor::grip_arm, Config::Motor::ArmVelControllerCfg),
    turn_vel_controller_(Device::Motor::grip_turn, Config::Motor::TurnVelControllerCfg),
    arm_trajectory_(&arm_vel_controller_, Config::Trajectory::ArmCfg, Config::Trajectory::ArmPDCfg),
    turn_trajectory_(&turn_vel_controller_,
                     Config::Trajectory::TurnCfg,
                     Config::Trajectory::TurnPDCfg),
    claw_{ GRIP_OUT_GPIO_Port, GRIP_OUT_Pin }, //
    calib_arm_(&arm_vel_controller_, &arm_trajectory_, Config::Calibration::ArmCalibCfg),
    calib_turn_(&turn_vel_controller_, &turn_trajectory_, Config::Calibration::TurnCalibCfg)
{
    openClaw();
}

void init()
{
    if (grip != nullptr)
        return;

    grip = new Grip();
}

bool Grip::enable()
{
    bool ok = true;
    ok &= arm_trajectory_.enable();
    ok &= turn_trajectory_.enable();

    if (!ok)
        disable();

    enabled_ = ok;
    return enabled_;
}

void Grip::disable()
{
    arm_trajectory_.disable();
    turn_trajectory_.disable();
    enabled_ = false;
}

void Grip::update_1kHz()
{
    calib_arm_.update_1kHz();
    calib_turn_.update_1kHz();
    arm_trajectory_.controllerUpdate();
    turn_trajectory_.controllerUpdate();
}

void Grip::update_500Hz()
{
    arm_trajectory_.errorUpdate();
    turn_trajectory_.errorUpdate();
}

void Grip::update_100Hz()
{
    arm_trajectory_.profileUpdate(0.01);
    turn_trajectory_.profileUpdate(0.01);
}

void Grip::startCalibration()
{
    if (!enabled())
        return;
    calib_arm_.startCalibration();
    calib_turn_.startCalibration();
}

bool Grip::toNoworkPose()
{
    const bool arm_ok  = arm_trajectory_.setTarget(Config::Position::ArmNowork);
    const bool turn_ok = turn_trajectory_.setTarget(Config::Position::TurnGrip);
    openClaw();
    return arm_ok && turn_ok;
}

bool Grip::toReadyPose()
{
    const bool arm_ok  = arm_trajectory_.setTarget(Config::Position::ArmReady);
    const bool turn_ok = turn_trajectory_.setTarget(Config::Position::TurnGrip);
    openClaw();
    return arm_ok && turn_ok;
}

bool Grip::toGripOutPose()
{
    closeClaw();
    const bool arm_ok  = arm_trajectory_.setTarget(Config::Position::ArmOut);
    const bool turn_ok = turn_trajectory_.setTarget(Config::Position::TurnGrip);
    return arm_ok && turn_ok;
}

bool Grip::toDockingPose()
{
    const bool arm_ok  = arm_trajectory_.setTarget(Config::Position::ArmReady);
    const bool turn_ok = turn_trajectory_.setTarget(Config::Position::TurnDocking);
    return arm_ok && turn_ok;
}

void Grip::openClaw()
{
    GPIO_ResetPin(&claw_);
}

void Grip::closeClaw()
{
    GPIO_SetPin(&claw_);
}

bool Grip::isFinished() const
{
    return arm_trajectory_.isFinished() && turn_trajectory_.isFinished();
}

void Grip::waitForFinish() const
{
    while (!isFinished())
        osDelay(10);
}

} // namespace Grip
