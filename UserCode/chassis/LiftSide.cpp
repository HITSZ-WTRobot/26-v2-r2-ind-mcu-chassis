/**
 * @file    LiftSide.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "LiftSide.hpp"

#include "Config.hpp"
#include "device.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

#include <algorithm>
#include <cmath>

namespace Lift
{

using namespace Chassis::Config;
using namespace Chassis::Config::Lift;

constexpr float toMotorSpeed(const float m)
{
    return m / GearRadius / M_PI * 180.0f;
}

constexpr Limit toMotorLimit(const Limit& limit)
{
    return { .max_spd  = toMotorSpeed(limit.max_spd),
             .max_acc  = toMotorSpeed(limit.max_acc),
             .max_jerk = toMotorSpeed(limit.max_jerk) };
}

// 4310 最大速度 1200deg/s
constexpr Limit max_motor_limit = toMotorLimit(DefaultLimit);

static_assert(max_motor_limit.max_spd <= 1200);

constexpr float CalibrationRpm = -CalibrationSpeed / GearRadius * 60 / (2 * M_PI); // 转换为RPM

/**
 *
 * @param z_pos 底盘 z 坐标 unit: m
 * @return 电机所在角度
 */
static constexpr float toMotorAngle(const float z_pos)
{
    return (std::clamp(z_pos, LiftMin, LiftMax) + LiftOffset) / GearRadius / M_PI * 180.0f;
}

static constexpr float toPosition(const float motor_angle)
{
    return motor_angle / 180.0f * M_PI * GearRadius - LiftOffset;
}

LiftSide::LiftSide(motors::IMotor* motor) :
    ctrl_(motor, { PIDCfg, controllers::ControlMode::ExternalPID }),
    traj_(&ctrl_, max_motor_limit, PDErrorCfg)
{
}

/**
 * 抬升到
 * @param position 目标位置，零点为辅助轮接地，往上为正
 * @return 用时，规划失败为 -1
 */
float LiftSide::to(const float position)
{
    if (!traj_.setTarget(toMotorAngle(position)))
        return -1;
    return traj_.getTotalTime();
}

/**
 * 抬升到
 * @param position 目标位置，零点为辅助轮接地，往上为正
 * @param limit 限制，单位 m/s^x
 * @return 用时，规划失败为 -1
 */
float LiftSide::to(const float position, const Limit& limit)
{
    if (!traj_.setTarget(toMotorAngle(position), toMotorLimit(limit)))
        return -1;
    return traj_.getTotalTime();
}

float LiftSide::getPosition() const
{
    return toPosition(traj_.getCurrentAvePosition());
}

void LiftSide::startCalibration()
{
    traj_.disable(); // 释放 SCurve 所有权
    ctrl_.getPID().setOutputMax(StalledCurrentMax);
    ctrl_.setRef(CalibrationRpm);
    ctrl_.enable(); // 单独控制速度
    calib_state_   = CalibState::Downing;
    stalled_ticks_ = 0;
}

void LiftSide::update_1kHz()
{
    if (calib_state_ == CalibState::Downing)
    {
        if (fabsf(StalledCurrentMax - ctrl_.getPID().getOutput()) < 10 &&
            fabsf(ctrl_.getMotor()->getVelocity()) < 0.1f * CalibrationRpm)
            stalled_ticks_++;
        else
            stalled_ticks_ = 0;
        if (stalled_ticks_ > StalledTicks)
        {
            ctrl_.getMotor()->resetAngle(); // 重置当前电机角度
            ctrl_.setRef(0);                // 停止
            traj_.enable();                 // traj 接管速度环
            traj_.setTarget(toMotorAngle(0));
            ctrl_.getPID().setOutputMax(PIDCfg.abs_output_max);
            calib_state_ = CalibState::Rising;
        }
    }
    else if (calib_state_ == CalibState::Rising && traj_.isFinished())
        calib_state_ = CalibState::Done;

    traj_.controllerUpdate();
}

} // namespace Lift