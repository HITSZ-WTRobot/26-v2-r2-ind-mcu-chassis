/**
 * @file    LiftSide.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "LiftSide.hpp"

#include "Config.hpp"
#include "device.hpp"
#include "homing_motor_trajectory.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

#include <algorithm>

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

constexpr Limit max_motor_limit = toMotorLimit(DefaultLimit);

static_assert(max_motor_limit.max_spd <= 2700.0f);

static constexpr float toTrajectoryTarget(const float z_pos)
{
    return std::clamp(z_pos, LiftMin, LiftMax) / GearRadius / M_PI * 180.0f;
}

static constexpr float toPosition(const float motor_angle)
{
    return motor_angle / 180.0f * M_PI * GearRadius - LiftOffset;
}

LiftSide::LiftSide(motors::IMotor* motor0, motors::IMotor* motor1) :
    ctrl_{ { motor0, { PIDCfg } }, { motor1, { PIDCfg } } },
    traj_(trajectory::MotorTrajectory<MotorNum>(ctrl_, max_motor_limit, PDErrorCfg), CalibrationCfg)
{
}

/**
 * 抬升到
 * @param position 目标位置，零点为辅助轮接地，往上为正
 * @return 用时，规划失败为 -1
 */
float LiftSide::to(const float position)
{
    return to(position, trajectory::LinkMode::PreviousCurve);
}

/**
 * 抬升到
 * @param position 目标位置，零点为辅助轮接地，往上为正
 * @param link_mode 轨迹衔接模式
 * @return 用时，规划失败为 -1
 */
float LiftSide::to(const float position, const trajectory::LinkMode link_mode)
{
    if (!traj_.setTarget(toTrajectoryTarget(position), link_mode))
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
    return to(position, limit, trajectory::LinkMode::PreviousCurve);
}

/**
 * 抬升到
 * @param position 目标位置，零点为辅助轮接地，往上为正
 * @param limit 限制，单位 m/s^x
 * @param link_mode 轨迹衔接模式
 * @return 用时，规划失败为 -1
 */
float LiftSide::to(const float position, const Limit& limit, const trajectory::LinkMode link_mode)
{
    if (!traj_.setTarget(toTrajectoryTarget(position), link_mode, toMotorLimit(limit)))
        return -1;
    return traj_.getTotalTime();
}

float LiftSide::getPosition() const
{
    return toPosition(traj_.getCurrentAvePosition());
}

void LiftSide::startCalibration()
{
    traj_.startCalibration();
}

void LiftSide::update_1kHz()
{
    traj_.controllerUpdate();
}

} // namespace Lift
