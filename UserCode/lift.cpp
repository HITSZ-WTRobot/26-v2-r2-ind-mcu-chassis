/**
 * @file    lift.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "lift.hpp"

#include "device.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

constexpr float pi = 3.141592654f;

constexpr PIDMotor::Config pid_cfg = {
    .Kp = 45.0f, .Ki = 0.15f, .Kd = 0.00f, .abs_output_max = 8000.0f
};

constexpr velocity_profile::SCurveProfile::Config profile_cfg = { .max_spd  = 360,
                                                                  .max_acc  = 480,
                                                                  .max_jerk = 1080 };

constexpr PD::Config pd_error_cfg{ .Kp = 5, .Kd = 3, .abs_output_max = 10 };

constexpr float LiftMax    = 330; // 最高抬升位置 unit mm
constexpr float LiftMin    = -10; // 最低抬升位置 unit mm
constexpr float LiftOffset = -15; // 到机械限位的偏移 unit mm
constexpr float GearRadius = 25;  // 抬升齿轮半径 mm

constexpr float CalibrationSpeed = 0.2f; // 校准归零速度 m/s
constexpr float CalibrationRpm = -CalibrationSpeed / GearRadius * 1000 * 60 / (2 * pi); // 转换为RPM
constexpr uint32_t StalledCurrentMax = 3000;
constexpr uint32_t StalledTicks      = 500;

Lift* lift;

/**
 *
 * @param z_pos 底盘 z 坐标 unit: m
 * @return 电机所在角度
 */
static constexpr float toMotorAngle(const float z_pos)
{
    return (z_pos + LiftOffset * 1e-3f) / GearRadius * 1000.0f / pi * 180.0f;
}

static constexpr float toPosition(const float motor_angle)
{
    return motor_angle / 180.0f * pi * GearRadius / 1000.0f - LiftOffset * 1e-3f;
}

float Lift::allTo(const float position)
{
    const auto f = front_.to(position);
    const auto r = rear_.to(position);
    return std::fmaxf(f, r);
}

Lift::LiftSide::LiftSide(motors::DJIMotor* motor) :
    ctrl_(motor, { pid_cfg }), traj_(&ctrl_, profile_cfg, pd_error_cfg)
{
}

float Lift::LiftSide::to(const float position)
{
    if (!traj_.setTarget(toMotorAngle(position)))
        return -1;
    return traj_.getTotalTime();
}

float Lift::LiftSide::getPosition() const
{
    return toPosition(traj_.getCurrentAvePosition());
}

void Lift::update_1kHz()
{
    ++prescaler_;
    if (prescaler_ == 2)
    {
        front_.update_500Hz();
        rear_.update_500Hz();
        prescaler_ = 0;
    }
    front_.update_1kHz();
    rear_.update_1kHz();
}

void Lift::update_100Hz()
{
    front_.update_100Hz();
    rear_.update_100Hz();
}

void Lift::startCalibration()
{
    front_.startCalibration();
    rear_.startCalibration();
}

bool Lift::isCalibrated() const
{
    return front_.isCalibrated() && rear_.isCalibrated();
}

void Lift::LiftSide::startCalibration()
{
    traj_.disable(); // 释放 SCurve 所有权
    ctrl_.getPID().setOutputMax(StalledCurrentMax);
    ctrl_.setRef(CalibrationRpm);
    ctrl_.enable(); // 单独控制速度
    calib_state_   = CalibState::Downing;
    stalled_ticks_ = 0;
}

void Lift::LiftSide::update_1kHz()
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
            ctrl_.getPID().setOutputMax(pid_cfg.abs_output_max);
            calib_state_ = CalibState::Rising;
        }
    }
    else if (calib_state_ == CalibState::Rising && traj_.isFinished())
        calib_state_ = CalibState::Done;

    traj_.controllerUpdate();
}

void APP_Lift_BeforeUpdate()
{
    lift = new Lift();
}
