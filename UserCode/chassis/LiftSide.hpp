/**
 * @file    LiftSide.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "Config.hpp"
#include "device.hpp"
#include "homing_motor_trajectory.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

namespace Lift
{
class LiftSide
{
public:
    LiftSide(motors::IMotor* motor0, motors::IMotor* motor1);

    float to(float position);
    float to(float position, trajectory::LinkMode link_mode);
    float to(float position, const Chassis::Config::Limit& limit);
    float to(float position, const Chassis::Config::Limit& limit, trajectory::LinkMode link_mode);

    void stop() { traj_.stop(); }

    [[nodiscard]] bool isFinished() const { return traj_.isFinished(); }

    /// 返回最近一次升降轨迹规划失败详情，供上层动作诊断展开 S 曲线错误。
    [[nodiscard]] const velocity_profile::SCurveProfile::FailureInfo& lastPlanFailureInfo() const
    {
        return traj_.lastFailureInfo();
    }

    void waitFinished() const
    {
        while (!isFinished())
            osDelay(1);
    }

    [[nodiscard]] float getPosition() const;

    bool enable() { return traj_.enable(); }

    [[nodiscard]] bool enabled() const { return ctrl_[0].enabled() && ctrl_[1].enabled(); }

    void disable() { traj_.disable(); }

    void               startCalibration();
    [[nodiscard]] bool isCalibrated() const { return traj_.isCalibrated(); }

    void update_1kHz();
    void update_500Hz() { traj_.errorUpdate(); }
    void update_100Hz() { traj_.profileUpdate(0.01); }

    [[nodiscard]] bool isGrounding() const { return grounding_; }

    void setGrounding(const bool grounding) { grounding_ = grounding; }

    /// 获取电机控制器引用（0/1），用于外部直接下发速度参考。
    [[nodiscard]] controllers::MotorVelController& motor(const size_t index)
    {
        return ctrl_[index];
    }

private:
    static constexpr size_t MotorNum = 2;

    controllers::MotorVelController             ctrl_[MotorNum];
    trajectory::HomingMotorTrajectory<MotorNum> traj_;

    bool grounding_ = true;
};
} // namespace Lift
