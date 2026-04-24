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

private:
    static constexpr size_t MotorNum = 2;

    controllers::MotorVelController             ctrl_[MotorNum];
    trajectory::HomingMotorTrajectory<MotorNum> traj_;

    bool grounding_ = true;
};
} // namespace Lift
