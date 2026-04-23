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
    explicit LiftSide(motors::IMotor* motor);

    float to(float position);
    float to(float position, const Chassis::Config::Limit& limit);

    void stop() { traj_.stop(); }

    [[nodiscard]] bool isFinished() const { return traj_.isFinished(); }

    void waitFinished() const
    {
        while (!isFinished())
            osDelay(1);
    }

    [[nodiscard]] float getPosition() const;

    bool enable() { return traj_.enable(); }

    [[nodiscard]] bool enabled() const { return ctrl_.enabled(); }

    void disable() { traj_.disable(); }

    void               startCalibration();
    [[nodiscard]] bool isCalibrated() const { return traj_.isCalibrated(); }

    void update_1kHz();
    void update_500Hz() { traj_.errorUpdate(); }
    void update_100Hz() { traj_.profileUpdate(0.01); }

    [[nodiscard]] bool isGrounding() const { return grounding_; }

    void setGrounding(const bool grounding) { grounding_ = grounding; }

private:
    controllers::MotorVelController      ctrl_;
    trajectory::HomingMotorTrajectory<1> traj_;

    bool grounding_ = true;
};
} // namespace Lift
