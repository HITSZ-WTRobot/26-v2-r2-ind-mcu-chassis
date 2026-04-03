/**
 * @file    LiftSide.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "Config.hpp"
#include "device.hpp"
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
    [[nodiscard]] bool isCalibrated() const { return calib_state_ == CalibState::Done; }

    void update_1kHz();
    void update_500Hz() { traj_.errorUpdate(); }
    void update_100Hz() { traj_.profileUpdate(0.01); }

private:
    controllers::MotorVelController ctrl_;
    trajectory::MotorTrajectory<1>  traj_;

    // 堵转检测来复位
    enum class CalibState
    {
        Idle,
        Downing, // 降低底盘寻找限位
        Rising,  // 抬升底盘到达零点
        Done,
    };
    CalibState calib_state_   = CalibState::Idle;
    uint32_t   stalled_ticks_ = 0;
};
} // namespace Lift