/**
 * @file    lift.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "device.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

class Lift
{
public:
    Lift() : front_(motor_lift_front), rear_(motor_lift_rear) {}

    void update_1kHz();

    void update_100Hz();

    void startCalibration();

    [[nodiscard]] bool isCalibrated() const;

    auto& front() { return front_; }
    auto& rear() { return rear_; }

    float allTo(float position);

    [[nodiscard]] bool isFinished() const { return front_.isFinished() && rear_.isFinished(); }

    class LiftSide
    {
    public:
        /**
         *
         * @param motor 大疆电机，因为需要外部速度环
         */
        explicit LiftSide(motors::DJIMotor* motor);

        float to(float position);

        [[nodiscard]] bool isFinished() const { return traj_.isFinished(); }

        [[nodiscard]] float getPosition() const;

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

        void startCalibration();
        void update_1kHz();
        void update_500Hz() { traj_.errorUpdate(); }
        void update_100Hz() { traj_.profileUpdate(0.01); }

        [[nodiscard]] bool isCalibrated() const { return calib_state_ == CalibState::Done; }

        friend class Lift;
    };

private:
    LiftSide front_, rear_;
    uint32_t prescaler_ = 0;
};
extern Lift* lift;

void APP_Lift_BeforeUpdate();