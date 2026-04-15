#pragma once
#include "motor_trajectory.hpp"
#include "stm32f4xx_hal.h"

#include <cmath>
#include <cstdint>

class MotorTrajCalibration
{
public:
    struct CalibrationConfig
    {
        float    speed;       // 堵转寻点速度
        float    max_current; // 堵转最大电流
        uint32_t min_ticks;   // 堵转最小持续时间

        float dead_angle = 0.1f; // 堵转检测过程允许的角度误差
    };
    enum class CalibrationState
    {
        Idle,
        Finding, // 检测堵转位置
        // Zeroing, // TODO: 回零
        Done,
    };

    MotorTrajCalibration(controllers::MotorVelController* ctrl,
                         trajectory::MotorTrajectory<1>*  traj,
                         const CalibrationConfig&         calib_config) :
        cfg_(calib_config), ctrl_(ctrl), traj_(traj)
    {
    }

    void startCalibration()
    {
        traj_->disable();

        old_max_current_ = ctrl_->getPID().getConfig().abs_output_max;
        ctrl_->getPID().setOutputMax(cfg_.max_current);
        ctrl_->setRef(cfg_.speed);

        ctrl_->enable();

        sampling_ = false;
        state_    = CalibrationState::Finding;
    }

    void update_1kHz()
    {
        if (state_ == CalibrationState::Finding)
        {
            ctrl_->update();

            if (std::abs(ctrl_->getPID().getOutput()) <= 0.99f * cfg_.max_current)
            {
                sampling_ = false;
                return;
            }

            const uint32_t now = HAL_GetTick();
            if (!sampling_)
            {
                sampling_     = true;
                sample_tick_  = now;
                sample_angle_ = ctrl_->getMotor()->getAngle();
                return;
            }

            if (now - sample_tick_ < cfg_.min_ticks)
                return;

            if (std::abs(ctrl_->getMotor()->getAngle() - sample_angle_) < cfg_.dead_angle)
            {
                state_ = CalibrationState::Done;
                ctrl_->getMotor()->resetAngle();
                ctrl_->setRef(0.0f);
                ctrl_->getPID().setOutputMax(old_max_current_);
                ctrl_->disable();
                traj_->enable();
                return;
            }

            sample_tick_  = now;
            sample_angle_ = ctrl_->getMotor()->getAngle();
        }
    }

    [[nodiscard]] bool isCalibrated() const { return state_ == CalibrationState::Done; }

private:
    CalibrationConfig cfg_;

    float old_max_current_{ 0 };

    controllers::MotorVelController* ctrl_;
    trajectory::MotorTrajectory<1>*  traj_;

    bool     sampling_     = false;
    uint32_t sample_tick_  = 0;
    float    sample_angle_ = 0.0f;

    CalibrationState state_ = CalibrationState::Idle;
};
