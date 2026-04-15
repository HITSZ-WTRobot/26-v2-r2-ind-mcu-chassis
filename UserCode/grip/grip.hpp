/**
 * @file    grip.hpp
 * @brief   夹取机构入口
 */
#pragma once

#include "Config.hpp"
#include "cmsis_os2.h"
#include "gpio_driver.h"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include "traits.hpp"

namespace Grip
{

class Grip : traits::NoCopy, traits::NoDelete
{
public:
    Grip();

    bool enable();
    void disable();

    [[nodiscard]] bool enabled() const { return enabled_; }

    void update_1kHz();
    void update_500Hz();
    void update_100Hz();

    void startCalibration();

    [[nodiscard]] bool isCalibrated() const
    {
        return calib_arm_.isCalibrated() && calib_turn_.isCalibrated();
    }

    bool toNoworkPose();
    bool toReadyPose();
    bool toGripOutPose();
    bool toDockingPose();

    void openClaw();
    void closeClaw();

    [[nodiscard]] bool isFinished() const;
    void               waitForFinish() const;

private:
    bool enabled_{ false };

    controllers::MotorVelController arm_vel_controller_;
    controllers::MotorVelController turn_vel_controller_;

    trajectory::MotorTrajectory<1> arm_trajectory_;
    trajectory::MotorTrajectory<1> turn_trajectory_;

    GPIO_t claw_{};

    MotorTrajCalibration calib_arm_, calib_turn_;
};

inline Grip* grip{};

void init();

} // namespace Grip
