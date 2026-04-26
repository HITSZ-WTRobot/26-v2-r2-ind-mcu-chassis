/**
 * @file    grip.hpp
 * @brief   夹取机构入口
 */
#pragma once

#include "Config.hpp"
#include "cmsis_os2.h"
#include "gpio_driver.h"
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
        return arm_trajectory_.isCalibrated() && turn_trajectory_.isCalibrated();
    }

    bool toNoworkPose();
    bool toReadyPose();
    bool toGripOutPose();
    bool toStorePose();
    bool toDockingPose();

    void openClaw();
    void closeClaw();

    [[nodiscard]] bool isFinished() const;
    void               waitForFinish() const;

private:
    bool enabled_{ false };

    controllers::MotorVelController arm_vel_controller_;
    controllers::MotorVelController turn_vel_controller_;

    trajectory::HomingMotorTrajectory<1> arm_trajectory_;
    trajectory::HomingMotorTrajectory<1> turn_trajectory_;

    GPIO_t claw_{};
};

inline Grip* grip{};

void init();

} // namespace Grip
