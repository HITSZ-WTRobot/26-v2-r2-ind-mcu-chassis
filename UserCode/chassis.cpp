/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"

#include "device.hpp"

constexpr PIDMotor::Config motor_wheel_vel_pid = { //
    .Kp             = 45.0f,
    .Ki             = 0.15f,
    .Kd             = 0.00f,
    .abs_output_max = 8000.0f
};

Chassis*                         chassis_;
controllers::MotorVelController* motor_vel_ctrl[4];

void APP_Chassis_Update_100Hz()
{
    chassis_->trajectoryUpdate();
}

void APP_Chassis_Update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    chassis_->feedbackUpdate(0.001);
    prescaler_500Hz++;
    if (prescaler_500Hz >= 2)
    {
        chassis_->errorUpdate();
        prescaler_500Hz = 0;
    }
    chassis_->controllerUpdate();
}

void APP_Chassis_BeforeUpdate()
{
    using chassis::Mecanum4;
    using controllers::MotorVelController;

    for (size_t i = 0; i < 4; ++i)
        motor_vel_ctrl[i] = new MotorVelController(motor_wheel[i], { .pid = motor_wheel_vel_pid });

    chassis_ = new Chassis(
    Mecanum4({
                .wheel_radius      = 77.0f,              ///< 轮子半径 (unit: mm)
                .wheel_distance_x  = 748.60f,            ///< 左右轮距 (unit: mm)
                .wheel_distance_y  = 500.00f,            ///< 前后轮距 (unit: mm)
                .chassis_type      = Mecanum4::ChassisType::OType, ///< 底盘构型
                .wheel_front_right = motor_vel_ctrl[2], ///< 右前方
                .wheel_front_left  = motor_vel_ctrl[3], ///< 左前方
                .wheel_rear_left   = motor_vel_ctrl[0], ///< 左后方
                .wheel_rear_right  = motor_vel_ctrl[1], ///< 右后方
            }, {
                    .feedback_source = {
                        .wz = &sensor_gyro_yaw->getWz(),
                        .x = &sensor_ops->getBodyX(),
                        .y = &sensor_ops->getBodyY(),
                        .yaw = &sensor_ops->getBodyYaw(),
                    },
            }), {
                .vx = {.Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f},
                .vy = {.Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f},
                .wz = {.Kp = 30, .Kd = 4.0f, .abs_output_max = 25.0f},
            });
}

void APP_Chassis_Init()
{
    if (!chassis_->enable())
        Error_Handler();

    if (!sensor_ops->resetWorldCoord())
        Error_Handler();
}
