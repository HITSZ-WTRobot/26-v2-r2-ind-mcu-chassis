/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"
#include "can.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846f
#endif

namespace Device
{

namespace
{
void sensor_init()
{
    using namespace sensors;

    // UartRxSync_RegisterCallback(Sensor::gyro_yaw, config::uart::SensorGyroYaw);
    // Sensor::gyro_yaw = new gyro::HWT101CT(config::uart::SensorGyroYaw);
    //
    // // 开启接收
    // if (!Sensor::gyro_yaw->startReceive())
    //     Error_Handler();
}

void can_init()
{
    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DMMotor::CAN_FilterInit(&hcan1, 1, 0x01);
    CAN_RegisterCallback(&hcan1, motors::DMMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DMMotor::CAN_FilterInit(&hcan2, 15, 0x01);
    CAN_RegisterCallback(&hcan2, motors::DMMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    CAN_InitMainCallback(&hcan1);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    CAN_InitMainCallback(&hcan2);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_wheel_config[4] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = false,
    },
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 3,
            .reverse = false,
    },
    {
            .hcan    = &hcan2,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 4,
            .reverse = true,
    },
};

void wheel_motor_init()
{
    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        Motor::wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

void motor_lift_init()
{
    using motors::DJIMotor, motors::DMMotor;

    Motor::lift_rear = new DMMotor({
            .hcan        = &hcan2,
            .id0         = 0xA,
            .type        = DMMotor::Type::J4310_2EC,
            .mode        = DMMotor::Mode::MIT, // MIT 退化为力矩控制
            .pos_max_rad = 3.14159,            // 3.14159f
            .vel_max_rad = 25,
            .tor_max     = 12,
            .auto_zero   = true,
            .reverse     = true,
    });

    Motor::lift_front = new DMMotor({
            .hcan        = &hcan1,
            .id0         = 0xB,
            .type        = DMMotor::Type::J4310_2EC,
            .mode        = DMMotor::Mode::MIT, // MIT 退化为力矩控制
            .pos_max_rad = 3.14159,            // 3.14159f
            .vel_max_rad = 25,
            .tor_max     = 12,
            .auto_zero   = true,
            .reverse     = true,
    });
}
} // namespace

void init()
{
    // sensor_init();

    can_init();

    wheel_motor_init();

    motor_lift_init();
}

bool isAllConnected()
{
    // if (!Sensor::gyro_yaw->isConnected())
    //     return false;

    // motors
    for (const auto& m : Motor::wheel)
        if (!m->isConnected())
            return false;

    // lifts
    if (!Motor::lift_front->isConnected())
        return false;
    if (!Motor::lift_rear->isConnected())
        return false;

    return true;
}

void waitAllConnections()
{
    while (!isAllConnected())
        osDelay(1);
}
void update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    // motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);

    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    // motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);

    Motor::lift_front->ping();
    Motor::lift_rear->ping();
}
} // namespace Device