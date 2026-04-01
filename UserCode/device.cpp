/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"
#include "can.h"
#include "chassis/Config.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846f
#endif

namespace Device
{

namespace
{
void can_init()
{
    // CAN 初始化
    motors::DMMotor::CAN_FilterInit(&hcan1, 1, 0x01);
    CAN_RegisterCallback(&hcan1, motors::DMMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
    // CAN_Fifo0ReceiveCallback); CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void motor_lift_init()
{
    using motors::DMMotor;

    Motor::lift_front = new DMMotor({
            .hcan        = &hcan1,
            .id0         = 0xA,
            .type        = DMMotor::Type::J4310_2EC,
            .mode        = DMMotor::Mode::MIT, // MIT 退化为力矩控制
            .pos_max_rad = 3.14159,            // 3.14159f
            .vel_max_rad = 25,
            .tor_max     = 12,
            .auto_zero   = true,
            .reverse     = true,
    });

    Motor::lift_rear = new DMMotor({
            .hcan        = &hcan1,
            .id0         = 0xB,
            .type        = DMMotor::Type::J4310_2EC,
            .mode        = DMMotor::Mode::MIT, // MIT 退化为力矩控制
            .pos_max_rad = 3.14159,            // 3.14159f
            .vel_max_rad = 25,
            .tor_max     = 12,
            .auto_zero   = true,
            .reverse     = false,
    });
}
} // namespace

void init()
{
    can_init();

    motor_lift_init();
}

bool isAllConnected()
{
    // lifts
    // 只连接一个升降机构
    if constexpr (Chassis::Config::useFrontLift)
        if (!Motor::lift_front->isConnected())
            return false;

    if constexpr (Chassis::Config::useRearLift)
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
    if constexpr (Chassis::Config::useFrontLift)
        Motor::lift_front->ping();
    if constexpr (Chassis::Config::useRearLift)
        Motor::lift_rear->ping();
}
} // namespace Device