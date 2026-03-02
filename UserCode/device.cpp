/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"

#include "can.h"

sensors::gyro::HWT101CT* sensor_gyro_yaw;
sensors::ops::ActionOPS* sensor_ops;
motors::DJIMotor*        motor_wheel[4];

// 定义串口回调
UartRxSync_DefineCallback(sensor_gyro_yaw);
UartRxSync_DefineCallback(sensor_ops);

static void sensor_init()
{
    using namespace sensors;

    UartRxSync_RegisterCallback(sensor_gyro_yaw, DEVICE_SENSOR_GYRO_YAW_UART);
    sensor_gyro_yaw = new gyro::HWT101CT(DEVICE_SENSOR_GYRO_YAW_UART);

    UartRxSync_RegisterCallback(sensor_ops, DEVICE_SENSOR_OPS_UART);
    sensor_ops = new ops::ActionOPS(DEVICE_SENSOR_OPS_UART,
                                    { .x_offset   = -276.0f,
                                      .y_offset   = 0.0f,
                                      .yaw_offset = -90.0f,
                                      .yaw_car    = &sensor_gyro_yaw->getYaw() });

    // 开启接收
    if (!sensor_gyro_yaw->startReceive())
        Error_Handler();
    if (!sensor_ops->startReceive())
        Error_Handler();
}

static void can_init()
{
    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_wheel_config[4] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = false,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 3,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 4,
            .reverse = false,
    },
};

static void wheel_motor_init()
{
    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        motor_wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

void Device_Init()
{
    sensor_init();

    can_init();

    wheel_motor_init();
}

bool Device_isAllConnected()
{
    if (!sensor_gyro_yaw->isConnected())
        return false;
    if (!sensor_ops->isConnected())
        return false;

    // motors
    for (const auto& m : motor_wheel)
        if (!m->isConnected())
            return false;

    return true;
}
void Device_WaitAllConnections()
{
    while (!Device_isAllConnected())
        osDelay(1);
}