/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"
#include "can.h"
#include "project_parts.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846f
#endif

namespace Device
{

namespace
{
[[nodiscard]] constexpr bool has_can_devices()
{
    return ProjectParts::EnableWheelChassis || ProjectParts::EnableLift || ProjectParts::EnableGrip;
}

void sensor_init()
{
    if constexpr (!ProjectParts::EnableGyro)
        return;

    using namespace sensors;

    UartRxSync_RegisterCallback(Sensor::gyro_yaw, config::uart::SensorGyroYaw);
    Sensor::gyro_yaw = new gyro::HWT101CT(config::uart::SensorGyroYaw);

    // 开启接收
    if (!Sensor::gyro_yaw->startReceive())
        Error_Handler();
}

void can_init()
{
    if constexpr (!has_can_devices())
        return;

    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

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
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        Motor::wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

// TODO: lift 电机型号 / 减速比 / 实际正方向尚未补充；
// 当前先按 DJI 电流控制链路占位接入，后续补齐这些参数即可。
constexpr motors::DJIMotor::Config motor_lift_config[4] = {
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 3,
            .reverse        = false,
            .reduction_rate = 1.0f,
    },
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 4,
            .reverse        = false,
            .reduction_rate = 1.0f,
    },
    {
            .hcan           = &hcan2,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 5,
            .reverse        = false,
            .reduction_rate = 1.0f,
    },
    {
            .hcan           = &hcan2,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 6,
            .reverse        = false,
            .reduction_rate = 1.0f,
    },
};

void motor_lift_init()
{
    if constexpr (!ProjectParts::EnableLift)
        return;

    using motors::DJIMotor;
    for (size_t i = 0; i < 4; ++i)
        Motor::lift[i] = new DJIMotor(motor_lift_config[i]);
}

void motor_grip_init()
{
    if constexpr (!ProjectParts::EnableGrip)
        return;

    constexpr motors::DJIMotor::Config ArmCfg{
        .hcan = &hcan2,
        .type = motors::DJIMotor::Type::M3508_C620,
        .id1  = 1,
    };

    constexpr motors::DJIMotor::Config TurnCfg{
        .hcan = &hcan2,
        .type = motors::DJIMotor::Type::M2006_C610,
        .id1  = 2,
    };
    Motor::grip_arm  = new motors::DJIMotor(ArmCfg);
    Motor::grip_turn = new motors::DJIMotor(TurnCfg);
}

[[nodiscard]] bool has_dji_motor_on_can1()
{
    return Motor::wheel[0] != nullptr || Motor::wheel[1] != nullptr || Motor::lift[0] != nullptr ||
           Motor::lift[1] != nullptr;
}

[[nodiscard]] bool has_dji_motor_on_can2_group_1_4()
{
    return Motor::wheel[2] != nullptr || Motor::wheel[3] != nullptr || Motor::grip_arm != nullptr ||
           Motor::grip_turn != nullptr;
}

[[nodiscard]] bool has_dji_motor_on_can2_group_5_8()
{
    return Motor::lift[2] != nullptr || Motor::lift[3] != nullptr;
}
} // namespace

void init()
{
    sensor_init();

    can_init();

    wheel_motor_init();

    motor_lift_init();

    motor_grip_init();
}

void update_1kHz()
{
    if (has_dji_motor_on_can1())
    {
        motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
        // motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    }

    if (has_dji_motor_on_can2_group_1_4())
    {
        motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    }

    if (has_dji_motor_on_can2_group_5_8())
    {
        motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    }
}
} // namespace Device
