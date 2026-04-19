/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"
#include "can.h"
#include "cmsis_os2.h"
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
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    using namespace motors;
    for (size_t i = 0; i < 4; ++i)
        Motor::wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

void motor_lift_init()
{
    if constexpr (!ProjectParts::EnableLift)
        return;

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
    return Motor::wheel[0] != nullptr || Motor::wheel[1] != nullptr;
}

[[nodiscard]] bool has_dji_motor_on_can2()
{
    return Motor::wheel[2] != nullptr || Motor::wheel[3] != nullptr || Motor::grip_arm != nullptr ||
           Motor::grip_turn != nullptr;
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

bool isAllConnected()
{
    if constexpr (ProjectParts::EnableGyro)
    {
        if (Sensor::gyro_yaw == nullptr || !Sensor::gyro_yaw->isConnected())
            return false;
    }

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        for (const auto& m : Motor::wheel)
            if (m == nullptr || !m->isConnected())
                return false;
    }

    if constexpr (ProjectParts::EnableLift)
    {
        if (Motor::lift_front == nullptr || !Motor::lift_front->isConnected())
            return false;
        if (Motor::lift_rear == nullptr || !Motor::lift_rear->isConnected())
            return false;
    }

    if constexpr (ProjectParts::EnableGrip)
    {
        if (Motor::grip_arm == nullptr || !Motor::grip_arm->isConnected())
            return false;
        if (Motor::grip_turn == nullptr || !Motor::grip_turn->isConnected())
            return false;
    }

    return true;
}

void waitAllConnections()
{
    while (!isAllConnected())
        osDelay(1);
}
void update_1kHz()
{
    if (has_dji_motor_on_can1())
    {
        motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
        // motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    }

    if (has_dji_motor_on_can2())
    {
        motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
        // motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    }

    if constexpr (ProjectParts::EnableLift)
    {
        if (Motor::lift_front != nullptr)
            Motor::lift_front->ping();
        if (Motor::lift_rear != nullptr)
            Motor::lift_rear->ping();
    }
}
} // namespace Device
