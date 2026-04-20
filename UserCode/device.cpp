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

template <typename T> [[nodiscard]] bool is_connected(const T* device)
{
    return device != nullptr && device->isConnected();
}

[[nodiscard]] constexpr uint16_t connection_mask(const ConnectionBit bit)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(bit));
}

void set_connection_bit(uint16_t& table, const ConnectionBit bit, const bool connected)
{
    if (connected)
        table |= connection_mask(bit);
}

[[nodiscard]] constexpr uint16_t required_connection_mask()
{
    uint16_t mask = 0;

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        mask |= connection_mask(ConnectionBit::Wheel0);
        mask |= connection_mask(ConnectionBit::Wheel1);
        mask |= connection_mask(ConnectionBit::Wheel2);
        mask |= connection_mask(ConnectionBit::Wheel3);
    }

    if constexpr (ProjectParts::EnableLift)
    {
        mask |= connection_mask(ConnectionBit::LiftFront);
        mask |= connection_mask(ConnectionBit::LiftRear);
    }

    if constexpr (ProjectParts::EnableGrip)
    {
        mask |= connection_mask(ConnectionBit::GripArm);
        mask |= connection_mask(ConnectionBit::GripTurn);
    }

    if constexpr (ProjectParts::EnableGyro)
        mask |= connection_mask(ConnectionBit::GyroYaw);

    return mask;
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

void updateConnectionTable()
{
    uint16_t table = 0;

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        set_connection_bit(table, ConnectionBit::Wheel0, is_connected(Motor::wheel[0]));
        set_connection_bit(table, ConnectionBit::Wheel1, is_connected(Motor::wheel[1]));
        set_connection_bit(table, ConnectionBit::Wheel2, is_connected(Motor::wheel[2]));
        set_connection_bit(table, ConnectionBit::Wheel3, is_connected(Motor::wheel[3]));
    }

    if constexpr (ProjectParts::EnableLift)
    {
        set_connection_bit(table, ConnectionBit::LiftFront, is_connected(Motor::lift_front));
        set_connection_bit(table, ConnectionBit::LiftRear, is_connected(Motor::lift_rear));
    }

    if constexpr (ProjectParts::EnableGrip)
    {
        set_connection_bit(table, ConnectionBit::GripArm, is_connected(Motor::grip_arm));
        set_connection_bit(table, ConnectionBit::GripTurn, is_connected(Motor::grip_turn));
    }

    if constexpr (ProjectParts::EnableGyro)
        set_connection_bit(table, ConnectionBit::GyroYaw, is_connected(Sensor::gyro_yaw));

    connection_table = table;
}

bool isAllConnected()
{
    const uint16_t table = connection_table;

    constexpr uint16_t mask = required_connection_mask();

    return (table & mask) == mask;
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
