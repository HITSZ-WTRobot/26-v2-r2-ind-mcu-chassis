/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "device.hpp"
#include "can.h"
#include "grip/Config.hpp"
#include "i2c.hpp"
#include "project_parts.hpp"
#include "suction/Config.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846f
#endif

namespace Device
{

namespace
{

/// 当前所有 DM 电机反馈目标 ID，必须与驱动器端 Master ID 配置一致。
inline constexpr uint32_t DmMasterId = 0x114U;

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

void pressure_sensor_init()
{
    // 气压计当前未启用/实现，占位保留。
    (void)ProjectParts::EnableGripSuctionPressureSensor;
}

void grip_suction_init()
{
    if constexpr (!ProjectParts::EnableGripSuction)
        return;

    Suction::grip = new ::Suction::SuctionCup(
            { .pump_gpio                     = { RELAY3_GPIO_Port, RELAY3_Pin },
              .pressure_stale_ms             = 120U,
              .object_detect_on_pressure_pa  = ::Suction::Config::DetectOnPressurePa,
              .object_detect_off_pressure_pa = ::Suction::Config::DetectOffPressurePa },
            nullptr);
}

void abdomen_suction_init()
{
    if constexpr (!ProjectParts::EnableAbdomenSuction)
        return;

    Suction::abdomen = new ::Suction::SuctionCup(
            { .pump_gpio                     = { RELAY0_GPIO_Port, RELAY0_Pin },
              .pressure_stale_ms             = 120U,
              .object_detect_on_pressure_pa  = ::Suction::Config::DetectOnPressurePa,
              .object_detect_off_pressure_pa = ::Suction::Config::DetectOffPressurePa },
            nullptr);
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
    if constexpr (ProjectParts::EnableGrip)
    {
        motors::DMMotor::CAN_FilterInit(&hcan2, 15, DmMasterId);
        CAN_RegisterCallback(&hcan2, motors::DMMotor::CANBaseReceiveCallback);
    }

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

constexpr motors::DJIMotor::Config motor_lift_config[4] = {
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 3,
            .reverse        = true,
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
            .id1            = 1,
            .reverse        = true,
            .reduction_rate = 1.0f,
    },
    {
            .hcan           = &hcan2,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 2,
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

    Motor::grip_arm  = new motors::DMMotor({
             .hcan               = &hcan2,
             .id0                = 0x09U,
             .type               = motors::DMMotor::Type::J4310_2EC,
             .mode               = motors::DMMotor::Mode::Vel,
             .pos_max_rad        = 3.14159,
             .vel_max_rad        = 25,
             .tor_max            = 12,
             .default_angle_zero = Grip::Config::Motor::ArmAngleZeroDeg,
             .auto_zero          = false,
             .reverse            = false,
             .reduction_rate     = 1.0f,
    });
    Motor::grip_turn = new motors::DMMotor({
            .hcan           = &hcan2,
            .id0            = 0x0AU,
            .type           = motors::DMMotor::Type::S2325_1EC,
            .mode           = motors::DMMotor::Mode::MIT,
            .pos_max_rad    = 3.14159,
            .vel_max_rad    = 63,
            .tor_max        = 5,
            .auto_zero      = true,
            .reverse        = false,
            .reduction_rate = 1.0f,
    });
}

[[nodiscard]] bool has_dji_motor_on_can1()
{
    return Motor::wheel[0] != nullptr || Motor::wheel[1] != nullptr || Motor::lift[0] != nullptr ||
           Motor::lift[1] != nullptr;
}

[[nodiscard]] bool has_dji_motor_on_can2()
{
    return Motor::wheel[2] != nullptr || Motor::wheel[3] != nullptr || Motor::lift[2] != nullptr ||
           Motor::lift[3] != nullptr;
}

} // namespace

void init()
{
    sensor_init();
    pressure_sensor_init();
    grip_suction_init();
    abdomen_suction_init();

    can_init();

    wheel_motor_init();

    motor_lift_init();

    motor_grip_init();
}

void update_1kHz()
{
    if constexpr (ProjectParts::EnableGrip)
    {
        if (Motor::grip_arm != nullptr)
            Motor::grip_arm->ping();
        if (Motor::grip_turn != nullptr)
            Motor::grip_turn->ping();
    }

    if (has_dji_motor_on_can1())
    {
        motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
        // motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
    }

    if (has_dji_motor_on_can2())
    {
        motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    }
}
} // namespace Device
