/**
 * @file    connection.cpp
 * @author  syhanjin
 * @date    2026-04-22
 */
#include "connection.hpp"

#include "cmsis_os2.h"
#include "device.hpp"
#include "project_parts.hpp"
#include "protocol.hpp"

namespace Connection
{
namespace
{
template <typename T> [[nodiscard]] bool is_connected(const T* object)
{
    return object != nullptr && object->isConnected();
}

[[nodiscard]] constexpr uint16_t mask(const Bit bit)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(bit));
}

void set_bit(uint16_t& current, const Bit bit, const bool connected)
{
    if (connected)
        current |= mask(bit);
}

[[nodiscard]] constexpr uint16_t required_mask()
{
    uint16_t required = 0;

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        required |= mask(Bit::Wheel0);
        required |= mask(Bit::Wheel1);
        required |= mask(Bit::Wheel2);
        required |= mask(Bit::Wheel3);
    }

    if constexpr (ProjectParts::EnableLift)
    {
        required |= mask(Bit::LiftFront0);
        required |= mask(Bit::LiftFront1);
        required |= mask(Bit::LiftRear0);
        required |= mask(Bit::LiftRear1);
    }

    if constexpr (ProjectParts::EnableGrip)
    {
        required |= mask(Bit::GripArm);
        required |= mask(Bit::GripTurn);
    }

    if constexpr (ProjectParts::EnableGyro)
        required |= mask(Bit::GyroYaw);

    if constexpr (ProjectParts::EnablePcLocalization)
        required |= mask(Bit::UpperHostLocalization);

    if constexpr (ProjectParts::EnableUpperHostProtocol)
        required |= mask(Bit::UpperHost);

    return required;
}
} // namespace

void updateTable()
{
    uint16_t current = 0;

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        set_bit(current, Bit::Wheel0, is_connected(Device::Motor::wheel[0]));
        set_bit(current, Bit::Wheel1, is_connected(Device::Motor::wheel[1]));
        set_bit(current, Bit::Wheel2, is_connected(Device::Motor::wheel[2]));
        set_bit(current, Bit::Wheel3, is_connected(Device::Motor::wheel[3]));
    }

    if constexpr (ProjectParts::EnableLift)
    {
        set_bit(current, Bit::LiftFront0, is_connected(Device::Motor::lift[0]));
        set_bit(current, Bit::LiftFront1, is_connected(Device::Motor::lift[1]));
        set_bit(current, Bit::LiftRear0, is_connected(Device::Motor::lift[2]));
        set_bit(current, Bit::LiftRear1, is_connected(Device::Motor::lift[3]));
    }

    if constexpr (ProjectParts::EnableGrip)
    {
        set_bit(current, Bit::GripArm, is_connected(Device::Motor::grip_arm));
        set_bit(current, Bit::GripTurn, is_connected(Device::Motor::grip_turn));
    }

    if constexpr (ProjectParts::EnableGyro)
        set_bit(current, Bit::GyroYaw, is_connected(Device::Sensor::gyro_yaw));

    if constexpr (ProjectParts::EnablePcLocalization)
        set_bit(current, Bit::UpperHostLocalization, Protocol::isPcLocalizationConnected());

    if constexpr (ProjectParts::EnableUpperHostProtocol)
        set_bit(current, Bit::UpperHost, is_connected(Protocol::pc_rx));

    table = current;
}

bool isAllConnected()
{
    const uint16_t current = table;

    constexpr uint16_t required = required_mask();

    return (current & required) == required;
}

void waitAll()
{
    while (!isAllConnected())
        osDelay(1);
}
} // namespace Connection
