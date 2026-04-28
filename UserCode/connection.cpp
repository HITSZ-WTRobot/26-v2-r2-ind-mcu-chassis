/**
 * @file    connection.cpp
 * @author  syhanjin
 * @date    2026-04-22
 */
#include "connection.hpp"

#include "cmsis_os2.h"
#include "device.hpp"
#include "i2c.hpp"
#include "project_parts.hpp"
#include "protocol.hpp"

namespace Connection
{
namespace
{
constexpr uint8_t  ConnectionTableI2CAddress7bit = 0x10U;
constexpr bool     ConnectionTableI2CUseMemWrite = false;
constexpr uint8_t  ConnectionTableI2CRegister    = 0x00U;
constexpr bool     ConnectionTableI2CBigEndian   = true;
constexpr uint32_t ConnectionTableI2CPeriodMs    = 200U; // 5Hz
constexpr uint32_t ConnectionTableI2CPhaseMs     = 0U;
constexpr uint32_t ConnectionTableI2CTimeoutMs   = 20U;

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

void encode_connection_table(uint8_t payload[2], const uint16_t value)
{
    if constexpr (ConnectionTableI2CBigEndian)
    {
        payload[0] = static_cast<uint8_t>(value >> 8);
        payload[1] = static_cast<uint8_t>(value);
    }
    else
    {
        payload[0] = static_cast<uint8_t>(value);
        payload[1] = static_cast<uint8_t>(value >> 8);
    }
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

class ConnectionTableI2CDevice final : public I2CDevice
{
public:
    [[nodiscard]] const char* name() const override { return "connection-table"; }

    [[nodiscard]] uint8_t address7bit() const override { return ConnectionTableI2CAddress7bit; }

    bool init(I2CBusDMA& bus, const uint32_t timeout_ms) override
    {
        return transmit(bus, timeout_ms);
    }

protected:
    bool onRead(I2CBusDMA& bus, const uint32_t now_ms, const uint32_t timeout_ms) override
    {
        (void)now_ms;
        return transmit(bus, timeout_ms);
    }

private:
    bool transmit(I2CBusDMA& bus, const uint32_t timeout_ms) const
    {
        const uint16_t current = table;
        uint8_t        payload[2]{};
        encode_connection_table(payload, current);

        if constexpr (ConnectionTableI2CUseMemWrite)
        {
            return bus.memWrite(address7bit(),
                                ConnectionTableI2CRegister,
                                payload,
                                sizeof(payload),
                                timeout_ms);
        }

        return bus.write(address7bit(), payload, sizeof(payload), timeout_ms);
    }
};

ConnectionTableI2CDevice& connection_table_i2c_device()
{
    static ConnectionTableI2CDevice device;
    return device;
}
} // namespace

void init()
{
    updateTable();

    if constexpr (!ProjectParts::EnableConnectionTableI2CTx)
        return;

    static bool registered = false;
    if (registered)
        return;

    if (!AppI2C::manager1().registerDevice(connection_table_i2c_device(),
                                           ConnectionTableI2CPeriodMs,
                                           ConnectionTableI2CPhaseMs,
                                           ConnectionTableI2CTimeoutMs))
    {
        Error_Handler();
    }

    registered = true;
}

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
