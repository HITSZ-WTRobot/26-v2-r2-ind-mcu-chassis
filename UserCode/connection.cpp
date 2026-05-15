/**
 * @file    connection.cpp
 * @author  syhanjin
 * @date    2026-04-22
 */
#include "connection.hpp"

#include "cmsis_os2.h"
#include "device.hpp"
#include "grip/grip.hpp"
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
    // 只把“校准前必须在线的本地硬件”纳入 required。
    // 上位机连接和上位机定位流由 System::Init 后续判断。
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

    if constexpr (ProjectParts::EnableGripSuctionPressureSensor)
    {
        required |= mask(Bit::GripSuctionPressure);
    }

    if constexpr (ProjectParts::EnableGyro)
        required |= mask(Bit::GyroYaw);

    // 上位机链路状态仍会进入 connection table，但不参与这里的“校准前本地链路就绪”判定。
    // 相关初始化要求统一放到 `System::Init::inited()` 的最终门槛中处理。

    return required;
}

class ConnectionTableI2CDevice final : public I2CDevice
{
public:
    // 这是一个“虚拟 I2C 设备”：它没有被读取的寄存器，只负责周期发送连接表。
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
        // 发送前取一次快照，避免 table 在编码两个字节之间变化。
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
    // manager 注册时需要稳定引用，因此用函数内 static。
    static ConnectionTableI2CDevice device;
    return device;
}
} // namespace

void init()
{
    // 先刷新一次，保证外部第一次读取就能拿到当前连接状态。
    updateTable();

    if constexpr (!ProjectParts::EnableConnectionTableI2CTx)
        return;

    static bool registered = false;
    if (registered)
        return;

    if (!AppI2C::manager2().registerDevice(connection_table_i2c_device(),
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
    // 重新构造整张表，而不是在旧值上增量修改，避免已断开的 bit 残留。
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

    if constexpr (ProjectParts::EnableGripSuctionPressureSensor)
    {
        set_bit(current,
                Bit::GripSuctionPressure,
                Device::Sensor::grip_suction_pressure != nullptr &&
                        Device::Sensor::grip_suction_pressure->isOnline());
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

    // required 为空时返回 true，这样“全关调试形态”不会卡在连接等待。
    return (current & required) == required;
}

void waitAll()
{
    while (!isAllConnected())
        osDelay(1);
}
} // namespace Connection
