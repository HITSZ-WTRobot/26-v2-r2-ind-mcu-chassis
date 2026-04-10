/**
 * @file    PCProtocol.cpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#include "PCProtocol.hpp"

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "system.hpp"

#include <cstring>

namespace Protocol
{
namespace
{
uint32_t read_u32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) << 24 | static_cast<uint32_t>(data[1]) << 16 |
           static_cast<uint32_t>(data[2]) << 8 | static_cast<uint32_t>(data[3]);
}

uint16_t read_u16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) << 8 | static_cast<uint16_t>(data[1]);
}

int16_t read_i16(const uint8_t* data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) << 8 |
                                static_cast<uint16_t>(data[1]));
}
} // namespace

bool PCProtocol::decode(const uint8_t data[PayloadLen])
{
    const uint16_t crc_in_data = read_u16(&data[PayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, PayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    return rx_buffer_.push(
            [&](Frame& frame)
            {
                frame.rx_timestamp = HAL_GetTick();
                frame.cmd          = data[0];
                frame.tx_timestamp = read_u32(&data[13]);
                frame.crc16        = crc_in_data;
                std::memcpy(frame.data.data(), data + 1, frame.data.size());
            });
}

void PCProtocol::cmdHandler(Frame& frame)
{
    clock_.align(static_cast<float>(frame.rx_timestamp),
                 static_cast<float>(frame.tx_timestamp) + transitionDelayMS());

    ++msg_cnt_;
    if (msg_cnt_ < 50)
        return;

    switch (frame.cmd)
    {
    case 0x01: // ping / 对时保活
        break;
    case 0x10: // 停止底盘
    case 0x11: // 重设坐标系
        if (Chassis::ctrl != nullptr)
            Chassis::ctrl->stop();
        break;
    case 0x12: // push 底盘轨迹点
        break;
    case 0x21: // 雷达定位点
    {
        constexpr auto to_pos =
                [](const int16_t value) { return static_cast<float>(value) / 2000.0f; };
        constexpr auto to_angle =
                [](const int16_t value) { return static_cast<float>(value) / 100.0f; };

        const auto& data = frame.data;

        const chassis::Posture pos = { .x   = to_pos(read_i16(&data[0])),
                                       .y   = to_pos(read_i16(&data[2])),
                                       .yaw = to_angle(read_i16(&data[4])) };

        debug_.lidar.last_received_posture = pos;

        const uint32_t lidar_time      = read_u32(data.data() + 6);
        const uint32_t lidar_self_time = clock_.pcTime2SelfTime(lidar_time);

        debug_.lidar.last_received_posture_timestamp = lidar_self_time;
        debug_.lidar.last_received_timestamp         = HAL_GetTick();
        debug_.lidar.last_received_delay =
                static_cast<int32_t>(debug_.lidar.last_received_timestamp) -
                static_cast<int32_t>(lidar_self_time);

        if (Chassis::motion == nullptr || !Chassis::motion->isReady())
            return;

        if (!System::Init::postureReceived)
        {
            if (Device::Sensor::gyro_yaw == nullptr || !Device::Sensor::gyro_yaw->isConnected())
                return;

            System::Init::posture         = pos;
            System::Init::postureReceived = true;
            System::Init::initPostureReceive();
            return;
        }

        if (Chassis::loc != nullptr)
            Chassis::loc->updateLidar(pos, lidar_self_time);
        break;
    }
    default:
        break;
    }
}

[[noreturn]] void PCProtocol::loop()
{
    for (;;)
    {
        while (!rx_buffer_.empty())
        {
            auto* frame = rx_buffer_.pop();
            if (frame != nullptr)
                cmdHandler(*frame);
        }

        osDelay(1);
    }
}

constexpr osThreadAttr_t processor_attr{
    .name       = "pc-cmd-processor",
    .stack_size = 1024 * 4,
    .priority   = osPriorityRealtime,
};

void init()
{
    if (pc_rx != nullptr)
        return;

    pc_rx = new PCProtocol(config::uart::UpperHost);
    UartRxSync_RegisterCallback(pc_rx, config::uart::UpperHost);

    if (!pc_rx->startReceive())
        Error_Handler();

    osThreadNew(PCProtocol::TaskEntry, pc_rx, &processor_attr);
}

} // namespace Protocol
