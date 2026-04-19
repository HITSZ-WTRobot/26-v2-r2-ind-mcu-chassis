/**
 * @file    PCProtocol.cpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#include "PCProtocol.hpp"

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "chassis/actions/Step.hpp"
#include "project_parts.hpp"
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
                frame.cmd          = static_cast<PCCommand>(data[0]);
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

    constexpr auto to_pos = [](const int16_t value) { return static_cast<float>(value) / 2000.0f; };
    constexpr auto to_angle = [](const int16_t v) { return static_cast<float>(v) / 100.0f; };

    const auto& data = frame.data;

    switch (frame.cmd)
    {
    case PCCommand::Ping:
        break;
    case PCCommand::StopChassis:
        // StopChassis 属于“上位机控制命令”范畴；
        // 当该能力关闭时，协议层收到该帧也会直接忽略。
        if constexpr (ProjectParts::EnablePcControl)
        {
            if (Chassis::ctrl != nullptr)
                Chassis::ctrl->stop();
        }
        break;
    case PCCommand::SlavePushChassisTrajectory:
        break;
    case PCCommand::LidarPosture:
    {
        // LidarPosture 只在“上位机定位包”能力启用时才参与处理。
        // 否则当前工程使用本地定位，不消费外部位姿观测。
        if constexpr (!ProjectParts::EnablePcLocalization)
            break;

        const chassis::Posture pos = { .x   = to_pos(read_i16(&data[0])),
                                       .y   = to_pos(read_i16(&data[2])),
                                       .yaw = to_angle(read_i16(&data[4])) };

        debug_.lidar.last_received_posture = pos;

        const uint32_t lidar_time      = read_u32(data.data() + 6);
        const uint32_t lidar_self_time = clock_.pcTime2SelfTime(lidar_time);

        debug_.lidar.last_received_posture_timestamp = lidar_self_time;
        debug_.lidar.last_received_timestamp         = HAL_GetTick();

        debug_.lidar.last_received_delay = static_cast<int32_t>(
                                                   debug_.lidar.last_received_timestamp) -
                                           static_cast<int32_t>(lidar_self_time);

        if (Chassis::motion == nullptr || !Chassis::motion->isReady())
            return;

        // 第一帧外部位姿的职责是“定义系统初值”：
        // - 记录 posture
        // - 触发 `System::Init::initPostureReceive()`
        // - 完成 Loc / Controller 的延迟构造
        if (!System::Init::postureReceived)
        {
            if (Device::Sensor::gyro_yaw == nullptr || !Device::Sensor::gyro_yaw->isConnected())
                return;

            System::Init::posture         = pos;
            System::Init::postureReceived = true;
            System::Init::initPostureReceive();
            return;
        }

        Chassis::updateLidar(pos, lidar_self_time);
        break;
    }
    case PCCommand::StepUp:
    {
        // 台阶动作不是单一协议能力，而是“控制命令 + 底盘 + 升降”的组合能力。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        const float             startDistance = to_pos(read_i16(&data[0]));
        const float             endDistance   = to_pos(read_i16(&data[2]));
        const uint16_t          dir           = read_u16(&data[4]);
        const bool              willTake      = static_cast<bool>(read_u16(&data[6]));
        Action::Step::Direction direction;
        if (dir == 0)
            direction = Action::Step::Direction::Forward;
        else if (dir == 1)
            direction = Action::Step::Direction::Backward;
        else
            break;
        Action::Step::inst().up(startDistance, endDistance, direction, willTake);
        break;
    }
    case PCCommand::StepUpResume:
    {
        // 与 StepUp 相同，只有完整动作链启用时才处理恢复命令。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        auto& step = Action::Step::inst();
        if (step.isWaitingTake())
        {
            step.resume_up();
        }
        break;
    }
    case PCCommand::StepDown:
    {
        // 与 StepUp 相同，只有完整动作链启用时才处理下台阶命令。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        const float             startDistance = to_pos(read_i16(&data[0]));
        const float             endDistance   = to_pos(read_i16(&data[2]));
        const uint16_t          dir           = read_u16(&data[4]);
        const bool              shouldReset   = static_cast<bool>(read_u16(&data[6]));
        Action::Step::Direction direction;
        if (dir == 0)
            direction = Action::Step::Direction::Forward;
        else if (dir == 1)
            direction = Action::Step::Direction::Backward;
        else
            break;
        Action::Step::inst().down(startDistance, endDistance, direction, shouldReset);
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
    // 若完全不需要上位机协议，则不创建串口接收对象和处理线程。
    if constexpr (!ProjectParts::EnableUpperHostProtocol)
        return;

    if (pc_rx != nullptr)
        return;

    assert(config::uart::UpperHost->Init.BaudRate == 230400);

    pc_rx = new PCProtocol(config::uart::UpperHost);
    UartRxSync_RegisterCallback(pc_rx, config::uart::UpperHost);

    if (!pc_rx->startReceive())
        Error_Handler();

    osThreadNew(PCProtocol::TaskEntry, pc_rx, &processor_attr);
}

} // namespace Protocol
