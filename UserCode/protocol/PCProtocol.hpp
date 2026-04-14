/**
 * @file    PCProtocol.hpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#pragma once

#include "IChassisDef.hpp"
#include "RingBuffer.hpp"
#include "UartRxSync.hpp"
#include "crc.hpp"
#include "sync/Clock.hpp"

#include <array>
#include <cstdint>

namespace Protocol
{

using CRC16Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen  = 2;
constexpr uint32_t PayloadLen = 1 + 2 * 6 + 4 + 2;
constexpr uint32_t FrameLen   = HeaderLen + PayloadLen;

enum class PCCommand : uint8_t
{
    /// 上位机对时信号
    /// TODO: 返回 Pong
    Ping = 0x01,

    /// 停止底盘
    StopChassis = 0x10,

    /// Slave 模式下 push 轨迹点
    /// | int16  | int16  |     int16    |  int16  |  int16  |    int16    |
    /// | x*2000 | y*2000 | yaw(deg)*100 | vx*2000 | vy*2000 | wz(deg)*100 |
    SlavePushChassisTrajectory = 0x12,

    /// 雷达位姿
    /// | int16  | int16  |     int16    |       uint32       |
    /// | x*2000 | y*2000 | yaw(deg)*100 | lidarTimestamp(ms) |
    LidarPosture = 0x21,

    /* 动作组 */
    /// 上台阶
    /// |        int16       |      int16       |   uint16  |  uint16  |
    /// | startDistance*2000 | endDistance*2000 | direction | willTake |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前上台阶
    ///                   1 Backward，车头朝后上台阶
    /// @param willTake: 0 连贯上台阶;
    ///                  1 中途将停下来取卷轴，发送 StepUpResume 指令后继续执行上台阶动作
    StepUp = 0x30,
    /// 继续上台阶动作
    StepUpResume = 0x31,
    /// 下台阶
    /// |        int16       |      int16       |   uint16  |    uint16   |
    /// | startDistance*2000 | endDistance*2000 | direction | shouldReset |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前下台阶
    ///                   1 Backward，车头朝后下台阶
    /// @param shouldReset: 1 下台阶之后底盘恢复到正常高度
    ///                     0 下台阶最后一步不回收底盘，底盘仍处于比台阶高的状态
    StepDown = 0x32,
};

class PCProtocol final : public protocol::UartRxSync<HeaderLen, FrameLen>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    static void TaskEntry(void* argument) { static_cast<PCProtocol*>(argument)->loop(); }

    [[nodiscard]] float transitionDelayMS() const
    {
        return static_cast<float>(FrameLen) * 10.0f * 1000.0f /
               static_cast<float>(huart()->Init.BaudRate);
    }

    [[nodiscard]] const Sync::Clock& clock() const { return clock_; }

protected:
    static constexpr std::array<uint8_t, HeaderLen> HEADER = { 0xAA, 0xBB };

    [[nodiscard]] const std::array<uint8_t, HeaderLen>& header() const override { return HEADER; }

    bool decode(const uint8_t data[PayloadLen]) override;

    [[nodiscard]] uint32_t timeout() const override { return 250; }

private:
    struct Frame
    {
        uint32_t                   rx_timestamp{};
        PCCommand                  cmd{};
        std::array<uint8_t, 2 * 6> data{};
        uint32_t                   tx_timestamp{};
        uint16_t                   crc16{};
    };

    libs::RingBuffer<Frame, 10> rx_buffer_{};

    uint32_t msg_cnt_{ 0 };

    struct
    {
        struct
        {
            chassis::Posture last_received_posture{};
            uint32_t         last_received_posture_timestamp{};
            uint32_t         last_received_timestamp{};
            int32_t          last_received_delay{};
        } lidar;
    } debug_{};

    [[noreturn]] void loop();

    void cmdHandler(Frame& frame);

    Sync::Clock clock_{};
};

inline PCProtocol* pc_rx{};

void init();

} // namespace Protocol
