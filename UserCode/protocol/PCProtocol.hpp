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
        uint8_t                    cmd{};
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
