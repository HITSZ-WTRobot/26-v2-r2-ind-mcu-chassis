/**
 * @file    PCProtocol.hpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#pragma once

#include "PCCommandDef.hpp"
#include "UartRxSync.hpp"
#include "sync/Clock.hpp"
#include <array>

namespace Protocol
{

class PCProtocol;

struct Frame
{
    PCProtocol*                protocol{};
    bool                       from_main_protocol{ false };
    uint32_t                   rx_timestamp{};
    PCCommand                  cmd{};
    std::array<uint8_t, 2 * 6> data{};
    uint32_t                   tx_timestamp{};
    uint16_t                   crc16{};
};

constexpr uint32_t MaxPCProtocolCount = 2;

class PCProtocol final : public protocol::UartRxSync<HeaderLen, FrameLen>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart, bool is_main_protocol = false);

    [[nodiscard]] float transitionDelayMS() const
    {
        return static_cast<float>(FrameLen) * 10.0f * 1000.0f /
               static_cast<float>(huart()->Init.BaudRate);
    }

    [[nodiscard]] bool isMainProtocol() const { return is_main_protocol_; }

    void transmitFeedbackFrame(const std::array<uint8_t, FeedbackFrameLen>& frame);
    void transmitIdentifyByte();
    void transmitTaskStep(const std::array<uint8_t, FeedbackFrameLen>& feedback_frame);

    void transmitCallback();

    bool startTransmit();

    void errorHandler();

protected:
    static constexpr std::array<uint8_t, HeaderLen> HEADER = { 0xAA, 0xBB };

    [[nodiscard]] const std::array<uint8_t, HeaderLen>& header() const override { return HEADER; }

    bool decode(const uint8_t data[PayloadLen]) override;

    [[nodiscard]] uint32_t timeout() const override { return 250; }

private:
    enum class TxState
    {
        Stopped,
        DMAActive,
        Idle,
    };

    volatile TxState tx_state_{ TxState::Stopped };

    bool is_main_protocol_{ false };

    std::array<uint8_t, 1> identify_tx_buffer_{ IdentifyInitByte };
};

/// 当前工程的 Main PCProtocol。
/// 兼容旧入口名：`pc_rx` 始终指向 main protocol。
inline PCProtocol* pc_rx{};

[[nodiscard]] const Sync::Clock& clock();

bool isPcLocalizationConnected();
bool isUpperHostIdentified();

void init();

} // namespace Protocol
