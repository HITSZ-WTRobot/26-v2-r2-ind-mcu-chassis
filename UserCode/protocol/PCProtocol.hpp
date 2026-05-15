/**
 * @file    PCProtocol.hpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#pragma once

// `PCProtocol` 是“某一路 UART 上的协议状态机”：
// - 接收时负责帧同步和 CRC 校验；
// - 发送时负责 DMA 状态管理；
// - 收到完整帧后把命令转交给 CommandHandler。

#include "PCCommandDef.hpp"
#include "UartRxSync.hpp"
#include "sync/Clock.hpp"
#include <array>

namespace Protocol
{

class PCProtocol;

struct Frame
{
    // 这帧属于哪个协议实例。
    PCProtocol*                protocol{};
    // 是否来自主上位机协议口。
    bool                       from_main_protocol{ false };
    // 下位机收到这帧的本地时间。
    uint32_t                   rx_timestamp{};
    // 命令编号。
    PCCommand                  cmd{};
    // 命令载荷，按字节保存，方便后续统一解析。
    std::array<uint8_t, 2 * 6> data{};
    // 上位机发送时的时间戳，用于对时。
    uint32_t                   tx_timestamp{};
    // 原始 CRC 值，便于调试。
    uint16_t                   crc16{};
};

constexpr uint32_t MaxPCProtocolCount = 2;

class PCProtocol final : public protocol::UartRxSync<HeaderLen, FrameLen>
{
public:
    explicit PCProtocol(UART_HandleTypeDef* huart, bool is_main_protocol = false);

    [[nodiscard]] float transitionDelayMS() const
    {
        // 帧在串口线上传输需要时间，对时的时候要把这部分补偿进去。
        return static_cast<float>(FrameLen) * 10.0f * 1000.0f /
               static_cast<float>(huart()->Init.BaudRate);
    }

    [[nodiscard]] bool isMainProtocol() const { return is_main_protocol_; }

    // 发送正常反馈帧。
    void transmitFeedbackFrame(const std::array<uint8_t, FeedbackFrameLen>& frame);
    // 辨识阶段只发单字节 0xAA。
    void transmitIdentifyByte();
    // 反馈任务每轮都会调用这里，由它决定发辨识字节还是发完整反馈帧。
    void transmitTaskStep(const std::array<uint8_t, FeedbackFrameLen>& feedback_frame);

    // DMA 发送完成回调。
    void transmitCallback();

    // 启动发送状态机。
    bool startTransmit();

    // 处理 UART 错误，重点恢复 TX DMA 异常。
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

    // 发送状态由中断和反馈线程共同访问，因此保持成一个轻量状态变量。
    volatile TxState tx_state_{ TxState::Stopped };

    bool is_main_protocol_{ false };

    // 辨识阶段固定发送的字节。
    std::array<uint8_t, 1> identify_tx_buffer_{ IdentifyInitByte };
};

/// 当前工程的 Main PCProtocol。
/// 兼容旧入口名：`pc_rx` 始终指向 main protocol。
inline PCProtocol* pc_rx{};

[[nodiscard]] const Sync::Clock& clock();

/** @brief 当前上位机定位链路是否仍被 watchdog 判定为在线。 */
bool isPcLocalizationConnected();
/** @brief 是否已经完成上位机辨识初始化。 */
bool isUpperHostIdentified();

/** @brief 初始化上位机协议相关对象。 */
void init();

} // namespace Protocol
