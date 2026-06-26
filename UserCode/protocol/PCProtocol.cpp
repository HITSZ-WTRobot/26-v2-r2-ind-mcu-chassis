/**
 * @file    PCProtocol.cpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#include "PCProtocol.hpp"

#include "PCCommandHandler.hpp"
#include "PCFeedback.hpp"
#include "device.hpp"
#include "project_parts.hpp"

#include <cassert>
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
} // namespace

bool PCProtocol::decode(const uint8_t data[PayloadLen])
{
    const uint16_t crc_in_data = read_u16(&data[PayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, PayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    // RX 回调只做最小解析和入队；具体命令语义交给 PCCommandHandler 线程处理。
    Frame frame{};
    frame.protocol           = this;
    frame.from_main_protocol = isMainProtocol();
    frame.rx_timestamp       = HAL_GetTick();
    frame.cmd                = static_cast<PCCommand>(data[0]);
    frame.tx_timestamp       = read_u32(&data[13]);
    frame.crc16              = crc_in_data;
    std::memcpy(frame.data.data(), data + 1, frame.data.size());

    return CommandHandler::enqueueFrame(frame);
}

PCProtocol::PCProtocol(UART_HandleTypeDef* huart, const bool is_main_protocol) :
    UartRxSync(huart), is_main_protocol_(is_main_protocol)
{
    Feedback::registerProtocol(this);
}

void PCProtocol::transmitFeedbackFrame(const std::array<uint8_t, FeedbackFrameLen>& frame)
{
    if (HAL_UART_Transmit_DMA(huart(), frame.data(), frame.size()) == HAL_OK)
    {
        tx_state_      = TxState::DMAActive;
        tx_watchdog_.feed(TxTimeoutTicks);
    }
    else
    {
        // HAL_UART_AbortTransmit 内部同步调用 TX Complete 回调，会先写入 tx_state_ = Idle，
        // 仅 abort 成功时才在此处安全重置状态。
        if (HAL_UART_AbortTransmit(huart()) == HAL_OK)
        {
            if (huart()->hdmatx != nullptr)
                huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
            tx_state_ = TxState::Idle;
        }
#ifdef DEBUG
        tx_fail_cnt_++;
#endif
    }
}

void PCProtocol::transmitIdentifyByte()
{
    if (HAL_UART_Transmit_DMA(huart(), identify_tx_buffer_.data(), identify_tx_buffer_.size()) ==
        HAL_OK)
    {
        tx_state_      = TxState::DMAActive;
        tx_watchdog_.feed(TxTimeoutTicks);
    }
    else
    {
        // HAL_UART_AbortTransmit 内部同步调用 TX Complete 回调，会先写入 tx_state_ = Idle，
        // 仅 abort 成功时才在此处安全重置状态。
        if (HAL_UART_AbortTransmit(huart()) == HAL_OK)
        {
            if (huart()->hdmatx != nullptr)
                huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
            tx_state_ = TxState::Idle;
        }
#ifdef DEBUG
        tx_fail_cnt_++;
#endif
    }
}

void PCProtocol::transmitCallback()
{
    tx_state_ = TxState::Idle;
}

void PCProtocol::transmitTaskStep(const std::array<uint8_t, FeedbackFrameLen>& feedback_frame)
{
    // 1. DMAActive 超时恢复：DMA Complete 中断丢失时看门狗复位
    if (tx_state_ == TxState::DMAActive)
    {
        if (!tx_watchdog_.isFed())
        {
            if (HAL_UART_AbortTransmit(huart()) == HAL_OK)
            {
                if (huart()->hdmatx != nullptr)
                    huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
                tx_state_ = TxState::Idle;
                // HAL_UART_AbortTransmit 内部同步调用 TX Complete 回调；此处再写入 Idle 是预期行为。
            }
#ifdef DEBUG
            tx_timeout_cnt_++;
#endif
        }
        else
        {
            return;
        }
    }

    if (tx_state_ == TxState::Stopped)
        return;

    // 2. gState 非 READY 恢复：HAL 状态机被污染时自愈
    if (huart()->gState != HAL_UART_STATE_READY)
    {
        if (huart()->gState == HAL_UART_STATE_BUSY_TX)
        {
            if (HAL_UART_AbortTransmit(huart()) == HAL_OK)
            {
                if (huart()->hdmatx != nullptr)
                    huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
                tx_state_ = TxState::Idle;
                // HAL_UART_AbortTransmit 内部同步调用 TX Complete 回调，transmitCallback()
                // 会在 AbortTransmit 返回前执行并写入 tx_state_ = Idle，此处的再写入是预期行为。
            }
        }
        if (huart()->gState != HAL_UART_STATE_READY)
            return;
    }

    if constexpr (ProjectParts::NeedUpperHostIdentifyInit)
    {
        if (isMainProtocol() && !isUpperHostIdentified())
            transmitIdentifyByte();
        else
            transmitFeedbackFrame(feedback_frame);
    }
    else
    {
        transmitFeedbackFrame(feedback_frame);
    }
}

bool PCProtocol::startTransmit()
{
    if (huart()->hdmatx == nullptr || huart()->hdmatx->Init.Mode != DMA_NORMAL)
        return false;

    tx_state_ = TxState::Idle;
    return true;
}

void PCProtocol::errorHandler()
{
    const bool has_tx_dma_error = (huart()->ErrorCode & HAL_UART_ERROR_DMA) != 0U &&
                                  huart()->hdmatx != nullptr &&
                                  huart()->hdmatx->ErrorCode != HAL_DMA_ERROR_NONE;

    if (has_tx_dma_error)
    {
        HAL_UART_AbortTransmit(huart());
        huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
        tx_state_                  = TxState::Idle;
    }

    protocol::UartRxSync<HeaderLen, FrameLen>::errorHandler();
}

static PCProtocol ctrl_rx{ config::uart::AuxControllerHost, false };

void init()
{
    // 若完全不需要上位机协议，则不创建串口接收对象和处理线程。
    if constexpr (!ProjectParts::EnableUpperHostProtocol)
        return;

    if (pc_rx != nullptr)
        return;

    assert(config::uart::UpperHost->Init.BaudRate == 230400);

    pc_rx = new PCProtocol(config::uart::UpperHost, true);

    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->receiveCallback(); });
    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_ERROR_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->errorHandler(); });
    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->transmitCallback(); });

    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.receiveCallback(); });
    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_ERROR_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.errorHandler(); });
    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.transmitCallback(); });

    CommandHandler::startTask();
    Feedback::startTask();

    if (!pc_rx->startReceive())
        Error_Handler();

    if (!pc_rx->startTransmit())
        Error_Handler();

    if (!ctrl_rx.startReceive())
        Error_Handler();

    if (!ctrl_rx.startTransmit())
        Error_Handler();
}

} // namespace Protocol
