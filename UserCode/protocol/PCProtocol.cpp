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
    // 先校验 CRC，再把命令打包交给命令处理线程。
    const uint16_t crc_in_data = read_u16(&data[PayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, PayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    // 来不及了不管这里的性能损失了
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
    // 每个协议实例创建时都要注册到反馈系统，方便反馈任务统一轮询发送。
    Feedback::registerProtocol(this);
}

void PCProtocol::transmitFeedbackFrame(const std::array<uint8_t, FeedbackFrameLen>& frame)
{
    if (HAL_UART_Transmit_DMA(huart(), frame.data(), frame.size()) == HAL_OK)
        tx_state_ = TxState::DMAActive;
}

void PCProtocol::transmitIdentifyByte()
{
    if (HAL_UART_Transmit_DMA(huart(), identify_tx_buffer_.data(), identify_tx_buffer_.size()) ==
        HAL_OK)
    {
        tx_state_ = TxState::DMAActive;
    }
}

void PCProtocol::transmitCallback()
{
    tx_state_ = TxState::Idle;
}

void PCProtocol::transmitTaskStep(const std::array<uint8_t, FeedbackFrameLen>& feedback_frame)
{
    // 只有空闲时才允许发下一帧，避免和 DMA 中断并发。
    if (tx_state_ == TxState::Stopped || tx_state_ == TxState::DMAActive ||
        huart()->gState != HAL_UART_STATE_READY)
        return;

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
    // 本实现依赖 DMA_NORMAL 模式；配置不符时直接拒绝启动。
    if (huart()->hdmatx == nullptr || huart()->hdmatx->Init.Mode != DMA_NORMAL)
        return false;

    tx_state_ = TxState::Idle;
    return true;
}

void PCProtocol::errorHandler()
{
    // 只对 TX DMA 异常做主动恢复，其余错误交给基类处理。
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

    // 主上位机链路的波特率是协议约定的一部分。
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

    // 命令处理和反馈发送都在独立线程里运行。
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
