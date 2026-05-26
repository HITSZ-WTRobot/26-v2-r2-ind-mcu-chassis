/**
 * @file    InfraredReceiver.cpp
 * @brief   Single-byte infrared UART receiver.
 */
#include "InfraredReceiver.hpp"

#include "device.hpp"
#include "grip/grip.hpp"
#include "main.h"
#include "project_parts.hpp"

#include <cassert>

namespace Infrared
{
namespace
{
constexpr uint8_t KeepAliveByte       = 0xA0U;
constexpr uint8_t DockingFinishedByte = 0xA1U;
constexpr uint8_t ReservedA2Byte      = 0xA2U;
constexpr uint8_t ReservedA3Byte      = 0xA3U;
constexpr uint8_t StableThreshold     = 3U;

volatile State state_{ State::KeepAlive };

uint8_t rx_byte_{};
uint8_t candidate_byte_{};
uint8_t candidate_count_{};
bool    has_candidate_{ false };
bool    started_{ false };

volatile bool     release_retract_pending_{ false };
volatile uint32_t release_retract_start_ms_{};

void trigger_docking_finished_action()
{
    if constexpr (!ProjectParts::EnableGrip)
        return;

    if (Grip::grip == nullptr)
        return;

    Grip::grip->openClaw();
    release_retract_start_ms_ = HAL_GetTick();
    release_retract_pending_  = true;
}

[[nodiscard]] bool decode_state(const uint8_t byte, State& out)
{
    switch (byte)
    {
    case KeepAliveByte:
        out = State::KeepAlive;
        return true;
    case DockingFinishedByte:
        out = State::DockingFinished;
        return true;
    case ReservedA2Byte:
        out = State::ReservedA2;
        return true;
    case ReservedA3Byte:
        out = State::ReservedA3;
        return true;
    default:
        return false;
    }
}

void apply_stable_state(const State next)
{
    const State prev = state_;
    state_           = next;

    if (prev != State::KeepAlive || next != State::DockingFinished)
        return;

    trigger_docking_finished_action();
}

void reset_candidate()
{
    has_candidate_   = false;
    candidate_byte_  = 0U;
    candidate_count_ = 0U;
}

void process_byte(const uint8_t byte)
{
    State next{};
    if (!decode_state(byte, next))
    {
        reset_candidate();
        return;
    }

    if (!has_candidate_ || byte != candidate_byte_)
    {
        has_candidate_   = true;
        candidate_byte_  = byte;
        candidate_count_ = 1U;
        return;
    }

    if (candidate_count_ < StableThreshold)
        ++candidate_count_;

    if (candidate_count_ >= StableThreshold)
        apply_stable_state(next);
}

void clear_rx_error_flags(UART_HandleTypeDef* huart)
{
    const uint32_t error_code = huart->ErrorCode;

    if ((error_code & HAL_UART_ERROR_PE) != 0U)
        __HAL_UART_CLEAR_PEFLAG(huart);
    if ((error_code & HAL_UART_ERROR_FE) != 0U)
        __HAL_UART_CLEAR_FEFLAG(huart);
    if ((error_code & HAL_UART_ERROR_NE) != 0U)
        __HAL_UART_CLEAR_NEFLAG(huart);
    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
        __HAL_UART_CLEAR_OREFLAG(huart);

    if ((error_code & HAL_UART_ERROR_DMA) != 0U && huart->hdmarx != nullptr)
        huart->hdmarx->ErrorCode = HAL_DMA_ERROR_NONE;
}

bool start_receive()
{
    UART_HandleTypeDef* const huart = config::uart::InfraredReceiver;

    if (huart->hdmarx == nullptr || huart->hdmarx->Init.Mode != DMA_CIRCULAR)
        return false;

    return HAL_UART_Receive_DMA(huart, &rx_byte_, sizeof(rx_byte_)) == HAL_OK;
}
} // namespace

void init()
{
    if constexpr (!ProjectParts::EnableInfraredReceiver)
        return;

    if (started_)
        return;

    assert(config::uart::InfraredReceiver->Init.BaudRate == 57600);

    HAL_UART_RegisterCallback(config::uart::InfraredReceiver,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart)
                              {
                                  (void)huart;
                                  receiveCallback();
                              });
    HAL_UART_RegisterCallback(config::uart::InfraredReceiver,
                              HAL_UART_ERROR_CB_ID,
                              [](UART_HandleTypeDef* huart)
                              {
                                  (void)huart;
                                  errorHandler();
                              });

    if (!start_receive())
        Error_Handler();

    started_ = true;
}

void update_100Hz()
{
    if constexpr (!ProjectParts::EnableInfraredReceiver || !ProjectParts::EnableGrip)
        return;

    if (!release_retract_pending_)
        return;

    const uint32_t now_ms     = HAL_GetTick();
    const uint32_t start_ms   = release_retract_start_ms_;
    const uint32_t elapsed_ms = now_ms - start_ms;

    if (elapsed_ms < Grip::Config::InfraredDocking::ReleaseRetractDelayMs)
        return;

    if (release_retract_start_ms_ != start_ms)
        return;

    release_retract_pending_ = false;

    if (Grip::grip != nullptr)
        (void)Grip::grip->toJointPose(Grip::Config::InfraredDocking::ReleaseRetractPose);
}

void receiveCallback()
{
    if constexpr (!ProjectParts::EnableInfraredReceiver)
        return;

    process_byte(rx_byte_);
}

void errorHandler()
{
    if constexpr (!ProjectParts::EnableInfraredReceiver)
        return;

    UART_HandleTypeDef* const huart = config::uart::InfraredReceiver;

    clear_rx_error_flags(huart);
    HAL_UART_AbortReceive(huart);

    if (!start_receive())
        Error_Handler();
}

State state()
{
    return state_;
}

uint16_t stateBits()
{
    if constexpr (!ProjectParts::EnableInfraredReceiver)
        return 0u;

    return static_cast<uint16_t>(state());
}

} // namespace Infrared
