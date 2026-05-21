/**
 * @file    InfraredReceiver.hpp
 * @brief   Single-byte infrared UART receiver.
 */
#pragma once

#include <cstdint>

namespace Infrared
{

enum class State : uint16_t
{
    KeepAlive       = 0u,
    DockingFinished = 1u,
    ReservedA2      = 2u,
    ReservedA3      = 3u,
};

void init();
void receiveCallback();
void errorHandler();

[[nodiscard]] State    state();
[[nodiscard]] uint16_t stateBits();

} // namespace Infrared
