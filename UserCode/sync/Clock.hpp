/**
 * @file    Clock.hpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#pragma once

#include "stm32f4xx_hal.h"

#include <cstdint>

namespace Sync
{

class Clock
{
public:
    [[nodiscard]] uint32_t pcTime2SelfTime(const uint32_t pc_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(pc_time) + 0.5f - offset_);
    }

    [[nodiscard]] uint32_t selfTime2PCTime(const uint32_t self_time) const
    {
        return static_cast<uint32_t>(static_cast<float>(self_time) + 0.5f + offset_);
    }

    [[nodiscard]] float pcTime2SelfTime(const float pc_time) const { return pc_time - offset_; }

    [[nodiscard]] float selfTime2PCTime(const float self_time) const { return self_time + offset_; }

    [[nodiscard]] uint32_t getPCTime() const { return selfTime2PCTime(HAL_GetTick()); }

    void align(const float self_time, const float pc_time)
    {
        const float delta = pc_time - self_time;

        offset_ = offset_ * (1.0f - alpha) + delta * alpha;
    }

private:
    float offset_{ 0.0f };

    constexpr static float alpha = 0.2f;
};

} // namespace Sync
