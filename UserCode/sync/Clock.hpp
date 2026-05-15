/**
 * @file    Clock.hpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#pragma once

// 这个时钟类只做一件事：把上位机时间和本地 HAL tick 做近似对齐。
// 它不负责网络同步，只负责给协议层一个足够稳定的时间偏移估计。

#include "stm32f4xx_hal.h"

#include <cmath>
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
    [[nodiscard]] bool     isStable() const { return stable_; }
    [[nodiscard]] float lastResidualBeforeAlignMS() const { return last_residual_before_align_ms_; }
    [[nodiscard]] float lastResidualAfterAlignMS() const { return last_residual_after_align_ms_; }

    void align(const float self_time, const float pc_time)
    {
        // 这里不是硬对齐，而是低通更新 offset，减少单帧抖动的影响。
        last_residual_before_align_ms_ = pc_time - self_time;

        const float delta = last_residual_before_align_ms_;

        offset_ = offset_ * (1.0f - alpha) + delta * alpha;

        last_residual_after_align_ms_ = pc_time - selfTime2PCTime(self_time);

        stable_ = std::fabs(last_residual_after_align_ms_) <= stable_threshold_ms_;
    }

private:
    float offset_{ 0.0f };
    float last_residual_before_align_ms_{ 0.0f };
    float last_residual_after_align_ms_{ 0.0f };
    bool  stable_{ false };

    constexpr static float alpha                = 0.2f;
    constexpr static float stable_threshold_ms_ = 20.0f;
};

} // namespace Sync
