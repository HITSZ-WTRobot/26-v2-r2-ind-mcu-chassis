/**
 * @file    trajectory_resampler.hpp
 * @author  syhanjin
 * @date    2026-06-28
 * @brief   离线轨迹 constexpr 重采样工具：500Hz → 100Hz，带速度倍率。
 */
#pragma once
#include "trajectory_point.hpp"
#include <array>
#include <cstddef>

namespace planning::trajectory
{

constexpr size_t computeResampledCount(const size_t original_count, const float speed_ratio)
{
    const float effective_duration = static_cast<float>(original_count - 1) * 0.002f / speed_ratio;
    const float count_f            = effective_duration * 100.0f;
    const auto  truncated          = static_cast<size_t>(count_f);
    return (count_f > static_cast<float>(truncated) ? truncated + 1 : truncated) + 1;
}

template <size_t NIn, size_t NOut>
constexpr auto resampleTrajectory(const std::array<TrajectoryPoint8, NIn>& original,
                                  const float speed_ratio) -> std::array<TrajectoryPoint8, NOut>
{
    std::array<TrajectoryPoint8, NOut> result{};
    constexpr float                    src_dt = 0.002f;
    constexpr float                    dst_dt = 0.01f;

    for (size_t i = 0; i < NOut; ++i)
    {
        const float effective_time = static_cast<float>(i) * dst_dt * speed_ratio;
        const float src_index_f    = effective_time / src_dt;
        const auto  src_idx        = static_cast<size_t>(src_index_f);
        const float frac           = src_index_f - static_cast<float>(src_idx);

        if (src_idx + 1 < NIn)
        {
            const auto& p0 = original[src_idx];
            const auto& p1 = original[src_idx + 1];
            // 位置直接插值，速度按 speed_ratio 缩放以匹配播放速率
            result[i]      = {
                p0.x + frac * (p1.x - p0.x),
                p0.y + frac * (p1.y - p0.y),
                p0.yaw + frac * (p1.yaw - p0.yaw),
                p0.h + frac * (p1.h - p0.h),
                (p0.dx + frac * (p1.dx - p0.dx)) * speed_ratio,
                (p0.dy + frac * (p1.dy - p0.dy)) * speed_ratio,
                (p0.dyaw + frac * (p1.dyaw - p0.dyaw)) * speed_ratio,
                (p0.dh + frac * (p1.dh - p0.dh)) * speed_ratio,
            };
        }
        else
        {
            result[i]      = original[NIn - 1];
            result[i].dx   *= speed_ratio;
            result[i].dy   *= speed_ratio;
            result[i].dyaw *= speed_ratio;
            result[i].dh   *= speed_ratio;
        }
    }
    return result;
}

} // namespace planning::trajectory
