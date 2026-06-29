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
            // 位置沿速度积分（二阶），替代线性插值以保证积分关系
            // pos(frac) = pos0 + vel0*frac*src_dt + 0.5*(vel1-vel0)/src_dt*(frac*src_dt)²
            //            = pos0 + frac * src_dt * (vel0 + 0.5 * (vel1 - vel0) * frac)
            const float x_integral   = p0.x + frac * src_dt * (p0.dx + 0.5f * (p1.dx - p0.dx) * frac);
            const float y_integral   = p0.y + frac * src_dt * (p0.dy + 0.5f * (p1.dy - p0.dy) * frac);
            const float yaw_integral = p0.yaw + frac * src_dt * (p0.dyaw + 0.5f * (p1.dyaw - p0.dyaw) * frac);
            const float h_integral   = p0.h + frac * src_dt * (p0.dh + 0.5f * (p1.dh - p0.dh) * frac);
            result[i]                = {
                x_integral,
                y_integral,
                yaw_integral,
                h_integral,
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
