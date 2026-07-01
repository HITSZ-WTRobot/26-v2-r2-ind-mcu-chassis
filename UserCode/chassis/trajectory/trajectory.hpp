/**
 * @file    trajectory.hpp
 * @author  syhanjin
 * @date    2026-06-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once
#include "Config.hpp"
#include "trajectory_resampler.hpp"
#include "trajectory_traj_1.hpp"
#include "trajectory_traj_2.hpp"
#include "trajectory_traj_3.hpp"
#include "trajectory_traj_4.hpp"
#include "trajectory_traj_5.hpp"
namespace Config::TrajectoryOffline
{

constexpr float dt = 0.01f;

namespace traj1
{

constexpr size_t PointCount =
        planning::trajectory::computeResampledCount(planning::trajectory::traj_1::kPointCount,
                                                    kSpeedRatio);

constexpr auto Points =
        planning::trajectory::resampleTrajectory<planning::trajectory::traj_1::kPointCount,
                                                 PointCount>(planning::trajectory::traj_1::kPoints,
                                                             kSpeedRatio);

constexpr float Duration = static_cast<float>(PointCount - 1) * dt;
} // namespace traj1
namespace traj2
{

constexpr size_t PointCount =
        planning::trajectory::computeResampledCount(planning::trajectory::traj_2::kPointCount,
                                                    kSpeedRatio);

constexpr auto Points =
        planning::trajectory::resampleTrajectory<planning::trajectory::traj_2::kPointCount,
                                                 PointCount>(planning::trajectory::traj_2::kPoints,
                                                             kSpeedRatio);

constexpr float Duration = static_cast<float>(PointCount - 1) * dt;
} // namespace traj2
namespace traj3
{

constexpr size_t PointCount =
        planning::trajectory::computeResampledCount(planning::trajectory::traj_3::kPointCount,
                                                    kSpeedRatio);

constexpr auto Points =
        planning::trajectory::resampleTrajectory<planning::trajectory::traj_3::kPointCount,
                                                 PointCount>(planning::trajectory::traj_3::kPoints,
                                                             kSpeedRatio);

constexpr float Duration = static_cast<float>(PointCount - 1) * dt;
} // namespace traj3
namespace traj4
{

constexpr size_t PointCount =
        planning::trajectory::computeResampledCount(planning::trajectory::traj_4::kPointCount,
                                                    kSpeedRatio);

constexpr auto Points =
        planning::trajectory::resampleTrajectory<planning::trajectory::traj_4::kPointCount,
                                                 PointCount>(planning::trajectory::traj_4::kPoints,
                                                             kSpeedRatio);

constexpr float Duration = static_cast<float>(PointCount - 1) * dt;
} // namespace traj4
namespace traj5
{

constexpr size_t PointCount =
        planning::trajectory::computeResampledCount(planning::trajectory::traj_5::kPointCount,
                                                    kSpeedRatio);

constexpr auto Points =
        planning::trajectory::resampleTrajectory<planning::trajectory::traj_5::kPointCount,
                                                 PointCount>(planning::trajectory::traj_5::kPoints,
                                                             kSpeedRatio);

constexpr float Duration = static_cast<float>(PointCount - 1) * dt;
} // namespace traj5

// 查看重采样后轨迹内存占用：
//   arm-none-eabi-nm -S build/*.elf | grep -E "traj[123].*Points"
// 或在 .map 文件中搜索 traj1::Points / traj2::Points / traj3::Points

} // namespace Config::TrajectoryOffline
