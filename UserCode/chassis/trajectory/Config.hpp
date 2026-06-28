/**
 * @file    Config.hpp
 * @author  syhanjin
 * @date    2026-06-28
 * @brief   离线轨迹配置：速度倍率 + PD 参数。
 */
#pragma once
#include "pid_pd.hpp"
#include "Slave.hpp"

namespace Config::TrajectoryOffline
{

/// 离线轨迹播放速度倍率（1.0 = 实时）。
constexpr float kSpeedRatio = 0.3f;

using ChassisSlaveOffline = chassis::controller::Slave<0, true>;

constexpr ChassisSlaveOffline::PDConfig PDCfg = {
    .vx = { .Kp = 5.0f, .Kd = 25.0f, .abs_output_max = 0.6f },
    .vy = { .Kp = 5.0f, .Kd = 25.0f, .abs_output_max = 0.6f },
    .wz = { .Kp = 5.0f, .Kd = 25.0f, .abs_output_max = 135.0f },
};

/// 抬升电机轨迹 PD 配置（电机角度域，4 个电机共用）。
constexpr PD::Config LiftMotorPDCfg = { .Kp = 5.0f, .Kd = 25.0f, .abs_output_max = 60.0f };

} // namespace Config::TrajectoryOffline
