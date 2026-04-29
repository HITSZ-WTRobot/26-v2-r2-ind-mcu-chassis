/**
 * @file    Config.hpp
 * @brief   吸盘模块公共参数
 */
#pragma once

#include <cstdint>

namespace Suction::Config
{

/// XGZP6847D 旧版 K 表等效量程，取 max(abs(Pmin), abs(Pmax))，单位 kPa。
constexpr float PressureRangeKPa = 100.0f;
/// 压力传感器默认 7-bit I2C 地址。
constexpr uint8_t PressureAddress7bit = 0x6DU;
/// 周期采样参数：20 ms 更新一次已足够覆盖吸附建立判定。
constexpr uint32_t PressureUpdatePeriodMs = 20U;
constexpr uint32_t PressureTimeoutMs      = 20U;

/// 当前未吸住时，压力低于该阈值则判定为“已吸住”。
constexpr float DetectOnPressurePa = -5000.0f;
/// 当前已吸住时，压力高于该阈值则判定为“已放开 / 已滑落”。
constexpr float DetectOffPressurePa = -2000.0f;

} // namespace Suction::Config
