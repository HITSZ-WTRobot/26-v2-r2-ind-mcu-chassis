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

/// 未使用气压计，或气压样本不新鲜时，延时多久认为已经吸住，单位 ms。
/// 这两个延时会被吸盘组件内部直接消费，KFS 状态机不再自己维护重复计时。
constexpr uint32_t ObjectDetectDelayMs = 300U;
/// 关闭吸盘后，延时多久认为已经放开，单位 ms。
constexpr uint32_t ObjectReleaseDelayMs = 100U;

/// 由未吸附状态到吸附状态的触发气压
constexpr float DetectOnPressurePa = -5000.0f;
/// 由吸附状态到未吸附状态的触发气压
constexpr float DetectOffPressurePa = -2000.0f;

} // namespace Suction::Config
