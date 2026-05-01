/**
 * @file    i2c.hpp
 * @author  syhanjin
 * @date    2026-04-28
 */
#pragma once

#include "I2CUpdateManager.hpp"

namespace AppI2C
{
/**
 * 共享 I2C2 总线对象。
 *
 * 所有走 I2C2 的周期设备都应通过这个 bus / manager 访问，避免多线程并发抢占同一总线。
 */
I2CBusDMA& bus2();

/**
 * I2C2 对应的共享周期调度器。
 *
 * 设备必须在 `start_bus2_manager()` 前通过 `manager2().registerDevice(...)`
 * 完成注册；manager 运行后不支持动态注册。
 */
I2CUpdateManager& manager2();

/**
 * 启动 I2C2 的共享 manager。
 *
 * 若当前没有任何已注册设备，则直接返回 true，不创建后台线程。
 */
bool start_bus2_manager();
} // namespace AppI2C
