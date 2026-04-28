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
 * 共享 I2C1 总线对象。
 *
 * 所有走 I2C1 的周期设备都应通过这个 bus / manager 访问，避免多线程并发抢占同一总线。
 */
I2CBusDMA& bus1();

/**
 * I2C1 对应的共享周期调度器。
 *
 * 设备必须在 `start_bus1_manager()` 前通过 `manager1().registerDevice(...)`
 * 完成注册；manager 运行后不支持动态注册。
 */
I2CUpdateManager& manager1();

/**
 * 启动 I2C1 的共享 manager。
 *
 * 若当前没有任何已注册设备，则直接返回 true，不创建后台线程。
 */
bool start_bus1_manager();
} // namespace AppI2C
