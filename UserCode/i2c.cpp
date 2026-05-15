/**
 * @file    i2c.cpp
 * @author  syhanjin
 * @date    2026-04-28
 */
#include "i2c.hpp"

#include "i2c.h"

namespace
{
// 当前项目只把 I2C2 暴露成共享总线。
constexpr auto                     Bus2Handle = &hi2c2;
constexpr I2CUpdateManager::Config Bus2ManagerConfig{
    .task_name        = "i2c2-mgr",
    .stack_size_bytes = 384U * sizeof(uint32_t),
    .priority         = osPriorityLow,
    .max_sleep_ms     = 500U,
};
} // namespace

namespace AppI2C
{
I2CBusDMA& bus2()
{
    // 函数内 static 保证对象只构造一次，同时避免全局初始化顺序问题。
    static I2CBusDMA bus(Bus2Handle);
    return bus;
}

I2CUpdateManager& manager2()
{
    // 所有 I2C2 周期设备共享同一个 manager。
    static I2CUpdateManager manager(bus2());
    return manager;
}

bool start_bus2_manager()
{
    auto& manager = manager2();

    if (manager.isRunning())
        return true;

    if (manager.deviceCount() == 0U)
        // 没有已注册设备时不需要创建后台线程。
        return true;

    return manager.start(Bus2ManagerConfig);
}
} // namespace AppI2C
