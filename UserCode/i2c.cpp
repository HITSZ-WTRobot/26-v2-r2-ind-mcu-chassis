/**
 * @file    i2c.cpp
 * @author  syhanjin
 * @date    2026-04-28
 */
#include "i2c.hpp"

#include "i2c.h"

namespace
{
constexpr auto                     Bus1Handle = &hi2c1;
constexpr I2CUpdateManager::Config Bus1ManagerConfig{
    .task_name        = "i2c1-mgr",
    .stack_size_bytes = 384U * sizeof(uint32_t),
    .priority         = osPriorityLow,
    .max_sleep_ms     = 500U,
};
} // namespace

namespace AppI2C
{
I2CBusDMA& bus1()
{
    static I2CBusDMA bus(Bus1Handle);
    return bus;
}

I2CUpdateManager& manager1()
{
    static I2CUpdateManager manager(bus1());
    return manager;
}

bool start_bus1_manager()
{
    auto& manager = manager1();

    if (manager.isRunning())
        return true;

    if (manager.deviceCount() == 0U)
        return true;

    return manager.start(Bus1ManagerConfig);
}
} // namespace AppI2C
