/**
 * @file    i2c.cpp
 * @author  syhanjin
 * @date    2026-04-28
 */
#include "i2c.hpp"

#include "i2c.h"

namespace
{
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
    static I2CBusDMA bus(Bus2Handle);
    return bus;
}

I2CUpdateManager& manager2()
{
    static I2CUpdateManager manager(bus2());
    return manager;
}

bool start_bus2_manager()
{
    auto& manager = manager2();

    if (manager.isRunning())
        return true;

    if (manager.deviceCount() == 0U)
        return true;

    return manager.start(Bus2ManagerConfig);
}
} // namespace AppI2C
