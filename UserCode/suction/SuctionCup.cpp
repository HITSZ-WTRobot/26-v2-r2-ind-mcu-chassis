/**
 * @file    SuctionCup.cpp
 * @brief   可复用吸盘组件实现
 */
#include "SuctionCup.hpp"

#include "main.h"

namespace Suction
{
SuctionCup::SuctionCup(const Config& config, XGZP6847DDevice* pressure_sensor) :
    config_(config), pressure_sensor_(pressure_sensor)
{
}

void SuctionCup::activate()
{
    GPIO_SetPin(&config_.pump_gpio);
}

void SuctionCup::deactivate()
{
    GPIO_ResetPin(&config_.pump_gpio);
}

bool SuctionCup::hasObject()
{
    if (pressure_sensor_ == nullptr)
        return false;

    const uint32_t now_ms = HAL_GetTick();

    const XGZP6847DDevice::Sample sample = pressure_sensor_->snapshot();

    if (!sample.valid || !pressure_sensor_->isDataFresh(now_ms, config_.pressure_stale_ms))
        return false;

    bool expected = has_object_.load(std::memory_order_relaxed);
    for (;;)
    {
        bool desired = expected;
        if (!expected)
        {
            if (sample.pressure_pa <= config_.object_detect_on_pressure_pa)
                desired = true;
        }
        else if (sample.pressure_pa >= config_.object_detect_off_pressure_pa)
        {
            desired = false;
        }

        if (has_object_.compare_exchange_weak(
                    expected, desired, std::memory_order_relaxed, std::memory_order_relaxed))
        {
            return desired;
        }
    }
}
} // namespace Suction
