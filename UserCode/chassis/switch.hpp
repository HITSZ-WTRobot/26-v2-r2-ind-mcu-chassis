/**
 * @file    switch.hpp
 * @author  syhanjin
 * @date    2026-07-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once
#include "gpio_driver.hpp"

#include <cstdint>

class InfraredSwitch : public bsp::gpio::GpioPinInput
{
public:
    using GpioPinInput::GpioPinInput;

    static constexpr uint32_t DebounceMs = 5U;

    void initExtiDebounce()
    {
        stable_triggered_   = isRawTriggered();
        exti_triggered_     = isRawTriggered();
        confirm_after_tick_ = 0U;
        user_data           = this;
        bsp::gpio::RegisterExtiCallback(this, extiCallback);
    }

    [[nodiscard]] bool isTriggered() const
    {
        if (HAL_GetTick() - confirm_after_tick_ <= DebounceMs)
            return stable_triggered_;
        stable_triggered_ = exti_triggered_;
        return stable_triggered_;
    }

    [[nodiscard]] bool isRawTriggered() const { return isReset(); }

private:
    static void extiCallback(const bsp::gpio::GpioPinInput* gpio, uint32_t counter)
    {
        (void)counter;

        if (gpio == nullptr || gpio->user_data == nullptr)
            return;

        static_cast<InfraredSwitch*>(gpio->user_data)->exti();
    }

    void exti()
    {
        confirm_after_tick_ = HAL_GetTick() + DebounceMs;
        exti_triggered_     = isRawTriggered();
    }

    mutable volatile bool stable_triggered_   = false;
    volatile bool         exti_triggered_     = false;
    volatile uint32_t     confirm_after_tick_ = 0U;
};
