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

class InfraredSwitch : public bsp::gpio::GpioPinInput
{
public:
    using GpioPinInput::GpioPinInput;
    [[nodiscard]] bool isTriggered() const { return isReset(); }
};