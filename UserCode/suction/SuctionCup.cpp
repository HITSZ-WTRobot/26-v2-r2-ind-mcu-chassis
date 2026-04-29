/**
 * @file    SuctionCup.cpp
 * @brief   可复用吸盘组件实现
 */
#include "SuctionCup.hpp"

#include "I2CUpdateManager.hpp"
#include "i2c.hpp"
#include "main.h"
#include "project_parts.hpp"

namespace Suction
{
namespace
{
[[nodiscard]] bool elapsed(const uint32_t now_ms, const uint32_t since_ms, const uint32_t delay_ms)
{
    return (now_ms - since_ms) >= delay_ms;
}
} // namespace

SuctionCup::SuctionCup(const OwnerConfig& config) :
    config_(config),
    pressure_sensor_(::Suction::Config::PressureRangeKPa, ::Suction::Config::PressureAddress7bit)
{
    if constexpr (ProjectParts::EnableGripSuctionPressureSensor)
    {
        // 传感器注册属于吸盘内部实现细节；上层只关心“能不能判断吸住”。
        if (!registerPressureSensor(AppI2C::manager1()))
            Error_Handler();
    }
}

void SuctionCup::activate()
{
    GPIO_t pump_gpio = config_.pump_gpio;
    GPIO_SetPin(&pump_gpio);

    taskENTER_CRITICAL();
    pump_active_              = true;
    pump_state_changed_at_ms_ = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SuctionCup::deactivate()
{
    GPIO_t pump_gpio = config_.pump_gpio;
    GPIO_ResetPin(&pump_gpio);

    taskENTER_CRITICAL();
    pump_active_              = false;
    pump_state_changed_at_ms_ = HAL_GetTick();
    taskEXIT_CRITICAL();
}

bool SuctionCup::hasObject() const
{
    refreshObjectDetectedState();

    taskENTER_CRITICAL();
    const bool detected = object_detected_;
    taskEXIT_CRITICAL();
    return detected;
}

bool SuctionCup::registerPressureSensor(I2CUpdateManager& manager)
{
    if (i2c_registered_)
        return true;

    i2c_registered_ = manager.registerDevice(pressure_sensor_,
                                             ::Suction::Config::PressureUpdatePeriodMs,
                                             config_.pressure_update_phase_ms,
                                             ::Suction::Config::PressureTimeoutMs);
    return i2c_registered_;
}

bool SuctionCup::isPressureSensorOnline() const
{
    if constexpr (!ProjectParts::EnableGripSuctionPressureSensor)
        return false;

    return pressure_sensor_.isOnline();
}

void SuctionCup::refreshObjectDetectedState() const
{
    const uint32_t now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    bool     pump_active         = pump_active_;
    uint32_t state_changed_at_ms = pump_state_changed_at_ms_;
    taskEXIT_CRITICAL();

    if constexpr (ProjectParts::EnableGripSuctionPressureSensor)
    {
        const PressureSample sample = pressure_sensor_.snapshot();
        if (sample.valid && pressure_sensor_.isDataFresh(now_ms, config_.pressure_stale_ms))
        {
            // 只要样本可用，就优先相信气压计，延时只作为编译期/运行期回退方案。
            refreshObjectDetectedFromPressure(sample);
            return;
        }
    }

    // 无气压计，或本周期暂时没有新鲜压力样本时，退化为时间确认。
    refreshObjectDetectedFromDelay(pump_active, state_changed_at_ms, now_ms);
}

void SuctionCup::refreshObjectDetectedFromPressure(const PressureSample& sample) const
{
    taskENTER_CRITICAL();
    if (!object_detected_)
    {
        if (sample.pressure_pa <= config_.object_detect_on_pressure_pa)
            object_detected_ = true;
    }
    else if (sample.pressure_pa >= config_.object_detect_off_pressure_pa)
    {
        object_detected_ = false;
    }
    taskEXIT_CRITICAL();
}

void SuctionCup::refreshObjectDetectedFromDelay(const bool     pump_active,
                                                const uint32_t state_changed_at_ms,
                                                const uint32_t now_ms) const
{
    taskENTER_CRITICAL();
    if (pump_active)
    {
        // 只要持续开吸达到阈值，就把状态锁存为“已吸住”。
        if (elapsed(now_ms, state_changed_at_ms, config_.object_detect_delay_ms))
            object_detected_ = true;
    }
    else if (elapsed(now_ms, state_changed_at_ms, config_.object_release_delay_ms))
    {
        // 关闭吸盘后延时清除锁存，避免上层在切换瞬间误判。
        object_detected_ = false;
    }
    taskEXIT_CRITICAL();
}

} // namespace Suction
