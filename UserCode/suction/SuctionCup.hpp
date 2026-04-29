/**
 * @file    SuctionCup.hpp
 * @brief   可复用吸盘组件
 */
#pragma once

#include "Config.hpp"
#include "XGZP6847DDevice.hpp"
#include "gpio_driver.h"
#include "traits.hpp"

#include <atomic>

namespace Suction
{

/**
 * @brief 单个吸盘组件
 *
 * 负责：
 * - 控制气泵 GPIO 开关；
 * - 在内部使用可选气压计和施密特触发判断“是否吸住物体”；
 * - 向上层只暴露吸盘开关与吸附状态这三个高层语义。
 *
 * 通用气压计参数统一由 `suction/Config.hpp` 提供；
 * 每个持有者负责给出自己这只吸盘的特化配置，并在构造前完成气压计注册。
 */
class SuctionCup : traits::NoCopy, traits::NoDelete
{
public:
    struct Config
    {
        GPIO_t   pump_gpio;
        uint32_t pressure_stale_ms{ 120U };
        float    object_detect_on_pressure_pa;
        float    object_detect_off_pressure_pa;
    };

    explicit SuctionCup(const Config& config, XGZP6847DDevice* pressure_sensor = nullptr);

    /** @brief 启动气泵。 */
    void activate();
    /** @brief 关闭气泵。 */
    void deactivate();

    /**
     * @brief 判断当前是否已吸住物体
     *
     * - 有气压计且当前样本新鲜时：按当前压力和上次判定状态做施密特触发判断；
     * - 没有气压计，或当前拿不到新鲜压力样本时：固定返回 false。
     */
    [[nodiscard]] bool hasObject();
    [[nodiscard]] bool canDetectObject() const { return pressure_sensor_ != nullptr; }

private:
    Config           config_{};
    XGZP6847DDevice* pressure_sensor_{ nullptr };
    std::atomic_bool has_object_{ false };
};

} // namespace Suction
