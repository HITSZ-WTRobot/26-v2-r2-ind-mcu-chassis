/**
 * @file    SuctionCup.hpp
 * @brief   可复用吸盘组件
 */
#pragma once

// SuctionCup 只负责吸盘本身，不知道自己属于哪个机构。
// 它接受一个可选压力传感器指针，并通过迟滞逻辑提供“是否吸住”的判断。

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
        // 气泵 GPIO 的所有权在 owner，SuctionCup 这里只持有一个拷贝。
        GPIO_t   pump_gpio;
        // 如果样本超过这个时间没刷新，就认为它已经不可靠。
        uint32_t pressure_stale_ms{ 120U };
        // 低压阈值触发“吸住”，高压阈值触发“释放”，形成迟滞区间。
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
    /** @brief 当前这个吸盘是否具备气压计判定能力。 */
    [[nodiscard]] bool canDetectObject() const { return pressure_sensor_ != nullptr; }

private:
    Config           config_{};
    // 传感器生命周期由外部管理，这里只保存观察用裸指针。
    XGZP6847DDevice* pressure_sensor_{ nullptr };
    // 最近一次“是否吸住”的锁存状态。
    std::atomic_bool has_object_{ false };
};

} // namespace Suction
