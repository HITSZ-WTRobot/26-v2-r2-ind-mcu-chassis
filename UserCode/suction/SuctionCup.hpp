/**
 * @file    SuctionCup.hpp
 * @brief   可复用吸盘组件
 */
#pragma once

#include "Config.hpp"
#include "XGZP6847DDevice.hpp"
#include "gpio_driver.h"
#include "traits.hpp"

class I2CUpdateManager;
namespace Grip::Action
{
class KfsStore;
}

namespace Suction
{

/**
 * @brief 单个吸盘组件
 *
 * 负责：
 * - 控制气泵 GPIO 开关；
 * - 在内部决定使用气压计还是延时来判断“是否吸住物体”；
 * - 向上层只暴露吸盘开关与吸附状态这三个高层语义。
 *
 * 通用气压计参数统一由 `suction/Config.hpp` 提供；
 * 每个持有者负责给出自己这只吸盘的特化配置。
 */
class SuctionCup : traits::NoCopy, traits::NoDelete
{
public:
    struct OwnerConfig
    {
        GPIO_t   pump_gpio{};
        uint32_t pressure_update_phase_ms{ 0U };
        uint32_t pressure_stale_ms{ 120U };
        float    object_detect_on_pressure_pa;
        float    object_detect_off_pressure_pa;
        /// 未使用气压计，或当前压力样本不新鲜时，延时多久认为“已经吸住”。
        uint32_t object_detect_delay_ms;
        /// 关闭吸盘后，延时多久认为“已经放开”。
        uint32_t object_release_delay_ms;
    };

    explicit SuctionCup(const OwnerConfig& config);

    /** @brief 启动气泵。 */
    void activate();
    /** @brief 关闭气泵。 */
    void deactivate();

    /**
     * @brief 判断当前是否已吸住物体
     *
     * - 启用气压计时：内部使用压力施密特触发判定；
     * - 未启用气压计，或当前拿不到新鲜压力样本时：退化为开吸/关吸延时判定。
     */
    [[nodiscard]] bool hasObject() const;

private:
    using PressureSample = XGZP6847DDevice::Sample;

    /// 仅在启用气压计编译开关时调用；约束：必须在 manager start 前注册。
    [[nodiscard]] bool registerPressureSensor(I2CUpdateManager& manager);
    [[nodiscard]] bool isPressureSensorOnline() const;
    /// 统一刷新“是否吸住”锁存状态，并在内部决定走压力还是延时判定。
    void refreshObjectDetectedState() const;
    /// 压力路径使用施密特触发，避免阈值附近抖动造成状态反复翻转。
    void refreshObjectDetectedFromPressure(const PressureSample& sample) const;
    /// 延时路径用于未启用气压计，或当前暂时拿不到可用压力样本的场景。
    void refreshObjectDetectedFromDelay(bool     pump_active,
                                        uint32_t state_changed_at_ms,
                                        uint32_t now_ms) const;

    friend class ::Grip::Action::KfsStore;

    OwnerConfig     config_{};
    XGZP6847DDevice pressure_sensor_;
    bool            i2c_registered_{ false };
    /// 记录逻辑上的吸盘开关状态；不要让上层再直接读 GPIO 决定业务语义。
    bool     pump_active_{ false };
    uint32_t pump_state_changed_at_ms_{ 0U };
    /// “是否吸住物体”的稳定锁存结果，由压力或延时路径共同维护。
    mutable bool object_detected_{ false };
};

} // namespace Suction
