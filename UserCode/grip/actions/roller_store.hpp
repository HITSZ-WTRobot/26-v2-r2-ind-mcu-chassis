/**
 * @file    roller_store.hpp
 * @brief   Grip 卷轴临时存放动作组
 */
#pragma once

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "gpio_driver.h"
#include "traits.hpp"

namespace Grip::Action
{
/**
 * @brief 卷轴临时存放动作
 *
 * 负责：卷轴吸盘的无参暂存 / 释放动作，底盘不移动
 */
class KfsStore : traits::NoCopy, traits::NoDelete
{
public:
    KfsStore();
    static KfsStore& inst();

    /**
     * @brief 开始卷轴临时存放动作
     */
    void store();

    /** @brief 开始卷轴释放动作 */
    void release();

    [[nodiscard]] bool isIdle() const;
    [[nodiscard]] bool isFinished() const;
    [[nodiscard]] bool isRunning() const;
    [[nodiscard]] bool isSuctionActive() const;
    void               waitForFinish() const;

private:
    enum class State
    {
        Idle,
        MovingToPickupPose,
        WaitingSuctionBuildUp,
        MovingToStorePose,
        MovingToReleasePose,
        MovingToStandbyPose,
        Done
    };

    struct SuctionCup
    {
        /** @brief 吸盘使能封装 */
        SuctionCup();

        /** @brief 激活吸盘 */
        void activate();

        /** @brief 关闭吸盘 */
        void deactivate();

        [[nodiscard]] bool isActive() const;

    private:
        GPIO_t gpio_;
    };

    static void TaskEntry(void* self) { static_cast<KfsStore*>(self)->loop(); }

    void              update();
    [[noreturn]] void loop();

    /** @brief 检查动作是否可启动 */
    [[nodiscard]] bool canStart() const;

    osThreadId_t task_{};
    State        state_ = State::Idle;
    SuctionCup   suction_{};
    uint32_t     delay_ms_remaining_{ 0 };
};

} // namespace Grip::Action
