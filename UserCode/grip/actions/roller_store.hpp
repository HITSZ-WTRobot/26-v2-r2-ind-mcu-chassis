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
 * 负责：通过机械臂抬升到吸盘存储姿态，然后启动吸盘吸取卷轴，底盘不移动
 */
class RollerStore : traits::NoCopy, traits::NoDelete
{
public:
    RollerStore();
    static RollerStore& inst();

    /**
     * @brief 开始卷轴临时存放动作
     *
     * @param storage_distance_x 保留参数，当前只执行机械臂吸盘存储，不移动底盘
     */
    void store(float storage_distance_x);

    /** @brief 释放吸盘 */
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
        MovingToStorePose,
        ActivatingSuction,
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
        [[nodiscard]] bool isAvailable() const;

    private:
        GPIO_t gpio_;
        bool   active_;
        bool   available_;
    };

    static void TaskEntry(void* self) { static_cast<RollerStore*>(self)->loop(); }

    void              update();
    [[noreturn]] void loop();

    /** @brief 检查动作是否可启动 */
    [[nodiscard]] bool canStart() const;

    osThreadId_t task_{};
    State        state_ = State::Idle;
    SuctionCup   suction_{};
};

} // namespace Grip::Action
