/**
 * @file    grip_actions.hpp
 * @brief   夹取整体所有操作
 */
#pragma once

#include "cmsis_os2.h"
#include "gpio_driver.h"
#include "traits.hpp"
#include "grip/grip.hpp"
#include "chassis/chassis.hpp"

namespace Grip::Action
{
/**
 * @brief 矛头夹取动作
 *
 * 负责：底盘移动到准备位置 -> 机械臂到位 -> 前进夹取 -> 后退 -> 机械臂回到对接状态
 */
class SpearGrab : traits::NoCopy, traits::NoDelete
{
public:
    SpearGrab();
    static SpearGrab& inst();

    /**
     * @brief 开始矛头夹取动作
     *
     * @param prepare_distance_x 车体先移动至准备位置的相对 x 距离
     * @param prepare_distance_y 车体先移动至准备位置的相对 y 距离
     * @param advance_distance   夹取矛头时前进的距离
     * @param backoff_distance   夹取完成后后退的距离
     */
    void grab(float prepare_distance_x,
              float prepare_distance_y,
              float advance_distance,
              float backoff_distance);

    /** @brief 是否为空闲状态 */
    [[nodiscard]] bool isIdle() const;

    /** @brief 是否完成所有步骤 */
    [[nodiscard]] bool isFinished() const;

    /** @brief 是否仍在运行 */
    [[nodiscard]] bool isRunning() const;

    /** @brief 等待动作完成 */
    void               waitForFinish() const;

private:
    enum class State
    {
        Idle,
        DrivingToReady,
        LiftToGrabHeight,
        ArmReady,
        DrivingForward,
        Grabbing,
        DrivingBack,
        Docking,
        Done
    };

    static void TaskEntry(void* self) { static_cast<SpearGrab*>(self)->loop(); }

    void update();
    [[noreturn]] void loop();

    [[nodiscard]] chassis::Posture relativePosture(float x, float y) const;
    [[nodiscard]] float        currentRelativeX() const;
    [[nodiscard]] bool         canStart() const;

    osThreadId_t task_{};
    State        state_ = State::Idle;

    float          prepare_distance_x_ = 0.0f;
    float          prepare_distance_y_ = 0.0f;
    float          advance_distance_   = 0.0f;
    float          backoff_distance_   = 0.0f;
    float          grab_target_x_      = 0.0f;
    float          back_target_x_      = 0.0f;
    chassis::Posture start_pos_{};
};

/**
 * @brief 取卷轴并存储动作
 *
 * 负责：通过机械臂抬升到吸盘存储姿态，然后启动吸盘吸取卷轴，底盘不移动
 */
class RollerStore : traits::NoCopy, traits::NoDelete
{
public:
    RollerStore();
    static RollerStore& inst();

    /**
     * @brief 开始卷轴存储动作
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

    void update();
    [[noreturn]] void loop();

    /** @brief 检查动作是否可启动 */
    [[nodiscard]] bool canStart() const;

    chassis::Posture relativePosture(float x) const;

    osThreadId_t task_{};
    State        state_ = State::Idle;
    chassis::Posture start_pos_{};
    SuctionCup   suction_{};
};
}

