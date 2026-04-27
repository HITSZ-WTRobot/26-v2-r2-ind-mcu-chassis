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
 * 负责：底盘切入 prepare 区 -> 夹爪进入 ready -> 到 target_pos 夹取 ->
 *      先沿 target_pos 的 x 方向离开危险区 -> 再移动到 end_pos 并对接
 */
class SpearGrab : traits::NoCopy, traits::NoDelete
{
public:
    SpearGrab();
    static SpearGrab& inst();

    /**
     * @brief 开始矛头夹取动作
     *
     * @param target_pos 待取矛头所在的世界系绝对位姿
     * @param end_pos    动作完成后的世界系绝对位姿，需保证其位于安全区内
     * @param safe_distance 相对于 target_pos 的安全 x 距离
     */
    void grab(const chassis::Posture& target_pos,
              const chassis::Posture& end_pos,
              float                   safe_distance);

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
        MovingToPrepare,
        MovingToTarget,
        Grabbing,
        LeavingTargetToSafeX,
        MovingToEnd,
        Done
    };

    static void TaskEntry(void* self) { static_cast<SpearGrab*>(self)->loop(); }

    void update();
    [[noreturn]] void loop();

    [[nodiscard]] chassis::Posture postureRelativeToTargetInWorld(const chassis::Posture& rel_pos)
            const;
    [[nodiscard]] chassis::Posture currentRelativeToTarget() const;
    [[nodiscard]] bool             canStart() const;

    osThreadId_t task_{};
    State        state_ = State::Idle;

    float           safe_distance_ = 0.0f;
    chassis::Posture target_pos_{};
    chassis::Posture end_pos_{};
    chassis::Posture end_pos_rel_to_target_{};
    chassis::Posture prepare_pos_{};
    chassis::Posture leave_target_x_only_pos_{};
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
