/**
 * @file    spear_grab.hpp
 * @brief   Grip 取矛头动作组
 */
#pragma once

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "traits.hpp"

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
     * @param end_pos 动作完成后的世界系绝对位姿，需保证其位于安全区内
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
    void waitForFinish() const;

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

    [[nodiscard]] chassis::Posture postureRelativeToTargetInWorld(
            const chassis::Posture& rel_pos) const;

    [[nodiscard]] chassis::Posture currentRelativeToTarget() const;

    [[nodiscard]] bool canStart() const;

    osThreadId_t task_{};
    State        state_ = State::Idle;

    float            safe_distance_ = 0.0f;
    chassis::Posture target_pos_{};
    chassis::Posture end_pos_{};
    chassis::Posture end_pos_rel_to_target_{};
    chassis::Posture prepare_pos_{};
    chassis::Posture leave_target_x_only_pos_{};
};

} // namespace Grip::Action
