/**
 * @file    roller_store.hpp
 * @brief   Grip 卷轴临时存放动作组
 */
#pragma once

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "suction/SuctionCup.hpp"
#include "traits.hpp"

namespace Grip::Action
{

/**
 * @brief 卷轴临时存放动作
 *
 * 负责：卷轴吸盘的无参暂存 / 释放动作，底盘不移动。
 *
 * 状态机会串联以下流程：
 * - 暂存：吸盘启动 -> 去拾取位 -> 等待气压确认或无气压计保底延时 -> 去暂存位；
 * - 释放：去释放位 -> 关闭吸盘 -> 等待气压确认或无气压计保底延时 -> 回待机位。
 */
class KfsStore : traits::NoCopy, traits::NoDelete
{
public:
    enum class WorkflowPhase
    {
        Idle,
        Store,
        Release,
    };

    /** @brief 创建后台状态机线程。 */
    KfsStore();
    /** @brief 获取全局唯一动作实例。 */
    static KfsStore& inst();

    /**
     * @brief 开始卷轴临时存放动作
     */
    void store();

    /** @brief 开始卷轴释放动作 */
    void release();

    /** @brief 当前是否为空闲状态。 */
    [[nodiscard]] bool isIdle() const;
    /** @brief 当前是否已结束。 */
    [[nodiscard]] bool isFinished() const;
    /** @brief 当前是否仍在执行动作。 */
    [[nodiscard]] bool isRunning() const;
    /** @brief 当前高层流程归属：暂存、回放或尚未进入动作流。 */
    [[nodiscard]] WorkflowPhase workflowPhase() const { return workflow_phase_; }
    /** @brief 阻塞等待动作结束。 */
    void waitForFinish() const;

private:
    /** @brief KFS 暂存 / 释放流程状态。 */
    enum class State
    {
        Idle,                 ///< 空闲。
        MovingToPickupPose,   ///< 正在移动到 KFS 拾取姿态。
        WaitingObjectAttach,  ///< 已到拾取位，等待气压确认或无气压计保底延时。
        MovingToStorePose,    ///< 正在移动到 KFS 暂存姿态。
        MovingToReleasePose,  ///< 正在移动到 KFS 释放姿态。
        WaitingObjectRelease, ///< 已关闭吸盘，等待气压确认或无气压计保底延时。
        MovingToStandbyPose,  ///< 释放后回系统待机姿态。
        Done                  ///< 流程结束。
    };

    static void TaskEntry(void* self) { static_cast<KfsStore*>(self)->loop(); }

    /** @brief 推进一步状态机。由后台线程 1 ms 周期调用。 */
    void update();
    /** @brief 后台线程主循环，等待动作启动后推进状态机。 */
    [[noreturn]] void loop();

    /** @brief 检查动作是否可启动 */
    [[nodiscard]] bool canStart() const;

    /// 后台状态机线程句柄。
    osThreadId_t task_{};
    /// 当前 KFS 动作阶段。
    State state_ = State::Idle;
    /// 面向上位机反馈的动作流归属；暂存完成后会保持为 Store，直到发起回放。
    WorkflowPhase workflow_phase_ = WorkflowPhase::Idle;
    /// KFS 持有的吸盘组件。
    Suction::SuctionCup kfs_suction_cup_;
    /// 进入等待吸上 / 放下确认阶段时的时间戳，用于无气压计保底判定。
    uint32_t wait_state_since_ms_{};
    /// 无气压计时使用的流程内持物状态；有气压计时仅镜像最近一次确认结果。
    bool assumed_has_object_ = false;
};

} // namespace Grip::Action
