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
 *      先沿 target_pos 的 x 方向离开危险区 -> 再移动到 end_pos 并对接。
 *
 * 该类只编排高层状态机，不直接实现底层闭环：
 * - 底盘轨迹由 Chassis::ctrl 执行；
 * - 升降目标由 Chassis::motion 负责；
 * - 机械臂关节姿态由 Grip::grip 负责。
 */
class SpearGrab : traits::NoCopy, traits::NoDelete
{
public:
    /** @brief 创建后台状态机线程。 */
    SpearGrab();
    /** @brief 获取全局唯一动作实例。 */
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

    /** @brief 是否为空闲状态，表示尚未接收动作或已被复位。 */
    [[nodiscard]] bool isIdle() const;

    /** @brief 是否完成所有步骤。注意：中止也会进入 Done。 */
    [[nodiscard]] bool isFinished() const;

    /** @brief 是否仍在运行中。 */
    [[nodiscard]] bool isRunning() const;

    /** @brief 阻塞等待动作结束。 */
    void waitForFinish() const;

private:
    /** @brief SpearGrab 高层阶段机。 */
    enum class State
    {
        Idle,                 ///< 空闲，尚未开始。
        MovingToPrepare,      ///< 底盘切入 prepare 区，同时 lift / grip 调整到预备状态。
        MovingToTarget,       ///< 由 prepare 区继续切入目标位姿。
        Grabbing,             ///< 底盘已到位，机械臂执行夹取。
        LeavingTargetToSafeX, ///< 沿目标 x 方向先脱离危险区。
        MovingToEnd,          ///< 从安全 x 位置移动到最终结束位姿。
        Done                  ///< 全流程结束，或因失败中止。
    };

    static void TaskEntry(void* self) { static_cast<SpearGrab*>(self)->loop(); }

    /** @brief 推进一步状态机。由后台线程 1 ms 周期调用。 */
    void update();

    /** @brief 后台线程主循环，等待启动标志后持续推进状态机。 */
    [[noreturn]] void loop();

    /** @brief 将目标局部坐标系下的相对位姿转换到世界系。 */
    [[nodiscard]] chassis::Posture postureRelativeToTargetInWorld(
            const chassis::Posture& rel_pos) const;

    /** @brief 获取当前车体相对于 target_pos_ 的位姿误差。 */
    [[nodiscard]] chassis::Posture currentRelativeToTarget() const;

    /** @brief 检查当前硬件与功能开关是否允许启动动作。 */
    [[nodiscard]] bool canStart() const;
    /** @brief 发生不可继续的错误时停车并收敛到 Done。 */
    void abort();

    /// 后台状态机线程句柄。
    osThreadId_t task_{};
    /// 当前高层动作阶段。
    State state_ = State::Idle;

    /// 目标点前的安全 x 距离，单位 m。
    float safe_distance_ = 0.0f;
    /// 待取矛头的世界系位姿。
    chassis::Posture target_pos_{};
    /// 流程结束后应到达的世界系位姿。
    chassis::Posture end_pos_{};
    /// end_pos_ 相对 target_pos_ 的位姿，用于安全区规划。
    chassis::Posture end_pos_rel_to_target_{};
    /// 进入夹取前的 prepare 位姿。
    chassis::Posture prepare_pos_{};
    /// 仅沿目标 x 方向先撤离后的中间位姿。
    chassis::Posture leave_target_x_only_pos_{};
};

} // namespace Grip::Action
