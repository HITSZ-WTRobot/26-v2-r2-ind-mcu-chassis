/**
 * @file    spear_grab.hpp
 * @brief   Grip 取矛头动作组
 */
#pragma once

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "diagnostics/SpearGrabActionDiagnostics.hpp"
#include "traits.hpp"

namespace Grip::Action
{

/**
 * @brief 矛头夹取动作
 *
 * 负责：启动时底盘切入 target_pos 安全距离边界、lift 抬升、夹爪进入预备 1 ->
 *      lift 到位后进入近距离 prepare 区 -> 到达 prepare 后切预备 2 ->
 *      grip / y / yaw / lift 就位后到 target_pos，合爪并保持 ->
 *      保持完成后抬升 lift，lift 就位后先沿 target_pos 的 x 方向离开危险区 ->
 *      再移动到 end_pos 并对接。
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
     * @param lift_execute 夹取前 lift 执行高度，单位 m
     */
    void grab(const chassis::Posture& target_pos,
              const chassis::Posture& end_pos,
              float                   lift_execute);

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
        MovingGripToPrepare1, ///< 等待 arm 达到预备 1 高度。
        MovingToPrepare,      ///< 等待 lift 到执行高度且 yaw 到位，再切近距离 prepare 区。
        MovingGripToPrepare2, ///< 等待 chassis 到达 prepare 后切 grip 预备 2。
        WaitingTargetEntry,   ///< 等待 grip / y / yaw / lift 就位，再切入目标位姿。
        MovingToTarget,       ///< 等待 chassis 到 target，随后合爪并开始保持。
        WaitingClawClose,     ///< 合爪后等待夹紧保持时间。
        RaisingLiftAfterGrab, ///< 合爪后继续抬升 lift，等待矛头脱离。
        LeavingTargetToSafeX, ///< lift 就位后沿目标 x 方向先脱离危险区。
        MovingToEnd,          ///< 从安全 x 位置移动到最终结束位姿。
        WaitingSettle,        ///< 等待就位稳定
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
    void abort(Diagnostics::SpearGrabAction::Reason reason);
    /** @brief Grip 双轴姿态规划失败时上报详细轴信息并中止动作。 */
    void reportGripPlanFailure();
    /** @brief 底盘 Master 轨迹规划失败时上报详细轴信息并中止动作。 */
    void reportChassisPlanFailure();
    /** @brief 设置带限速的底盘目标，失败时自动走诊断上报路径。 */
    [[nodiscard]] bool setChassisTarget(const chassis::Posture&                            target,
                                        Chassis::ChassisController::TrajectoryLinkMode     mode,
                                        const Chassis::ChassisController::TrajectoryLimit& limit);
    /** @brief 设置普通底盘目标，失败时自动走诊断上报路径。 */
    [[nodiscard]] bool setChassisTarget(const chassis::Posture&                        target,
                                        Chassis::ChassisController::TrajectoryLinkMode mode);

    [[nodiscard]] Diagnostics::SpearGrabAction::Context diagnosticContext() const
    {
        return { .stage = static_cast<uint8_t>(state_) };
    }

    /// 后台状态机线程句柄。
    osThreadId_t task_{};
    /// 当前高层动作阶段。
    State state_ = State::Idle;
    /// 本轮动作是否以失败方式收敛；Done 只表示状态机已经停下。
    bool failed_ = false;

    /// 待取矛头的世界系位姿。
    chassis::Posture target_pos_{};
    /// 流程结束后应到达的世界系位姿。
    chassis::Posture end_pos_{};
    /// end_pos_ 相对 target_pos_ 的位姿，用于安全区规划。
    chassis::Posture end_pos_rel_to_target_{};
    /// lift 到位前允许靠近的安全距离边界位姿。
    chassis::Posture prepare_safe_pos_{};
    /// lift 到位后进入夹取前的近距离 prepare 位姿。
    chassis::Posture prepare_approach_pos_{};
    /// 仅沿目标 x 方向先撤离后的中间位姿。
    chassis::Posture leave_target_x_only_pos_{};
    /// 本轮动作传入的 lift 执行高度。
    float lift_execute_{};
    /// 本轮合爪后继续抬升的 lift 目标高度。
    float post_grab_lift_pos_{};
    /// 进入合爪保持阶段时的时间戳。
    uint32_t wait_state_since_ms_{};
    uint32_t wait_settle_since_ms_{};
};

} // namespace Grip::Action
