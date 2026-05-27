/**
 * @file    Step.hpp
 * @author  syhanjin Luoyue777
 * @date    2026-04-03
 * @brief   上台阶动作
 */
#pragma once
#include "traits.hpp"
#include "chassis/LiftSide.hpp"
#include "chassis/chassis.hpp"

namespace Action
{

class Step : traits::NoCopy, traits::NoDelete
{
public:
    Step();

    static Step& inst();

    enum class Direction
    {
        Forward,
        Backward
    };

    enum class Height
    {
        Step200,
        Step400
    };

    void up(float     startDistance2Step,
            float     endDistance2Step,
            Direction dir      = Direction::Forward,
            bool      willTake = false,
            Height    height   = Height::Step200);

    void up(const chassis::Posture& stepTargetPos,
            const chassis::Posture& endPos,
            Direction               dir      = Direction::Forward,
            bool                    willTake = false,
            Height                  height   = Height::Step200);

    void down(float     startDistance2Step,
              float     endDistance2Step,
              Direction dir         = Direction::Forward,
              bool      shouldReset = true,
              Height    height      = Height::Step200);

    void down(const chassis::Posture& stepTargetPos,
              const chassis::Posture& endPos,
              Direction               dir         = Direction::Forward,
              bool                    shouldReset = true,
              Height                  height      = Height::Step200);

    void resume_up();

    static void TaskEntry(void* self) { static_cast<Step*>(self)->loop(); }

    [[noreturn]] void loop();

    [[nodiscard]] bool isIdle() const
    {
        return chassis_state_ == ChassisState::Idle && front_state_ == LiftState::Idle &&
               rear_state_ == LiftState::Idle;
    }

    [[nodiscard]] bool isWaitingTake() const
    {
        return will_take_ && chassis_state_ == ChassisState::Up2;
    }

    [[nodiscard]] bool isFinished() const
    {
        return chassis_state_ == ChassisState::Done && front_state_ == LiftState::Done &&
               rear_state_ == LiftState::Done;
    }

    [[nodiscard]] bool isRunning() const { return !isFinished() && !isIdle(); }

    /**
     * 等待执行完成
     */
    void waitForFinish() const
    {
        while (!isFinished())
            osDelay(10);
    }

    /**
     * 等待中间停止取件
     *
     * TODO: 好奇怪的名字，改一下
     */
    void waitForPause() const
    {
        if (will_take_)
            while (!isWaitingTake())
                osDelay(10);
    }

private:
    osThreadId_t task_;

    void prepare(const chassis::Posture& stepTargetPos,
                 const chassis::Posture& endPos,
                 Direction               dir);

    void update();

    enum class ChassisState
    {
        Idle,

        // 目标 R{-(HalfChassisDiagonal + SafeDistance), 0, dirRelativeYaw}；
        // 前后腿同步抬升到台阶高度；yaw 到阈值后切换到 Up1。
        Up0,
        // 目标 R{-(HalfChassisDistanceX + SafeDistance), 0, dirRelativeYaw}；
        // 等待前后腿抬升完成且 y 到阈值后切换到原上台阶动作链。
        Up1,
        // 目标 R{-(AbsWheelOuterEdgeX + SafeDistance), 0, dirRelativeYaw}；
        // 前辅助轮越过台阶边缘，willTake 时在此等待 0x31 恢复。
        Up2,
        // 目标 R{AbsWheelInnerEdgeX - 3 * SafeDistance, 0, dirRelativeYaw}；
        // 等待后腿收起，之后切到保持台阶 yaw 的 EndPos 相对 x。
        Up3,
        // 目标 R{end_rel.x, 0, dirRelativeYaw}；
        // 等待后腿放下，之后切到保持台阶 yaw 的 EndPos 相对 x/y。
        Up4,
        // 目标 R{end_rel.x, end_rel.y, dirRelativeYaw}；
        // 前后腿恢复 Normal，离开台阶安全 x 后切最终 EndPos。
        Up5,
        // 目标 EndPos；等待底盘轨迹和前后腿都完成。
        Up6,

        // 目标 R{-(HalfWheelDiagonal + WheelRadius + 3 * SafeDistance), 0, dirRelativeYaw}；
        // 前后腿同步到过渡高度；yaw 到阈值后切换到 Down1。
        Down0,
        // 目标 R{-(AbsAuxInnerWheelX + 3 * SafeDistance), 0, dirRelativeYaw}；
        // 等待前后腿到过渡高度后进入原下台阶动作链。
        Down1,
        // 继续前侧辅助轮到达台阶边缘的推进；等待前腿下放完成。
        Down2,
        // 目标 R{AbsAuxOuterWheelX - 3 * SafeDistance, 0, dirRelativeYaw}；
        // 等待后腿下放完成。
        Down3,
        // 目标 W{end_pos.x, end_pos.y, step_target_pos.yaw + dirRelativeYaw}；
        // 腿高度保持不变，离开台阶安全 x 后切最终 EndPos。
        Down4,
        // 目标 EndPos；shouldReset 时前后腿恢复 Normal，否则保持高度等待底盘完成。
        Down5,

        Done
    };

    enum class LiftState
    {
        Idle,

        Up1_抬升ing,
        Up2_等待收起,
        Up3_收起ing,
        Up4_等待放下,
        Up5_放下ing,
        Up6_等待恢复到Normal,
        Up7_恢复到Normaling,

        Down1_等待放下,
        Down2_放下ing,
        Down3_等待回收到正常位置,
        Down4_回收ing,

        Done
    };

    // 上台阶过程是否停下取物
    bool will_take_ = false;

    // 下台阶是否复位底盘
    bool should_reset_ = true;

    Height height_ = Height::Step200;

    Direction direction_ = Direction::Forward;

    float dir_relative_yaw_ = 0.0f;

    ChassisState chassis_state_ = ChassisState::Idle;
    LiftState    front_state_   = LiftState::Idle;
    LiftState    rear_state_    = LiftState::Idle;

    chassis::Posture step_target_pos_{};
    chassis::Posture end_pos_{};

    Lift::LiftSide* front_ = nullptr;
    Lift::LiftSide* rear_  = nullptr;

    [[nodiscard]] chassis::Posture stepRelativePosture(const float x) const
    {
        return stepRelativePosture({ x, 0.0f, dir_relative_yaw_ });
    }

    [[nodiscard]] chassis::Posture stepRelativePosture(const chassis::Posture& rel_pos) const
    {
        return chassis::loc::IChassisLoc::RelativePosture2WorldPosture(step_target_pos_, rel_pos);
    }

    [[nodiscard]] chassis::Posture currentRelativeToStep() const
    {
        return Chassis::loc->CurrentPostureRelativeTo(step_target_pos_);
    }

    [[nodiscard]] float currentRelativeX() const { return currentRelativeToStep().x; }

    [[nodiscard]] bool yawPrepared() const;

    [[nodiscard]] chassis::Posture endXWithStepYaw() const;

    [[nodiscard]] chassis::Posture endXYWithStepYaw() const;

    [[nodiscard]] float stepUpPosition() const;
};

} // namespace Action
