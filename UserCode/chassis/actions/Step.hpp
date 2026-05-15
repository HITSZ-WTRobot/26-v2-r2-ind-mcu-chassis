/**
 * @file    Step.hpp
 * @author  syhanjin Luoyue777
 * @date    2026-04-03
 * @brief   上台阶动作
 */
#pragma once

// Step 是一个“底盘 + 前后 lift 协同”的高层动作状态机。
// 它不直接做连续控制，只负责把动作拆成多个阶段并在合适时机切换目标。

#include "traits.hpp"
#include "chassis/LiftSide.hpp"
#include "chassis/chassis.hpp"

namespace Action
{

class Step : traits::NoCopy, traits::NoDelete
{
public:
    Step();

    // 单例入口，便于协议层和测试入口共用同一个动作状态机。
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

    // 上台阶：从台阶前方切入，必要时可以在中途暂停去取件。
    void up(float     startDistance2Step,
            float     endDistance2Step,
            Direction dir      = Direction::Forward,
            bool      willTake = false,
            Height    height   = Height::Step200);

    // 下台阶：从高处回到低处，必要时可选择是否恢复到底盘正常高度。
    void down(float     startDistance2Step,
              float     endDistance2Step,
              Direction dir         = Direction::Forward,
              bool      shouldReset = true,
              Height    height      = Height::Step200);

    // 继续上台阶中途暂停后的后半段。
    void resume_up();

    static void TaskEntry(void* self) { static_cast<Step*>(self)->loop(); }

    // 后台线程只负责 1 ms 粒度推进状态机。
    [[noreturn]] void loop();

    [[nodiscard]] bool isIdle() const
    {
        return chassis_state_ == ChassisState::Idle && front_state_ == LiftState::Idle &&
               rear_state_ == LiftState::Idle;
    }

    [[nodiscard]] bool isWaitingTake() const
    {
        return will_take_ &&
               chassis_state_ == ChassisState::Up2_前进将前辅助轮悬于台阶上方_等待前轮收起;
    }

    [[nodiscard]] bool isFinished() const
    {
        return chassis_state_ == ChassisState::Done && front_state_ == LiftState::Done &&
               rear_state_ == LiftState::Done;
    }

    [[nodiscard]] bool isRunning() const { return !isFinished() && !isIdle(); }

    /** @brief 阻塞等待动作完全结束。 */
    void waitForFinish() const
    {
        while (!isFinished())
            osDelay(10);
    }

    /** @brief 等待上台阶中途暂停到取件窗口。 */
    void waitForPause() const
    {
        if (will_take_)
            while (!isWaitingTake())
                osDelay(10);
    }

private:
    // 独立线程句柄。
    osThreadId_t task_;

    // 在动作开始前，把左右前后 lift 的角色和起始位姿都整理好。
    void prepare(float startDistance2Step, float endDistance2Step, Direction dir);

    // 状态机推进逻辑。
    void update();

    enum class ChassisState
    {
        Idle,

        Up1_等待底盘到达台阶高度_同时往台阶位移,
        Up2_前进将前辅助轮悬于台阶上方_等待前轮收起,
        Up3_前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起,
        Up4_前进使底盘完全登上台阶,

        Down0_前进使前轮前边缘到达台阶边缘_等待底盘降为过渡高度,
        Down1_前进使中前辅助轮到达台阶边缘_等待前轮放下,
        Down2_前进使后辅助轮到达台阶边缘_等待后轮放下,
        Down3_前进使底盘完全走下台阶,

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

    // 上台阶过程是否停下取物。
    bool will_take_ = false;

    // 下台阶是否复位底盘。
    bool should_reset_ = true;

    Height height_ = Height::Step200;

    Direction direction_ = Direction::Forward;

    // 统一表示车体中心到台阶的距离。
    float startDistance2Step_ = 0;
    float endDistance2Step_   = 0;

    float x_sign_ = 1.0f;

    ChassisState chassis_state_ = ChassisState::Idle;
    LiftState    front_state_   = LiftState::Idle;
    LiftState    rear_state_    = LiftState::Idle;

    // 动作开始时的世界系位姿，后续所有相对计算都以它为基准。
    chassis::Posture start_pos_{};

    // 根据前进 / 后退方向，front_ / rear_ 会指向实际意义上的前后两侧 lift。
    Lift::LiftSide* front_ = nullptr;
    Lift::LiftSide* rear_  = nullptr;

    [[nodiscard]] chassis::Posture relativePosture(const float x) const
    {
        return Chassis::loc->RelativePosture2WorldPosture(start_pos_, { x_sign_ * x, 0, 0 });
    }

    [[nodiscard]] float currentRelativeX() const
    {
        return x_sign_ * Chassis::loc->CurrentPostureRelativeTo(start_pos_).x;
    }

    [[nodiscard]] float stepUpPosition() const;
};

} // namespace Action
