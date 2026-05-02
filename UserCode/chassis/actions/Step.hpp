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

    void up(float     startDistance2Step,
            float     endDistance2Step,
            Direction dir      = Direction::Forward,
            bool      willTake = false);

    void down(float     startDistance2Step,
              float     endDistance2Step,
              Direction dir         = Direction::Forward,
              bool      shouldReset = true);

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
        return will_take_ &&
               chassis_state_ == ChassisState::Up2_前进将前辅助轮悬于台阶上方_等待前轮收起;
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

    void prepare(float startDistance2Step, float endDistance2Step, Direction dir);

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

    // 上台阶过程是否停下取物
    bool will_take_ = false;

    // 下台阶是否复位底盘
    bool should_reset_ = true;

    Direction direction_ = Direction::Forward;

    // 统一表示车体中心到台阶的距离。
    float startDistance2Step_ = 0;
    float endDistance2Step_   = 0;

    float x_sign_ = 1.0f;

    ChassisState chassis_state_ = ChassisState::Idle;
    LiftState    front_state_   = LiftState::Idle;
    LiftState    rear_state_    = LiftState::Idle;

    chassis::Posture start_pos_{};

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
};

} // namespace Action
