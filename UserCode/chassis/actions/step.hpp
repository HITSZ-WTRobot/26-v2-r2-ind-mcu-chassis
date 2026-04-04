/**
 * @file    step.hpp
 * @author  syhanjin
 * @date    2026-04-03
 * @brief 与台阶相关的动作集合
 */
#pragma once
#include "chassis/LiftSide.hpp"
#include "chassis/chassis.hpp"

namespace Action
{

class UpStep
{
public:
    UpStep();

    static UpStep& inst();

    enum class Direction
    {
        Front,
        Rear
    };

    void start(float     startDistance2Step,
               float     endDistance2Step,
               Direction dir      = Direction::Front,
               bool      willTake = false);

    void resume();

    static void TaskEntry(void* self) { static_cast<UpStep*>(self)->loop(); }

    [[noreturn]] void loop();

    [[nodiscard]] bool isIdle() const
    {
        return chassis_state_ == ChassisState::Idle && lift_front_state_ == LiftState::Idle &&
               lift_rear_state_ == LiftState::Idle;
    }

    [[nodiscard]] bool isWaitingTake() const
    {
        return will_take_ &&
               chassis_state_ == ChassisState::B前进将前辅助轮悬于台阶上方_等待前轮收起;
    }

    [[nodiscard]] bool isFinished() const
    {
        return chassis_state_ == ChassisState::Done && lift_front_state_ == LiftState::Done &&
               lift_rear_state_ == LiftState::Done;
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

    void update();

    enum class ChassisState
    {
        Idle,

        A等待底盘到达台阶高度_同时往台阶位移,
        B前进将前辅助轮悬于台阶上方_等待前轮收起,
        C前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起,
        D前进使底盘完全登上台阶,

        Done
    };

    enum class LiftState
    {
        Idle,

        A抬升ing,
        B等待收起,
        C收起ing,
        D等待放下,
        E放下ing,

        Done
    };

    bool will_take_ = false;

    Direction direction_ = Direction::Front;

    float startDistance2Step_ = 0;
    float endDistance2Step_   = 0;

    float x_sign_ = 1.0f;

    ChassisState chassis_state_    = ChassisState::Idle;
    LiftState    lift_front_state_ = LiftState::Idle;
    LiftState    lift_rear_state_  = LiftState::Idle;

    chassis::Posture start_pos_{};

    Lift::LiftSide* front_ = nullptr;
    Lift::LiftSide* rear_  = nullptr;

    [[nodiscard]] chassis::Posture relativePosture(const float x) const
    {
        return { start_pos_.x + x_sign_ * x, start_pos_.y, start_pos_.yaw };
    }

    [[nodiscard]] float currentRelativeX() const
    {
        return x_sign_ * (Chassis::loc->postureInWorld().x - start_pos_.x);
    }
};

} // namespace Action