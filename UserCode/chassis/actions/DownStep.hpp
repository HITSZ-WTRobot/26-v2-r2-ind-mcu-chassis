/**
 * @file    DownStep.hpp
 * @author  syhanjin
 * @date    2026-04-05
 * @brief   下台阶动作
 */
#pragma once
#include "IChassisDef.hpp"
#include "cmsis_os2.h"
#include "traits.hpp"
#include "chassis/LiftSide.hpp"
#include "chassis/chassis.hpp"

namespace Action
{

class DownStep : traits::NoCopy, traits::NoDelete
{
public:
    DownStep();

    static DownStep& inst();

    enum class Direction
    {
        Forward,
        Backward
    };

    void start(float     startDistance2Step,
               float     endDistance2Step,
               Direction dir = Direction::Forward);

    static void TaskEntry(void* self) { static_cast<DownStep*>(self)->loop(); }

    [[noreturn]] void loop();

    [[nodiscard]] bool isIdle() const
    {
        return chassis_state_ == ChassisState::Idle && lift_front_state_ == LiftState::Idle &&
               lift_rear_state_ == LiftState::Idle;
    }

    [[nodiscard]] bool isFinished() const
    {
        return chassis_state_ == ChassisState::Done && lift_front_state_ == LiftState::Done &&
               lift_rear_state_ == LiftState::Done;
    }

    [[nodiscard]] bool isRunning() const { return !isFinished() && !isIdle(); }

private:
    osThreadId_t task_;

    void update();

    enum class ChassisState
    {
        Idle,

        A前进使中前辅助轮到达台阶边缘_等待前轮放下,
        B前进使后辅助轮到达台阶边缘_等待后轮放下,
        C前进使底盘完全走下台阶,

        Done,
    };

    enum class LiftState
    {
        Idle,

        A等待放下,
        B放下ing,
        C等待回收到正常位置,
        D回收ing,

        Done,
    };

    Direction direction_ = Direction::Forward;

    float startDistance2Step_ = 0.0f;
    float endDistance2Step_   = 0.0f;

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
