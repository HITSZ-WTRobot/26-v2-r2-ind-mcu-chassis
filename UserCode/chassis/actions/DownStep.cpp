/**
 * @file    DownStep.cpp
 * @author  syhanjin
 * @date    2026-04-05
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "DownStep.hpp"

namespace Action
{
namespace
{
constexpr uint32_t FlagStart = 1 << 0;

using namespace Chassis::Config::ChassisInfo;
constexpr float 前边缘到中前辅助轮前边缘 = ChassisFrontEdge - AuxWheelMidFrontX -
                                           AuxiliaryWheelRadius;
constexpr float 前边缘到后辅助轮前边缘 = ChassisFrontEdge - AuxWheelRearX - AuxiliaryWheelRadius;

constexpr float 前边缘到前轮后边缘 = ChassisFrontEdge - WheelFrontEdgeRear;
constexpr float 前边缘到后轮后边缘 = ChassisFrontEdge - WheelRearEdgeRear;
constexpr float 前边缘到前轮前边缘 = ChassisFrontEdge - WheelFrontEdgeFront;
constexpr float 前边缘到后轮前边缘 = ChassisFrontEdge - WheelRearEdgeFront;

constexpr Chassis::Config::Limit 放腿速度{ Chassis::Config::Lift::MaxSpeed,
                                           Chassis::Config::Lift::MaxNoloadAccel,
                                           Chassis::Config::Lift::MaxNoloadAccel * 50 };

using chassis::controller::Master;
} // namespace

DownStep& DownStep::inst()
{
    static DownStep inst;
    return inst;
}

/**
 *
 * @param startDistance2Step 开始时车体前边缘距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体边缘距离台阶距离 unit: m
 * @param dir 下台阶方向
 *
 */
void DownStep::start(const float     startDistance2Step,
                     const float     endDistance2Step,
                     const Direction dir)
{
    if (isRunning())
        return;

    direction_          = dir;
    x_sign_             = dir == Direction::Forward ? 1 : -1;
    startDistance2Step_ = startDistance2Step;
    endDistance2Step_   = endDistance2Step;
    start_pos_          = Chassis::loc->postureInWorld();

    if (dir == Direction::Forward)
    {
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }
    else
    {
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }

    chassis_state_    = ChassisState::A前进使中前辅助轮到达台阶边缘_等待前轮放下;
    lift_front_state_ = LiftState::A等待放下;
    lift_rear_state_  = LiftState::A等待放下;

    Chassis::ctrl->setTargetPostureInWorld(
            relativePosture(startDistance2Step_ + 前边缘到中前辅助轮前边缘 - 3 * SafeDistance));

    osThreadFlagsSet(task_, FlagStart);
}

void DownStep::update()
{
    switch (chassis_state_)
    {
    case ChassisState::Idle:
        break;
    case ChassisState::A前进使中前辅助轮到达台阶边缘_等待前轮放下:
        if (lift_front_state_ == LiftState::C等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(
                    startDistance2Step_ + 前边缘到后辅助轮前边缘 - 3 * SafeDistance));

            chassis_state_ = ChassisState::B前进使后辅助轮到达台阶边缘_等待后轮放下;
        }
        break;
    case ChassisState::B前进使后辅助轮到达台阶边缘_等待后轮放下:
        if (lift_rear_state_ == LiftState::C等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + endDistance2Step_ + ChassisDistanceX));

            chassis_state_ = ChassisState::C前进使底盘完全走下台阶;
        }
        break;
    case ChassisState::C前进使底盘完全走下台阶:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    case ChassisState::Done:
        break;
    }
    switch (lift_front_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::A等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮前边缘)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮后边缘 + SafeDistance)
        {
            front_->to(Chassis::Config::Lift::Position::UpStep, 放腿速度);
            lift_front_state_ = LiftState::B放下ing;
        }
        break;
    case LiftState::B放下ing:
        if (front_->isFinished())
        {
            front_->setGrounding(true);
            lift_front_state_ = LiftState::C等待回收到正常位置;
        }
        break;
    case LiftState::C等待回收到正常位置:
        if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
        {
            front_->to(Chassis::Config::Lift::Position::Normal);
            lift_front_state_ = LiftState::D回收ing;
        }
        break;
    case LiftState::D回收ing:
        if (front_->isFinished())
        {
            lift_front_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }
    switch (lift_rear_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::A等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮前边缘)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮后边缘 + SafeDistance)
        {
            rear_->to(Chassis::Config::Lift::Position::UpStep, 放腿速度);
            lift_rear_state_ = LiftState::B放下ing;
        }
        break;
    case LiftState::B放下ing:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true);
            lift_rear_state_ = LiftState::C等待回收到正常位置;
        }
        break;
    case LiftState::C等待回收到正常位置:
        if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
        {
            rear_->to(Chassis::Config::Lift::Position::Normal);
            lift_rear_state_ = LiftState::D回收ing;
        }
        break;
    case LiftState::D回收ing:
        if (rear_->isFinished())
        {
            lift_rear_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }
}

void DownStep::loop()
{
    for (;;)
    {
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);

        while (!isFinished())
        {
            update();
            osDelay(1);
        }
    }
}
} // namespace Action