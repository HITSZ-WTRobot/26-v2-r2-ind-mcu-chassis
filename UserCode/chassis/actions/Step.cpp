/**
 * @file    Step.cpp
 * @author  syhanjin
 * @date    2026-04-03
 */
#include "Step.hpp"
#include "chassis/chassis.hpp"

#include <cmath>

namespace Action
{
namespace
{
constexpr uint32_t FlagStart  = 1 << 0;
constexpr uint32_t FlagResume = 1 << 1;

using namespace Chassis::Config::ChassisInfo;
using namespace Chassis::Config::Lift;

constexpr Chassis::Config::Limit DeployLiftLimit{ MaxSpeed, MaxOnloadAccel, MaxOnloadAccel * 50 };

using chassis::controller::Master;
} // namespace

Step::Step()
{
    constexpr osThreadAttr_t attr{
        .stack_size = 1024 * 4,
        .priority   = osPriorityNormal,
    };
    task_ = osThreadNew(TaskEntry, this, &attr);
}

Step& Step::inst()
{
    static Step inst;
    return inst;
}

void Step::prepare(const chassis::Posture& stepTargetPos,
                   const chassis::Posture& endPos,
                   const Direction         dir)
{
    direction_        = dir;
    dir_relative_yaw_ = dir == Direction::Forward ? 0.0f : 180.0f;
    step_target_pos_  = stepTargetPos;
    end_pos_          = endPos;

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
}

bool Step::yawPrepared() const
{
    return std::fabs(currentRelativeToStep().yaw - dir_relative_yaw_) < StepPrepareYawThreshold;
}

chassis::Posture Step::endXWithStepYaw() const
{
    const auto end_rel = chassis::loc::IChassisLoc::WorldPosture2RelativePosture(step_target_pos_,
                                                                                 end_pos_);
    return stepRelativePosture({ end_rel.x, 0.0f, dir_relative_yaw_ });
}

chassis::Posture Step::endXYWithStepYaw() const
{
    const auto end_rel = chassis::loc::IChassisLoc::WorldPosture2RelativePosture(step_target_pos_,
                                                                                 end_pos_);
    return stepRelativePosture({ end_rel.x, end_rel.y, dir_relative_yaw_ });
}

float Step::stepUpPosition() const
{
    switch (height_)
    {
    case Height::Step400:
        return Position::StepUp400;
    case Height::Step200:
    default:
        return Position::StepUp200;
    }
}

/**
 * 上台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 上台阶方向
 * @param willTake 中间是否会停下来取卷轴
 * @param height 台阶高度
 *
 */
void Step::up(const float     startDistance2Step,
              const float     endDistance2Step,
              const Direction dir,
              const bool      willTake,
              const Height    height)
{
    const chassis::Posture start_pos = Chassis::loc->postureInWorld();
    const float            x_sign    = dir == Direction::Forward ? 1.0f : -1.0f;
    const float            step_yaw  = dir == Direction::Forward ? 0.0f : -180.0f;

    const chassis::Posture step_target = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * startDistance2Step, 0.0f, step_yaw });
    const chassis::Posture end = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * (startDistance2Step + endDistance2Step), 0.0f, 0.0f });

    up(step_target, end, dir, willTake, height);
}

void Step::up(const chassis::Posture& stepTargetPos,
              const chassis::Posture& endPos,
              const Direction         dir,
              const bool              willTake,
              const Height            height)
{
    if (isRunning())
        return;

    prepare(stepTargetPos, endPos, dir);
    will_take_    = willTake;
    should_reset_ = true;
    height_       = height;

    chassis_state_ = ChassisState::Up0;
    front_state_   = LiftState::Up1_Lifting;
    rear_state_    = LiftState::Up1_Lifting;

    front_->to(stepUpPosition(), OnloadLimit);
    rear_->to(stepUpPosition(), OnloadLimit);

    Chassis::ctrl->setTargetPostureInWorld(
            stepRelativePosture(-(HalfChassisDiagonal + SafeDistance)));

    osThreadFlagsSet(task_, FlagStart);
}
/**
 * 下台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 下台阶方向
 * @param shouldReset 最后是否复位底盘高度
 * @param height 台阶高度
 *
 */
void Step::down(const float     startDistance2Step,
                const float     endDistance2Step,
                const Direction dir,
                const bool      shouldReset,
                const Height    height)
{
    const chassis::Posture start_pos = Chassis::loc->postureInWorld();
    const float            x_sign    = dir == Direction::Forward ? 1.0f : -1.0f;
    const float            step_yaw  = dir == Direction::Forward ? 0.0f : -180.0f;

    const chassis::Posture step_target = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * startDistance2Step, 0.0f, step_yaw });
    const chassis::Posture end = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * (startDistance2Step + endDistance2Step), 0.0f, 0.0f });

    down(step_target, end, dir, shouldReset, height);
}

void Step::down(const chassis::Posture& stepTargetPos,
                const chassis::Posture& endPos,
                const Direction         dir,
                const bool              shouldReset,
                const Height            height)
{
    if (isRunning())
        return;

    prepare(stepTargetPos, endPos, dir);
    will_take_    = false;
    should_reset_ = shouldReset;
    height_       = height;

    chassis_state_ = ChassisState::Down0;
    front_state_   = LiftState::Down1_WaitDeploy;
    rear_state_    = LiftState::Down1_WaitDeploy;

    front_->to(Position::StepTransition, OnloadLimit);
    rear_->to(Position::StepTransition, OnloadLimit);

    Chassis::ctrl->setTargetPostureInWorld(
            stepRelativePosture(-(HalfWheelDiagonal + WheelRadius + 3 * SafeDistance)));

    osThreadFlagsSet(task_, FlagStart);
}

/**
 * 继续上台阶
 */
void Step::resume_up()
{
    will_take_ = false;
    osThreadFlagsSet(task_, FlagResume);
}

void Step::update()
{
    switch (chassis_state_)
    {
    case ChassisState::Idle:
        break;
    case ChassisState::Up0:
        if (yawPrepared())
        {
            Chassis::ctrl->setTargetPostureInWorld(stepRelativePosture(
                                                           -(HalfChassisDistanceX + SafeDistance)),
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            chassis_state_ = ChassisState::Up1;
        }
        break;
    case ChassisState::Up1:
        if (front_->isFinished() && rear_->isFinished() &&
            std::fabs(currentRelativeToStep().y) < StepPrepareYThreshold)
        {
            Chassis::ctrl->setTargetPostureInWorld(stepRelativePosture(-(AbsWheelOuterEdgeX +
                                                                         3 * SafeDistance)),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up2;

            if (will_take_)
                osThreadFlagsWait(FlagResume, osFlagsWaitAll, osWaitForever);
        }
        break;
    case ChassisState::Up2:
        if (front_state_ == LiftState::Up4_WaitDeploy)
        {
            Chassis::ctrl->setTargetPostureInWorld(stepRelativePosture(AbsWheelInnerEdgeX -
                                                                       3 * SafeDistance),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up3;
        }
        break;
    case ChassisState::Up3:
        if (rear_state_ == LiftState::Up4_WaitDeploy)
        {
            Chassis::ctrl->setTargetPostureInWorld(endXWithStepYaw(),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up4;
        }
        break;
    case ChassisState::Up4:
        if (rear_state_ == LiftState::Up6_WaitRestoreNormal || rear_state_ == LiftState::Done)
        {
            Chassis::ctrl->setTargetPostureInWorld(endXYWithStepYaw(),
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            chassis_state_ = ChassisState::Up5;
        }
        break;
    case ChassisState::Up5:
        if (currentRelativeX() > HalfWheelDiagonal + WheelRadius + 3 * SafeDistance)
        {
            Chassis::ctrl->setTargetPostureInWorld(end_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            chassis_state_ = ChassisState::Up6;
        }
        break;
    case ChassisState::Up6:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    case ChassisState::Down0:
        if (yawPrepared())
        {
            Chassis::ctrl->setTargetPostureInWorld(stepRelativePosture(
                                                           -(AbsAuxInnerWheelX + 3 * SafeDistance)),
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            chassis_state_ = ChassisState::Down1;
        }
        break;
    case ChassisState::Down1:
        if (front_->isFinished() && rear_->isFinished())
        {
            chassis_state_ = ChassisState::Down2;
        }
        break;
    case ChassisState::Down2:
        if (front_state_ == LiftState::Down3_WaitRestoreNormal)
        {
            Chassis::ctrl->setTargetPostureInWorld(stepRelativePosture(AbsAuxOuterWheelX -
                                                                       3 * SafeDistance),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Down3;
        }
        break;
    case ChassisState::Down3:
        if (rear_state_ == LiftState::Down3_WaitRestoreNormal)
        {
            Chassis::ctrl->setTargetPostureInWorld(endXYWithStepYaw(),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Down4;
        }
        break;
    case ChassisState::Down4:
        if (currentRelativeX() > HalfChassisDiagonal + SafeDistance)
        {
            Chassis::ctrl->setTargetPostureInWorld(end_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            chassis_state_ = ChassisState::Down5;
        }
        break;
    case ChassisState::Down5:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;

    case ChassisState::Done:
        break;
    }

    switch (front_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::Up1_Lifting:
        if (front_->isFinished())
            front_state_ = LiftState::Up2_WaitRetract;
        break;
    case LiftState::Up2_WaitRetract:
        // 等待取物
        if (will_take_)
            break;
        // TODO: 这里不应该这样扩大安全距离
        if (currentRelativeX() > -AbsAuxOuterWheelX + AuxWheelRadius + 3 * SafeDistance)
        {
            front_->to(LiftMin, NoloadLimit);
            front_->setGrounding(false); // 离地
            front_state_ = LiftState::Up3_Retracting;
        }
        break;
    case LiftState::Up3_Retracting:
        if (front_->isFinished())
            front_state_ = LiftState::Up4_WaitDeploy;
        break;
    case LiftState::Up4_WaitDeploy:
        // 前轮已经完全登上
        if (currentRelativeX() > -AbsWheelInnerEdgeX + 3 * SafeDistance)
        {
            front_->to(Position::StepTransition, DeployLiftLimit);
            front_state_ = LiftState::Up5_Deploying;
        }
        break;
    case LiftState::Up5_Deploying:
        if (front_->isFinished())
        {
            front_->setGrounding(true); // 着地
            if constexpr (Position::StepTransition == Position::Normal)
                front_state_ = LiftState::Done;
            else
                front_state_ = LiftState::Up6_WaitRestoreNormal;
        }
        break;
    case LiftState::Up6_WaitRestoreNormal:
        // 在 rear 触发
        break;
    case LiftState::Up7_RestoringNormal:
        if (front_->isFinished())
            front_state_ = LiftState::Done;
        break;
    case LiftState::Down1_WaitDeploy:
        if (chassis_state_ == ChassisState::Down0 || chassis_state_ == ChassisState::Down1)
            break;

        if (currentRelativeX() > -AbsWheelOuterEdgeX)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > -AbsWheelInnerEdgeX + 3 * SafeDistance)
        {
            front_->to(stepUpPosition(), NoloadLimit);
            front_state_ = LiftState::Down2_Deploying;
        }
        break;
    case LiftState::Down2_Deploying:
        if (front_->isFinished())
        {
            front_->setGrounding(true);
            front_state_ = LiftState::Down3_WaitRestoreNormal;
        }
        break;
    case LiftState::Down3_WaitRestoreNormal:
        if (should_reset_)
        {
            if (chassis_state_ == ChassisState::Down5)
            {
                front_->to(Position::Normal, OnloadLimit);
                front_state_ = LiftState::Down4_RestoringNormal;
            }
        }
        else
        {
            // 如果不复位底盘，则在底盘完成后立即完成
            if (chassis_state_ == ChassisState::Done)
            {
                front_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::Down4_RestoringNormal:
        if (front_->isFinished())
        {
            front_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }

    switch (rear_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::Up1_Lifting:
        if (rear_->isFinished())
            rear_state_ = LiftState::Up2_WaitRetract;
        break;
    case LiftState::Up2_WaitRetract:
        // 等待取物，此处可以不加该判断，这里为了对称
        if (will_take_)
            break;
        if ((currentRelativeX() > AbsAuxInnerWheelX + AuxWheelRadius + 3 * SafeDistance) &&
            (front_state_ == LiftState::Done || front_state_ == LiftState::Up6_WaitRestoreNormal))
        {
            rear_->to(LiftMin, NoloadLimit);
            rear_->setGrounding(false);
            rear_state_ = LiftState::Up3_Retracting;
        }
        break;
    case LiftState::Up3_Retracting:
        if (rear_->isFinished())
            rear_state_ = LiftState::Up4_WaitDeploy;
        break;
    case LiftState::Up4_WaitDeploy:
        // 后轮已经完全登上
        if (currentRelativeX() > AbsWheelOuterEdgeX + 3 * SafeDistance)
        {
            if (front_state_ == LiftState::Up6_WaitRestoreNormal)
            {
                front_->to(Position::Normal, DeployLiftLimit);
                front_state_ = LiftState::Up7_RestoringNormal;
            }
            rear_->to(Position::Normal, DeployLiftLimit);
            rear_state_ = LiftState::Up5_Deploying;
        }
        break;
    case LiftState::Up5_Deploying:
        if (rear_->isFinished())
        {
            rear_state_ = LiftState::Done;
            rear_->setGrounding(true); // 着地
        }
        break;
    case LiftState::Up6_WaitRestoreNormal:
    case LiftState::Up7_RestoringNormal:
        break;
    case LiftState::Down1_WaitDeploy:
        if (chassis_state_ == ChassisState::Down0 || chassis_state_ == ChassisState::Down1)
            break;

        if (currentRelativeX() > AbsWheelInnerEdgeX)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > AbsWheelOuterEdgeX + 3 * SafeDistance)
        {
            rear_->to(stepUpPosition(), NoloadLimit);
            rear_state_ = LiftState::Down2_Deploying;
        }
        break;
    case LiftState::Down2_Deploying:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true);
            rear_state_ = LiftState::Down3_WaitRestoreNormal;
        }
        break;
    case LiftState::Down3_WaitRestoreNormal:
        if (should_reset_)
        {
            if (chassis_state_ == ChassisState::Down5)
            {
                rear_->to(Position::Normal, OnloadLimit);
                rear_state_ = LiftState::Down4_RestoringNormal;
            }
        }
        else
        {
            // 如果不复位底盘，则在底盘完成后立即完成
            if (chassis_state_ == ChassisState::Done)
            {
                rear_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::Down4_RestoringNormal:
        if (rear_->isFinished())
        {
            rear_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }
}

[[noreturn]] void Step::loop()
{
    for (;;)
    {
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);
        osThreadFlagsClear(FlagResume);

        while (isRunning())
        {
            update();
            osDelay(1);
        }
    }
}
} // namespace Action
