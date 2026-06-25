/**
 * @file    Step.cpp
 * @author  syhanjin
 * @date    2026-04-03
 */
#include "Step.hpp"
#include "chassis/chassis.hpp"
#include "main.h"

#include <cmath>

namespace Action
{
namespace
{
constexpr uint32_t FlagStart = 1 << 0;

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
    return std::fabs(chassis::Posture::yawError(currentRelativeToStep().yaw, dir_relative_yaw_)) <
           StepPrepareYawThreshold;
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
    case Height::R1:
        return Position::UpR1;
    case Height::Step200:
    default:
        return Position::StepUp200;
    }
}

float Step::selectedFinalPosition() const
{
    switch (final_height_)
    {
    case FinalHeight::R1:
        return Position::UpR1EndHeight;
    case FinalHeight::High:
        return Position::StepFinalHigh;
    case FinalHeight::Low:
    default:
        return Position::StepFinalLow;
    }
}

void Step::abort()
{
    // 失败后立即停止所有相关执行器，避免底盘和 lift 继续朝旧目标发散。
    if (Chassis::ctrl != nullptr)
        Chassis::ctrl->stop();
    if (front_ != nullptr)
        front_->stop();
    if (rear_ != nullptr && rear_ != front_)
        rear_->stop();
    failed_ = true;
}

bool Step::setChassisTarget(const chassis::Posture& target)
{
    if (Chassis::ctrl == nullptr)
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        abort();
        return false;
    }
    bool plan_succeeded = false;
    if (height_ == Height::R1)
    {
        plan_succeeded = Chassis::ctrl->setTargetPostureInWorld(
                target, Chassis::Config::Control::UpR1TrajectoryLimit);
    }
    else
    {
        plan_succeeded = Chassis::ctrl->setTargetPostureInWorld(target);
    }

    if (!plan_succeeded)
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::ChassisPlanFailed,
                                        Chassis::ctrl->lastSCurveFailure());
        abort();
        return false;
    }
    return true;
}

bool Step::moveLift(Lift::LiftSide* side, const float position, const Chassis::Config::Limit& limit)
{
    if (side == nullptr)
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        abort();
        return false;
    }

    if (side->to(position, limit) < 0.0f)
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::LiftPlanFailed,
                                        side->lastPlanFailureInfo());
        abort();
        return false;
    }

    return true;
}

/**
 * 上台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 上台阶方向
 * @param endHeight 动作结束后的底盘高度
 * @param height 台阶高度
 *
 */
void Step::up(const float       startDistance2Step,
              const float       endDistance2Step,
              const Direction   dir,
              const FinalHeight endHeight,
              const Height      height)
{
    if (!dependencyReady())
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        return;
    }

    const chassis::Posture start_pos = Chassis::loc->postureInWorld();
    const float            x_sign    = dir == Direction::Forward ? 1.0f : -1.0f;
    const float            step_yaw  = dir == Direction::Forward ? 0.0f : -180.0f;

    const chassis::Posture step_target = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * startDistance2Step, 0.0f, step_yaw });
    const chassis::Posture end = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * (startDistance2Step + endDistance2Step), 0.0f, 0.0f });

    up(step_target, end, dir, endHeight, height);
}

void Step::up(const chassis::Posture& stepTargetPos,
              const chassis::Posture& endPos,
              const Direction         dir,
              const FinalHeight       endHeight,
              const Height            height)
{
    if (isRunning())
    {
        Diagnostics::StepAction::report(diagnosticContext(), Diagnostics::StepAction::Reason::Busy);
        return;
    }
    if (!dependencyReady())
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        return;
    }

    failed_ = false;
    prepare(stepTargetPos, endPos, dir);
    final_height_ = endHeight;
    height_       = height;

    chassis_state_ = ChassisState::Up0_PrepareYaw;
    front_state_   = LiftState::Up1_Lifting;
    rear_state_    = LiftState::Up1_Lifting;

    if (!moveLift(front_, stepUpPosition(), OnloadLimit))
        return;
    if (!moveLift(rear_, stepUpPosition(), OnloadLimit))
        return;

    if (!setChassisTarget(stepRelativePosture(-(HalfChassisDiagonal + SafeDistance))))
        return;

    osThreadFlagsSet(task_, FlagStart);
}

void Step::upR1(const chassis::Posture& stepTargetPos, const Direction dir)
{
    if (isRunning())
    {
        Diagnostics::StepAction::report(diagnosticContext(), Diagnostics::StepAction::Reason::Busy);
        return;
    }
    if (!dependencyReady())
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        return;
    }

    const chassis::Posture endPos =
            chassis::loc::IChassisLoc::RelativePosture2WorldPosture(stepTargetPos,
                                                                    UpR1EndRelativePos);

    failed_ = false;
    prepare(stepTargetPos, endPos, dir);
    final_height_ = FinalHeight::R1;
    height_       = Height::R1;

    chassis_state_ = ChassisState::Up0_PrepareYaw;
    front_state_   = LiftState::Up1_Lifting;
    rear_state_    = LiftState::Up1_Lifting;

    if (!moveLift(front_, stepUpPosition(), OnloadLimit))
        return;
    if (!moveLift(rear_, stepUpPosition(), OnloadLimit))
        return;

    if (!setChassisTarget(stepRelativePosture(-(HalfChassisDiagonal + SafeDistance))))
        return;

    osThreadFlagsSet(task_, FlagStart);
}

/**
 * 下台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 下台阶方向
 * @param endHeight 动作结束后的底盘高度
 * @param height 台阶高度
 *
 */
void Step::down(const float       startDistance2Step,
                const float       endDistance2Step,
                const Direction   dir,
                const FinalHeight endHeight,
                const Height      height)
{
    if (isRunning())
    {
        Diagnostics::StepAction::report(diagnosticContext(), Diagnostics::StepAction::Reason::Busy);
        return;
    }
    if (!dependencyReady())
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        return;
    }

    const chassis::Posture start_pos = Chassis::loc->postureInWorld();
    const float            x_sign    = dir == Direction::Forward ? 1.0f : -1.0f;
    const float            step_yaw  = dir == Direction::Forward ? 0.0f : -180.0f;

    const chassis::Posture step_target = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * startDistance2Step, 0.0f, step_yaw });
    const chassis::Posture end = chassis::loc::IChassisLoc::RelativePosture2WorldPosture(
            start_pos, { x_sign * (startDistance2Step + endDistance2Step), 0.0f, 0.0f });

    down(step_target, end, dir, endHeight, height);
}

void Step::down(const chassis::Posture& stepTargetPos,
                const chassis::Posture& endPos,
                const Direction         dir,
                const FinalHeight       endHeight,
                const Height            height)
{
    if (isRunning())
    {
        Diagnostics::StepAction::report(diagnosticContext(), Diagnostics::StepAction::Reason::Busy);
        return;
    }
    if (!dependencyReady())
    {
        Diagnostics::StepAction::report(diagnosticContext(),
                                        Diagnostics::StepAction::Reason::DependencyNotReady);
        return;
    }

    failed_ = false;
    prepare(stepTargetPos, endPos, dir);
    final_height_ = endHeight;
    height_       = height;

    chassis_state_ = ChassisState::Down0_PrepareYaw;
    front_state_   = LiftState::Down1_WaitDeploy;
    rear_state_    = LiftState::Down1_WaitDeploy;

    if (!moveLift(front_, Position::StepTransition, OnloadLimit))
        return;
    if (!moveLift(rear_, Position::StepTransition, OnloadLimit))
        return;

    if (!setChassisTarget(
                stepRelativePosture(-(HalfWheelDiagonal + WheelRadius + 3 * SafeDistance))))
        return;

    osThreadFlagsSet(task_, FlagStart);
}

void Step::update()
{
    switch (chassis_state_)
    {
    case ChassisState::Idle:
        break;

    // 上台阶预备：先到台阶前的安全预备点，并等待 yaw 对准台阶方向。
    case ChassisState::Up0_PrepareYaw:
        if (yawPrepared())
        {
            if (!setChassisTarget(stepRelativePosture(-(HalfChassisDistanceX + SafeDistance))))
                return;
            chassis_state_ = ChassisState::Up1_ApproachEdge;
        }
        break;

    // 上台阶靠近边缘：等待两侧升到台阶高度，并确认 y 偏差足够小。
    case ChassisState::Up1_ApproachEdge:
        if (front_->isFinished() && rear_->isFinished() &&
            std::fabs(currentRelativeToStep().y) < StepPrepareYThreshold)
        {
            if (!setChassisTarget(stepRelativePosture(-(AbsWheelOuterEdgeX + 3 * SafeDistance))))
                return;

            chassis_state_ = ChassisState::Up2_WaitFrontRetract;
        }
        break;

    // 等前侧腿收起：前辅助轮越过边缘后，等待前侧腿完成收起。
    case ChassisState::Up2_WaitFrontRetract:
        if (front_state_ == LiftState::Up4_WaitDeploy)
        {
            if (!setChassisTarget(stepRelativePosture(AbsWheelInnerEdgeX - 3 * SafeDistance)))
                return;

            chassis_state_ = ChassisState::Up3_WaitRearRetract;
        }
        break;

    // 等后侧腿收起：继续推进到后侧跨越条件，等待后侧腿收起完成。
    case ChassisState::Up3_WaitRearRetract:
        if (rear_state_ == LiftState::Up4_WaitDeploy)
        {
            if (!setChassisTarget(endXWithStepYaw()))
                return;

            chassis_state_ = ChassisState::Up4_MoveToEndX;
        }
        break;

    // 保持台阶 yaw 先对齐终点 x，等后侧腿放下或恢复到位。
    case ChassisState::Up4_MoveToEndX:
        if (rear_state_ == LiftState::Up6_WaitRestoreNormal || rear_state_ == LiftState::Done)
        {
            if (!setChassisTarget(endXYWithStepYaw()))
                return;
            chassis_state_ = ChassisState::Up5_MoveToEndXY;
        }
        break;

    // 保持台阶 yaw 走到终点 x/y，离开台阶安全范围后再切最终位姿。
    case ChassisState::Up5_MoveToEndXY:
        if (currentRelativeX() > HalfWheelDiagonal + WheelRadius + 3 * SafeDistance)
        {
            if (!setChassisTarget(end_pos_))
                return;
            chassis_state_ = ChassisState::Up6_FinalizePose;
        }
        break;

    // 上台阶收尾：等待底盘最终位姿轨迹完成。
    case ChassisState::Up6_FinalizePose:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;

    // 下台阶预备：先到下台阶预备点，此时进行降底盘和前进到主动轮外边缘的并行。
    case ChassisState::Down0_PrepareYaw:
        if (yawPrepared())
        {
            if (!setChassisTarget(stepRelativePosture(-(AbsWheelOuterEdgeX + 3 * SafeDistance))))
                return;
            chassis_state_ = ChassisState::Down1_ApproachFrontAux;
        }
        break;

    // 等待两侧腿到过渡高度，之后下台阶靠近辅助轮内侧前侧边缘，准备下放前腿。
    case ChassisState::Down1_ApproachFrontAux:
        if (front_->isFinished() && rear_->isFinished())
        {
            if (!setChassisTarget(stepRelativePosture(-(AbsAuxInnerWheelX + 3 * SafeDistance))))
                return;
            chassis_state_ = ChassisState::Down2_WaitFrontDeploy;
        }
        break;

    // 等前侧腿下放：前侧腿支撑到位后，底盘继续推进到后侧外辅助轮边缘。
    case ChassisState::Down2_WaitFrontDeploy:
        if (front_state_ == LiftState::Down3_WaitRestoreNormal)
        {
            if (!setChassisTarget(stepRelativePosture(AbsAuxOuterWheelX - 3 * SafeDistance)))
                return;

            chassis_state_ = ChassisState::Down3_WaitRearDeploy;
        }
        break;

    // 等后侧腿下放：后侧腿支撑到位后，保持台阶 yaw 走到终点 x/y。
    case ChassisState::Down3_WaitRearDeploy:
        if (rear_state_ == LiftState::Down3_WaitRestoreNormal)
        {
            if (!setChassisTarget(endXYWithStepYaw()))
                return;

            chassis_state_ = ChassisState::Down4_MoveToEndXY;
        }
        break;

    // 下台阶离开边缘：离开台阶安全范围后再切最终位姿。
    case ChassisState::Down4_MoveToEndXY:
        if (currentRelativeX() > HalfChassisDiagonal + SafeDistance)
        {
            if (!setChassisTarget(end_pos_))
                return;
            chassis_state_ = ChassisState::Down5_FinalizePose;
        }
        break;

    // 下台阶收尾：等待底盘最终位姿轨迹完成。
    case ChassisState::Down5_FinalizePose:
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
        // TODO: 这里不应该这样扩大安全距离
        if (currentRelativeX() > -AbsAuxOuterWheelX + AuxWheelRadius +
                                         3 * SafeDistance) // 这里是判断前侧辅助轮有没有上台阶
        {
            if (final_height_ == FinalHeight::R1 && Chassis::loc_ekf != nullptr)
            {
                Chassis::loc_ekf->setGyroEnabled(false);
                Chassis::loc_ekf->setLidarEnabled(false);
            }
            if (!moveLift(front_, LiftMin, NoloadLimit))
                return;
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
            if (!moveLift(front_, Position::StepTransition, DeployLiftLimit))
                return;
            front_state_ = LiftState::Up5_Deploying;
        }
        break;
    case LiftState::Up5_Deploying:
        if (front_->isFinished())
        {
            front_->setGrounding(true); // 着地
            front_state_ = LiftState::Up6_WaitRestoreNormal;
        }
        break;
    case LiftState::Up6_WaitRestoreNormal:
        // 等待 rear 同步进入过渡态后统一恢复目标高度。
        break;
    case LiftState::Up7_RestoringNormal:
        if (front_->isFinished())
            front_state_ = LiftState::Done;
        break;
        // 等待离地之后并具备放下前腿条件之后开始放腿
    case LiftState::Down1_WaitDeploy:
        if (chassis_state_ == ChassisState::Down0_PrepareYaw ||
            chassis_state_ == ChassisState::Down1_ApproachFrontAux)
            break;

        if (currentRelativeX() > -AbsWheelOuterEdgeX)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > -AbsWheelInnerEdgeX + 3 * SafeDistance)
        {
            if (!moveLift(front_, stepUpPosition(), NoloadLimit))
                return;
            front_state_ = LiftState::Down2_Deploying;
        }
        break;
        // 等前侧腿下放完成判定状态，并更改底盘支撑状态
    case LiftState::Down2_Deploying:
        if (front_->isFinished())
        {
            front_->setGrounding(true);
            front_state_ = LiftState::Down3_WaitRestoreNormal;
        }
        break;
        // 等待 rear 同步进入该状态后统一恢复目标高度。
    case LiftState::Down3_WaitRestoreNormal:
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
        if ((currentRelativeX() > AbsAuxInnerWheelX + AuxWheelRadius + 3 * SafeDistance) &&
            (front_state_ == LiftState::Done || front_state_ == LiftState::Up6_WaitRestoreNormal))
        {
            if (!moveLift(rear_, LiftMin, NoloadLimit))
                return;
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
            if (!moveLift(rear_, Position::StepTransition, DeployLiftLimit))
                return;
            rear_state_ = LiftState::Up5_Deploying;
        }
        break;
    case LiftState::Up5_Deploying:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true); // 着地
            rear_state_ = LiftState::Up6_WaitRestoreNormal;
        }
        break;
    case LiftState::Up6_WaitRestoreNormal:
    {
        const float final_position = selectedFinalPosition();
        if (!moveLift(front_, final_position, DeployLiftLimit))
            return;
        if (!moveLift(rear_, final_position, DeployLiftLimit))
            return;
        front_state_ = LiftState::Up7_RestoringNormal;
        rear_state_  = LiftState::Up7_RestoringNormal;
        break;
    }
    case LiftState::Up7_RestoringNormal:
        if (rear_->isFinished())
            rear_state_ = LiftState::Done;
        break;
        // 等待离地之后并具备放下后腿条件之后开始放腿
    case LiftState::Down1_WaitDeploy:
        if (chassis_state_ == ChassisState::Down0_PrepareYaw ||
            chassis_state_ == ChassisState::Down1_ApproachFrontAux)
            break;

        if (currentRelativeX() > AbsWheelInnerEdgeX)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > AbsWheelOuterEdgeX + 3 * SafeDistance)
        {
            if (!moveLift(rear_, stepUpPosition(), NoloadLimit))
                return;
            rear_state_ = LiftState::Down2_Deploying;
        }
        break;
        // 等待后侧腿下放完成判定状态，并更改底盘支撑状态
    case LiftState::Down2_Deploying:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true);
            rear_state_ = LiftState::Down3_WaitRestoreNormal;
        }
        break;
    case LiftState::Down3_WaitRestoreNormal:
        if (chassis_state_ == ChassisState::Down5_FinalizePose)
        {
            const float final_position = selectedFinalPosition();
            if (!moveLift(front_, final_position, OnloadLimit))
                return;
            if (!moveLift(rear_, final_position, OnloadLimit))
                return;
            front_state_ = LiftState::Down4_RestoringNormal;
            rear_state_  = LiftState::Down4_RestoringNormal;
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

        while (isRunning())
        {
            update();
            osDelay(1);
        }
    }
}
} // namespace Action
