/**
 * @file    ActionState.cpp
 * @author  syhanjin
 * @date    2026-04-23
 */
#include "ActionState.hpp"

#include "chassis/actions/Step.hpp"
#include "chassis/chassis.hpp"
#include "grip/actions/roller_store.hpp"
#include "grip/actions/spear_grab.hpp"
#include "grip/grip.hpp"
#include "project_parts.hpp"

namespace Protocol::ActionState
{
namespace
{
StepStatus currentStepStatus()
{
    if constexpr (!ProjectParts::EnableStepAction)
        return StepStatus::Idle;

    auto& step = Action::Step::inst();

    if (step.isWaitingTake())
        return StepStatus::WaitingTake;

    if (step.isRunning())
        return StepStatus::Running;

    if (step.isFinished())
        return StepStatus::Done;

    return StepStatus::Idle;
}

ChassisMode currentChassisMode()
{
    // 当前工程仍只实例化 Master 控制器，因此这里暂时只会返回 Stop / Velocity / Position。
    // `ChassisMode::Slave = 3u` 已预留给后续接入从机控制器时使用。
    if constexpr (!ProjectParts::EnableWheelChassis)
        return ChassisMode::Stop;

    if (Chassis::ctrl == nullptr)
        return ChassisMode::Stop;

    switch (Chassis::ctrl->controlMode())
    {
    case Chassis::ChassisController::CtrlMode::Velocity:
        return ChassisMode::Velocity;
    case Chassis::ChassisController::CtrlMode::Posture:
        return ChassisMode::Position;
    case Chassis::ChassisController::CtrlMode::Stopped:
    default:
        return ChassisMode::Stop;
    }
}

bool isChassisCurveFinished()
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return true;

    if (Chassis::ctrl == nullptr)
        return true;

    return Chassis::ctrl->controlMode() != Chassis::ChassisController::CtrlMode::Posture ||
           Chassis::ctrl->isTrajectoryFinished();
}

LiftStatus currentLiftStatus()
{
    if constexpr (!ProjectParts::EnableLift)
        return LiftStatus::NotEnabled;

    if (Chassis::motion == nullptr || !Chassis::motion->isReady())
        return LiftStatus::Calibrating;

    return Chassis::motion->isLiftAllFinished() ? LiftStatus::Ready : LiftStatus::Running;
}

GripStatus currentGripStatus()
{
    if constexpr (!ProjectParts::EnableGrip)
        return GripStatus::Idle;

    if (::Grip::grip == nullptr || !::Grip::grip->isCalibrated())
        return GripStatus::Calibrating;

    const auto& spear = Grip::Action::SpearGrab::inst();

    if constexpr (ProjectParts::EnableKfsAction)
    {
        const auto& kfs = Grip::Action::KfsStore::inst();

        if (kfs.workflowPhase() == Grip::Action::KfsStore::WorkflowPhase::Release)
        {
            return kfs.isRunning() ? GripStatus::KfsRelease
                                   : (kfs.isFinished() ? GripStatus::Done : GripStatus::Idle);
        }

        if (kfs.workflowPhase() == Grip::Action::KfsStore::WorkflowPhase::Store)
        {
            if (kfs.isRunning())
                return GripStatus::KfsStore;

            if (kfs.isFinished())
                return GripStatus::Done;
        }
    }

    if (spear.isRunning())
        return GripStatus::TakingSpear;

    if (spear.isFinished())
        return GripStatus::Done;

    return GripStatus::Idle;
}

bool currentGripSuctionHasObject()
{
    if constexpr (!ProjectParts::EnableGripSuctionPressureSensor)
        return false;

    return Grip::Action::KfsStore::inst().hasDetectedObject();
}
} // namespace

void updateTable()
{
    table = pack(currentStepStatus(),
                 currentChassisMode(),
                 isChassisCurveFinished(),
                 currentLiftStatus(),
                 currentGripStatus(),
                 currentGripSuctionHasObject());
}
} // namespace Protocol::ActionState
