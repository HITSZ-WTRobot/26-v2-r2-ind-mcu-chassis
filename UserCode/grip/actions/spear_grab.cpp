#include "spear_grab.hpp"

#include "grip/Config.hpp"
#include "grip/grip.hpp"
#include "project_parts.hpp"

#include <cassert>
#include <cmath>

namespace Grip::Action
{
namespace
{
constexpr uint32_t FlagStart = 1u << 0;

using chassis::controller::Master;
} // namespace

SpearGrab::SpearGrab()
{
    constexpr osThreadAttr_t attr{
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    task_ = osThreadNew(TaskEntry, this, &attr);
}

SpearGrab& SpearGrab::inst()
{
    static SpearGrab instance;
    return instance;
}

bool SpearGrab::canStart() const
{
    // SpearGrab 必须依赖完整的“平面底盘 + 升降”动作链，缺一都不允许启动。
    if constexpr (!ProjectParts::EnableWheelChassis || !ProjectParts::EnableLift)
        return false;

    // 必须保证动作等待完成或空闲，同时机械臂与底盘控制器已就绪
    const bool basic_ready = (state_ == State::Idle || state_ == State::Done) &&
                             ::Grip::grip != nullptr && ::Grip::grip->enabled() &&
                             Chassis::ctrl != nullptr && Chassis::loc != nullptr;

    if (!basic_ready)
        return false;

    return Chassis::motion != nullptr;
}

void SpearGrab::grab(const chassis::Posture& target_pos,
                     const chassis::Posture& end_pos,
                     float                   safe_distance)
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    safe_distance_           = safe_distance;
    target_pos_              = target_pos;
    end_pos_                 = end_pos;
    end_pos_rel_to_target_   = Chassis::ChassisLoc::WorldPosture2RelativePosture(target_pos_,
                                                                               end_pos_);
    prepare_pos_             = postureRelativeToTargetInWorld({ safe_distance_, 0.0f, 0.0f });
    leave_target_x_only_pos_ = postureRelativeToTargetInWorld(
            { end_pos_rel_to_target_.x, 0.0f, 0.0f });

    assert(safe_distance_ > 0.0f);
    assert(end_pos_rel_to_target_.x > safe_distance_);

    Chassis::ctrl->setTargetPostureInWorld(prepare_pos_);

    if constexpr (ProjectParts::EnableLift)
    {
        Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftExecute,
                                   Chassis::Config::Lift::OnloadLimit);
    }

    ::Grip::grip->toReadyPose();

    state_ = State::MovingToPrepare;
    osThreadFlagsSet(task_, FlagStart);
}

bool SpearGrab::isIdle() const
{
    return state_ == State::Idle;
}

bool SpearGrab::isFinished() const
{
    return state_ == State::Done;
}

bool SpearGrab::isRunning() const
{
    return state_ != State::Idle && state_ != State::Done;
}

void SpearGrab::waitForFinish() const
{
    while (!isFinished())
        osDelay(10);
}

chassis::Posture SpearGrab::postureRelativeToTargetInWorld(const chassis::Posture& rel_pos) const
{
    return Chassis::ChassisLoc::RelativePosture2WorldPosture(target_pos_, rel_pos);
}

chassis::Posture SpearGrab::currentRelativeToTarget() const
{
    return Chassis::loc->CurrentPostureRelativeTo(target_pos_);
}

void SpearGrab::update()
{
    const auto is_lift_finished = []() -> bool
    {
        if constexpr (ProjectParts::EnableLift)
            return Chassis::motion->isLiftAllFinished();

        return true;
    };

    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::MovingToPrepare:
    {
        const auto rel_pos = currentRelativeToTarget();

        // 底盘先对准 prepare 轨迹；侧向/偏航收敛且 lift 到位后，就直接切入 target。
        if (std::fabs(rel_pos.y) < ::Grip::Config::SpearGrab::PrepareYThreshold &&
            std::fabs(rel_pos.yaw) < ::Grip::Config::SpearGrab::PrepareYawThreshold &&
            is_lift_finished() && ::Grip::grip->isFinished())
        {
            // TODO: 不同运行阶段应使用不同的底盘速度限制，而不是沿用同一套默认轨迹参数。
            Chassis::ctrl->setTargetPostureInWorld(target_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            state_ = State::MovingToTarget;
        }
        break;
    }
    case State::MovingToTarget:
        // 到达矛头位置后，执行从 ready 到 grip-out 的夹取动作。
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            ::Grip::grip->toGripOutPose();
            state_ = State::Grabbing;
        }
        break;
    case State::Grabbing:
        if (::Grip::grip->isFinished())
        {
            if constexpr (ProjectParts::EnableLift && (::Grip::Config::SpearGrab::LiftDocking >
                                                       ::Grip::Config::SpearGrab::LiftExecute))
            {
                Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                           Chassis::Config::Lift::OnloadLimit);
            }

            Chassis::ctrl->setTargetPostureInWorld(leave_target_x_only_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            // TODO: 确保对接时机械臂不会更低。
            ::Grip::grip->toDockingPose();
            state_ = State::LeavingTargetToSafeX;
        }
        break;
    case State::LeavingTargetToSafeX:
    {
        const auto rel_pos = currentRelativeToTarget();

        if (rel_pos.x > safe_distance_)
        {
            if constexpr (ProjectParts::EnableLift && (::Grip::Config::SpearGrab::LiftDocking <=
                                                       ::Grip::Config::SpearGrab::LiftExecute))
            {
                Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                           Chassis::Config::Lift::OnloadLimit);
            }
            Chassis::ctrl->setTargetPostureInWorld(end_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            state_ = State::MovingToEnd;
        }
        break;
    }
    case State::MovingToEnd:
        if (Chassis::ctrl->isTrajectoryFinished() && ::Grip::grip->isFinished() &&
            is_lift_finished())
        {
            state_ = State::Done;
        }
        break;
    }
}

[[noreturn]] void SpearGrab::loop()
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
} // namespace Grip::Action
