#include "spear_grab.hpp"

#include "grip/Config.hpp"
#include "grip/grip.hpp"
#include "main.h"
#include "project_parts.hpp"

#include <cassert>
#include <cmath>

namespace Grip::Action
{
namespace
{
// 外部仅需置位一次启动标志，后台线程随后独占推进整个状态机。
constexpr uint32_t FlagStart = 1u << 0;

using chassis::controller::Master;
} // namespace

SpearGrab::SpearGrab()
{
    // 独立线程使 SpearGrab 可以与主控线程解耦，避免在协议或主循环里手写阶段推进。
    constexpr osThreadAttr_t attr{
        .stack_size = 1024 * 4,
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

void SpearGrab::abort(const Diagnostics::SpearGrabAction::Reason reason)
{
    // 失败后只保留诊断信息，不继续推进任何轨迹。
    Diagnostics::SpearGrabAction::report(diagnosticContext(), reason);
    // 机械臂轨迹失败后不能继续跑底盘轨迹，否则会出现机构姿态与车体位置脱节。
    if (Chassis::ctrl != nullptr)
        Chassis::ctrl->stop();
    if (::Grip::grip != nullptr)
        ::Grip::grip->stop();
    state_  = State::Done;
    failed_ = true;
}

void SpearGrab::reportGripPlanFailure()
{
    if (::Grip::grip == nullptr)
    {
        abort(Diagnostics::SpearGrabAction::Reason::DependencyNotReady);
        return;
    }

    const uint8_t mask = ::Grip::grip->lastPlanFailureMask();
    if (mask == 0U)
    {
        // Grip 没有给出可拆解的轴失败信息时，退化成普通姿态规划失败。
        abort(Diagnostics::SpearGrabAction::Reason::GripPlanFailed);
        return;
    }

    Diagnostics::SpearGrabAction::GripFailureRecords axes{};
    // Grip 姿态由 arm / turn 两轴组成，对应 axis0 / axis1。
    for (uint8_t axis = 0; axis < Diagnostics::SpearGrabAction::GripAxisCount; ++axis)
    {
        if ((mask & static_cast<uint8_t>(1U << axis)) != 0U)
        {
            axes[axis] = {
                .valid = true,
                .axis  = axis,
                .info  = ::Grip::grip->lastPlanFailureInfo(axis),
            };
        }
    }
    Diagnostics::SpearGrabAction::last_spear_grab_diagnostics = {
        diagnosticContext(),
        Diagnostics::SpearGrabAction::Reason::GripPlanFailed,
        {},
        axes,
    };
    if (Chassis::ctrl != nullptr)
        Chassis::ctrl->stop();
    ::Grip::grip->stop();
    state_  = State::Done;
    failed_ = true;
}

void SpearGrab::reportChassisPlanFailure()
{
    if (Chassis::ctrl == nullptr)
    {
        abort(Diagnostics::SpearGrabAction::Reason::DependencyNotReady);
        return;
    }

    const auto& failure = Chassis::ctrl->lastSCurveFailure();
    if (!failure.valid || failure.axis_mask == 0U)
    {
        // 底盘控制器没有可展开的 S 曲线失败细节时，只保留摘要。
        abort(Diagnostics::SpearGrabAction::Reason::ChassisPlanFailed);
        return;
    }

    Diagnostics::SpearGrabAction::last_spear_grab_diagnostics = {
        diagnosticContext(),
        Diagnostics::SpearGrabAction::Reason::ChassisPlanFailed,
        failure,
        {},
    };
    Chassis::ctrl->stop();
    if (::Grip::grip != nullptr)
        ::Grip::grip->stop();
    state_  = State::Done;
    failed_ = true;
}

bool SpearGrab::setChassisTarget(const chassis::Posture&                              target,
                                 const Chassis::ChassisController::TrajectoryLinkMode mode,
                                 const Chassis::ChassisController::TrajectoryLimit&   limit)
{
    if (Chassis::ctrl == nullptr || !Chassis::ctrl->setTargetPostureInWorld(target, mode, limit))
    {
        reportChassisPlanFailure();
        return false;
    }
    return true;
}

bool SpearGrab::setChassisTarget(const chassis::Posture&                              target,
                                 const Chassis::ChassisController::TrajectoryLinkMode mode)
{
    if (Chassis::ctrl == nullptr || !Chassis::ctrl->setTargetPostureInWorld(target, mode))
    {
        reportChassisPlanFailure();
        return false;
    }
    return true;
}

void SpearGrab::grab(const chassis::Posture& target_pos,
                     const chassis::Posture& end_pos,
                     const float             lift_execute)
{
    // 若上次已结束，则允许复位到 Idle 重新启动一轮动作。
    if (!canStart())
    {
        Diagnostics::SpearGrabAction::report(
                diagnosticContext(),
                isRunning() ? Diagnostics::SpearGrabAction::Reason::Busy
                            : Diagnostics::SpearGrabAction::Reason::DependencyNotReady);
        return;
    }

    if (state_ == State::Done)
        state_ = State::Idle;
    failed_ = false;

    constexpr float pre_grab_approach_distance = ::Grip::Config::SpearGrab::PreGrabApproachDistance;
    constexpr float safe_distance              = ::Grip::Config::SpearGrab::SafeDistance;
    static_assert(pre_grab_approach_distance > 0.0f);
    static_assert(safe_distance > 0.0f);
    static_assert(pre_grab_approach_distance < safe_distance);

    target_pos_            = target_pos;
    end_pos_               = end_pos;
    lift_execute_          = lift_execute;
    post_grab_lift_pos_    = lift_execute_ + ::Grip::Config::SpearGrab::PostGrabLiftRaise;
    end_pos_rel_to_target_ = Chassis::ChassisLoc::WorldPosture2RelativePosture(target_pos_,
                                                                               end_pos_);
    if (end_pos_rel_to_target_.x <= safe_distance)
    {
        Diagnostics::SpearGrabAction::report(diagnosticContext(),
                                             Diagnostics::SpearGrabAction::Reason::InvalidArgument);
        return;
    }

    prepare_safe_pos_     = postureRelativeToTargetInWorld({ safe_distance, 0.0f, 0.0f });
    prepare_approach_pos_ = postureRelativeToTargetInWorld(
            { pre_grab_approach_distance, 0.0f, 0.0f });
    leave_target_x_only_pos_ = postureRelativeToTargetInWorld(
            { end_pos_rel_to_target_.x, 0.0f, 0.0f });

    assert(end_pos_rel_to_target_.x > safe_distance);

    // 启动后底盘、抬升、grip 可同时动作；底盘第一目标只到安全距离边界。
    ::Grip::grip->openClaw();
    if (!::Grip::grip->toJointPose(::Grip::Config::Poses::PrepareGrab1))
    {
        reportGripPlanFailure();
        return;
    }

    if (!setChassisTarget(prepare_safe_pos_,
                          Master::TrajectoryLinkMode::CurrentState,
                          Config::SpearGrab::PrepareGrabLimit))
        return;

    if constexpr (ProjectParts::EnableLift)
    {
        // 夹取前先把 lift 提到执行高度，为后续靠近目标留出机构空间。
        Chassis::motion->liftAllTo(lift_execute_, Chassis::Config::Lift::OnloadLimit);
    }

    state_ = State::MovingGripToPrepare1;
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
    auto [x, y, yaw] = Chassis::loc->CurrentPostureRelativeTo(target_pos_);
    while (yaw > 180.0f)
        yaw -= 360.0f;
    while (yaw < -180.0f)
        yaw += 360.0f;
    if (yaw == -180.0f)
        yaw = 180.0f;
    return { x, y, yaw };
}

void SpearGrab::update()
{
    // lift 不是所有配置都启用，因此统一折叠成一个“是否完成”的判断。
    constexpr auto is_lift_finished = []() -> bool
    {
        if constexpr (ProjectParts::EnableLift)
            return Chassis::motion->isLiftAllFinished();

        return true;
    };

    const auto is_lift_at_execute_position = [&]() -> bool
    {
        if constexpr (ProjectParts::EnableLift)
        {
            const auto lift_settled = [&](const Lift::LiftSide& l)
            {
                return std::fabs(l.getPosition() - lift_execute_) <
                       Config::SpearGrab::PrepareLiftZThreshold;
            };

            return is_lift_finished() &&
                   lift_settled(Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front)) &&
                   lift_settled(Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear));
        }

        return true;
    };

    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::MovingGripToPrepare1:
        if (::Grip::grip->armPosition() >= ::Grip::Config::Poses::PrepareGrab2.arm_pos + 5)
        {
            state_ = State::MovingToPrepare;
        }
        break;
    case State::MovingToPrepare:
    {
        const auto rel_pos = currentRelativeToTarget();

        // lift 和 yaw 到位后才允许进入安全距离以内；此处不等待 y 收敛。
        if (is_lift_at_execute_position() && ::Grip::grip->isFinished() &&
            std::fabs(rel_pos.yaw) < ::Grip::Config::SpearGrab::PrepareYawThreshold)
        {
            if (!setChassisTarget(prepare_approach_pos_,
                                  Master::TrajectoryLinkMode::PreviousCurve,
                                  Config::SpearGrab::PrepareGrabLimit))
                break;
            state_ = State::MovingGripToPrepare2;
        }
        break;
    }
    case State::MovingGripToPrepare2:
    {
        const auto rel_pos = currentRelativeToTarget();

        // chassis 到达 approach，且 lift / yaw 仍保持就位后，才把 grip 切到第二预备姿态。
        if (Chassis::ctrl->isTrajectoryFinished() && is_lift_at_execute_position() &&
            std::fabs(rel_pos.yaw) < ::Grip::Config::SpearGrab::PrepareYawThreshold)
        {
            if (!::Grip::grip->toJointPose(::Grip::Config::Poses::PrepareGrab2))
            {
                reportGripPlanFailure();
                break;
            }
            state_ = State::WaitingTargetEntry;
        }
        break;
    }
    case State::WaitingTargetEntry:
    {
        const auto rel_pos = currentRelativeToTarget();

        // grip、侧向、yaw、lift 均就位后，才允许底盘从 approach 切入最终夹取位。
        if (::Grip::grip->isFinished() && is_lift_at_execute_position() &&
            std::fabs(rel_pos.y) < ::Grip::Config::SpearGrab::PrepareYThreshold &&
            std::fabs(rel_pos.yaw) < ::Grip::Config::SpearGrab::PrepareYawThreshold)
        {
            if (!setChassisTarget(target_pos_,
                                  Master::TrajectoryLinkMode::PreviousCurve,
                                  Config::SpearGrab::GrabLimit))
                break;
            state_ = State::MovingToTarget;
        }
        break;
    }
    case State::MovingToTarget:
        // chassis 到达目标位后先合爪，避免行进中提前夹住矛头。
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            ::Grip::grip->closeClaw();
            wait_state_since_ms_ = HAL_GetTick();
            state_               = State::WaitingClawClose;
        }
        break;
    case State::WaitingClawClose:
        if (HAL_GetTick() - wait_state_since_ms_ >= ::Grip::Config::SpearGrab::ClawCloseDelayMs)
        {
            // 合爪保持完成后再启动 lift 抬升，让夹爪先建立稳定夹持。
            Chassis::motion->liftAllTo(post_grab_lift_pos_, Chassis::Config::Lift::OnloadLimit);
            state_ = State::RaisingLiftAfterGrab;
        }
        break;
    case State::RaisingLiftAfterGrab:
        if (is_lift_finished())
        {
            // 合爪和抬升完成后先切到对接姿态；若姿态切换失败，整套动作立即中止。
            if (!::Grip::grip->toDockingPose())
            {
                reportGripPlanFailure();
                break;
            }

            if constexpr (ProjectParts::EnableLift)
            {
                if (::Grip::Config::SpearGrab::LiftDocking > post_grab_lift_pos_)
                {
                    // 如果对接高度高于夹后抬升高度，则在撤离前就先抬升
                    Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                               Chassis::Config::Lift::OnloadLimit);
                }
            }

            // 第一段撤离只放开 x 方向，尽快离开危险区，再进入最终轨迹。
            if (!setChassisTarget(leave_target_x_only_pos_,
                                  Master::TrajectoryLinkMode::PreviousCurve))
                break;
            state_ = State::LeavingTargetToSafeX;
        }
        break;
    case State::LeavingTargetToSafeX:
    {
        const auto rel_pos = currentRelativeToTarget();

        if (rel_pos.x > ::Grip::Config::SpearGrab::SafeDistance)
        {
            if constexpr (ProjectParts::EnableLift)
            {
                if (::Grip::Config::SpearGrab::LiftDocking <= post_grab_lift_pos_)
                {
                    // 若对接高度不高于夹后抬升高度，则等撤到安全 x 之后再调整 lift。
                    Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                               Chassis::Config::Lift::OnloadLimit);
                }
            }
            // 到达安全区后再切终点轨迹，避免在目标附近做大幅横移。
            if (!setChassisTarget(end_pos_, Master::TrajectoryLinkMode::PreviousCurve))
                break;
            state_ = State::MovingToEnd;
        }
        break;
    }
    case State::MovingToEnd:
    {
        const auto current_pos = Chassis::loc->postureInWorld();

        // 只有底盘、grip、lift 三者都收敛，才认为整套夹取动作真正完成。
        if (Chassis::ctrl->isTrajectoryFinished() && ::Grip::grip->isFinished() &&
            is_lift_finished() &&
            std::fabs(current_pos.x - end_pos_.x) < ::Grip::Config::SpearGrab::EndXYThreshold &&
            std::fabs(current_pos.y - end_pos_.y) < ::Grip::Config::SpearGrab::EndXYThreshold &&
            std::fabs(chassis::Posture::yawError(current_pos.yaw, end_pos_.yaw)) <
                    ::Grip::Config::SpearGrab::EndYawThreshold)
        {
            Chassis::ctrl->setVelocityInBody(chassis::Velocity::zero(), false);
            state_ = State::Done;
        }
        break;
    }
    }
}

[[noreturn]] void SpearGrab::loop()
{
    for (;;)
    {
        // 空闲时阻塞等待启动，避免线程空转占用 CPU。
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);
        while (!isFinished())
        {
            update();
            // 当前状态机依赖 1 ms 粒度推进，delay 同时也作为时间基准。
            osDelay(1);
        }
    }
}
} // namespace Grip::Action
