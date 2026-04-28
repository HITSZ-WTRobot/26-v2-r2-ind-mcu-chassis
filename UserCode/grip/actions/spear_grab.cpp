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
// 外部仅需置位一次启动标志，后台线程随后独占推进整个状态机。
constexpr uint32_t FlagStart = 1u << 0;

using chassis::controller::Master;
} // namespace

SpearGrab::SpearGrab()
{
    // 独立线程使 SpearGrab 可以与主控线程解耦，避免在协议或主循环里手写阶段推进。
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

void SpearGrab::abort()
{
    // 机械臂轨迹失败后不能继续跑底盘轨迹，否则会出现机构姿态与车体位置脱节。
    Chassis::ctrl->stop();
    state_ = State::Done;
}

void SpearGrab::grab(const chassis::Posture& target_pos,
                     const chassis::Posture& end_pos,
                     float                   safe_distance)
{
    // 若上次已结束，则允许复位到 Idle 重新启动一轮动作。
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

    // 先确认 grip 能进入准备姿态，再发底盘 / 抬升目标，避免流程半启动。
    if (!::Grip::grip->toPrepareGrabPose())
        return;

    // 底盘先切入 prepare 位；这里不要求先完全到位，状态机后续会边走边判断。
    Chassis::ctrl->setTargetPostureInWorld(prepare_pos_);

    if constexpr (ProjectParts::EnableLift)
    {
        // 夹取前先把 lift 提到执行高度，为后续靠近目标留出机构空间。
        Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftExecute,
                                   Chassis::Config::Lift::OnloadLimit);
    }

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
    // lift 不是所有配置都启用，因此统一折叠成一个“是否完成”的判断。
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

        // prepare 阶段关注横向与朝向误差，而非要求底盘完全停在 prepare 点。
        // 一旦侧向 / 偏航收敛，且 lift / grip 都已到位，就可以直接衔接目标轨迹。
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
        // 底盘到达目标位后，再让机械臂执行最终夹取，避免提早闭合或推进。
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            if (!::Grip::grip->toGrabPose())
            {
                abort();
                break;
            }
            state_ = State::Grabbing;
        }
        break;
    case State::Grabbing:
        if (::Grip::grip->isFinished())
        {
            // 夹取完成后先切到对接姿态；若姿态切换失败，整套动作立即中止。
            if (!::Grip::grip->toDockingPose())
            {
                abort();
                break;
            }

            if constexpr (ProjectParts::EnableLift && (::Grip::Config::SpearGrab::LiftDocking >
                                                       ::Grip::Config::SpearGrab::LiftExecute))
            {
                // 如果对接高度高于夹取高度，则在撤离前就先抬升，减少后续碰撞风险。
                Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                           Chassis::Config::Lift::OnloadLimit);
            }

            // 第一段撤离只放开 x 方向，尽快离开危险区，再进入最终轨迹。
            Chassis::ctrl->setTargetPostureInWorld(leave_target_x_only_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
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
                // 若对接高度不高于夹取高度，则等撤到安全 x 之后再调整 lift。
                Chassis::motion->liftAllTo(::Grip::Config::SpearGrab::LiftDocking,
                                           Chassis::Config::Lift::OnloadLimit);
            }
            // 到达安全区后再切终点轨迹，避免在目标附近做大幅横移。
            Chassis::ctrl->setTargetPostureInWorld(end_pos_,
                                                   Master::TrajectoryLinkMode::PreviousCurve);
            state_ = State::MovingToEnd;
        }
        break;
    }
    case State::MovingToEnd:
        // 只有底盘、grip、lift 三者都收敛，才认为整套夹取动作真正完成。
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
