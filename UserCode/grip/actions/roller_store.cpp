#include "roller_store.hpp"

#include "device.hpp"
#include "grip/grip.hpp"
#include "main.h"
#include "project_parts.hpp"

namespace Grip::Action
{
namespace
{
// 暂存 / 释放动作由独立线程推进，外部只负责发出一次启动信号。
constexpr uint32_t FlagStart = 1u << 0;
} // namespace

KfsStore::KfsStore()
{
    // 与 SpearGrab 一样，KFS 使用独立后台线程管理分阶段流程。
    constexpr osThreadAttr_t attr{
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };

    task_ = osThreadNew(TaskEntry, this, &attr);
}

KfsStore& KfsStore::inst()
{
    static KfsStore instance;
    return instance;
}

bool KfsStore::canStart() const
{
    if constexpr (!ProjectParts::EnableKfsAction)
        return false;

    // 只允许在空闲 / 已完成状态重启，并且要求底层 grip 已经就绪。
    // 这里不检查 suction 状态，因为 store/release 自身会负责吸盘开关与物体确认。
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void KfsStore::abort(const Diagnostics::KfsStoreAction::Reason reason)
{
    // 失败后统一停掉 grip 和吸盘，动作对象收敛到 Done，并保留模块内诊断。
    Diagnostics::KfsStoreAction::report(diagnosticContext(), reason);
    if (::Grip::grip != nullptr)
        ::Grip::grip->stop();
    if (state_ == State::MovingToPickupPose || state_ == State::WaitingObjectAttach)
        Device::Suction::grip->deactivate();
    state_  = State::Done;
    failed_ = true;
}

void KfsStore::reportGripPlanFailure()
{
    if (::Grip::grip == nullptr)
    {
        abort(Diagnostics::KfsStoreAction::Reason::DependencyNotReady);
        return;
    }

    const uint8_t mask = ::Grip::grip->lastPlanFailureMask();
    if (mask == 0U)
    {
        // Grip 没有留下可展开的轴信息时，只能上报普通姿态规划失败。
        abort(Diagnostics::KfsStoreAction::Reason::GripPlanFailed);
        return;
    }

    Diagnostics::KfsStoreAction::GripFailureRecords axes{};
    // Grip 组合姿态按 arm / turn 两轴展开到 S 曲线记录里。
    for (uint8_t axis = 0; axis < Diagnostics::KfsStoreAction::GripAxisCount; ++axis)
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
    Diagnostics::KfsStoreAction::last_kfs_store_diagnostics = {
        diagnosticContext(),
        Diagnostics::KfsStoreAction::Reason::GripPlanFailed,
        axes,
    };
    if (::Grip::grip != nullptr)
        ::Grip::grip->stop();
    state_  = State::Done;
    failed_ = true;
}

void KfsStore::store()
{
    if (!canStart())
    {
        Diagnostics::KfsStoreAction::report(
                diagnosticContext(),
                isRunning() ? Diagnostics::KfsStoreAction::Reason::Busy
                            : Diagnostics::KfsStoreAction::Reason::DependencyNotReady);
        return;
    }

    // 有气压计时以实时检测为准；无气压计时退化为流程内的默认持物状态。
    if (Device::Suction::grip->canDetectObject())
    {
        if (Device::Suction::grip->hasObject())
        {
            Diagnostics::KfsStoreAction::report(
                    diagnosticContext(),
                    Diagnostics::KfsStoreAction::Reason::ObjectAlreadyAttached);
            return;
        }
    }
    else if (assumed_has_object_)
    {
        Diagnostics::KfsStoreAction::report(
                diagnosticContext(),
                Diagnostics::KfsStoreAction::Reason::ObjectAlreadyAttached);
        return;
    }

    if (state_ == State::Done)
        state_ = State::Idle;
    failed_ = false;

    // 先启动吸盘，再把机构移向拾取位，避免刚到位时负压尚未建立。
    assumed_has_object_ = false;
    Device::Suction::grip->activate();
    if (::Grip::grip->toKfsPickupPose())
    {
        workflow_phase_ = WorkflowPhase::Store;
        state_          = State::MovingToPickupPose;
        osThreadFlagsSet(task_, FlagStart);
        return;
    }

    // 若姿态规划失败，则立即撤销本次吸盘动作，避免空吸。
    Device::Suction::grip->deactivate();
    reportGripPlanFailure();
}

void KfsStore::release()
{
    if (!canStart())
    {
        Diagnostics::KfsStoreAction::report(
                diagnosticContext(),
                isRunning() ? Diagnostics::KfsStoreAction::Reason::Busy
                            : Diagnostics::KfsStoreAction::Reason::DependencyNotReady);
        return;
    }

    // 有气压计时以实时检测为准；无气压计时退化为流程内的默认持物状态。
    if (Device::Suction::grip->canDetectObject())
    {
        if (!Device::Suction::grip->hasObject())
        {
            Diagnostics::KfsStoreAction::report(diagnosticContext(),
                                                Diagnostics::KfsStoreAction::Reason::ObjectMissing);
            return;
        }
    }
    else if (!assumed_has_object_)
    {
        Diagnostics::KfsStoreAction::report(diagnosticContext(),
                                            Diagnostics::KfsStoreAction::Reason::ObjectMissing);
        return;
    }

    if (state_ == State::Done)
        state_ = State::Idle;
    failed_ = false;

    // 释放流程先移动到释放位，真正关闭吸盘要等机构到位后再执行。
    if (::Grip::grip->toKfsReleasePose())
    {
        workflow_phase_ = WorkflowPhase::Release;
        state_          = State::MovingToReleasePose;
        osThreadFlagsSet(task_, FlagStart);
    }
    else
    {
        reportGripPlanFailure();
    }
}

bool KfsStore::isIdle() const
{
    return state_ == State::Idle;
}

bool KfsStore::isFinished() const
{
    return state_ == State::Done;
}

bool KfsStore::isRunning() const
{
    return state_ != State::Idle && state_ != State::Done;
}

bool KfsStore::hasDetectedObject()
{
    return Device::Suction::grip->canDetectObject() && Device::Suction::grip->hasObject();
}

void KfsStore::waitForFinish() const
{
    while (!isFinished())
        osDelay(10);
}

void KfsStore::update()
{
    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::MovingToPickupPose:
        if (::Grip::grip->isFinished())
        {
            // 机械臂到位后：有气压计继续等气压确认；无气压计则从此刻开始计保底延时。
            wait_state_since_ms_ = HAL_GetTick();
            state_               = State::WaitingObjectAttach;
        }
        break;
    case State::WaitingObjectAttach:
    {
        bool attached = false;
        if (Device::Suction::grip->canDetectObject())
        {
            attached = Device::Suction::grip->hasObject();
        }
        else
        {
            attached = HAL_GetTick() - wait_state_since_ms_ >=
                       ::Grip::Config::KfsStore::AttachConfirmDelayMs;
        }

        if (attached)
        {
            assumed_has_object_ = true;
            // 拾取确认后再切到暂存位，避免在没夹住工件时提前离开拾取位。
            if (::Grip::grip->toKfsStorePose())
            {
                state_ = State::MovingToStorePose;
            }
            else
            {
                reportGripPlanFailure();
            }
        }
        break;
    }
    case State::MovingToStorePose:
        // 暂存流程在到达暂存位后结束，吸盘保持工作状态。
        if (::Grip::grip->isFinished())
            state_ = State::Done;
        break;
    case State::MovingToReleasePose:
        if (::Grip::grip->isFinished())
        {
            Device::Suction::grip->deactivate();
            wait_state_since_ms_ = HAL_GetTick();

            // 先关闭吸盘，再进入释放确认等待，确认对象确实已经脱离。
            state_ = State::WaitingObjectRelease;
        }
        break;
    case State::WaitingObjectRelease:
    {
        bool released = false;
        if (Device::Suction::grip->canDetectObject())
        {
            released = !Device::Suction::grip->hasObject();
        }
        else
        {
            released = HAL_GetTick() - wait_state_since_ms_ >=
                       ::Grip::Config::KfsStore::ReleaseConfirmDelayMs;
        }

        if (released)
        {
            assumed_has_object_ = false;
            // 释放确认后再回待机位，避免工件尚未脱离就提前撤回。
            if (::Grip::grip->toStandbyPose())
            {
                state_ = State::MovingToStandbyPose;
            }
            else
            {
                reportGripPlanFailure();
            }
        }
        break;
    }
    case State::MovingToStandbyPose:
        if (::Grip::grip->isFinished())
            state_ = State::Done;
        break;
    }
}

[[noreturn]] void KfsStore::loop()
{
    for (;;)
    {
        // 空闲时休眠等待新动作，启动后再按 1 ms 周期推进状态机。
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);
        while (!isFinished())
        {
            update();
            osDelay(1);
        }
    }
}
} // namespace Grip::Action
