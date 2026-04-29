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

KfsStore::KfsStore() :
    kfs_suction_cup_(::Grip::Config::KfsStore::SuctionCupConfig,
                     Device::Sensor::grip_suction_pressure)
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
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void KfsStore::store()
{
    if (!canStart())
        return;

    // 有气压计时以实时检测为准；无气压计时退化为流程内的默认持物状态。
    if (kfs_suction_cup_.canDetectObject())
    {
        if (kfs_suction_cup_.hasObject())
            return;
    }
    else if (assumed_has_object_)
    {
        return;
    }

    if (state_ == State::Done)
        state_ = State::Idle;

    // 先启动吸盘，再把机构移向拾取位，避免刚到位时负压尚未建立。
    assumed_has_object_ = false;
    kfs_suction_cup_.activate();
    if (::Grip::grip->toKfsPickupPose())
    {
        workflow_phase_ = WorkflowPhase::Store;
        state_          = State::MovingToPickupPose;
        osThreadFlagsSet(task_, FlagStart);
        return;
    }

    // 若姿态规划失败，则立即撤销本次吸盘动作，避免空吸。
    kfs_suction_cup_.deactivate();
}

void KfsStore::release()
{
    if (!canStart())
        return;

    // 有气压计时以实时检测为准；无气压计时退化为流程内的默认持物状态。
    if (kfs_suction_cup_.canDetectObject())
    {
        if (!kfs_suction_cup_.hasObject())
            return;
    }
    else if (!assumed_has_object_)
    {
        return;
    }

    if (state_ == State::Done)
        state_ = State::Idle;

    // 释放流程先移动到释放位，真正关闭吸盘要等机构到位后再执行。
    if (::Grip::grip->toKfsReleasePose())
    {
        workflow_phase_ = WorkflowPhase::Release;
        state_          = State::MovingToReleasePose;
        osThreadFlagsSet(task_, FlagStart);
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
            state_ = State::WaitingObjectAttach;
        }
        break;
    case State::WaitingObjectAttach:
    {
        bool attached = false;
        if (kfs_suction_cup_.canDetectObject())
        {
            attached = kfs_suction_cup_.hasObject();
        }
        else
        {
            attached = HAL_GetTick() - wait_state_since_ms_ >=
                       ::Grip::Config::KfsStore::AttachConfirmDelayMs;
        }

        if (attached)
        {
            assumed_has_object_ = true;
            if (::Grip::grip->toKfsStorePose())
                state_ = State::MovingToStorePose;
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
            // TODO: 增加电磁阀，释放阶段应先切阀再关闭气泵 / 吸盘。
            kfs_suction_cup_.deactivate();
            wait_state_since_ms_ = HAL_GetTick();
            state_ = State::WaitingObjectRelease;
        }
        break;
    case State::WaitingObjectRelease:
    {
        bool released = false;
        if (kfs_suction_cup_.canDetectObject())
        {
            released = !kfs_suction_cup_.hasObject();
        }
        else
        {
            released = HAL_GetTick() - wait_state_since_ms_ >=
                       ::Grip::Config::KfsStore::ReleaseConfirmDelayMs;
        }

        if (released)
        {
            assumed_has_object_ = false;
            if (::Grip::grip->toStandbyPose())
                state_ = State::MovingToStandbyPose;
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
