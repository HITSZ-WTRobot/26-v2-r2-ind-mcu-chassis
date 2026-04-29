#include "roller_store.hpp"

#include "grip/grip.hpp"
#include "project_parts.hpp"

namespace Grip::Action
{
namespace
{
// 暂存 / 释放动作由独立线程推进，外部只负责发出一次启动信号。
constexpr uint32_t FlagStart = 1u << 0;
} // namespace

KfsStore::KfsStore() : kfs_suction_cup_(::Grip::Config::KfsStore::SuctionCupConfig)
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
    if constexpr (!ProjectParts::EnableGripSuction)
        return false;

    // 只允许在空闲 / 已完成状态重启，并且要求底层 grip 已经就绪。
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void KfsStore::store()
{
    // 是否已持有卷轴，统一以吸盘当前观测到的真实状态为准。
    if (!canStart() || kfs_suction_cup_.hasObject())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    // 先启动吸盘，再把机构移向拾取位，避免刚到位时负压尚未建立。
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
    // 释放是否有意义，也统一由吸盘当前是否仍持有物体来决定。
    if (!canStart() || !kfs_suction_cup_.hasObject())
        return;

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

bool KfsStore::isPressureSensorOnline() const
{
    if constexpr (!ProjectParts::EnableGripSuctionPressureSensor)
        return false;

    // 连接表仍然需要一个独立的“气压计在线”观测位，但它不再参与动作启动判断。
    return kfs_suction_cup_.isPressureSensorOnline();
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
            // 机械臂到位后，不再自己计时，统一等待吸盘内部给出“已吸住”语义。
            state_ = State::WaitingObjectAttach;
        }
        break;
    case State::WaitingObjectAttach:
        // 是否吸住以及用什么手段确认，统一由吸盘内部自己决定。
        if (kfs_suction_cup_.hasObject() && ::Grip::grip->toKfsStorePose())
            state_   = State::MovingToStorePose;
        break;
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
            state_   = State::WaitingObjectRelease;
        }
        break;
    case State::WaitingObjectRelease:
        // 放料确认也统一由吸盘内部完成；这样有无气压计时，上层流程都保持一致。
        if (!kfs_suction_cup_.hasObject() && ::Grip::grip->toStandbyPose())
            state_ = State::MovingToStandbyPose;
        break;
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
