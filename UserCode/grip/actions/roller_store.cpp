#include "roller_store.hpp"

#include "grip/grip.hpp"

namespace Grip::Action
{
namespace
{
// 暂存 / 释放动作由独立线程推进，外部只负责发出一次启动信号。
constexpr uint32_t FlagStart = 1u << 0;
} // namespace

KfsStore::SuctionCup::SuctionCup() : gpio_(::Grip::Config::KfsStore::SuctionGPIO) {}

void KfsStore::SuctionCup::activate()
{
    GPIO_SetPin(&gpio_);
}

void KfsStore::SuctionCup::deactivate()
{
    GPIO_ResetPin(&gpio_);
}

bool KfsStore::SuctionCup::isActive() const
{
    return HAL_GPIO_ReadPin(gpio_.port, gpio_.pin) == GPIO_PIN_SET;
}

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
    // 只允许在空闲 / 已完成状态重启，并且要求底层 grip 已经使能。
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void KfsStore::store()
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    // 先启动吸盘，再把机构移向拾取位，避免刚到位时负压尚未建立。
    suction_.activate();
    if (::Grip::grip->toKfsPickupPose())
    {
        delay_ms_remaining_ = 0;
        state_              = State::MovingToPickupPose;
        osThreadFlagsSet(task_, FlagStart);
        return;
    }

    // 若姿态规划失败，则立即撤销本次吸盘动作，避免空吸。
    suction_.deactivate();
}

void KfsStore::release()
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    // 释放流程先移动到释放位，真正关闭吸盘要等机构到位后再执行。
    if (::Grip::grip->toKfsReleasePose())
    {
        state_ = State::MovingToReleasePose;
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

bool KfsStore::isSuctionActive() const
{
    return suction_.isActive();
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
            // 到达拾取位后开始按毫秒计时，等待吸盘建立基础负压。
            delay_ms_remaining_ = ::Grip::Config::KfsStore::SuctionBuildUpDelayMs;
            state_              = State::WaitingSuctionBuildUp;
        }
        break;
    case State::WaitingSuctionBuildUp:
        if (delay_ms_remaining_ > 0)
        {
            --delay_ms_remaining_;
            break;
        }

        // TODO: 增加气压计，改为等待吸盘建立足够负压后再切到暂存位。
        if (::Grip::grip->toKfsStorePose())
            state_ = State::MovingToStorePose;
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
            suction_.deactivate();
            // 放料后回系统待机姿态，把机构留在统一的空闲位置。
            if (::Grip::grip->toStandbyPose())
                state_ = State::MovingToStandbyPose;
        }
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
