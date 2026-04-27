#include "roller_store.hpp"

#include "grip/grip.hpp"

namespace Grip::Action
{
namespace
{
constexpr uint32_t FlagStart = 1u << 0;
} // namespace

KfsStore::SuctionCup::SuctionCup() :
    gpio_(::Grip::Config::KfsStore::SuctionGPIO)
{
}

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
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void KfsStore::store()
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    suction_.activate();
    if (::Grip::grip->toKfsPickupPose())
    {
        delay_ms_remaining_ = 0;
        state_              = State::MovingToPickupPose;
        osThreadFlagsSet(task_, FlagStart);
        return;
    }

    suction_.deactivate();
}

void KfsStore::release()
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

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
        if (::Grip::grip->isFinished())
            state_ = State::Done;
        break;
    case State::MovingToReleasePose:
        if (::Grip::grip->isFinished())
        {
            // TODO: 增加电磁阀，释放阶段应先切阀再关闭气泵 / 吸盘。
            suction_.deactivate();
            if (::Grip::grip->toKfsStandbyPose())
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
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);
        while (!isFinished())
        {
            update();
            osDelay(1);
        }
    }
}
} // namespace Grip::Action
