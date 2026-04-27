#include "roller_store.hpp"

#include "grip/grip.hpp"

namespace Grip::Action
{
namespace
{
constexpr uint32_t FlagStart = 1u << 0;
} // namespace

RollerStore::SuctionCup::SuctionCup()
#ifdef GRIP_SUCTION_GPIO_Port
    : gpio_{ GRIP_SUCTION_GPIO_Port, GRIP_SUCTION_Pin }, active_(false), available_(true)
#else
    : gpio_{ nullptr, 0 }, active_(false), available_(false)
#endif
{
    // 如果没有定义吸盘 GPIO 引脚，那么吸盘动作为不可用
}

void RollerStore::SuctionCup::activate()
{
    if (!available_)
        return;
    GPIO_SetPin(&gpio_);
    active_ = true;
}

void RollerStore::SuctionCup::deactivate()
{
    if (!available_)
        return;
    GPIO_ResetPin(&gpio_);
    active_ = false;
}

bool RollerStore::SuctionCup::isActive() const
{
    return active_;
}

bool RollerStore::SuctionCup::isAvailable() const
{
    return available_;
}

RollerStore::RollerStore()
{
    constexpr osThreadAttr_t attr{
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    task_        = osThreadNew(TaskEntry, this, &attr);
    roller_store = this;
}

RollerStore& RollerStore::inst()
{
    static RollerStore instance;
    return instance;
}

bool RollerStore::canStart() const
{
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr &&
           ::Grip::grip->enabled();
}

void RollerStore::store(float /*storage_distance_x*/)
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    if (::Grip::grip->toStorePose())
    {
        state_ = State::MovingToStorePose;
        osThreadFlagsSet(task_, FlagStart);
    }
}

void RollerStore::release()
{
    suction_.deactivate();
}

bool RollerStore::isIdle() const
{
    return state_ == State::Idle;
}

bool RollerStore::isFinished() const
{
    return state_ == State::Done;
}

bool RollerStore::isRunning() const
{
    return state_ != State::Idle && state_ != State::Done;
}

bool RollerStore::isSuctionActive() const
{
    return suction_.isActive();
}

void RollerStore::waitForFinish() const
{
    while (!isFinished())
        osDelay(10);
}

void RollerStore::update()
{
    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::MovingToStorePose:
        // 机械臂到达卷轴临时存放姿态后，启动吸盘吸持卷轴
        if (::Grip::grip->isFinished())
        {
            if (suction_.isAvailable())
                suction_.activate();
            state_ = State::ActivatingSuction;
        }
        break;
    case State::ActivatingSuction:
        state_ = State::Done;
        break;
    }
}

[[noreturn]] void RollerStore::loop()
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
