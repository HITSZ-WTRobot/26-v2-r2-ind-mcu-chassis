#include "grip_actions.hpp"
#include "grip/Config.hpp"

#include "chassis/chassis.hpp"

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
    // 必须保证动作等待完成或空闲，同时机械臂与底盘控制器已就绪
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr && ::Grip::grip->enabled() && Chassis::ctrl != nullptr && Chassis::loc != nullptr;
}

void SpearGrab::grab(float prepare_distance_x,
                      float prepare_distance_y,
                      float advance_distance,
                      float backoff_distance)
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    prepare_distance_x_ = prepare_distance_x;
    prepare_distance_y_ = prepare_distance_y;
    advance_distance_   = advance_distance;
    backoff_distance_   = backoff_distance;
    grab_target_x_      = prepare_distance_x_ + advance_distance_;
    back_target_x_      = grab_target_x_ + backoff_distance_;
    start_pos_          = Chassis::loc->postureInWorld();

    Chassis::ctrl->setTargetPostureInWorld(relativePosture(prepare_distance_x_, prepare_distance_y_));
    state_ = State::DrivingToReady;
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

chassis::Posture SpearGrab::relativePosture(float x, float y) const
{
    return Chassis::loc->RelativePosture2WorldPosture(start_pos_, { x, y, 0.0f });
}

float SpearGrab::currentRelativeX() const
{
    return Chassis::loc->CurrentPostureRelativeTo(start_pos_).x;
}

void SpearGrab::update()
{
    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::DrivingToReady:
        // 底盘到达准备位置后，先抬升到底盘夹取高度，再摆出机械臂准备夹取
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            if constexpr (ProjectParts::EnableLift)
            {
                Chassis::motion->liftAllTo(::Grip::Config::Position::LiftGrab,
                                           Chassis::Config::Lift::OnloadLimit);
                state_ = State::LiftToGrabHeight;
            }
            else
            {
                ::Grip::grip->toReadyPose();
                state_ = State::ArmReady;
            }
        }
        break;
    case State::LiftToGrabHeight:
        if (Chassis::motion->isLiftAllFinished())
        {
            ::Grip::grip->toReadyPose();
            state_ = State::ArmReady;
        }
        break;
    case State::ArmReady:
        // 机械臂到位后，底盘继续前进到夹取目标位置
        if (::Grip::grip->isFinished())
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(grab_target_x_, prepare_distance_y_));
            state_ = State::DrivingForward;
        }
        break;
    case State::DrivingForward:
        // 到达矛头位置后执行夹取动作
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            ::Grip::grip->toGripOutPose();
            state_ = State::Grabbing;
        }
        break;
    case State::Grabbing:
        // 夹取完成后，让底盘后退并进入收回姿态
        if (::Grip::grip->isFinished())
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(back_target_x_, prepare_distance_y_), Master::TrajectoryLinkMode::PreviousCurve);
            state_ = State::DrivingBack;
        }
        break;
    case State::DrivingBack:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            ::Grip::grip->toDockingPose();
            if constexpr (ProjectParts::EnableLift)
            {
                Chassis::motion->liftAllTo(Chassis::Config::Lift::Position::Normal,
                                           Chassis::Config::Lift::OnloadLimit);
            }
            state_ = State::Docking;
        }
        break;
    case State::Docking:
        // 机械臂回到对接姿态后结束动作
        if (::Grip::grip->isFinished())
        {
            if constexpr (ProjectParts::EnableLift)
            {
                if (Chassis::motion->isLiftAllFinished())
                    state_ = State::Done;
            }
            else
            {
                state_ = State::Done;
            }
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
    task_ = osThreadNew(TaskEntry, this, &attr);
}

RollerStore& RollerStore::inst()
{
    static RollerStore instance;
    return instance;
}

bool RollerStore::canStart() const
{
    return (state_ == State::Idle || state_ == State::Done) && ::Grip::grip != nullptr && ::Grip::grip->enabled();
}

void RollerStore::store(float /*storage_distance_x*/)
{
    if (!canStart())
        return;

    if (state_ == State::Done)
        state_ = State::Idle;

    start_pos_ = Chassis::loc->postureInWorld();
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

chassis::Posture RollerStore::relativePosture(float x) const
{
    return Chassis::loc->RelativePosture2WorldPosture(start_pos_, { x, 0.0f, 0.0f });
}

void RollerStore::update()
{
    switch (state_)
    {
    case State::Idle:
    case State::Done:
        break;
    case State::MovingToStorePose:
        // 机械臂到达存储姿态后，启动吸盘吸持卷轴
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
