/**
 * @file    Step.cpp
 * @author  syhanjin
 * @date    2026-04-03
 */
#include "Step.hpp"
#include "chassis/chassis.hpp"

namespace Action
{
namespace
{
constexpr uint32_t FlagStart  = 1 << 0;
constexpr uint32_t FlagResume = 1 << 1;

using namespace Chassis::Config::ChassisInfo;
using namespace Chassis::Config::Lift;

constexpr Chassis::Config::Limit 放腿速度{ MaxSpeed, MaxOnloadAccel, MaxOnloadAccel * 50 };

using chassis::controller::Master;
} // namespace

Step::Step()
{
    constexpr osThreadAttr_t attr{
        .stack_size = 256 * 4,
        .priority   = osPriorityNormal,
    };
    task_ = osThreadNew(TaskEntry, this, &attr);
}

Step& Step::inst()
{
    static Step inst;
    return inst;
}

void Step::prepare(const float     startDistance2Step,
                   const float     endDistance2Step,
                   const Direction dir)
{
    direction_          = dir;
    x_sign_             = dir == Direction::Forward ? 1 : -1;
    startDistance2Step_ = startDistance2Step;
    endDistance2Step_   = endDistance2Step;

    start_pos_ = Chassis::loc->postureInWorld();

    if (dir == Direction::Forward)
    {
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }
    else
    {
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }
}

/**
 * 上台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 上台阶方向
 * @param willTake 中间是否会停下来取卷轴
 *
 */
void Step::up(const float     startDistance2Step,
              const float     endDistance2Step,
              const Direction dir,
              const bool      willTake)
{
    if (isRunning())
        return;

    prepare(startDistance2Step, endDistance2Step, dir);
    will_take_ = willTake;

    chassis_state_ = ChassisState::Up1_等待底盘到达台阶高度_同时往台阶位移;
    front_state_   = LiftState::Up1_抬升ing;
    rear_state_    = LiftState::Up1_抬升ing;

    front_->to(Position::UpStep, OnloadLimit);
    rear_->to(Position::UpStep, OnloadLimit);

    // 第一个坐标点为 车体前边缘贴着台阶
    Chassis::ctrl->setTargetPostureInWorld(
            relativePosture(startDistance2Step_ - HalfChassisDistanceX - SafeDistance));

    osThreadFlagsSet(task_, FlagStart);
}
/**
 * 下台阶
 * @param startDistance2Step 开始时车体中心距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体中心距离台阶的距离 unit: m
 * @param dir 下台阶方向
 * @param shouldReset 最后是否复位底盘高度
 *
 */
void Step::down(const float     startDistance2Step,
                const float     endDistance2Step,
                const Direction dir,
                const bool      shouldReset)
{
    if (isRunning())
        return;

    prepare(startDistance2Step, endDistance2Step, dir);
    should_reset_ = shouldReset;

    chassis_state_ = ChassisState::Down0_前进使前轮前边缘到达台阶边缘_等待底盘降为过渡高度;
    front_state_   = LiftState::Down1_等待放下;
    rear_state_    = LiftState::Down1_等待放下;

    front_->to(Position::StepTransition, OnloadLimit);
    rear_->to(Position::StepTransition, OnloadLimit);

    // 第一个坐标点只是底盘的前导引导点；当两侧 lift 到达过渡高度后，会立即重定向到下一段目标。
    Chassis::ctrl->setTargetPostureInWorld(
            relativePosture(startDistance2Step_ - AbsWheelOuterEdgeX - 3 * SafeDistance));

    osThreadFlagsSet(task_, FlagStart);
}

/**
 * 继续上台阶
 */
void Step::resume_up()
{
    will_take_ = false;
    osThreadFlagsSet(task_, FlagResume);
}

void Step::update()
{
    switch (chassis_state_)
    {
    case ChassisState::Idle:
        break;
    case ChassisState::Up1_等待底盘到达台阶高度_同时往台阶位移:
        if (front_->isFinished() && rear_->isFinished())
        {
            // TODO: 不同阶段底盘的速度限制应当不同
            // TODO: 如果 will_take_ = false, 则该目标值设置应当带末速度
            Chassis::ctrl->setTargetPostureInWorld(
                    // 使前主动轮外边缘贴着台阶
                    relativePosture(startDistance2Step_ - AbsWheelOuterEdgeX - SafeDistance),
                    Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up2_前进将前辅助轮悬于台阶上方_等待前轮收起;

            if (will_take_)
                osThreadFlagsWait(FlagResume, osFlagsWaitAll, osWaitForever);
        }
        break;
    case ChassisState::Up2_前进将前辅助轮悬于台阶上方_等待前轮收起:
        if (front_state_ == LiftState::Up4_等待放下)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    // 使后主动轮内边缘贴着台阶
                    relativePosture(startDistance2Step_ + AbsWheelInnerEdgeX - SafeDistance),
                    Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up3_前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起;
        }
        break;
    case ChassisState::Up3_前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起:
        if ((front_state_ == LiftState::Done || front_state_ == LiftState::Up6_等待恢复到Normal) &&
            rear_state_ == LiftState::Up4_等待放下)
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(startDistance2Step_ +
                                                                   endDistance2Step_),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up4_前进使底盘完全登上台阶;
        }
        break;
    case ChassisState::Up4_前进使底盘完全登上台阶:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    case ChassisState::Down0_前进使前轮前边缘到达台阶边缘_等待底盘降为过渡高度:
        if (front_->isFinished() && rear_->isFinished())
        {
            // lift 已经到达过渡高度，此时允许底盘跳过前导目标，直接切到正式的下台阶位移目标。
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ - AbsAuxInnerWheelX - 3 * SafeDistance));
            chassis_state_ = ChassisState::Down1_前进使中前辅助轮到达台阶边缘_等待前轮放下;
        }
        break;
    case ChassisState::Down1_前进使中前辅助轮到达台阶边缘_等待前轮放下:
        if (front_state_ == LiftState::Down3_等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld( // 外侧（后）辅助轮到达台阶边缘
                    relativePosture(startDistance2Step_ + AbsAuxOuterWheelX - 3 * SafeDistance),
                    Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Down2_前进使后辅助轮到达台阶边缘_等待后轮放下;
        }
        break;
    case ChassisState::Down2_前进使后辅助轮到达台阶边缘_等待后轮放下:
        if (rear_state_ == LiftState::Down3_等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(startDistance2Step_ +
                                                                   endDistance2Step_),
                                                   Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Down3_前进使底盘完全走下台阶;
        }
        break;
    case ChassisState::Down3_前进使底盘完全走下台阶:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;

    case ChassisState::Done:
        break;
    }

    switch (front_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::Up1_抬升ing:
        if (front_->isFinished())
            front_state_ = LiftState::Up2_等待收起;
        break;
    case LiftState::Up2_等待收起:
        // 等待取物
        if (will_take_)
            break;
        // TODO: 这里不应该这样扩大安全距离
        if (currentRelativeX() >
            startDistance2Step_ - AbsAuxOuterWheelX + AuxWheelRadius + 3 * SafeDistance)
        {
            front_->to(LiftMin, NoloadLimit);
            front_->setGrounding(false); // 离地
            front_state_ = LiftState::Up3_收起ing;
        }
        break;
    case LiftState::Up3_收起ing:
        if (front_->isFinished())
            front_state_ = LiftState::Up4_等待放下;
        break;
    case LiftState::Up4_等待放下:
        // 前轮已经完全登上
        if (currentRelativeX() > startDistance2Step_ - AbsWheelInnerEdgeX + 3 * SafeDistance)
        {
            front_->to(Position::StepTransition, 放腿速度);
            front_state_ = LiftState::Up5_放下ing;
        }
        break;
    case LiftState::Up5_放下ing:
        if (front_->isFinished())
        {
            front_->setGrounding(true); // 着地
            if constexpr (Position::StepTransition == Position::Normal)
                front_state_ = LiftState::Done;
            else
                front_state_ = LiftState::Up6_等待恢复到Normal;
        }
        break;
    case LiftState::Up6_等待恢复到Normal:
        // 在 rear 触发
        break;
    case LiftState::Up7_恢复到Normaling:
        if (front_->isFinished())
            front_state_ = LiftState::Done;
        break;
    case LiftState::Down1_等待放下:
        if (currentRelativeX() > startDistance2Step_ - AbsWheelOuterEdgeX)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ - AbsWheelInnerEdgeX + SafeDistance)
        {
            front_->to(Position::UpStep, NoloadLimit);
            front_state_ = LiftState::Down2_放下ing;
        }
        break;
    case LiftState::Down2_放下ing:
        if (front_->isFinished())
        {
            front_->setGrounding(true);
            front_state_ = LiftState::Down3_等待回收到正常位置;
        }
        break;
    case LiftState::Down3_等待回收到正常位置:
        if (should_reset_)
        {
            if (currentRelativeX() > startDistance2Step_ + HalfChassisDistanceX + SafeDistance)
            {
                front_->to(Position::Normal, OnloadLimit);
                front_state_ = LiftState::Down4_回收ing;
            }
        }
        else
        {
            // 如果不复位底盘，则在底盘完成后立即完成
            if (chassis_state_ == ChassisState::Done)
            {
                front_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::Down4_回收ing:
        if (front_->isFinished())
        {
            front_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }

    switch (rear_state_)
    {
    case LiftState::Idle:
        break;
    case LiftState::Up1_抬升ing:
        if (rear_->isFinished())
            rear_state_ = LiftState::Up2_等待收起;
        break;
    case LiftState::Up2_等待收起:
        // 等待取物，此处可以不加该判断，这里为了对称
        if (will_take_)
            break;
        // 中后辅助轮已登上台阶
        if (currentRelativeX() >
            startDistance2Step_ + AbsAuxInnerWheelX + AuxWheelRadius + 3 * SafeDistance)
        {
            rear_->to(LiftMin, NoloadLimit);
            rear_->setGrounding(false);
            rear_state_ = LiftState::Up3_收起ing;
        }
        break;
    case LiftState::Up3_收起ing:
        if (rear_->isFinished())
            rear_state_ = LiftState::Up4_等待放下;
        break;
    case LiftState::Up4_等待放下:
        // 后轮已经完全登上
        if (currentRelativeX() > startDistance2Step_ + AbsWheelOuterEdgeX + 3 * SafeDistance)
        {
            if (front_state_ == LiftState::Up6_等待恢复到Normal)
            {
                front_->to(Position::Normal, 放腿速度);
                front_state_ = LiftState::Up7_恢复到Normaling;
            }
            rear_->to(Position::Normal, 放腿速度);
            rear_state_ = LiftState::Up5_放下ing;
        }
        break;
    case LiftState::Up5_放下ing:
        if (rear_->isFinished())
        {
            rear_state_ = LiftState::Done;
            rear_->setGrounding(true); // 着地
        }
        break;
    case LiftState::Up6_等待恢复到Normal:
    case LiftState::Up7_恢复到Normaling:
        break;
    case LiftState::Down1_等待放下:
        if (currentRelativeX() > startDistance2Step_ + AbsWheelInnerEdgeX)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + AbsWheelOuterEdgeX + SafeDistance)
        {
            rear_->to(Position::UpStep, NoloadLimit);
            rear_state_ = LiftState::Down2_放下ing;
        }
        break;
    case LiftState::Down2_放下ing:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true);
            rear_state_ = LiftState::Down3_等待回收到正常位置;
        }
        break;
    case LiftState::Down3_等待回收到正常位置:
        if (should_reset_)
        {
            if (currentRelativeX() > startDistance2Step_ + HalfChassisDistanceX + SafeDistance)
            {
                rear_->to(Position::Normal, OnloadLimit);
                rear_state_ = LiftState::Down4_回收ing;
            }
        }
        else
        {
            // 如果不复位底盘，则在底盘完成后立即完成
            if (chassis_state_ == ChassisState::Done)
            {
                rear_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::Down4_回收ing:
        if (rear_->isFinished())
        {
            rear_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }
}

[[noreturn]] void Step::loop()
{
    for (;;)
    {
        osThreadFlagsWait(FlagStart, osFlagsWaitAll, osWaitForever);
        osThreadFlagsClear(FlagResume);

        while (isRunning())
        {
            update();
            osDelay(1);
        }
    }
}
}; // namespace Action
