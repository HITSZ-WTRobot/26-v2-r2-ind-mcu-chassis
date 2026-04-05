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

constexpr float 前边缘到前轮前边缘 = ChassisFrontEdge - WheelFrontEdgeFront;
constexpr float 前边缘到后轮前边缘 = ChassisFrontEdge - WheelRearEdgeFront;
constexpr float 前边缘到前轮后边缘 = ChassisFrontEdge - WheelFrontEdgeRear;
constexpr float 前边缘到后轮后边缘 = ChassisFrontEdge - WheelRearEdgeRear;

constexpr float 前边缘到前辅助轮后边缘 = 2 * AuxiliaryWheelRadius;
constexpr float 前边缘到后辅助轮前边缘 = ChassisFrontEdge - AuxWheelRearX - AuxiliaryWheelRadius;

constexpr float 前边缘到中前辅助轮前边缘 = ChassisFrontEdge - AuxWheelMidFrontX -
                                           AuxiliaryWheelRadius;
constexpr float 前边缘到中后辅助轮后边缘 = ChassisFrontEdge - AuxWheelMidRearX +
                                           AuxiliaryWheelRadius;

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
 * @param startDistance2Step 开始时车体前边缘距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体边缘距离台阶距离 unit: m
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
    Chassis::ctrl->setTargetPostureInWorld(relativePosture(startDistance2Step_ - SafeDistance));

    osThreadFlagsClear(FlagResume);
    osThreadFlagsSet(task_, FlagStart);
}
/**
 * 下台阶
 * @param startDistance2Step 开始时车体前边缘距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体边缘距离台阶距离 unit: m
 * @param dir 上台阶方向
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

    chassis_state_ = ChassisState::Down1_前进使中前辅助轮到达台阶边缘_等待前轮放下;
    front_state_   = LiftState::Down1_等待放下;
    rear_state_    = LiftState::Down1_等待放下;

    Chassis::ctrl->setTargetPostureInWorld(
            relativePosture(startDistance2Step_ + 前边缘到中前辅助轮前边缘 - 3 * SafeDistance));

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
                    relativePosture(startDistance2Step_ + 前边缘到前轮前边缘 - SafeDistance),
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
                    relativePosture(startDistance2Step_ + 前边缘到后轮前边缘 - SafeDistance),
                    Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Up3_前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起;
        }
        break;
    case ChassisState::Up3_前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起:
        if (front_state_ == LiftState::Done && rear_state_ == LiftState::Up4_等待放下)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + endDistance2Step_ + ChassisDistanceX),
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
    case ChassisState::Down1_前进使中前辅助轮到达台阶边缘_等待前轮放下:
        if (front_state_ == LiftState::Down3_等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld( //
                    relativePosture(startDistance2Step_ + 前边缘到后辅助轮前边缘 -
                                    3 * SafeDistance),
                    Master::TrajectoryLinkMode::PreviousCurve);

            chassis_state_ = ChassisState::Down2_前进使后辅助轮到达台阶边缘_等待后轮放下;
        }
        break;
    case ChassisState::Down2_前进使后辅助轮到达台阶边缘_等待后轮放下:
        if (rear_state_ == LiftState::Down3_等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + endDistance2Step_ + ChassisDistanceX),
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
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前辅助轮后边缘 + 3 * SafeDistance)
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
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮后边缘 + 3 * SafeDistance)
        {
            front_->to(Position::Normal, 放腿速度);
            front_state_ = LiftState::Up5_放下ing;
        }
        break;
    case LiftState::Up5_放下ing:
        if (front_->isFinished())
        {
            front_state_ = LiftState::Done;
            front_->setGrounding(true); // 着地
        }
        break;
    case LiftState::Down1_等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮前边缘)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮后边缘 + SafeDistance)
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
            if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
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
        if (currentRelativeX() > startDistance2Step_ + 前边缘到中后辅助轮后边缘 + 3 * SafeDistance)
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
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮后边缘 + 3 * SafeDistance)
        {
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
    case LiftState::Down1_等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮前边缘)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮后边缘 + SafeDistance)
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
            if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
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

        while (!isFinished())
        {
            update();
            osDelay(1);
        }
    }
}
}; // namespace Action