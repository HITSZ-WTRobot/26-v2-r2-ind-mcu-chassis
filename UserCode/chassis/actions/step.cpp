/**
 * @file    step.cpp
 * @author  syhanjin
 * @date    2026-04-03
 */
#include "step.hpp"
#include "chassis/chassis.hpp"

namespace Action
{
namespace
{
constexpr uint32_t FlagStart  = 1 << 0;
constexpr uint32_t FlagResume = 1 << 1;

using namespace Chassis::Config::ChassisInfo;
constexpr float 前边缘到中前辅助轮前边缘 = ChassisFrontEdge - AuxWheelMidFrontX -
                                           AuxiliaryWheelRadius;
constexpr float 前边缘到后辅助轮前边缘 = ChassisFrontEdge - AuxWheelRearX - AuxiliaryWheelRadius;
constexpr float 前边缘到前轮前边缘     = ChassisFrontEdge - WheelFrontEdgeFront;
constexpr float 前边缘到后轮前边缘     = ChassisFrontEdge - WheelRearEdgeFront;
constexpr float 前边缘到前轮后边缘     = ChassisFrontEdge - WheelFrontEdgeRear;
constexpr float 前边缘到后轮后边缘     = ChassisFrontEdge - WheelRearEdgeRear;

constexpr float 前边缘到前辅助轮后边缘   = 2 * AuxiliaryWheelRadius;
constexpr float 前边缘到中后辅助轮后边缘 = ChassisFrontEdge - AuxWheelMidRearX +
                                           AuxiliaryWheelRadius;

constexpr Chassis::Config::Limit 抬升速度{ Chassis::Config::Lift::MaxSpeed,
                                           Chassis::Config::Lift::MaxOnloadAccel,
                                           Chassis::Config::Lift::MaxOnloadAccel * 50 };

constexpr Chassis::Config::Limit 收腿速度{ Chassis::Config::Lift::MaxSpeed,
                                           Chassis::Config::Lift::MaxNoloadAccel,
                                           Chassis::Config::Lift::MaxNoloadAccel * 50 };

constexpr Chassis::Config::Limit 放腿速度{ Chassis::Config::Lift::MaxSpeed,
                                           Chassis::Config::Lift::MaxOnloadAccel,
                                           Chassis::Config::Lift::MaxOnloadAccel * 50 };
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

/**
 *
 * @param startDistance2Step 开始时车体前边缘距离台阶的距离 unit: m
 * @param endDistance2Step 结束时车体边缘距离台阶距离 unit: m
 * @param dir 机构相对台阶的朝向
 * @param willTake 中间是否会停下来取卷轴
 * @param stairs 上下台阶方向
 *
 */
void Step::start(const float     startDistance2Step,
                 const float     endDistance2Step,
                 const Direction dir,
                 bool            willTake,
                 const Stairs    stairs)
{
    if (isRunning())
        return;

    will_take_          = willTake;
    direction_          = dir;
    stairs_             = stairs;
    x_sign_             = dir == Direction::Front ? 1 : -1;
    startDistance2Step_ = startDistance2Step;
    endDistance2Step_   = endDistance2Step;

    start_pos_ = Chassis::loc->postureInWorld();

    if (dir == Direction::Front)
    {
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }
    else
    {
        rear_  = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front);
        front_ = &Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear);
    }

    if (stairs == Stairs::Up)
    {
        chassis_state_    = ChassisState::A等待底盘到达台阶高度_同时往台阶位移;
        lift_front_state_ = LiftState::A抬升ing;
        lift_rear_state_  = LiftState::A抬升ing;

        front_->to(Chassis::Config::Lift::Position::UpStep, 抬升速度);
        rear_->to(Chassis::Config::Lift::Position::UpStep, 抬升速度);
        Chassis::ctrl->setTargetPostureInWorld(relativePosture(startDistance2Step_ - SafeDistance));
    }
    else
    {
        chassis_state_    = ChassisState::E前进使中前辅助轮到达台阶边缘_等待前轮放下;
        lift_front_state_ = LiftState::D等待放下;
        lift_rear_state_  = LiftState::D等待放下;

        Chassis::ctrl->setTargetPostureInWorld(
                relativePosture(startDistance2Step_ + 前边缘到中前辅助轮前边缘 - 3 * SafeDistance));
    }
    osThreadFlagsClear(FlagResume);
    osThreadFlagsSet(task_, FlagStart);
}

void Step::resume()
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
    // 上台阶
    case ChassisState::A等待底盘到达台阶高度_同时往台阶位移:
        if (front_->isFinished() && rear_->isFinished())
        {
            // TODO: 不同阶段底盘的速度限制应当不同
            // TODO: 如果 will_take_ = false, 则该目标值设置应当带末速度
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + 前边缘到前轮前边缘 - SafeDistance));
            chassis_state_ = ChassisState::B前进将前辅助轮悬于台阶上方_等待前轮收起;

            if (will_take_)
                osThreadFlagsWait(FlagResume, osFlagsWaitAll, osWaitForever);
        }
        break;
    case ChassisState::B前进将前辅助轮悬于台阶上方_等待前轮收起:
        if (lift_front_state_ == LiftState::D等待放下)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + 前边缘到后轮前边缘 - SafeDistance));
            chassis_state_ = ChassisState::C前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起;
        }
        break;
    case ChassisState::C前进将中辅助轮悬于台阶上方_等待前轮放下_等待后轮收起:
        if (lift_front_state_ == LiftState::Done && lift_rear_state_ == LiftState::D等待放下)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + endDistance2Step_ + ChassisDistanceX));
            chassis_state_ = ChassisState::D前进使底盘完全登上台阶;
        }
        break;
    case ChassisState::D前进使底盘完全登上台阶:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    // 下台阶
    case ChassisState::E前进使中前辅助轮到达台阶边缘_等待前轮放下:
        if (lift_front_state_ == LiftState::H等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(relativePosture(
                    startDistance2Step_ + 前边缘到后辅助轮前边缘 - 3 * SafeDistance));

            chassis_state_ = ChassisState::F前进使后辅助轮到达台阶边缘_等待后轮放下;
        }
        break;
    case ChassisState::F前进使后辅助轮到达台阶边缘_等待后轮放下:
        if (lift_rear_state_ == LiftState::H等待回收到正常位置)
        {
            Chassis::ctrl->setTargetPostureInWorld(
                    relativePosture(startDistance2Step_ + endDistance2Step_ + ChassisDistanceX));

            chassis_state_ = ChassisState::G前进使底盘完全走下台阶_放下底盘;
        }
        break;
    case ChassisState::G前进使底盘完全走下台阶_放下底盘:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    case ChassisState::H前进使底盘完全走下台阶:
        if (Chassis::ctrl->isTrajectoryFinished())
        {
            chassis_state_ = ChassisState::Done;
        }
        break;
    case ChassisState::Done:
        break;
    }

    switch (lift_front_state_)
    {
    case LiftState::Idle:
        break;
    // 上台阶
    case LiftState::A抬升ing:
        if (front_->isFinished())
            lift_front_state_ = LiftState::B等待收起;
        break;
    case LiftState::B等待收起:
        // 等待取物
        if (will_take_)
            break;
        // TODO: 这里不应该这样扩大安全距离
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前辅助轮后边缘 + 3 * SafeDistance)
        {
            front_->to(Chassis::Config::Lift::Position::LiftMin, 收腿速度);
            front_->setGrounding(false); // 离地
            lift_front_state_ = LiftState::C收起ing;
        }
        break;
    case LiftState::C收起ing:
        if (front_->isFinished())
            lift_front_state_ = LiftState::D等待放下;
        break;
    case LiftState::D等待放下:
        // 前轮已经完全登上
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮后边缘 + 3 * SafeDistance)
        {
            front_->to(Chassis::Config::Lift::Position::Normal, 放腿速度);
            lift_front_state_ = LiftState::E放下ing;
        }
        break;
    case LiftState::E放下ing:
        if (front_->isFinished())
        {
            lift_front_state_ = LiftState::Done;
            front_->setGrounding(true); // 着地
        }
        break;
    // 下台阶
    case LiftState::F等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮前边缘)
        {
            // 离地判定
            front_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到前轮后边缘 + SafeDistance)
        {
            front_->to(Chassis::Config::Lift::Position::UpStep, 放腿速度);
            lift_front_state_ = LiftState::G放下ing;
        }
        break;
    case LiftState::G放下ing:
        if (front_->isFinished())
        {
            front_->setGrounding(true);
            lift_front_state_ = LiftState::H等待回收到正常位置;
        }
        break;
    case LiftState::H等待回收到正常位置:
        if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
        {
            if(stairs_ == Stairs::Down)
            {
                front_->to(Chassis::Config::Lift::Position::Normal);
                lift_front_state_ = LiftState::I回收ing;
            }
            else if(stairs_ == Stairs::Down_No_Rst)
            {
                lift_front_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::I回收ing:
        if (front_->isFinished())
        {
            lift_front_state_ = LiftState::Done;
        }
        break;
    case LiftState::Done:
        break;
    }

    switch (lift_rear_state_)
    {
    case LiftState::Idle:
        break;
    // 上台阶
    case LiftState::A抬升ing:
        if (rear_->isFinished())
            lift_rear_state_ = LiftState::B等待收起;
        break;
    case LiftState::B等待收起:
        // 等待取物，此处可以不加该判断，这里为了对称
        if (will_take_)
            break;
        // 中后辅助轮已登上台阶
        if (currentRelativeX() > startDistance2Step_ + 前边缘到中后辅助轮后边缘 + 3 * SafeDistance)
        {
            rear_->to(Chassis::Config::Lift::LiftMin, 收腿速度);
            rear_->setGrounding(false);
            lift_rear_state_ = LiftState::C收起ing;
        }
        break;
    case LiftState::C收起ing:
        if (rear_->isFinished())
            lift_rear_state_ = LiftState::D等待放下;
        break;
    case LiftState::D等待放下:
        // 后轮已经完全登上
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮后边缘 + 3 * SafeDistance)
        {
            rear_->to(Chassis::Config::Lift::Position::Normal, 放腿速度);
            lift_rear_state_ = LiftState::E放下ing;
        }
        break;
    case LiftState::E放下ing:
        if (rear_->isFinished())
        {
            lift_rear_state_ = LiftState::Done;
            rear_->setGrounding(true); // 着地
        }
        break;
    // 下台阶
    case LiftState::F等待放下:
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮前边缘)
        {
            // 离地判定
            rear_->setGrounding(false);
        }
        if (currentRelativeX() > startDistance2Step_ + 前边缘到后轮后边缘 + SafeDistance)
        {
            rear_->to(Chassis::Config::Lift::Position::UpStep, 放腿速度);
            lift_rear_state_ = LiftState::G放下ing;
        }
        break;
    case LiftState::G放下ing:
        if (rear_->isFinished())
        {
            rear_->setGrounding(true);
            lift_rear_state_ = LiftState::H等待回收到正常位置;
        }
        break;
    case LiftState::H等待回收到正常位置:
        if (currentRelativeX() > startDistance2Step_ + ChassisDistanceX + SafeDistance)
        {
            if(stairs_ == Stairs::Down)
            {
                rear_->to(Chassis::Config::Lift::Position::Normal);
                lift_rear_state_ = LiftState::I回收ing;
            }
            else if(stairs_ == Stairs::Down_No_Rst)
            {
                lift_front_state_ = LiftState::Done;
            }
        }
        break;
    case LiftState::I回收ing:
        if (rear_->isFinished())
        {
            lift_rear_state_ = LiftState::Done;
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