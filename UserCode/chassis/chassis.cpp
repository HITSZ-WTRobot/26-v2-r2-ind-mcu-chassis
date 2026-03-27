/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"
#include "system.hpp"
#include "device.hpp"
#include "JustEncoder.hpp"
#include "Config.hpp"

namespace Chassis
{

void update_100Hz()
{
    motion->update_100Hz();
    ctrl->profileUpdate(0.01);
}

void update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    loc->update(0.001);
    prescaler_500Hz++;
    if (prescaler_500Hz >= 2)
    {
        ctrl->errorUpdate();
        prescaler_500Hz = 0;
    }
    ctrl->controllerUpdate();

    motion->update_1kHz();
}

void init()
{
    using controllers::MotorVelController;

    motion = new IndLiftMecanum4();

    loc = new chassis::loc::JustEncoder(*motion);

    // slave mode
    // ctrl = new ChassisController(*motion,
    //                              *loc,
    //                              { {
    //                                      .vx = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
    //                                      .vy = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
    //                                      .wz = { .Kp = 30, .Kd = 4.0f, .abs_output_max = 25.0f },
    //                              } });

    ctrl = new ChassisController(*motion, *loc, Config::Control::masterCfg);
}

void enable()
{
    if (!ctrl->enable())
        Error_Handler();
}

} // namespace Chassis

// 系统初始化钩子
void System::Init::initPostureReceive()
{
    // nothing to do.
}