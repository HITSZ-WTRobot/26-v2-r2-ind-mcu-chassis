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
}

void update_1kHz()
{
    motion->update_1kHz();
}

void init()
{
    using controllers::MotorVelController;

    motion = new IndLiftMecanum4();
}

void enable()
{
    if (!motion->enable())
        Error_Handler();
}

} // namespace Chassis

// 系统初始化钩子
void System::Init::initPostureReceive()
{
    // nothing to do.
}