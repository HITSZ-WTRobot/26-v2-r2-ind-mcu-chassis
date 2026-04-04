/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "IndLiftMecanum4.hpp"
#include "JustEncoder.hpp"
#include "Master.hpp"

namespace Chassis
{

using ChassisController = chassis::controller::Master;
using ChassisLoc        = chassis::loc::JustEncoder;
using ChassisMotion     = IndLiftMecanum4;

inline ChassisLoc*        loc;
inline ChassisController* ctrl;
inline ChassisMotion*     motion;

// void motionInit();

// void locCtrlInit(const chassis::Posture& init_pos);

void init();

void enable();

void update_1kHz();

void update_100Hz();

} // namespace Chassis