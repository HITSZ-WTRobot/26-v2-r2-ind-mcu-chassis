/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "IndLiftMecanum4.hpp"
#include "LocEKF.hpp"
#include "Master.hpp"

namespace Chassis
{

using ChassisController = chassis::controller::Master;
using ChassisLoc        = chassis::loc::LocEKF;
using ChassisMotion     = IndLiftMecanum4;

inline ChassisLoc*        loc{};
inline ChassisController* ctrl{};
inline ChassisMotion*     motion{};

void init();

void initLocCtrl(const chassis::Posture& init_posture);

void enable();

void update_1kHz();

void update_100Hz();

} // namespace Chassis
