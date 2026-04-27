/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "IndLiftMecanum4.hpp"
#include "IChassisLoc.hpp"
#include "JustEncoder.hpp"
#include "LocEKF.hpp"
#include "Master.hpp"

#include <cstdint>

namespace Chassis
{

using ChassisController = chassis::controller::Master;
using ChassisLoc        = chassis::loc::IChassisLoc;
using ChassisLocEKF     = chassis::loc::LocEKF<256>;
using ChassisLocEncoder = chassis::loc::JustEncoder;
using ChassisMotion     = IndLiftMecanum4;

inline ChassisLoc*        loc{};
inline ChassisLocEKF*     loc_ekf{};
inline ChassisLocEncoder* loc_encoder{};
inline ChassisController* ctrl{};
inline ChassisMotion*     motion{};

void init();

void initLocCtrl(const chassis::Posture& init_posture);
void initStandaloneLocCtrl();

void updateLidar(const chassis::Posture& posture, uint32_t ticks);

void enable();

void update_1kHz();

void update_100Hz();

} // namespace Chassis
