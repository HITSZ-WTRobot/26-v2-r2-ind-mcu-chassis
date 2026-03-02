/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "Mecanum4.hpp"
#include "Slave.hpp"

/**
 * 底盘对象
 */
using Chassis = chassis::controller::Slave<chassis::Mecanum4, 500>;
extern Chassis* chassis_;

void APP_Chassis_Update_100Hz();
void APP_Chassis_Update_1kHz();
void APP_Chassis_BeforeUpdate();
void APP_Chassis_Init();