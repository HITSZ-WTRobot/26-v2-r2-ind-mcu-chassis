/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "HWT101CT.hpp"
#include "dji.hpp"
#include "dm.hpp"
#include "usart.h"

namespace config::uart
{
constexpr auto SensorGyroYaw = &huart2;
constexpr auto UpperHost     = &huart3;
} // namespace config::uart

namespace Device
{
// 传感器声明
namespace Sensor
{

/**
 * 陀螺仪
 * HWT101CT， UART2
 */
inline sensors::gyro::HWT101CT* gyro_yaw{};
} // namespace Sensor

// 电机声明
namespace Motor
{
/**
 * 底盘使用的轮子
 * 大疆电机 3508, CAN1 ID: 1 ~ 4
 */
inline motors::DJIMotor* wheel[4]{};

/**
 * 升降电机
 * 以使得底盘抬升的方向为正
 */
inline motors::DMMotor* lift_front{}; // 前抬升
inline motors::DMMotor* lift_rear{};  // 后抬升
} // namespace Motor

void init();
bool isAllConnected();
void waitAllConnections();
void update_1kHz();
} // namespace Device
