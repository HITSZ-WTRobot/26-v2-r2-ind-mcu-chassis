/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "HWT101CT.hpp"
#include "XGZP6847DDevice.hpp"
#include "dji.hpp"
#include "usart.h"

#include <cstddef>
#include <cstdint>

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

/**
 * Grip 吸盘气压计
 * XGZP6847D， I2C2
 */
inline XGZP6847DDevice* grip_suction_pressure{};
} // namespace Sensor

// 电机声明
namespace Motor
{
/**
 * 底盘使用的轮子。
 * 当前映射：
 * - wheel[0], wheel[1]：前轮驱动，CAN1 id1 = 1, 2
 * - wheel[2], wheel[3]：后轮驱动，CAN2 id1 = 3, 4
 */
inline motors::DJIMotor* wheel[4]{};

/**
 * 升降电机。
 * 当前按“前侧两台 + 后侧两台”的物理分组保存：
 * - lift[0], lift[1]：前侧抬升，CAN1 id1 = 3, 4
 * - lift[2], lift[3]：后侧抬升，CAN2 id1 = 5, 6
 *
 * TODO: 补充每台 lift 电机的具体左右对应关系与实际减速比。
 */
inline motors::DJIMotor* lift[4]{};

/**
 * 矛头电机
 */
inline motors::DJIMotor* grip_arm{};
inline motors::DJIMotor* grip_turn{};
} // namespace Motor

void init();
void update_1kHz();
} // namespace Device
