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
// UART 端口的硬件映射集中放在这里，其他模块只引用这些语义名字。
constexpr auto AuxControllerHost = &huart1;
constexpr auto SensorGyroYaw     = &huart2;
constexpr auto UpperHost         = &huart3;
} // namespace config::uart

namespace Device
{
// 传感器声明。这里保存的是全局单例式指针，具体创建在 device.cpp。
namespace Sensor
{

/**
 * 航向陀螺仪。
 *
 * 型号 HWT101CT，挂在 UART2。
 */
inline sensors::gyro::HWT101CT* gyro_yaw{};

/**
 * Grip 吸盘气压计。
 *
 * 型号 XGZP6847D，挂在共享 I2C2。
 */
inline XGZP6847DDevice* grip_suction_pressure{};
} // namespace Sensor

// 电机声明。对象由 Device::init() 按编译开关创建。
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
 * 这个分组和 `LiftSide` 的“前侧 / 后侧”控制模型一致。
 */
inline motors::DJIMotor* lift[4]{};

/**
 * grip 大臂电机。
 */
inline motors::DJIMotor* grip_arm{};
/**
 * grip 转向电机。
 */
inline motors::DJIMotor* grip_turn{};
} // namespace Motor

/** @brief 创建当前编译形态需要的传感器、电机和总线回调。 */
void init();
/** @brief 1 kHz 集中发送 DJI 电机电流指令。 */
void update_1kHz();
} // namespace Device
