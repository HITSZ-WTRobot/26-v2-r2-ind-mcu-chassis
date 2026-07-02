/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "HWT101CT.hpp"
#include "dji.hpp"
#include "dm.hpp"
#include "suction/SuctionCup.hpp"
#include "usart.h"
#include "chassis/switch.hpp"

#include <cstddef>
#include <cstdint>

namespace config::uart
{
constexpr auto AuxControllerHost = &huart1;
constexpr auto SensorGyroYaw     = &huart2;
constexpr auto UpperHost         = &huart3;
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

// 执行器声明
namespace Suction
{

/**
 * Grip 吸盘
 * 位于 grip 机构侧，气泵由 RELAY1 控制，当前不连接气压计。
 */
inline ::Suction::SuctionCup* grip{};

/**
 * 腹部吸盘
 * 位于车体腹部，气泵由 RELAY2 控制，当前不连接气压计。
 */
inline ::Suction::SuctionCup* abdomen{};
} // namespace Suction

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
 * - lift[2], lift[3]：后侧抬升，CAN2 id1 = 1, 2
 *
 * TODO: 补充每台 lift 电机的具体左右对应关系与实际减速比。
 */
inline motors::DJIMotor* lift[4]{};

/**
 * Grip 电机。
 * 当前映射：
 * - grip_arm：DM4310，CAN2 id0 = 0x09，内部速度模式
 * - grip_turn：DM2325，CAN2 id0 = 0x0A，MIT 力矩模式
 */
inline motors::DMMotor* grip_arm{};
inline motors::DMMotor* grip_turn{};
} // namespace Motor

namespace Switch
{
/// 4 个红外 GPIO switch，数组顺序就是 ActionState bit12..15 的反馈顺序。
inline InfraredSwitch infrared_switch[4]{
    GpioPinWithName(EXTI3),
    GpioPinWithName(EXTI2),
    GpioPinWithName(EXTI1),
    GpioPinWithName(EXTI0),
};
}

void init();
void dm_motor_pings();
void update_1kHz();
} // namespace Device
