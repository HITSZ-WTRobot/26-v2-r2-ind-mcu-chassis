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

#include <cstdint>

namespace config::uart
{
constexpr auto SensorGyroYaw = &huart2;
constexpr auto UpperHost     = &huart3;
} // namespace config::uart

namespace Device
{
enum class ConnectionBit : uint8_t
{
    Wheel0    = 0,
    Wheel1    = 1,
    Wheel2    = 2,
    Wheel3    = 3,
    LiftFront = 4,
    LiftRear  = 5,
    GripArm   = 6,
    GripTurn  = 7,
    GyroYaw   = 8,
};

/**
 * 两字节设备连接表，bit=1 表示对应设备已连接。
 *
 * 固定 bit 分配如下，未启用的设备对应 bit 恒为 0：
 * - bit0  : wheel[0]
 * - bit1  : wheel[1]
 * - bit2  : wheel[2]
 * - bit3  : wheel[3]
 * - bit4  : lift_front
 * - bit5  : lift_rear
 * - bit6  : grip_arm
 * - bit7  : grip_turn
 * - bit8  : gyro_yaw
 * - bit9~15: 预留
 */
inline volatile uint16_t connection_table{};

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

/**
 * 矛头电机
 */
inline motors::DJIMotor* grip_arm{};
inline motors::DJIMotor* grip_turn{};
} // namespace Motor

void init();
void updateConnectionTable();
bool isAllConnected();
void waitAllConnections();
void update_1kHz();
} // namespace Device
