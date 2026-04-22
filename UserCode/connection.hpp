/**
 * @file    connection.hpp
 * @author  syhanjin
 * @date    2026-04-22
 */
#pragma once

#include <cstdint>

namespace Connection
{
enum class Bit : uint8_t
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
    UpperHost = 15,
};

/**
 * 两字节连接表，bit=1 表示对应对象已连接。
 *
 * 固定 bit 分配如下，未启用的对象对应 bit 恒为 0：
 * - bit0  : wheel[0]
 * - bit1  : wheel[1]
 * - bit2  : wheel[2]
 * - bit3  : wheel[3]
 * - bit4  : lift_front
 * - bit5  : lift_rear
 * - bit6  : grip_arm
 * - bit7  : grip_turn
 * - bit8  : gyro_yaw
 * - bit9~14: 预留
 * - bit15 : upper_host
 */
inline volatile uint16_t table{};

void updateTable();
bool isAllConnected();
void waitAll();
} // namespace Connection
