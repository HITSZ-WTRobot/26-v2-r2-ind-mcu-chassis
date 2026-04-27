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
    Wheel0                = 0,
    Wheel1                = 1,
    Wheel2                = 2,
    Wheel3                = 3,
    LiftFront0            = 4,
    LiftFront1            = 5,
    LiftRear0             = 6,
    LiftRear1             = 7,
    GripArm               = 8,
    GripTurn              = 9,
    GyroYaw               = 10,
    UpperHostLocalization = 14,
    UpperHost             = 15,
};

/**
 * 两字节连接表，bit=1 表示对应对象已连接。
 *
 * 固定 bit 分配如下，未启用的对象对应 bit 恒为 0：
 * - bit0  : wheel[0]
 * - bit1  : wheel[1]
 * - bit2  : wheel[2]
 * - bit3  : wheel[3]
 * - bit4  : lift[0]（前侧抬升 0）
 * - bit5  : lift[1]（前侧抬升 1）
 * - bit6  : lift[2]（后侧抬升 0）
 * - bit7  : lift[3]（后侧抬升 1）
 * - bit8  : grip_arm
 * - bit9  : grip_turn
 * - bit10 : gyro_yaw
 * - bit11~13: 预留
 * - bit14 : upper_host_localization
 * - bit15 : upper_host
 */
inline volatile uint16_t table{};

void updateTable();
bool isAllConnected();
void waitAll();
} // namespace Connection
