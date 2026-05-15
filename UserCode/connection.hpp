/**
 * @file    connection.hpp
 * @author  syhanjin
 * @date    2026-04-22
 */
#pragma once

#include <cstdint>

namespace Connection
{
/**
 * 连接状态位分配。
 *
 * 这张表是对外协议的一部分，新增 / 删除设备时必须同步更新这里、
 * connection.cpp 的刷新逻辑，以及对应的协议文档。
 */
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
    GripSuctionPressure   = 11,
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
 * - bit11 : grip suction pressure（仅在启用吸盘气压计编译开关时有效）
 * - bit12~13: 预留
 * - bit14 : upper_host_localization
 * - bit15 : upper_host
 *
 * 注意：上位机相关 bit 只用于观测，不参与 Connection::waitAll() 的本地硬件等待。
 */
inline volatile uint16_t table{};

/** @brief 刷新一次连接表，并按配置注册 I2C 周期发送设备。 */
void init();
/** @brief 从各设备对象读取当前连接状态，更新 table。 */
void updateTable();
/** @brief 当前编译形态要求的本地硬件是否都在线。 */
bool isAllConnected();
/** @brief 阻塞等待本地硬件连接完成。 */
void waitAll();
} // namespace Connection
