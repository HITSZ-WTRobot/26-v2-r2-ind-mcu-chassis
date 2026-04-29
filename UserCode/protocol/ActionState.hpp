/**
 * @file    ActionState.hpp
 * @author  syhanjin
 * @date    2026-04-23
 */
#pragma once

#include <cstdint>

namespace Protocol::ActionState
{
/**
 * 两字节动作状态表，按固定子字段打包：
 *
 * - bit0..1   Step 状态：Idle / Done / Running / WaitingTake
 * - bit2..3   底盘模式：Stop / Velocity / Position / Slave
 * - bit4      底盘曲线是否完成
 * - bit5..6   Lift 状态：Calibrating / Running / Ready / NotEnabled
 * - bit7..9   Grip 状态：Calibrating / TakingSpear / KfsStore /
 *               KfsRelease / Idle / Done
 * - bit10     Grip suction 是否检测到物体（仅在启用吸盘气压计时有效）
 * - bit11..15 预留
 */
inline volatile uint16_t table{};

/** @brief 启动 ActionState 的 50 Hz 低优先级刷新任务。 */
void init();

/**
 * 台阶动作状态
 */
enum class StepStatus : uint16_t
{
    Idle        = 0u, /// 空闲
    Done        = 1u, /// 动作完成
    Running     = 2u, /// 正在执行动作
    WaitingTake = 3u, /// 正在等待取卷轴
};

/**
 * 底盘控制状态
 */
enum class ChassisMode : uint16_t
{
    Stop     = 0u, /// 静止状态
    Velocity = 1u, /// Master 速度控制模式
    Position = 2u, /// Master 位置控制模式
    Slave    = 3u, /// 从机控制模式
};

enum class LiftStatus : uint16_t
{
    Calibrating = 0u, /// 校准中
    Running     = 1u, /// 正在执行（变更底盘高度）
    Ready       = 2u, /// 就位
    NotEnabled  = 3u, /// 保留，
};

enum class GripStatus : uint16_t
{
    Calibrating = 0u, /// 校准中
    TakingSpear = 1u, /// 取矛头中
    KfsStore    = 2u, /// 暂存卷轴中（从腹部吸取卷轴暂存）
    KfsRelease  = 3u, /// 释放卷轴中（从尾部释放卷轴到腹部）
    Idle        = 4u, /// 无动作
    Done        = 5u, /// 动作执行完成
};

namespace Layout
{
inline constexpr uint16_t StepShift                = 0u;
inline constexpr uint16_t ChassisModeShift         = 2u;
inline constexpr uint16_t ChassisCurveFinishedBit  = 4u;
inline constexpr uint16_t LiftShift                = 5u;
inline constexpr uint16_t GripShift                = 7u;
inline constexpr uint16_t GripSuctionHasObjectBit  = 10u;
inline constexpr uint16_t StepMask                 = 0x3u << StepShift;
inline constexpr uint16_t ChassisModeMask          = 0x3u << ChassisModeShift;
inline constexpr uint16_t ChassisCurveFinishedMask = 0x1u << ChassisCurveFinishedBit;
inline constexpr uint16_t LiftMask                 = 0x3u << LiftShift;
inline constexpr uint16_t GripMask                 = 0x7u << GripShift;
inline constexpr uint16_t GripSuctionHasObjectMask = 0x1u << GripSuctionHasObjectBit;
} // namespace Layout

constexpr uint16_t pack(const StepStatus  step,
                        const ChassisMode chassis_mode,
                        const bool        chassis_curve_finished,
                        const LiftStatus  lift,
                        const GripStatus  grip,
                        const bool        grip_suction_has_object)
{
    return static_cast<uint16_t>(
            (static_cast<uint16_t>(step) << Layout::StepShift) |
            (static_cast<uint16_t>(chassis_mode) << Layout::ChassisModeShift) |
            ((chassis_curve_finished ? 1u : 0u) << Layout::ChassisCurveFinishedBit) |
            (static_cast<uint16_t>(lift) << Layout::LiftShift) |
            (static_cast<uint16_t>(grip) << Layout::GripShift) |
            ((grip_suction_has_object ? 1u : 0u) << Layout::GripSuctionHasObjectBit));
}

void updateTable();
} // namespace Protocol::ActionState
