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
 * - bit0..1   Step 状态：Idle / Done / Running / WaitingTake(旧保留值)
 * - bit2..3   底盘模式：Stop / Velocity / Position / Slave
 * - bit4      底盘曲线是否完成（时间结束且跟踪误差在 Master 跟踪阈值内）
 * - bit5..6   Lift 状态：Calibrating / Running / Ready / NotEnabled
 * - bit7..9   Grip 状态：Calibrating / TakingSpear / KfsStore /
 *               KfsRelease / Done / Running
 * - bit10..11 离线轨迹状态：Idle / Running / Finished / Interrupted
 * - bit12..15 4 个红外 switch 触发状态，按 Device::Switch::infrared_switch[0..3] 顺序打包
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
    WaitingTake = 3u, /// 旧等待取卷轴流程保留值，当前不会主动产生
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
    Done        = 5u, /// 动作执行完成 / 空闲
    Running     = 6u, /// 直接姿态控制运行中（SetGripPose / SetGripPresetPose 等）
};

/**
 * 离线轨迹跟随状态
 */
enum class TrajectoryOfflineState : uint16_t
{
    Idle        = 0u, /// 无离线轨迹运行
    Running     = 1u, /// 离线轨迹执行中
    Finished    = 2u, /// 离线轨迹已完成
    Interrupted = 3u, /// 离线轨迹被中断
};

namespace Layout
{
inline constexpr uint16_t StepShift                   = 0u;
inline constexpr uint16_t ChassisModeShift            = 2u;
inline constexpr uint16_t ChassisCurveFinishedBit     = 4u;
inline constexpr uint16_t LiftShift                   = 5u;
inline constexpr uint16_t GripShift                   = 7u;
inline constexpr uint16_t TrajectoryOfflineStateShift = 10u;
inline constexpr uint16_t InfraredSwitchStateShift    = 12u;
inline constexpr uint16_t StepMask                    = 0x3u << StepShift;
inline constexpr uint16_t ChassisModeMask             = 0x3u << ChassisModeShift;
inline constexpr uint16_t ChassisCurveFinishedMask    = 0x1u << ChassisCurveFinishedBit;
inline constexpr uint16_t LiftMask                    = 0x3u << LiftShift;
inline constexpr uint16_t GripMask                    = 0x7u << GripShift;
inline constexpr uint16_t TrajectoryOfflineStateMask  = 0x3u << TrajectoryOfflineStateShift;
inline constexpr uint16_t InfraredSwitchStateMask     = 0xFu << InfraredSwitchStateShift;
} // namespace Layout

constexpr uint16_t pack(const StepStatus             step,
                        const ChassisMode            chassis_mode,
                        const bool                   chassis_curve_finished,
                        const LiftStatus             lift,
                        const GripStatus             grip,
                        const TrajectoryOfflineState trajectory_offline_state,
                        const uint16_t               infrared_switch_state)
{
    return static_cast<uint16_t>(
            (static_cast<uint16_t>(step) << Layout::StepShift) |
            (static_cast<uint16_t>(chassis_mode) << Layout::ChassisModeShift) |
            ((chassis_curve_finished ? 1u : 0u) << Layout::ChassisCurveFinishedBit) |
            (static_cast<uint16_t>(lift) << Layout::LiftShift) |
            (static_cast<uint16_t>(grip) << Layout::GripShift) |
            (static_cast<uint16_t>(trajectory_offline_state)
             << Layout::TrajectoryOfflineStateShift) |
            ((infrared_switch_state & 0xFu) << Layout::InfraredSwitchStateShift));
}

void updateTable();
} // namespace Protocol::ActionState
