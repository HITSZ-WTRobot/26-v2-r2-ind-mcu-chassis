/**
 * @file    PCCommandDef.hpp
 * @author  syhanjin
 * @date    2026-05-09
 */
#pragma once
#include "crc.hpp"

using CRC16Modbus = crc::CRCX<16, 0x8005, 0xFFFF, true, true, 0x0000>;

constexpr uint32_t HeaderLen  = 2;
constexpr uint32_t PayloadLen = 1 + 2 * 6 + 4 + 2;
constexpr uint32_t FrameLen   = HeaderLen + PayloadLen;

constexpr uint8_t IdentifyInitByte = 0xAA;

/// 反馈帧：
/// AA BB | timestamp(uint32) | x*2000(int16) | y*2000(int16) | yaw*100(int16) |
/// frontHeight*2000(int16) | rearHeight*2000(int16) | action state(uint16, packed fields) |
/// connection state(uint16) | CRC16
constexpr uint32_t FeedbackPayloadLen = 4 + 2 * 5 + 2 + 2 + 2;
constexpr uint32_t FeedbackFrameLen   = HeaderLen + FeedbackPayloadLen;

enum class PCCommand : uint8_t
{
    /// 上位机对时信号
    /// TODO: 返回 Pong
    Ping = 0x01,

    /// 停止底盘
    StopChassis = 0x10,

    /// 设置底盘离地高度
    /// |       int16        |   uint16   |   uint16   | uint16 |   uint16   |
    /// | chassisHeight*2000 | v_max*1000 | a_max*100  | j_max  |  linkMode  |
    /// @note 上位机使用底盘离地高度；下位机 lift 零点为辅助轮接地位置。
    /// @note v_max / a_max / j_max 为 0 时，分别回退到带载参数。
    /// @note linkMode: 0=默认(PreviousCurve), 1=CurrentState, 2=PreviousCurve.
    SetChassisHeight = 0x11,

    /// Slave 模式下 push 轨迹点
    /// | int16  | int16  |     int16    |  int16  |  int16  |    int16    |
    /// | x*2000 | y*2000 | yaw(deg)*100 | vx*2000 | vy*2000 | wz(deg)*100 |
    SlavePushChassisTrajectory = 0x12,

    /// Master 模式下设置底盘目标位姿，轨迹从当前状态衔接
    /// | int16  | int16  |     int16    |    uint12   |    uint12   | uint12  | uint12  |
    /// | x*2000 | y*2000 | yaw(deg)*100 | xy_vmax*200 | xy_amax*200 | yaw_vmax | yaw_amax |
    /// @note 4 个 uint12 按高位优先连续打包：
    ///       [a11:a4] [a3:a0|b11:b8] [b7:b0] [c11:c4] [c3:c0|d11:d8] [d7:d0]
    SetMasterChassisTargetCurrentState = 0x13,

    /// Master 模式下设置底盘目标位姿，轨迹沿上一条曲线衔接
    /// 参数格式与 0x13 完全一致。
    SetMasterChassisTargetPreviousCurve = 0x14,

    /// Master 模式下设置底盘车体系速度
    /// | int16  | int16  |    int16    |  uint16 |  uint16 |  uint16 |
    /// | vx*2000| vy*2000| wz(deg)*100 | reserve | reserve | reserve |
    /// @note 速度按车体系解释，内部调用 `setVelocityInBody(..., false)`。
    SetMasterChassisVelocity = 0x15,

    /// 设置 GripPose
    /// |      int16       |      int16       |   uint16  | uint16  | uint16  | uint16  |
    /// | arm_pos(deg)*100 | turn_pos(deg)*100| clawMode  | reserve | reserve | reserve |
    /// @param clawMode: 0=保持当前夹爪状态，1=张开夹爪，2=闭合夹爪。
    SetGripPose = 0x16,

    /// 设置 Grip 到预设姿态
    /// |  uint16  | uint16  | uint16  | uint16  | uint16  | uint16  |
    /// | presetId | reserve | reserve | reserve | reserve | reserve |
    /// @param presetId: 0=Standby, 1=PrepareGrab, 2=Grab, 3=Docking,
    ///                  4=KfsPickup, 5=KfsStore, 6=KfsRelease.
    SetGripPresetPose = 0x17,

    /// 雷达位姿
    /// | int16  | int16  |     int16    |       uint32       |
    /// | x*2000 | y*2000 | yaw(deg)*100 | lidarTimestamp(ms) |
    LidarPosture = 0x21,

    /* 动作组 */
    /// 上 200mm 台阶
    /// |        int16       |      int16       |   uint16  |  uint16  |
    /// | startDistance*2000 | endDistance*2000 | direction | willTake |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前上台阶
    ///                   1 Backward，车头朝后上台阶
    /// @param willTake: 0 连贯上台阶;
    ///                  1 中途将停下来取卷轴，发送 StepUpResume 指令后继续执行上台阶动作
    StepUp200 = 0x30,
    /// 继续上台阶动作，200mm / 400mm 共用
    StepUpResume = 0x31,
    /// 下 200mm 台阶
    /// |        int16       |      int16       |   uint16  |    uint16   |
    /// | startDistance*2000 | endDistance*2000 | direction | shouldReset |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前下台阶
    ///                   1 Backward，车头朝后下台阶
    /// @param shouldReset: 1 下台阶之后底盘恢复到正常高度
    ///                     0 下台阶最后一步不回收底盘，底盘仍处于比台阶高的状态
    StepDown200 = 0x32,
    /// 上 400mm 台阶，数据格式与 StepUp200 相同
    StepUp400 = 0x33,
    /// 下 400mm 台阶，数据格式与 StepDown200 相同
    StepDown400 = 0x34,

    /// 取矛头
    /// |     int16   |     int16   |        int16      |   int16  |   int16  |      int16     |
    /// |target_x*2000|target_y*2000|target_yaw(deg)*100|end_x*2000|end_y*2000|end_yaw(deg)*100|
    TakeSpear = 0x40,
    /// 固定矛位取矛头
    /// | uint16  |    int16   |    int16   |       int16      | uint16  | uint16  |
    /// | spearId | end_x*2000 | end_y*2000 | end_yaw(deg)*100 | reserve | reserve |
    /// @param spearId: 固定矛位索引，对应 `Grip::Config::SpearGrab::TargetPoses`
    /// @note 后 2 个 `uint16` 数据位当前保留并忽略。
    TakeSpearById = 0x41,
    /// 启动 KFS 暂存动作组。
    StoreKFS = 0x42,
    /// 启动 KFS 释放动作组。
    ReleaseKFS = 0x43,
};
