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
    /// | startDistance*2000 | endDistance*2000 | direction | endHeight |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前上台阶
    ///                   1 Backward，车头朝后上台阶
    /// @param endHeight: 0 Low; 1 High
    StepUp200 = 0x30,
    /// 旧恢复上台阶命令，当前保留无动作。
    StepUpResume = 0x31,
    /// 下 200mm 台阶
    /// |        int16       |      int16       |   uint16  |    uint16   |
    /// | startDistance*2000 | endDistance*2000 | direction | endHeight |
    /// @param startDistance: 开始时车体中心距离台阶边缘的距离
    /// @param endDistance: 结束时车体中心距离台阶边缘的距离
    /// @param direction: 0 Forward，车头朝前下台阶
    ///                   1 Backward，车头朝后下台阶
    /// @param endHeight: 0 Low; 1 High
    StepDown200 = 0x32,
    /// 上 400mm 台阶，数据格式与 StepUp200 相同
    StepUp400 = 0x33,
    /// 下 400mm 台阶，数据格式与 StepDown200 相同
    StepDown400 = 0x34,

    /// 上R1台阶
    /// |        int16       |        int16       |          int16          |   uint16  |
    /// | stepTarget_x*2000  | stepTarget_y*2000  | stepTarget_yaw(deg)*100 | direction |
    /// @param direction: 0 Forward，车头朝前上台阶
    ///                   1 Backward，车头朝后上台阶
    StepUpR1 = 0x35,

    /// 上R1台阶（直接版本），使用当前位置作为收腿点
    /// |   uint16  | uint16  | uint16  | uint16  | uint16  | uint16  |
    /// | direction | reserve | reserve | reserve | reserve | reserve |
    /// @param direction: 0 Forward，1 Backward
    StepUpR1Direct = 0x36,

    /// `0x50..0x5F` 为平面台阶动作命令组，不在枚举中逐项展开，统一由命令分发层按位解析：
    /// cmd = 0x50 | type(1bit)<<3 | dir(1bit)<<2 | height(1bit)<<1 | finalHeight(1bit)
    /// type: 0=up, 1=down; dir: 0=Forward, 1=Backward; height: 0=Step200, 1=Step400.
    /// finalHeight: 0=Low, 1=High, up/down 统一表示动作结束后的底盘高度.
    /// |        int16       |        int16       |          int16          |
    /// | stepTarget_x*2000  | stepTarget_y*2000  | stepTarget_yaw(deg)*100 |
    /// |      int16     |      int16     |        int16        |
    /// | end_x*2000    | end_y*2000    | end_yaw(deg)*100    |

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
    /// 控制 Grip 吸盘开关
    /// | uint16  | uint16  | uint16  | uint16  | uint16  | uint16  |
    /// | on(0/1) | reserve | reserve | reserve | reserve | reserve |
    SetGripSuction = 0x44,
    /// 控制腹部吸盘开关
    /// | uint16  | uint16  | uint16  | uint16  | uint16  | uint16  |
    /// | on(0/1) | reserve | reserve | reserve | reserve | reserve |
    SetAbdomenSuction = 0x45,
    /// 独立控制夹爪开合，不改变 Grip 关节位置
    /// |  uint16  | uint16  | uint16  | uint16  | uint16  | uint16  |
    /// | clawMode | reserve | reserve | reserve | reserve | reserve |
    /// @param clawMode: 0=张开夹爪，1=闭合夹爪。
    SetGripClaw = 0x46,
};
