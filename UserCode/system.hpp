#pragma once
#include "IChassisDef.hpp"
#include "project_parts.hpp"

namespace System
{

namespace Init
{

/**
 * 这里保存的是“最终系统初始化门槛”需要用到的上位机状态。
 *
 * 注意它和 Connection::waitAll() 的职责不同：
 * - Connection 只等本地硬件链路，为 enable / calibration 服务；
 * - System::Init 负责等上位机辨识、首帧定位等高层条件。
 */

// 是否已完成上位机串口辨识。
inline bool upperHostIdentified = false;
// 是否接收到可用于初始化定位的上位机首帧位置数据。
inline bool postureReceived = false;
// 上位机给出的初始世界系位置。
inline chassis::Posture posture;
// 首帧位姿到达后的初始化钩子，由 chassis.cpp 提供实现。
extern void initPostureReceive();

/**
 * 判断系统初始化是否完成。
 *
 * 规则：
 * - 若启用了上位机串口辨识初始化，则必须先收到任意合法上位机帧；
 * - 若启用了上位机定位包，则必须等待首个满足当前接入条件的上位机位姿；
 * - 上述要求只影响“初始化完成”判定，不参与下位机本地校准前的连接等待。
 */
inline bool inited()
{
    if constexpr (ProjectParts::NeedUpperHostIdentifyInit)
    {
        if (!upperHostIdentified)
            return false;
    }

    if constexpr (ProjectParts::NeedUpperHostInitPosture)
        return postureReceived;

    return true;
}
} // namespace Init
} // namespace System
