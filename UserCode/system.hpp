#pragma once
#include "IChassisDef.hpp"
#include "project_parts.hpp"

namespace System
{

namespace Init
{

/* 定位初始位置 */
// 是否已完成上位机串口辨识
inline bool upperHostIdentified = false;
// 是否接收到上位机初始位置数据
inline bool postureReceived = false;
// 初始位置
inline chassis::Posture posture;
// hook
extern void initPostureReceive();

/**
 * 判断系统初始化是否完成。
 *
 * 规则：
 * - 若启用了上位机串口辨识初始化，则必须先收到任意合法上位机帧；
 * - 若启用了上位机定位包，则必须等待首帧上位机位姿；
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
