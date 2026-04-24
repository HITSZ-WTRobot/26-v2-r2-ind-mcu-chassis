#pragma once
#include "IChassisDef.hpp"
#include "project_parts.hpp"

namespace System
{

namespace Init
{

/* 定位初始位置 */
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
 * - 若启用了上位机定位包，则必须等待首帧上位机位姿；
 * - 否则认为无需等待外部初始位姿，底盘会走本地初始化路径。
 */
inline bool inited()
{
    if constexpr (ProjectParts::NeedUpperHostInitPosture)
        return postureReceived;

    return true;
}
} // namespace Init
} // namespace System
