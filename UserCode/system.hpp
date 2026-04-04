#pragma once
#include "IChassisDef.hpp"

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

// 初始化完成函数
inline bool inited()
{
    return postureReceived;
}
} // namespace Init
} // namespace System