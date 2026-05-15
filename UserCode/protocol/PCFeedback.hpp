/**
 * @file    PCFeedback.hpp
 * @author  syhanjin
 * @date    2026-05-08
 */
#pragma once

// 反馈系统只负责两件事：
// - 记录有哪些协议实例需要收反馈；
// - 周期性打包并发送统一格式的反馈帧。

#include "PCProtocol.hpp"

namespace Protocol::Feedback
{

void registerProtocol(PCProtocol* protocol);
/** @brief 启动反馈发送线程。 */
void startTask();

} // namespace Protocol::Feedback
