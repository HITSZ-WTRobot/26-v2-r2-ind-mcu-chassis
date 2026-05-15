/**
 * @file    PCCommandHandler.hpp
 * @author  syhanjin
 * @date    2026-05-08
 */
#pragma once

// 命令处理器把“已经通过 CRC 的协议帧”翻译成模块调用。
// 它不直接读 UART，也不直接发反馈。

#include "PCProtocol.hpp"

namespace Protocol::CommandHandler
{

bool enqueueFrame(const Frame& frame);
void startTask();

} // namespace Protocol::CommandHandler
