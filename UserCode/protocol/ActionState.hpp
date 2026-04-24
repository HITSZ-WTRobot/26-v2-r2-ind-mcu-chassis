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
 * 两字节动作状态表，bit=1 表示对应状态当前成立。
 *
 * 当前版本暂未定义具体动作位；bit0~15 全部预留。
 */
inline volatile uint16_t table{};

void updateTable();
} // namespace Protocol::ActionState
