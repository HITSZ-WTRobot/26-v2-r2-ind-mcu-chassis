/**
 * @file    PCFeedback.hpp
 * @author  syhanjin
 * @date    2026-05-08
 */
#pragma once

#include "PCProtocol.hpp"

namespace Protocol::Feedback
{

void registerProtocol(PCProtocol* protocol);
void startTask();

} // namespace Protocol::Feedback
