/**
 * @file    PCCommandHandler.hpp
 * @author  syhanjin
 * @date    2026-05-08
 */
#pragma once

#include "PCProtocol.hpp"

namespace Protocol::CommandHandler
{

bool enqueueFrame(const Frame& frame);
void startTask();

} // namespace Protocol::CommandHandler
