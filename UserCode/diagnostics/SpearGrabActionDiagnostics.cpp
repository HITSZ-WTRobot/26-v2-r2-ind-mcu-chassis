/**
 * @file    SpearGrabActionDiagnostics.cpp
 * @brief   SpearGrab action diagnostic helpers.
 */
#include "SpearGrabActionDiagnostics.hpp"

namespace Diagnostics::SpearGrabAction
{

SpearGrabDiagnostics last_spear_grab_diagnostics;

void report(const Context& ctx, const Reason reason)
{
    last_spear_grab_diagnostics = { ctx, reason, {}, {} };
}

} // namespace Diagnostics::SpearGrabAction
