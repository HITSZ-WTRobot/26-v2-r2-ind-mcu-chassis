/**
 * @file    KfsStoreActionDiagnostics.cpp
 * @brief   KFS store/release action diagnostic helpers.
 */
#include "KfsStoreActionDiagnostics.hpp"

namespace Diagnostics::KfsStoreAction
{

KfsStoreDiagnostics last_kfs_store_diagnostics;

void report(const Context& ctx, const Reason reason)
{
    last_kfs_store_diagnostics = { ctx, reason, {} };
}

} // namespace Diagnostics::KfsStoreAction
