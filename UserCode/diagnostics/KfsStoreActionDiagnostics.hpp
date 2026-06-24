/**
 * @file    KfsStoreActionDiagnostics.hpp
 * @brief   KFS store/release action diagnostic helpers.
 */
#pragma once

#include "s_curve.hpp"

#include <array>
#include <cstdint>

namespace Diagnostics::KfsStoreAction
{

enum class Reason : uint8_t
{
    None                  = 0,
    Busy                  = 1,
    DependencyNotReady    = 2,
    GripPlanFailed        = 10,
    ObjectAlreadyAttached = 30,
    ObjectMissing         = 31,
};

struct Context
{
    uint8_t stage{ 0 };
    uint8_t workflow_phase{ 0 };
};

using GripSCurveFailureInfo = velocity_profile::SCurveProfile::FailureInfo;

inline constexpr uint8_t GripAxisCount = 2U;

struct GripAxisFailureRecord
{
    bool                  valid{ false };
    uint8_t               axis{ 0 };
    GripSCurveFailureInfo info{};
};

using GripFailureRecords = std::array<GripAxisFailureRecord, GripAxisCount>;

struct KfsStoreDiagnostics
{
    Context ctx;
    Reason  reason{ Reason::None };

    GripFailureRecords gripFailureRecord{};
};

extern KfsStoreDiagnostics last_kfs_store_diagnostics;

void report(const Context& ctx, Reason reason);

} // namespace Diagnostics::KfsStoreAction
