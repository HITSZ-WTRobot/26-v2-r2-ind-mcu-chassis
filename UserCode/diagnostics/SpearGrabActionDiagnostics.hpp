/**
 * @file    SpearGrabActionDiagnostics.hpp
 * @brief   SpearGrab action diagnostic helpers.
 */
#pragma once

#include "Master.hpp"
#include "s_curve.hpp"

#include <array>
#include <cstdint>

namespace Diagnostics::SpearGrabAction
{

enum class Reason : uint8_t
{
    None               = 0,
    Busy               = 1,
    DependencyNotReady = 2,
    InvalidArgument    = 3,
    ChassisPlanFailed  = 10,
    GripPlanFailed     = 11,
};

struct Context
{
    uint8_t stage{ 0 };
};

using ChassisSCurveFailureRecord = chassis::controller::Master::SCurveFailureRecord;
using GripSCurveFailureInfo      = velocity_profile::SCurveProfile::FailureInfo;

inline constexpr uint8_t GripAxisCount = 2U;

struct GripAxisFailureRecord
{
    bool                   valid{ false };
    uint8_t                axis{ 0 };
    GripSCurveFailureInfo  info{};
};

using GripFailureRecords = std::array<GripAxisFailureRecord, GripAxisCount>;

struct SpearGrabDiagnostics
{
    Context ctx;
    Reason  reason{ Reason::None };

    ChassisSCurveFailureRecord chassisFailureRecord{};
    GripFailureRecords         gripFailureRecord{};
};

extern SpearGrabDiagnostics last_spear_grab_diagnostics;

void report(const Context& ctx, Reason reason);

} // namespace Diagnostics::SpearGrabAction
