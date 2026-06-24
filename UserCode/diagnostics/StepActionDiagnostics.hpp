/**
 * @file    StepActionDiagnostics.hpp
 * @brief   Step action diagnostic helpers.
 */
#pragma once

#include "chassis/LiftSide.hpp"

namespace Diagnostics::StepAction
{

enum class Reason : uint8_t
{
    None               = 0,
    Busy               = 1,
    DependencyNotReady = 2,
    InvalidArgument    = 3,
    ChassisPlanFailed  = 10,
    LiftPlanFailed     = 11,
};

[[nodiscard]] constexpr uint8_t reasonCode(const Reason reason)
{
    return static_cast<uint8_t>(reason);
}

struct Context
{
    uint8_t chassis_stage{ 0 };
    uint8_t front_lift_stage{ 0 };
    uint8_t rear_lift_stage{ 0 };
};

using ChassisSCurveFailureRecord = chassis::controller::Master::SCurveFailureRecord;
using LiftScurveFailureInfo      = velocity_profile::SCurveProfile::FailureInfo;

struct ChassisDiagnostics
{
    Context ctx;
    Reason  reason = { Reason::None };

    ChassisSCurveFailureRecord chassisFailureRecord{};

    LiftScurveFailureInfo liftFailureRecord{};
};

void report(const Context& ctx, Reason reason);

void report(const Context&                    ctx,
            Reason                            reason,
            const ChassisSCurveFailureRecord& chassisFailureRecord);

void report(const Context& ctx, Reason reason, const LiftScurveFailureInfo& liftFailureRecord);

} // namespace Diagnostics::StepAction
