/**
 * @file    StepActionDiagnostics.cpp
 * @brief   Step action diagnostic helpers.
 */
#include "StepActionDiagnostics.hpp"

#include "chassis/chassis.hpp"

namespace Diagnostics::StepAction
{

ChassisDiagnostics last_chassis_diagnostics;

void report(const Context&                    ctx,
            const Reason                      reason,
            const ChassisSCurveFailureRecord& chassisFailureRecord,
            const LiftScurveFailureInfo&      liftFailureRecord)
{
    last_chassis_diagnostics = { ctx, reason, chassisFailureRecord, liftFailureRecord };
}

void report(const Context& ctx, const Reason reason)
{
    report(ctx, reason, {}, {});
}

void report(const Context&                    ctx,
            const Reason                      reason,
            const ChassisSCurveFailureRecord& chassisFailureRecord)
{
    report(ctx, reason, chassisFailureRecord, {});
}
void report(const Context& ctx, const Reason reason, const LiftScurveFailureInfo& liftFailureRecord)
{
    report(ctx, reason, {}, liftFailureRecord);
}

} // namespace Diagnostics::StepAction
