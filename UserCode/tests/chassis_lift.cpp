/**
 * @file    chassis_lift.cpp
 * @brief   底盘 + 升降联合调试测试
 */
#include "tests.hpp"

#include "chassis/Config.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "project_parts.hpp"

#include <cstddef>

namespace Tests::ChassisLift
{

// 0 stop
// 1 vel
// 2 pos
/// 底盘测试模式选择。Ozone 修改后立即生效。
size_t s_chassis = 0;

// 0 前后协同
// 1 控制前腿
// 2 控制后腿
/// 升降测试模式选择。0 时使用 `z_pos`，1/2 时分别使用 `z_front_pos` / `z_rear_pos`。
size_t s_lift      = 0;
/// 前后协同时的统一升降目标位置，单位 m。
float  z_pos       = Chassis::Config::Lift::Position::Normal;
/// 仅控制前腿时的目标位置，单位 m。
float  z_front_pos = z_pos;
/// 仅控制后腿时的目标位置，单位 m。
float  z_rear_pos  = z_pos;

/// 升降动作使用的速度/加速度/加加速度限制。
Chassis::Config::Limit lift_limit = Chassis::Config::Lift::DefaultLimit;

/// 底盘位置模式目标位姿；仅在 `s_chassis == 2` 且值变化时下发，单位 m / deg。
chassis::Posture  pos{};
/// 底盘速度模式目标速度；仅在 `s_chassis == 1` 且值变化时下发，单位 m/s / deg/s。
chassis::Velocity vel{};

namespace
{

void TestTask(void* argument)
{
    (void)argument;

    if constexpr (ProjectParts::EnableWheelChassis)
    {
        if (Chassis::loc != nullptr)
            pos = Chassis::loc->postureInWorld();
    }

    chassis::Posture  posture_cache    = pos;
    chassis::Velocity velocity_cache   = vel;
    float             lift_all_cache   = z_pos;
    float             lift_front_cache = z_front_pos;
    float             lift_rear_cache  = z_rear_pos;

    for (;;)
    {
        if constexpr (ProjectParts::EnableWheelChassis)
        {
            if (Chassis::ctrl != nullptr)
            {
                if (s_chassis == 0)
                {
                    Chassis::ctrl->stop();
                }
                else if (s_chassis == 1)
                {
                    if (velocity_cache.vx != vel.vx || velocity_cache.vy != vel.vy ||
                        velocity_cache.wz != vel.wz)
                    {
                        velocity_cache = vel;
                        Chassis::ctrl->setVelocityInBody(velocity_cache, false);
                    }
                }
                else if (s_chassis == 2)
                {
                    if (posture_cache.x != pos.x || posture_cache.y != pos.y ||
                        posture_cache.yaw != pos.yaw)
                    {
                        posture_cache = pos;
                        Chassis::ctrl->setTargetPostureInWorld(posture_cache);
                    }
                }
            }
        }

        if constexpr (ProjectParts::EnableLift)
        {
            if (Chassis::motion != nullptr)
            {
                if (s_lift == 0)
                {
                    if (lift_all_cache != z_pos)
                    {
                        lift_all_cache   = z_pos;
                        lift_front_cache = z_pos;
                        lift_rear_cache  = z_pos;
                        Chassis::motion->liftAllTo(z_pos, lift_limit);
                    }
                }
                else if (s_lift == 1)
                {
                    if (lift_front_cache != z_front_pos)
                    {
                        lift_front_cache = z_front_pos;
                        Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front)
                                .to(z_front_pos, lift_limit);
                    }
                }
                else if (s_lift == 2)
                {
                    if (lift_rear_cache != z_rear_pos)
                    {
                        lift_rear_cache = z_rear_pos;
                        Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear)
                                .to(z_rear_pos, lift_limit);
                    }
                }
            }
        }

        osDelay(1);
    }
}

} // namespace

void init()
{
    if constexpr (!Tests::EnableChassisLift)
        return;

    if constexpr (!ProjectParts::EnableWheelChassis && !ProjectParts::EnableLift)
        return;

    constexpr osThreadAttr_t attr{
        .name       = "test-cl",
        .stack_size = 512 * 4,
        .priority   = osPriorityNormal,
    };

    osThreadNew(TestTask, nullptr, &attr);
}

} // namespace Tests::ChassisLift
