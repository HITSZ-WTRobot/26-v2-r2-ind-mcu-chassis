/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"
#include "Config.hpp"
#include "system.hpp"

namespace Chassis
{
namespace
{
// constexpr float sq(const float value)
// {
//     return value * value;
// }
//
// chassis::loc::LocEKF::Config make_loc_ekf_config(const chassis::Posture& init_posture)
// {
//     const float init_gyro_yaw = Device::Sensor::gyro_yaw->getYaw();
//
//     return {
//         .x_init = { .x          = init_posture.x,
//                     .y          = init_posture.y,
//                     .yaw        = init_gyro_yaw,
//                     .yaw_offset = init_posture.yaw - init_gyro_yaw },
//         .covP   = { .xy = sq(0.1f), .yaw = sq(0.1f), .yaw_offset = sq(10.0f) },
//         .noiseQ = { .xy = sq(0.05f), .yaw = sq(0.5f), .yaw_offset = sq(0.01f) },
//         .noiseR = {
//             .gyro  = { .yaw = sq(0.1f) },
//             .lidar = { .xy = sq(0.01f), .yaw = sq(0.5f) },
//         },
//     };
// }
} // namespace


void update_100Hz()
{
    motion->update_100Hz();

    if (ctrl != nullptr)
        ctrl->profileUpdate(0.01f);
}

void update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    if (loc != nullptr)
        loc->update(0.001f);

    if (ctrl != nullptr)
    {
        prescaler_500Hz++;
        if (prescaler_500Hz >= 2)
        {
            ctrl->errorUpdate();
            prescaler_500Hz = 0;
        }
        ctrl->controllerUpdate();
    }

    motion->update_1kHz();
}

void init()
{
    motion = new IndLiftMecanum4();
}

void initLocCtrl(const chassis::Posture& init_posture)
{
    if (loc != nullptr || ctrl != nullptr)
        return;

    (void)init_posture;

    loc = new chassis::loc::JustEncoder(*motion);
    ctrl = new ChassisController(*motion, *loc, Config::Control::masterCfg);
}

void enable()
{
    if (ctrl == nullptr || !ctrl->enable())
        Error_Handler();
}

} // namespace Chassis

// 系统初始化钩子
void System::Init::initPostureReceive()
{
    Chassis::initLocCtrl(posture);
}
