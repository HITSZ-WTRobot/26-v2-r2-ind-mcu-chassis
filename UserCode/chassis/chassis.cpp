/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"
#include "Config.hpp"
#include "LocEKF.hpp"
#include "device.hpp"
#include "project_parts.hpp"
#include "system.hpp"

namespace Chassis
{
namespace
{
constexpr float sq(const float value)
{
    return value * value;
}

ChassisLocEKF::Config make_loc_ekf_config(const chassis::Posture& init_posture)
{
    // EKF 初值由上位机位置 + 当前陀螺仪 yaw 组成。
    // yaw_offset 表示“上位机世界系 yaw”和“陀螺仪本体 yaw”的固定偏差。
    const float init_gyro_yaw = Device::Sensor::gyro_yaw->getYaw();

    return {
        .x_init = { .x          = init_posture.x,
                    .y          = init_posture.y,
                    .yaw        = init_gyro_yaw,
                    .yaw_offset = init_posture.yaw - init_gyro_yaw },
        .covP   = { .xy = sq(0.1f), .yaw = sq(0.1f), .yaw_offset = sq(10.0f) },
        .noiseQ = { .xy = sq(0.05f), .yaw = sq(0.5f), .yaw_offset = sq(0.01f) },
        .noiseR = {
            .gyro  = { .yaw = sq(0.1f) },
            .lidar = { .xy = sq(0.01f), .yaw = sq(0.5f) },
        },
    };
}
[[nodiscard]] chassis::Posture default_init_posture()
{
    // 没有上位机定位首帧时，从世界系原点起步。
    return { .x = 0.0f, .y = 0.0f, .yaw = 0.0f };
}
} // namespace

void update_100Hz()
{
    // 100 Hz 主要推进轨迹曲线；速度环等快速控制不放在这里。
    if (motion != nullptr)
        motion->update_100Hz();

    if (ctrl != nullptr)
        ctrl->profileUpdate(0.01f);
}

void update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    // 定位先更新，控制器随后使用最新反馈。
    if (loc_ekf != nullptr)
        loc_ekf->update();
    else if (loc_encoder != nullptr)
        loc_encoder->update(0.001f);

    if (ctrl != nullptr)
    {
        // 主控制器的误差修正 500 Hz 即可，输出控制仍保持 1 kHz。
        prescaler_500Hz++;
        if (prescaler_500Hz >= 2)
        {
            ctrl->errorUpdate();
            prescaler_500Hz = 0;
        }
        ctrl->controllerUpdate();
    }

    if (motion != nullptr)
        motion->update_1kHz();
}

void init()
{
    // 当前工程把“底盘轮组”和“升降机构”都挂在同一个 Motion 对象下。
    // 因此只要这两部分有任意一项启用，就需要创建 Motion。
    if constexpr (!ProjectParts::EnableChassisMotion)
        return;

    motion = new IndLiftMecanum4();
}

void initLocCtrl(const chassis::Posture& init_posture)
{
    // 没有四轮底盘时，不需要 Loc / Controller。
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    if (loc != nullptr || ctrl != nullptr)
        return;

    if (motion == nullptr)
        return;

    // 有陀螺仪时走 EKF；没有陀螺仪时退化为 JustEncoder。
    if constexpr (ProjectParts::EnableEkfLocalization)
    {
        if (Device::Sensor::gyro_yaw == nullptr)
            return;

        loc_ekf = new ChassisLocEKF(*motion,
                                    make_loc_ekf_config(init_posture),
                                    *Device::Sensor::gyro_yaw,
                                    1);
        loc     = loc_ekf;
    }
    else if constexpr (ProjectParts::EnableJustEncoderLocalization)
    {
        loc_encoder = new chassis::loc::JustEncoder(*motion);
        loc         = loc_encoder;
    }

    if (loc == nullptr)
        return;

    // 控制器只依赖统一的 motion / loc 接口，因此不关心底层定位实现。
    ctrl = new ChassisController(*motion, *loc, Config::Control::masterCfg);
}

void initStandaloneLocCtrl()
{
    // 没有底盘时，不存在独立底盘定位初始化。
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    // 启用了上位机定位包时，必须等 `System::Init::initPostureReceive()`
    // 拿到首帧外部位姿后再构造 EKF，这里不能抢先初始化。
    if constexpr (ProjectParts::NeedUpperHostInitPosture)
        return;

    initLocCtrl(default_init_posture());
}

void updateLidar(const chassis::Posture& posture, const uint32_t ticks)
{
    // 只有 EKF 模式才会消费外部位姿观测。
    if (loc_ekf != nullptr)
        loc_ekf->updateLidar(posture, ticks);
}

void enable()
{
    // 没有四轮底盘时，不存在底盘控制器使能动作。
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    if (ctrl == nullptr || !ctrl->enable())
        Error_Handler();
}

} // namespace Chassis

// 系统初始化钩子
void System::Init::initPostureReceive()
{
    // 收到第一帧上位机位姿后才允许创建 EKF / controller。
    Chassis::initLocCtrl(posture);
}
