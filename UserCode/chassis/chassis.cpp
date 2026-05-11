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

#include <cmath>

namespace Chassis
{
namespace
{
constexpr float sq(const float value)
{
    return value * value;
}

struct SourceFrame
{
    bool            valid{ false };
    chassis::Posture origin{};
};

constexpr uint32_t SourceFrameCount = 2U;

constexpr uint32_t source_index(const ExternalSource source)
{
    return (source == ExternalSource::Vision) ? 1U : 0U;
}

[[nodiscard]] chassis::Posture source_to_world(const chassis::Posture& origin,
                                               const chassis::Posture& posture_in_source)
{
    return ChassisLoc::RelativePosture2WorldPosture(origin, posture_in_source);
}

ExternalSource active_source = ProjectParts::EnablePcLocalization ? ExternalSource::Lidar
                                                                   : ExternalSource::None;

SourceFrame source_frames[SourceFrameCount] = {
    { true, { 0.0f, 0.0f, 0.0f } },     // Lidar 作为默认的外部定位源，初始时有效，且原点在世界坐标系下的 (0, 0, 0)。Vision 作为后续可选的外部定位源，初始时无效。
    { false, { 0.0f, 0.0f, 0.0f } },    // Vision 作为后续可选的外部定位源，初始时无效。
};

ChassisLocEKF::Config make_loc_ekf_config(const chassis::Posture& init_posture)
{
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
    return { .x = 0.0f, .y = 0.0f, .yaw = 0.0f };
}
} // namespace

void update_100Hz()
{
    if (motion != nullptr)
        motion->update_100Hz();

    if (ctrl != nullptr)
        ctrl->profileUpdate(0.01f);
}

void update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    if (loc_ekf != nullptr)
        loc_ekf->update();
    else if (loc_encoder != nullptr)
        loc_encoder->update(0.001f);
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

        loc_ekf = new ChassisLocEKF(*motion, make_loc_ekf_config(init_posture),
                                    *Device::Sensor::gyro_yaw, 1);
        loc = loc_ekf;
    }
    else if constexpr (ProjectParts::EnableJustEncoderLocalization)
    {
        loc_encoder = new chassis::loc::JustEncoder(*motion);
        loc         = loc_encoder;
    }

    if (loc == nullptr)
        return;

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
ExternalSource externalSource()
{
    return active_source;
}

void switchExternalSource(const ExternalSource source)
{
    if constexpr (!ProjectParts::EnablePcLocalization)
        return;

    if (source == active_source)
        return;

    active_source = source;
    if (ctrl != nullptr)
        ctrl->setVelocityInBody(chassis::Velocity::zero(), false);

    if (source == ExternalSource::Lidar || source == ExternalSource::Vision)
    {
        // 切换到新源时，首帧观测将重建该源的参考系并重置 EKF 状态。
        auto& frame = source_frames[source_index(source)];
        frame.origin = { .x = 0.0f, .y = 0.0f, .yaw = 0.0f };
        frame.valid  = false;
    }
}

bool needsExternalInitPosture()
{
    if constexpr (!ProjectParts::NeedUpperHostInitPosture)
        return false;

    return active_source != ExternalSource::None;
}

chassis::Posture externalObservationToWorldForInit(const ExternalSource source,
                                                   const chassis::Posture& posture)
{
    if (source == ExternalSource::None)
        return posture;

    if (source == ExternalSource::Lidar || source == ExternalSource::Vision)
    {
        auto& frame = source_frames[source_index(source)];
        frame.origin = { .x = 0.0f, .y = 0.0f, .yaw = 0.0f };  // 传感器坐标系原点在世界坐标系下的位姿，初始时假设与世界坐标系重合（Lidar）。后续会在 updateExternalObservation 中根据首帧观测更新为正确的位姿。
        frame.valid  = true;
        return source_to_world(frame.origin, posture);
    }

    return posture;
}

void (const ExternalSource source,
                               const chassis::Posture& posture,
                               const uint32_t ticks)
{
    if (loc_ekf == nullptr)
        return;

    if (source == ExternalSource::None)
        return;

    auto& frame = source_frames[source_index(source)];
    if (!frame.valid)
    {
        // 切换后首帧观测：用外部位姿直接重置 EKF，并同步更新协方差。
        frame.valid = true;
        const chassis::Posture posture_in_world = source_to_world(frame.origin, posture);       // 获取body在world下的坐标
        loc_ekf->resetToPosture(posture_in_world, Config::Control::ExternalSourceSwitchCovScale);
        return;
    }

    const chassis::Posture posture_in_world = source_to_world(frame.origin, posture);
    loc_ekf->updateLidar(posture_in_world, ticks);
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
    Chassis::initLocCtrl(posture);
}
