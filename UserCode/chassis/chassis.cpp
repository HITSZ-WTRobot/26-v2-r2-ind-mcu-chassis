/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"
#include "Config.hpp"
#include "LocEKF.hpp"
#include "chassis/trajectory/OfflineTrajectoryFollower.hpp"
#include "chassis/trajectory/trajectory.hpp"
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
            .lidar = { .xy = sq(0.03f), .yaw = sq(1.0f) },
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

    if (offline_trajectory->isActive())
        offline_trajectory->update();
    else if (ctrl != nullptr)
        ctrl->profileUpdate(0.01f);
}

void update_1kHz()
{
    static uint32_t prescaler_500Hz = 0;

    if (loc_ekf != nullptr)
        loc_ekf->update();
    else if (loc_encoder != nullptr)
        loc_encoder->update(0.001f);

    if (offline_trajectory->isActive())
    {
        prescaler_500Hz++;
        if (prescaler_500Hz >= 2)
        {
            offline_trajectory->errorUpdate();
            prescaler_500Hz = 0;
        }
        offline_trajectory->controllerUpdate();
    }
    else if (ctrl != nullptr)
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

    ctrl = new ChassisController(*motion, *loc, Config::Control::masterCfg);

    // 创建离线轨迹跟随器（永不销毁，后续通过 start() 重用）
    {
        ::controllers::MotorVelController* lift_motors[4];
        for (size_t i = 0; i < 4; ++i)
            lift_motors[i] = &motion->liftMotor(i);
        offline_trajectory = new controller::OfflineTrajectoryFollower(*motion, *loc, lift_motors);
    }
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

void startOfflineTrajectory(const int traj_id, const bool mirror)
{
    using namespace ::Config::TrajectoryOffline;

    if (Chassis::offline_trajectory == nullptr)
        return;

    // 1. 释放 Master 控制权（首次切换时需要，后续为 no-op）
    if (Chassis::ctrl != nullptr)
    {
        Chassis::ctrl->stop();
        Chassis::ctrl->releaseControl();
    }

    // 2. 禁用抬升（释放电机控制权，首次切换时需要）
    Chassis::motion->disableLift();

    // 3. 根据 traj_id 选择轨迹数据
    const planning::trajectory::TrajectoryPoint8* points = nullptr;
    size_t                                        count  = 0;
    switch (traj_id)
    {
    case 1:
        points = traj1::Points.data();
        count  = traj1::PointCount;
        break;
    case 2:
        points = traj2::Points.data();
        count  = traj2::PointCount;
        break;
    case 3:
        points = traj3::Points.data();
        count  = traj3::PointCount;
        break;
    case 4:
        points = traj4::Points.data();
        count  = traj4::PointCount;
        break;
    default:
        goto fail;
    }

    // 4. 启动轨迹（内部会先 cleanup 旧轨迹再启动新轨迹）
    if (!Chassis::offline_trajectory->start(points, count, mirror))
        goto fail;
    return;

fail:
    Chassis::motion->enableLift();
    if (Chassis::ctrl != nullptr)
        Chassis::ctrl->enable();
}

void stopOfflineTrajectory()
{
    if (offline_trajectory == nullptr || !offline_trajectory->isActive())
        return;

    offline_trajectory->cleanup();

    if (motion != nullptr)
        motion->enableLift();

    if (ctrl != nullptr)
        ctrl->enable();
}
} // namespace Chassis

// 系统初始化钩子
void System::Init::initPostureReceive()
{
    Chassis::initLocCtrl(posture);
}
