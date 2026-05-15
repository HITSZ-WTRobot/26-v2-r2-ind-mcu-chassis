/**
 * @file    IndLiftMecanum4.cpp
 * @author  syhanjin
 * @date    2026-03-26
 */
#include "IndLiftMecanum4.hpp"
#include "Config.hpp"

namespace Chassis
{

namespace
{
constexpr float deg2rad(const float deg)
{
    return deg * M_PI / 180.0f;
}

/**
 * rad/s to round/min.
 * @param rps rad/s
 */
constexpr float rps2rpm(const float rps)
{
    return rps * 60.0f / (2 * M_PI);
}
/**
 * round/min to rad/s
 * @param rpm round/min
 */
constexpr float rpm2rps(const float rpm)
{
    return rpm / 60.0f * (2 * M_PI);
}
/**
 * round/min to deg/s
 * @param rpm round/min
 */
constexpr float rpm2dps(const float rpm)
{
    return rpm / 60.0f * 360.0f;
}

} // namespace

IndLiftMecanum4::IndLiftMecanum4() :
    wheel_{
        { Device::Motor::wheel[0], { Config::Motion::MotorWheelVelPIDCfg } }, // 右前轮
        { Device::Motor::wheel[1], { Config::Motion::MotorWheelVelPIDCfg } }, // 左前轮
        { Device::Motor::wheel[2], { Config::Motion::MotorWheelVelPIDCfg } }, // 左后轮
        { Device::Motor::wheel[3], { Config::Motion::MotorWheelVelPIDCfg } }, // 右后轮
    },
    lift_{ Lift::LiftSide(Device::Motor::lift[0], Device::Motor::lift[1]),
           Lift::LiftSide(Device::Motor::lift[2], Device::Motor::lift[3]) }
{
    // 运动学几何参数统一换算成 SI 单位，后续控制链不再混用 mm。
    wheel_radius_ = Config::Motion::WheelRadiusMM * 1e-3f;

    constexpr float half_x = Config::Motion::WheelDistanceXMM * 1e-3f * 0.5f;
    constexpr float half_y = Config::Motion::WheelDistanceYMM * 1e-3f * 0.5f;

    k_omega_ = half_x + half_y;
}

bool IndLiftMecanum4::enable()
{
    bool enabled = true;

    // 轮组和 lift 任意一个 enable 失败，整个 motion 都回退为 disable。
    if constexpr (ProjectParts::EnableWheelChassis)
        for (auto& w : wheel_)
            enabled &= w.enable();
    if constexpr (ProjectParts::EnableLift)
        for (auto& l : lift_)
            enabled &= l.enable();

    if (!enabled)
    {
        if constexpr (ProjectParts::EnableWheelChassis)
            for (auto& w : wheel_)
                w.disable();
        if constexpr (ProjectParts::EnableLift)
            for (auto& l : lift_)
                l.disable();
    }
    enabled_ = enabled;
    return enabled;
}

void IndLiftMecanum4::disable()
{
    // 停止时同时关掉所有启用的执行机构，保持 motion 内部状态一致。
    if constexpr (ProjectParts::EnableWheelChassis)
        for (auto& w : wheel_)
            w.disable();
    if constexpr (ProjectParts::EnableLift)
        for (auto& l : lift_)
            l.disable();
    enabled_ = false;
}

void IndLiftMecanum4::update_1kHz()
{
    if (!enabled())
        return;

    // 轮速环每 1 ms 更新一次。
    if constexpr (ProjectParts::EnableWheelChassis)
        for (auto& wheel : wheel_)
            wheel.update();

    if constexpr (ProjectParts::EnableLift)
    {
        // lift 的误差修正 500 Hz；速度环仍保持 1 kHz。
        ++prescaler_;
        if (prescaler_ == 2)
        {
            prescaler_ = 0;
            for (auto& l : lift_)
                l.update_500Hz();
        }

        for (auto& l : lift_)
            l.update_1kHz();
    }
}

void IndLiftMecanum4::update_100Hz()
{
    // lift 的 S 曲线轨迹推进放在 100 Hz。
    if constexpr (ProjectParts::EnableLift)
        for (auto& l : lift_)
            l.update_100Hz();
}

chassis::Velocity IndLiftMecanum4::forwardGetVelocity()
{
    chassis::Velocity vel{};

    if constexpr (!ProjectParts::EnableWheelChassis)
        return vel;

    if constexpr (!ProjectParts::EnableLift)
    {
        // 没有 lift 时，四个轮子都默认接地，按标准四麦轮反解。
        vel.vx = wheel(WheelType::FrontRight).getMotor()->getVelocity() +
                 wheel(WheelType::FrontLeft).getMotor()->getVelocity() +
                 wheel(WheelType::RearRight).getMotor()->getVelocity() +
                 wheel(WheelType::RearLeft).getMotor()->getVelocity();
        vel.vy = wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                 wheel(WheelType::FrontLeft).getMotor()->getVelocity() -
                 wheel(WheelType::RearRight).getMotor()->getVelocity() +
                 wheel(WheelType::RearLeft).getMotor()->getVelocity();
        vel.wz = wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                 wheel(WheelType::FrontLeft).getMotor()->getVelocity() +
                 wheel(WheelType::RearRight).getMotor()->getVelocity() -
                 wheel(WheelType::RearLeft).getMotor()->getVelocity();

        constexpr float factor = 0.25f;

        vel.vx = rpm2rps(wheel_radius_ * vel.vx) * factor;
        vel.vy = rpm2rps(wheel_radius_ * vel.vy) * factor;
        vel.wz = rpm2dps(wheel_radius_ / k_omega_ * vel.wz) * factor;
        return vel;
    }

    // 有 lift 时，某一侧 lift 收起后对应轮组可能离地。
    // 速度估计只使用 grounding=true 的轮组，并用 factor 做平均。
    float factor = 1.0f;

    if (lift(LiftType::Front).isGrounding())
    {
        factor *= 0.5f;
        vel.vx += wheel(WheelType::FrontRight).getMotor()->getVelocity() +
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
        vel.vy += wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
        vel.wz += wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
    }
    if (lift(LiftType::Rear).isGrounding())
    {
        factor *= 0.5f;
        vel.vx += wheel(WheelType::RearRight).getMotor()->getVelocity() +
                  wheel(WheelType::RearLeft).getMotor()->getVelocity();
        vel.vy += -wheel(WheelType::RearRight).getMotor()->getVelocity() +
                  wheel(WheelType::RearLeft).getMotor()->getVelocity();
        vel.wz += wheel(WheelType::RearRight).getMotor()->getVelocity() -
                  wheel(WheelType::RearLeft).getMotor()->getVelocity();
    }
    vel.vx = rpm2rps(wheel_radius_ * vel.vx) * factor;
    vel.vy = rpm2rps(wheel_radius_ * vel.vy) * factor;
    vel.wz = rpm2dps(wheel_radius_ / k_omega_ * vel.wz) * factor;

    return vel;
}

/**
 * 设置底盘速度
 */
void IndLiftMecanum4::applyVelocity(const chassis::Velocity& velocity)
{
    if constexpr (!ProjectParts::EnableWheelChassis)
        return;

    const auto& [vx, vy, wz] = velocity;
    /** Mecanum4 O 型运动学解算
     * w_fr = (+ vx + vy + (w + h) * ω) / r
     * w_fl = (+ vx - vy - (w + h) * ω) / r
     * w_rl = (+ vx + vy - (w + h) * ω) / r
     * w_rr = (+ vx - vy + (w + h) * ω) / r
     */
    wheel(WheelType::FrontRight)
            .setRef(rps2rpm((vx + vy + k_omega_ * deg2rad(wz)) / wheel_radius_));
    wheel(WheelType::FrontLeft).setRef(rps2rpm((vx - vy - k_omega_ * deg2rad(wz)) / wheel_radius_));
    wheel(WheelType::RearLeft).setRef(rps2rpm((vx + vy - k_omega_ * deg2rad(wz)) / wheel_radius_));
    wheel(WheelType::RearRight).setRef(rps2rpm((vx - vy + k_omega_ * deg2rad(wz)) / wheel_radius_));
}
} // namespace Chassis
