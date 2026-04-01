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
 * rad/s to round/min
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
    lift_{ Lift::LiftSide(Device::Motor::lift_front), Lift::LiftSide(Device::Motor::lift_rear) }
{
    wheel_radius_ = Config::Motion::wheelRadius * 1e-3f;

    constexpr float half_x = Config::Motion::wheelDistanceX * 1e-3f * 0.5f;
    constexpr float half_y = Config::Motion::wheelDistanceY * 1e-3f * 0.5f;

    k_omega_ = half_x + half_y;
}

bool IndLiftMecanum4::enable()
{
    bool enabled = true;

    if constexpr (Config::useFrontLift)
        enabled &= lift(LiftType::Front).enable();

    if constexpr (Config::useRearLift)
        enabled &= lift(LiftType::Rear).enable();

    enabled_ = enabled;
    return enabled;
}

void IndLiftMecanum4::disable()
{
    if constexpr (Config::useFrontLift)
        lift(LiftType::Front).disable();

    if constexpr (Config::useRearLift)
        lift(LiftType::Rear).disable();
    enabled_ = false;
}

void IndLiftMecanum4::update_1kHz()
{
    if (!enabled())
        return;

    ++prescaler_;
    if (prescaler_ == 2)
    {
        prescaler_ = 0;
        if constexpr (Config::useFrontLift)
            lift(LiftType::Front).update_500Hz();

        if constexpr (Config::useRearLift)
            lift(LiftType::Rear).update_500Hz();
    }
    if constexpr (Config::useFrontLift)
        lift(LiftType::Front).update_1kHz();

    if constexpr (Config::useRearLift)
        lift(LiftType::Rear).update_1kHz();
}

void IndLiftMecanum4::update_100Hz()
{
    if constexpr (Config::useFrontLift)
        lift(LiftType::Front).update_100Hz();

    if constexpr (Config::useRearLift)
        lift(LiftType::Rear).update_100Hz();
}

chassis::Velocity IndLiftMecanum4::forwardGetVelocity()
{
    chassis::Velocity vel{};

    float factor = 1.0f;

    if (isLiftGrounding(LiftType::Front))
    {
        factor *= 0.5f;
        vel.vx += wheel(WheelType::FrontRight).getMotor()->getVelocity() +
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
        vel.vy += wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
        vel.wz += wheel(WheelType::FrontRight).getMotor()->getVelocity() -
                  wheel(WheelType::FrontLeft).getMotor()->getVelocity();
    }
    if (isLiftGrounding(LiftType::Rear))
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