/**
 * @file    IndLiftMecanum4.hpp
 * @author  syhanjin
 * @date    2026-03-26
 * @brief   前后独立底盘，应用于 WTR RC 2026。此为麦轮版本
 *
 * @note 由于这种底盘不太可能重复使用，如有相似设计必然需要重写，故本驱动不做复用设计。
 *
 */
#pragma once
#include "IChassisMotion.hpp"
#include "LiftSide.hpp"
#include "motor_vel_controller.hpp"
#include "project_parts.hpp"

#include <cstddef>

namespace Chassis
{

class IndLiftMecanum4 : public chassis::motion::IChassisMotion
{
public:
    enum class WheelType : size_t
    {
        FrontRight = 0U, ///< 右前轮
        FrontLeft,       ///< 左前轮
        RearLeft,        ///< 左后轮
        RearRight,       ///< 右后轮
        Max
    };

    enum class LiftType : size_t
    {
        Front = 0U,
        Rear,
        Max
    };

    explicit IndLiftMecanum4();

    bool enable() override;
    void disable() override;
    void update_1kHz();
    void update_100Hz();

    [[nodiscard]] bool enabled() const override { return enabled_; }

    chassis::Velocity forwardGetVelocity() override;

    [[nodiscard]] bool isReady() const override
    {
        if constexpr (ProjectParts::EnableLift)
        {
            for (auto& l : lift_)
                if (!l.isCalibrated())
                    return false;
        }
        return true;
    }

    void startCalibration()
    {
        if constexpr (ProjectParts::EnableLift)
            for (auto& l : lift_)
                l.startCalibration();
    }

    Lift::LiftSide& lift(const LiftType type) { return lift_[static_cast<size_t>(type)]; }

    float liftAllTo(const float pos)
    {
        if constexpr (!ProjectParts::EnableLift)
            return 0.0f;

        return std::max(lift(LiftType::Front).to(pos), lift(LiftType::Rear).to(pos));
    }

    float liftAllTo(const float pos, const trajectory::LinkMode link_mode)
    {
        if constexpr (!ProjectParts::EnableLift)
            return 0.0f;

        return std::max(lift(LiftType::Front).to(pos, link_mode),
                        lift(LiftType::Rear).to(pos, link_mode));
    }

    float liftAllTo(const float pos, const Config::Limit& limit)
    {
        if constexpr (!ProjectParts::EnableLift)
            return 0.0f;

        return std::max(lift(LiftType::Front).to(pos, limit), lift(LiftType::Rear).to(pos, limit));
    }

    float liftAllTo(const float                pos,
                    const Config::Limit&       limit,
                    const trajectory::LinkMode link_mode)
    {
        if constexpr (!ProjectParts::EnableLift)
            return 0.0f;

        return std::max(lift(LiftType::Front).to(pos, limit, link_mode),
                        lift(LiftType::Rear).to(pos, limit, link_mode));
    }

    [[nodiscard]] bool isLiftAllFinished()
    {
        if constexpr (ProjectParts::EnableLift)
        {
            for (auto& l : lift_)
                if (!l.isFinished())
                    return false;
        }
        return true;
    }

    void waitLiftAllFinished()
    {
        while (!isLiftAllFinished())
            osDelay(1);
    }

protected:
    void applyVelocity(const chassis::Velocity& velocity) override;

private:
    uint32_t prescaler_ = 0;

    bool  enabled_{ false };
    float wheel_radius_; ///< 轮子半径 (unit: m)
    float k_omega_;      ///< O 型：半前后 + 半左右

    static constexpr size_t WheelNum = static_cast<size_t>(WheelType::Max);
    static constexpr size_t LiftNum  = static_cast<size_t>(LiftType::Max);

    controllers::MotorVelController& wheel(const WheelType type)
    {
        return wheel_[static_cast<size_t>(type)];
    }

    controllers::MotorVelController wheel_[WheelNum];

    Lift::LiftSide lift_[LiftNum];
};

} // namespace Chassis
