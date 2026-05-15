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

// 当前项目把“麦轮底盘”和“前后两组 lift”合并成一个 motion 对象。
// 这样上层控制器、定位器和 step 动作都可以只依赖一个统一的运动接口。
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
        Front = 0U, ///< 前侧 lift。
        Rear,       ///< 后侧 lift。
        Max
    };

    explicit IndLiftMecanum4();

    bool enable() override;
    void disable() override;
    void update_1kHz();
    void update_100Hz();

    [[nodiscard]] bool enabled() const override { return enabled_; }

    /** @brief 由轮速反馈反算车体系速度。 */
    chassis::Velocity forwardGetVelocity() override;

    [[nodiscard]] bool isReady() const override
    {
        // 当前“ready”主要由 lift 校准完成决定；未启用 lift 时不阻塞。
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
        // 轮组无回零需求，只有 lift 需要校准。
        if constexpr (ProjectParts::EnableLift)
            for (auto& l : lift_)
                l.startCalibration();
    }

    Lift::LiftSide& lift(const LiftType type) { return lift_[static_cast<size_t>(type)]; }

    float liftAllTo(const float pos)
    {
        if constexpr (!ProjectParts::EnableLift)
            return 0.0f;

        // 返回两侧规划耗时的最大值，方便上层估计整体动作时间。
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
        // 简单轮询等待；调用方应在任务线程中使用，不要在 ISR 中调用。
        while (!isLiftAllFinished())
            osDelay(1);
    }

protected:
    void applyVelocity(const chassis::Velocity& velocity) override;

private:
    // lift 的 errorUpdate 由 1 kHz 定时器分频到 500 Hz。
    uint32_t prescaler_ = 0;

    bool  enabled_{ false };
    float wheel_radius_; ///< 轮子半径 (unit: m)
    float k_omega_;      ///< O 型麦轮解算中的“半前后轮距 + 半左右轮距”

    static constexpr size_t WheelNum = static_cast<size_t>(WheelType::Max);
    static constexpr size_t LiftNum  = static_cast<size_t>(LiftType::Max);

    controllers::MotorVelController& wheel(const WheelType type)
    {
        return wheel_[static_cast<size_t>(type)];
    }

    // 四个轮电机各自一个速度环。
    controllers::MotorVelController wheel_[WheelNum];

    // 两个 LiftSide 分别代表前侧和后侧。
    Lift::LiftSide lift_[LiftNum];
};

} // namespace Chassis
