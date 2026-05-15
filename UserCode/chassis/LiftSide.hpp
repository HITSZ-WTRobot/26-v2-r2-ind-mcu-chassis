/**
 * @file    LiftSide.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once

// LiftSide 表示“前侧”或“后侧”的一组双电机 lift。
// 上层只给它一个线性高度目标，内部负责换算成电机角度并同步两个电机。
#include "Config.hpp"
#include "device.hpp"
#include "homing_motor_trajectory.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

namespace Lift
{
class LiftSide
{
public:
    LiftSide(motors::IMotor* motor0, motors::IMotor* motor1);

    /** @brief 使用默认限幅移动到目标高度。 */
    float to(float position);
    /** @brief 使用默认限幅，并指定轨迹衔接模式。 */
    float to(float position, trajectory::LinkMode link_mode);
    /** @brief 使用指定限幅移动到目标高度。 */
    float to(float position, const Chassis::Config::Limit& limit);
    /** @brief 使用指定限幅和衔接模式移动到目标高度。 */
    float to(float position, const Chassis::Config::Limit& limit, trajectory::LinkMode link_mode);

    void stop() { traj_.stop(); }

    [[nodiscard]] bool isFinished() const { return traj_.isFinished(); }

    void waitFinished() const
    {
        while (!isFinished())
            osDelay(1);
    }

    /** @brief 当前 lift 高度，单位 m，零点为辅助轮接地位置。 */
    [[nodiscard]] float getPosition() const;

    bool enable() { return traj_.enable(); }

    [[nodiscard]] bool enabled() const { return ctrl_[0].enabled() && ctrl_[1].enabled(); }

    void disable() { traj_.disable(); }

    /** @brief 启动双电机回零校准。 */
    void               startCalibration();
    [[nodiscard]] bool isCalibrated() const { return traj_.isCalibrated(); }

    /** @brief 1 kHz 速度环更新。 */
    void update_1kHz();
    /** @brief 500 Hz 误差修正。 */
    void update_500Hz() { traj_.errorUpdate(); }
    /** @brief 100 Hz 轨迹推进。 */
    void update_100Hz() { traj_.profileUpdate(0.01); }

    /** @brief 当前这侧辅助轮 / 主动轮是否参与接地速度估计。 */
    [[nodiscard]] bool isGrounding() const { return grounding_; }

    /** @brief 由台阶状态机在离地 / 着地节点显式设置。 */
    void setGrounding(const bool grounding) { grounding_ = grounding; }

private:
    static constexpr size_t MotorNum = 2;

    controllers::MotorVelController             ctrl_[MotorNum];
    trajectory::HomingMotorTrajectory<MotorNum> traj_;

    // true 表示该侧轮组被认为仍然接地，可用于底盘速度反馈。
    bool grounding_ = true;
};
} // namespace Lift
