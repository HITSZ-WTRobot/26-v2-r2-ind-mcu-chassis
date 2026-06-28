/**
 * @file    OfflineTrajectoryFollower.hpp
 * @author  syhanjin
 * @date    2026-06-28
 * @brief   4-DOF 离线轨迹跟随器，组合 ChassisSlaveOffline 和 MotorTrajectorySlave。
 */
#pragma once
#include <cmath>
#include "../Config.hpp"
#include "Config.hpp"
#include "Slave.hpp"
#include "isr_lock.h"
#include "motor_trajectory_slave.hpp"
#include "motor_vel_controller.hpp"
#include "trajectory_point.hpp"

namespace Chassis::controller
{

class OfflineTrajectoryFollower
{
public:
    using ChassisSlave     = ::Config::TrajectoryOffline::ChassisSlaveOffline;
    using MotorSlave       = trajectory::MotorTrajectorySlave<4, 0, true>;
    using TrajectoryPoint8 = planning::trajectory::TrajectoryPoint8;

    OfflineTrajectoryFollower(chassis::motion::IChassisMotion& motion,
                              chassis::loc::IChassisLoc&       loc,
                              controllers::MotorVelController* lift_motors[4]) :
        chassis_slave_(motion, loc, ::Config::TrajectoryOffline::PDCfg),
        motor_slave_(lift_motors, ::Config::TrajectoryOffline::LiftMotorPDCfg)
    {
    }

    /// 开始跟随一条轨迹。如已有轨迹在运行，会自动停止旧轨迹。
    bool start(const TrajectoryPoint8* points, const size_t count, const bool mirror = false)
    {
        if (points == nullptr || count == 0)
            return false;

        // 1. 停止 ISR 侧对旧轨迹的访问
        {
            ISRGuard lock{};
            stopped_ = true;
        }

        // 2. 释放旧轨迹的控制权（ISR 已不会访问，可在关中断外操作）
        chassis_slave_.releaseControl();
        motor_slave_.disable();

        // 3. 使能新轨迹的 slave
        if (!chassis_slave_.enable())
            return false;
        if (!motor_slave_.enable())
        {
            chassis_slave_.disable();
            return false;
        }

        // 4. 计算首个轨迹点
        const auto& [x, y, yaw, h, dx, dy, dyaw, dh] = points[0];

        chassis::Posture        p_ref = mirrorPosture(x, y, yaw);
        const chassis::Velocity v_ref = mirrorVelocity(dx, dy, dyaw);

        const auto [wx, wy, wyaw] = chassis_slave_.postureInWorld();
        const float offset = static_cast<float>(std::lround((wyaw - p_ref.yaw) / 360.0f)) * 360.0f;
        p_ref.yaw += offset;

        // 5. 原子写入全部状态，最后置 stopped_=false 激活 ISR 访问
        {
            ISRGuard lock{};
            points_     = points;
            count_      = count;
            index_      = 0;
            mirror_     = mirror;
            yaw_offset_ = offset;
            stopped_    = false;
        }

        // 6. 注入首点到两个 slave（ISR 此时可访问，profileUpdate 本身 ISR 安全）
        chassis_slave_.profileUpdate({ p_ref, v_ref });
        motor_slave_.profileUpdate({ Chassis::Config::Lift::chassisHeightToMotorAngle(h),
                                     Chassis::Config::Lift::chassisHeightVelToMotorDPS(dh) });

        return true;
    }

    /// 推进轨迹索引，更新参考值（100Hz 调用）。
    void update()
    {
        if (stopped_ || points_ == nullptr)
            return;

        ++index_;
        if (index_ >= count_)
        {
            index_ = count_ - 1;
            return;
        }

        const auto& [x, y, yaw, h, dx, dy, dyaw, dh] = points_[index_];

        // 拆分平面参考
        chassis::Posture p = mirrorPosture(x, y, yaw);
        p.yaw += yaw_offset_;
        const ChassisSlave::TrajectoryPoint chassis_pt = { p, mirrorVelocity(dx, dy, dyaw) };

        chassis_slave_.profileUpdate(chassis_pt);
        motor_slave_.profileUpdate({ Config::Lift::chassisHeightToMotorAngle(h),
                                     Config::Lift::chassisHeightVelToMotorDPS(dh) });
    }

    /// 误差跟踪（500Hz 调用），委托给两个 slave。
    void errorUpdate()
    {
        if (stopped_ || points_ == nullptr)
            return;
        chassis_slave_.errorUpdate();
        motor_slave_.errorUpdate();
    }

    /// 电机速度 PID 更新（1kHz 调用），委托给抬升 slave。
    void controllerUpdate()
    {
        if (stopped_ || points_ == nullptr)
            return;
        motor_slave_.controllerUpdate();
    }

    /// 停止跟踪，锁定当前位置。
    void stop()
    {
        {
            ISRGuard lock{};
            stopped_ = true;
        }
        chassis_slave_.stop();
        motor_slave_.stop();
    }

    /// 完整清理：停止 + 释放控制权 + 关闭电机控制器。
    void cleanup()
    {
        {
            ISRGuard lock{};
            stopped_ = true;
        }
        chassis_slave_.releaseControl();
        motor_slave_.disable();
    }

    [[nodiscard]] bool   isActive() const { return !stopped_ && points_ != nullptr; }
    [[nodiscard]] bool   isFinished() const { return index_ >= count_ - 1; }
    [[nodiscard]] size_t currentIndex() const { return index_; }

    ChassisSlave& chassisSlave() { return chassis_slave_; }
    MotorSlave&   motorSlave() { return motor_slave_; }

private:
    [[nodiscard]] chassis::Posture mirrorPosture(const float x,
                                                 const float y,
                                                 const float yaw) const
    {
        return mirror_ ? chassis::Posture{ x, -y, -yaw } : chassis::Posture{ x, y, yaw };
    }

    [[nodiscard]] chassis::Velocity mirrorVelocity(const float dx,
                                                   const float dy,
                                                   const float dyaw) const
    {
        return mirror_ ? chassis::Velocity{ dx, -dy, -dyaw } : chassis::Velocity{ dx, dy, dyaw };
    }

    ChassisSlave chassis_slave_;
    MotorSlave   motor_slave_;

    const TrajectoryPoint8* points_{ nullptr };
    size_t                  count_{ 0 };
    size_t                  index_{ 0 };

    bool  mirror_{ false };
    bool  stopped_{ true };
    float yaw_offset_{ 0.0f };
};

} // namespace Chassis::controller
