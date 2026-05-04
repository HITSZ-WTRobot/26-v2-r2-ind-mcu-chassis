/**
 * @file    grip.hpp
 * @brief   夹取机构入口
 */
#pragma once

#include "Config.hpp"
#include "cmsis_os2.h"
#include "gpio_driver.h"
#include "motor_vel_controller.hpp"
#include "traits.hpp"

namespace Grip
{

/**
 * @brief Grip 低层机构对象
 *
 * 负责：
 * - 封装大臂 / 转向两个单轴轨迹对象；
 * - 维护夹爪 GPIO 的开合；
 * - 提供上层动作可直接调用的语义姿态接口；
 * - 统一处理双轴规划失败时的立即停车策略。
 */
class Grip : traits::NoCopy, traits::NoDelete
{
public:
    /** @brief 构造底层控制对象并默认张开夹爪。 */
    Grip();

    /** @brief 使能两个关节轨迹控制器。仅允许在校准完成后调用。 */
    bool enable();
    /** @brief 关闭两个关节控制器并停止后续输出。 */
    void disable();

    /** @brief 当前 grip 底层轨迹控制是否已整体使能。 */
    [[nodiscard]] bool enabled() const { return enabled_; }

    /** @brief 1 kHz：更新速度环控制输出。 */
    void update_1kHz();
    /** @brief 500 Hz：更新位置误差补偿。 */
    void update_500Hz();
    /** @brief 100 Hz：推进 S 曲线轨迹。 */
    void update_100Hz();

    /** @brief 同时启动大臂与转向两轴的回零校准。 */
    void startCalibration();

    /** @brief 两轴都完成校准后，grip 才算校准完成。 */
    [[nodiscard]] bool isCalibrated() const
    {
        return arm_trajectory_.isCalibrated() && turn_trajectory_.isCalibrated();
    }

    /** @brief 前往系统待机姿态，并张开夹爪。 */
    bool toStandbyPose();
    /** @brief 前往准备夹取姿态，并张开夹爪等待底盘切入。 */
    bool toPrepareGrabPose();
    /** @brief 前往夹取执行姿态，并先闭合夹爪。 */
    bool toGrabPose();
    /** @brief 前往对接姿态。 */
    bool toDockingPose();
    /** @brief 前往 KFS 拾取姿态。 */
    bool toKfsPickupPose();
    /** @brief 前往 KFS 暂存姿态。 */
    bool toKfsStorePose();
    /** @brief 前往 KFS 释放姿态。 */
    bool toKfsReleasePose();
    /** @brief 不附带夹爪语义，直接规划到指定双轴关节位姿。 */
    bool toJointPose(const Config::JointPose& pose);

    /** @brief 直接控制夹爪 GPIO 打开。 */
    void openClaw();
    /** @brief 直接控制夹爪 GPIO 闭合。 */
    void closeClaw();
    /** @brief 立即停止当前双轴轨迹。 */
    void stop();

    /** @brief 两轴均校准完成且轨迹都跑完时返回 true。 */
    [[nodiscard]] bool isFinished() const;
    /** @brief 阻塞等待当前双轴动作结束。 */
    void waitForFinish() const;

private:
    bool enabled_{ false };

    /// 大臂电机速度环控制器。
    controllers::MotorVelController arm_vel_controller_;
    /// 转向电机速度环控制器。
    controllers::MotorVelController turn_vel_controller_;

    /// 大臂单轴回零 + 轨迹封装。
    trajectory::HomingMotorTrajectory<1> arm_trajectory_;
    /// 转向单轴回零 + 轨迹封装。
    trajectory::HomingMotorTrajectory<1> turn_trajectory_;

    /// 夹爪气缸 / 电磁控制 GPIO。
    GPIO_t claw_{};

    /**
     * @brief 按一组关节姿态统一规划两轴目标
     *
     * 约束：
     * - 两轴规划必须同时成功才算成功；
     * - 任一轴规划失败时，立即 stop 两轴，避免另一轴继续沿旧轨迹运行。
     */
    bool planPose(const Config::JointPose& pose);
};

/// 全局单例风格 grip 入口，由 Grip::init() 负责创建。
inline Grip* grip{};

/** @brief 初始化 grip 单例；重复调用安全。 */
void init();

} // namespace Grip
