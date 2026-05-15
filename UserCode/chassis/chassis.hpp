/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once

// chassis 命名空间集中暴露底盘系统的几个核心单例式入口：
// - motion: 物理运动对象，拥有四轮和前后 lift；
// - loc: 定位接口，可能是 JustEncoder 或 LocEKF；
// - ctrl: 主控制器，把目标姿态 / 速度变成 motion 输出。
#include "IndLiftMecanum4.hpp"
#include "IChassisLoc.hpp"
#include "JustEncoder.hpp"
#include "LocEKF.hpp"
#include "Master.hpp"

#include <cstdint>

namespace Chassis
{

using ChassisController = chassis::controller::Master;
using ChassisLoc        = chassis::loc::IChassisLoc;
using ChassisLocEKF     = chassis::loc::LocEKF<256>;
using ChassisLocEncoder = chassis::loc::JustEncoder;
using ChassisMotion     = IndLiftMecanum4;

inline ChassisLoc*        loc{};
inline ChassisLocEKF*     loc_ekf{};
inline ChassisLocEncoder* loc_encoder{};
inline ChassisController* ctrl{};
inline ChassisMotion*     motion{};

/** @brief 按当前 ProjectParts 创建底盘 / lift 运动对象。 */
void init();

/** @brief 用给定初始世界位姿创建定位层和主控制器。 */
void initLocCtrl(const chassis::Posture& init_posture);
/** @brief 不依赖上位机首帧时，使用默认位姿创建定位层和主控制器。 */
void initStandaloneLocCtrl();

/** @brief 把上位机定位帧作为 EKF 外部观测输入。 */
void updateLidar(const chassis::Posture& posture, uint32_t ticks);

/** @brief 使能底盘主控制器。 */
void enable();

/** @brief 1 kHz 控制链更新。 */
void update_1kHz();

/** @brief 100 Hz 轨迹推进更新。 */
void update_100Hz();

} // namespace Chassis
