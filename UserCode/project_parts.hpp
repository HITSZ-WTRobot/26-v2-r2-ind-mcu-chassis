/**
 * @file    project_parts.hpp
 * @brief   项目模块启用开关
 *
 * 本文件是“组件启用逻辑”的唯一入口。
 *
 * 使用规则：
 * - 只修改下面 7 个 0 / 1 宏；
 * - 业务代码不要直接重新组合原始宏，统一使用本文件中派生出来的
 *   `ProjectParts::EnableXxx` / `NeedXxx` 常量；
 * - 派生常量的存在意义，是把“模块开关”翻译成“系统能力”。
 *
 * 七个一级开关分别代表：
 * 1. 底盘（四个底盘电机组成的 mecanum4 平面运动部分）
 * 2. 升降（两个抬升电机组成的抬升机构）
 * 3. grip（夹取机构，负责取矛头和卷轴临时存放）
 * 4. 陀螺仪（当前为航向陀螺仪）
 * 5. 上位机定位包（上位机下发的外部位姿观测）
 * 6. 上位机控制指令（上位机其他控制命令）
 * 7. connection table I2C 周期发送
 *
 * 常见组合：
 * - 仅底盘调试：
 *   `WHEEL_CHASSIS=1, LIFT=0, GRIP=0, GYRO=0, PC_LOCALIZATION=0, PC_CONTROL=0`
 *   => 使用 JustEncoder 做纯编码器定位
 * - 底盘 + 升降：
 *   `WHEEL_CHASSIS=1, LIFT=1`
 *   => 可独立调试整车下部机构，但不依赖 grip / 上位机
 * - 仅升降：
 *   `WHEEL_CHASSIS=0, LIFT=1`
 *   => 只保留抬升机构初始化、校准和更新逻辑
 * - 仅 grip：
 *   `GRIP=1`，其余可全关
 * - 底盘 + 陀螺仪：
 *   `WHEEL_CHASSIS=1, GYRO=1, PC_LOCALIZATION=0`
 *   => 使用下位机本地 EKF，不等待上位机首帧位姿
 * - 底盘 + 陀螺仪 + 上位机定位包：
 *   `WHEEL_CHASSIS=1, GYRO=1, PC_LOCALIZATION=1`
 *   => 使用上下位机融合定位，并等待上位机首帧初始位姿
 * - 完整项目：
 *   全部置 1
 */
#pragma once

/// 启用四轮底盘部分。它决定是否初始化轮电机、是否构造底盘 Loc / Controller。
#ifndef PROJECT_PART_ENABLE_WHEEL_CHASSIS
#    define PROJECT_PART_ENABLE_WHEEL_CHASSIS 1
#endif

/// 启用前后两个抬升电机组成的升降机构。
#ifndef PROJECT_PART_ENABLE_LIFT
#    define PROJECT_PART_ENABLE_LIFT 1
#endif

/// 启用夹取机构，包括 arm / turn 电机、夹爪 GPIO，以及取矛头 / 存放卷轴动作组。
#ifndef PROJECT_PART_ENABLE_GRIP
#    define PROJECT_PART_ENABLE_GRIP 1
#endif

/// 启用陀螺仪输入。关闭时底盘定位退化为 JustEncoder。
#ifndef PROJECT_PART_ENABLE_GYRO
#    define PROJECT_PART_ENABLE_GYRO 1
#endif

/// 启用上位机外部定位包接入。它要求底盘与陀螺仪同时启用。
#ifndef PROJECT_PART_ENABLE_PC_LOCALIZATION
#    define PROJECT_PART_ENABLE_PC_LOCALIZATION 1
#endif

/// 启用上位机控制命令接入，例如 Stop / StepUp / StepDown 等。
#ifndef PROJECT_PART_ENABLE_PC_CONTROL
#    define PROJECT_PART_ENABLE_PC_CONTROL 1
#endif

/// 启用 connection table 的 I2C 周期发送。
#ifndef PROJECT_PART_ENABLE_CONNECTION_TABLE_I2C_TX
#    define PROJECT_PART_ENABLE_CONNECTION_TABLE_I2C_TX 1
#endif

namespace ProjectParts
{

/// 一级开关：四轮底盘。
inline constexpr bool EnableWheelChassis = PROJECT_PART_ENABLE_WHEEL_CHASSIS != 0;
/// 一级开关：升降机构。
inline constexpr bool EnableLift         = PROJECT_PART_ENABLE_LIFT != 0;
/// 一级开关：grip 机构。
inline constexpr bool EnableGrip         = PROJECT_PART_ENABLE_GRIP != 0;
/// 一级开关：陀螺仪。
inline constexpr bool EnableGyro         = PROJECT_PART_ENABLE_GYRO != 0;
/// 一级开关：上位机定位包。
inline constexpr bool EnablePcLocalization = PROJECT_PART_ENABLE_PC_LOCALIZATION != 0;
/// 一级开关：上位机控制命令。
inline constexpr bool EnablePcControl      = PROJECT_PART_ENABLE_PC_CONTROL != 0;
/// 一级开关：connection table I2C 周期发送。
inline constexpr bool EnableConnectionTableI2CTx =
        PROJECT_PART_ENABLE_CONNECTION_TABLE_I2C_TX != 0;

/**
 * 是否需要构造当前工程的底盘运动对象。
 *
 * 这里使用 `||` 的原因是当前 `IndLiftMecanum4` 被当作“底盘轮组 + 升降机构”的
 * 统一承载对象：
 * - 仅底盘调试时，需要它来驱动四轮；
 * - 仅升降调试时，也需要它来承载 `LiftSide` 更新逻辑；
 * - 两者都关时，整个底盘运动对象不创建。
 */
inline constexpr bool EnableChassisMotion = EnableWheelChassis || EnableLift;

/// 只要上位机任一通道打开，就需要启动上位机串口协议任务。
inline constexpr bool EnableUpperHostProtocol = EnablePcLocalization || EnablePcControl;

/**
 * 只要启用了四轮底盘，就认为“需要定位层”。
 *
 * 原因是底盘控制器 `Master` 强依赖 Loc 反馈；即便只是单独调底盘，也需要一个
 * 最简单的定位后端来提供位姿 / 速度反馈接口。
 */
inline constexpr bool EnableChassisLocalization = EnableWheelChassis;

/// 纯编码器定位模式：有底盘，但没有陀螺仪。
inline constexpr bool EnableJustEncoderLocalization =
        EnableChassisLocalization && !EnableGyro;

/// EKF 定位模式：有底盘，且有陀螺仪。
inline constexpr bool EnableEkfLocalization = EnableChassisLocalization && EnableGyro;

/**
 * 是否必须等待上位机首帧位姿后，才允许系统认为“初始化完成”。
 *
 * 仅在 `PC_LOCALIZATION` 打开时需要等待：
 * - 关闭时：说明使用本地下位机定位，不需要外部初始位姿；
 * - 打开时：需要把首帧上位机位姿作为 EKF 初始参考。
 */
inline constexpr bool NeedUpperHostInitPosture = EnablePcLocalization;

/**
 * 台阶动作组是否可用。
 *
 * `StepUp/StepDown` 同时依赖：
 * - 上位机控制命令入口
 * - 底盘平面运动
 * - 升降机构
 *
 * 三者缺一都无法形成完整动作链，因此统一派生成一个能力开关。
 */
inline constexpr bool EnableStepAction = EnablePcControl && EnableWheelChassis && EnableLift;

/**
 * 编译期配置约束：
 * 上位机定位包不是独立能力，它本质上是“对 EKF 的外部观测输入”。
 * 因此没有底盘或没有陀螺仪时，不允许单独打开。
 */
static_assert(!EnablePcLocalization || EnableEkfLocalization,
              "PROJECT_PART_ENABLE_PC_LOCALIZATION requires wheel chassis and gyro enabled.");

} // namespace ProjectParts
