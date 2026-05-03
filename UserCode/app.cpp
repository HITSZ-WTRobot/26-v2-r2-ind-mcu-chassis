/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "can.h"
#include "chassis/chassis.hpp"
#include "chassis/actions/Step.hpp"
#include "cmsis_os2.h"
#include "connection.hpp"
#include "device.hpp"
#include "grip/actions/roller_store.hpp"
#include "grip/actions/spear_grab.hpp"
#include "grip/grip.hpp"
#include "i2c.hpp"
#include "project_parts.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tests/tests.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    static uint32_t grip_prescaler_500Hz = 0;

    // 上半拍统一完成本周期控制计算，给下半拍的集中 CAN 发送准备好最新输出。
    Chassis::update_1kHz();
    if (Grip::grip != nullptr)
    {
        grip_prescaler_500Hz++;
        if (grip_prescaler_500Hz >= 2)
        {
            Grip::grip->update_500Hz();
            grip_prescaler_500Hz = 0;
        }
        Grip::grip->update_1kHz();
    }
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim)
{
    // 下半拍集中处理总线发送和管理刷新，避免把 CAN 发送再拆散到两个中断里。
    Device::update_1kHz();

    Connection::updateTable();
    service::Watchdog::EatAll();
}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    Chassis::update_100Hz();

    if (Grip::grip != nullptr)
        Grip::grip->update_100Hz();
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    Device::init();

    Chassis::init();

    Protocol::init();

    if constexpr (ProjectParts::EnableGrip)
        Grip::init();

    if constexpr (ProjectParts::EnableStepAction)
        (void)Action::Step::inst();

    if constexpr (ProjectParts::EnableGrip)
    {
        // 这些高层动作的构造函数内部会创建后台线程，必须在线程上下文预创建；
        // 后续 ISR 路径只读取 `::inst()` 暴露的状态，不再承担首次构造职责。
        (void)Grip::Action::SpearGrab::inst();
    }

    if constexpr (ProjectParts::EnableGripSuction)
    {
        // KFS 吸盘状态需要在线程上下文预创建，后续 ISR / 反馈路径只读取状态，不首次构造对象。
        (void)Grip::Action::KfsStore::inst();
    }

    // ActionState 低优先级任务只读取这些已预创建好的动作 / 机构状态，因此放在它们之后启动。
    Protocol::ActionState::init();

    Connection::init();

    if (!AppI2C::start_bus2_manager())
        Error_Handler();

    // 检查看门狗是否已满
    if (service::Watchdog::isFull())
        Error_Handler();

    // 启动定时器
    // 使用 1 kHz 周期中断 + 半周期后比较中断，把“控制计算”和“CAN 发送 / 管理刷新”分到两个半拍。
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // 这里只等待下位机本地硬件链路在线；上位机辨识/定位首帧等要求统一放到最终初始化完成门槛。
    Connection::waitAll();

    // 等待更新
    osDelay(2000);

    // Motion 非空表示当前调试形态包含“底盘轮组”或“升降机构”至少一个。
    if (Chassis::motion != nullptr)
    {
        if (!Chassis::motion->enable())
            Error_Handler();

        osDelay(1000);

        // 只有升降机构开启时，才需要执行底盘下部抬升校准。
        if constexpr (ProjectParts::EnableLift)
            Chassis::motion->startCalibration();
    }

    // grip 完全独立于底盘 / 升降，可单独校准。
    if constexpr (ProjectParts::EnableGrip)
    {
        if (Grip::grip != nullptr)
            Grip::grip->startCalibration(); // 电机堵转到限位处进行初始化
    }

    while (true)
    {
        // “底盘 ready”在当前工程里等价为：若启用了升降，则两个 LiftSide 已校准完成；
        // 若未启用升降，则不阻塞启动流程。
        const bool chassis_ready = !ProjectParts::EnableLift ||
                                   (Chassis::motion != nullptr && Chassis::motion->isReady());
        // grip 未启用时不阻塞；启用时必须等待其归零完成。
        const bool grip_ready = !ProjectParts::EnableGrip ||
                                (Grip::grip != nullptr && Grip::grip->isCalibrated());

        if (chassis_ready && grip_ready)
            break;

        osDelay(1);
    }

    // TODO: 向上位机返回校准结果

    // 若当前不依赖上位机首帧位姿，则在这里直接构造 Loc / Controller：
    // - 无陀螺仪：构造 JustEncoder
    // - 有陀螺仪但无上位机定位包：构造本地下位机 EKF
    Chassis::initStandaloneLocCtrl();

    // 这里统一等待“系统初始化完成”：
    // - 若启用了上位机串口辨识初始化，则必须先收到任意合法上位机帧；
    // - 若启用了上位机定位包，则还必须等待首帧位姿。
    while (!System::Init::inited())
        osDelay(1);

    osDelay(50);

    // 仅在底盘轮组启用时，才真正使能底盘控制器。
    Chassis::enable();
    if constexpr (ProjectParts::EnableGrip)
    {
        if (Grip::grip == nullptr || !Grip::grip->enable())
            Error_Handler();
    }

    // 等待启动
    osDelay(1000);

    // 创建其他 tasks
    Tests::init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
