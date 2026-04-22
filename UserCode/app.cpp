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
#include "grip/grip.hpp"
#include "project_parts.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    static uint32_t grip_prescaler_500Hz = 0;

    Connection::updateTable();

    Chassis::update_1kHz();

    Device::update_1kHz();

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

    service::Watchdog::EatAll();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim) {}

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

    // 检查看门狗是否已满
    if (service::Watchdog::isFull())
        Error_Handler();

    // 启动定时器
    // 实现 1kHz 定时器交替触发，避免总线上同时控制多电机导致需要同时发送超过3条消息的情况
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // 等待设备和上位机等已启用连接对象在线
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

    // 若当前启用了上位机定位包，则会在此等待首帧位姿；
    // 否则 `System::Init::inited()` 立即返回 true。
    while (!System::Init::inited())
        osDelay(1);

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
    // constexpr osThreadAttr_t autoTaskAttr{
    //     .name       = "auto-task",
    //     .stack_size = 1024 * 4,
    //     .priority   = osPriorityNormal,
    // };

    /* 初始化完成后退出线程 */
    osThreadExit();
}
