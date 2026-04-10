/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "can.h"
#include "chassis/chassis.hpp"
#include "chassis/actions/Step.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    Chassis::update_1kHz();

    Device::update_1kHz();

    service::Watchdog::EatAll();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim) {}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    Chassis::update_100Hz();
}

void AutoTask(void* argument)
{
    constexpr float distance2step = 0.375f; // 前端离台阶的距离 m

    auto& step = Action::Step::inst();

    step.up(distance2step, 0.2, Action::Step::Direction::Forward, false);

    step.waitForFinish();

    for (;;)
    {
        osDelay(1000);
    }
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

    // 等待设备连接
    Device::waitAllConnections();

    // 等待更新
    osDelay(2000);

    // 先完成底盘校准
    if (!Chassis::motion->enable())
        Error_Handler();

    osDelay(1000);

    Chassis::motion->startCalibration();

    while (!Chassis::motion->isReady())
        osDelay(1);

    // TODO: 向上位机返回校准结果

    while (!System::Init::inited())
        osDelay(1);

    // 初始化控制器
    Chassis::enable();

    // 等待启动
    osDelay(1000);

    // 创建其他 tasks
    constexpr osThreadAttr_t autoTaskAttr{
        .name       = "auto-task",
        .stack_size = 1024 * 4,
        .priority   = osPriorityNormal,
    };
    osThreadNew(AutoTask, nullptr, &autoTaskAttr);

    /* 初始化完成后退出线程 */
    osThreadExit();
}
