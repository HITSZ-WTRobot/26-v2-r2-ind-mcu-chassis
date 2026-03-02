/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim) {}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim) {}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim) {}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    Device_Init();

    APP_Chassis_BeforeUpdate();

    // 启动定时器
    // 实现 1kHz 定时器交替触发，避免总线上同时控制多电机导致需要同时发送超过3条消息的情况
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);

    // 等待设备连接
    Device_WaitAllConnections();

    // 等待更新
    osDelay(2000);

    // 初始化机构
    APP_Chassis_Init();

    // 等待启动
    osDelay(1000);

    // 创建其他 tasks

    /* 初始化完成后退出线程 */
    osThreadExit();
}