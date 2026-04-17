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
#include "grip/grip.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    static uint32_t grip_prescaler_500Hz = 0;

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

    // Protocol::init();

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

    // 等待设备连接
    Device::waitAllConnections();

    // 等待更新
    osDelay(2000);

    // 先完成底盘校准
    if (!Chassis::motion->enable())
        Error_Handler();

    if (!Grip::grip->enable())
        Error_Handler();

    osDelay(1000);

    Chassis::motion->startCalibration();

    Grip::grip->startCalibration(); // 电机堵转到限位处进行初始化

    while (!Chassis::motion->isReady() || !Grip::grip->isCalibrated())
        osDelay(1);

    // TODO: 向上位机返回校准结果

    System::Init::posture         = {};
    System::Init::postureReceived = true;
    System::Init::initPostureReceive();

    // while (!System::Init::inited())
    //     osDelay(1);

    // 初始化控制器
    Chassis::enable();
    if (!Grip::grip->enable())
        Error_Handler();

    Grip::grip->toNoworkPose();
    Grip::grip->waitForFinish();

    osDelay(1000);

    Chassis::ctrl->setTargetPostureInWorld({ -2.0f, 0.0f, 0.0f });
    Chassis::ctrl->waitTrajectoryFinish();

    osDelay(500);

    Chassis::ctrl->setTargetPostureInWorld({ -2.0f, -1.2f, 0.0f });
    Chassis::ctrl->waitTrajectoryFinish();

    osDelay(500);

    Chassis::ctrl->setTargetPostureInWorld({ -2.0f, -1.2f, -180.0f });
    Chassis::ctrl->waitTrajectoryFinish();
    osDelay(500);

    Chassis::ctrl->setTargetPostureInWorld({ 0.0f, 0.0f, 0.0f },
                                           { .x   = { 0.5, 0.6, 30 },
                                             .y   = { 0.5, 0.6, 30 },
                                             .yaw = { 45, 60, 360 } });
    Chassis::ctrl->waitTrajectoryFinish();
    //
    // Grip::grip->toReadyPose();
    // Grip::grip->waitForFinish();
    //
    // osDelay(1000);
    //
    // Chassis::ctrl->setTargetPostureInWorld({ -0.20, 0, 0 });
    // Chassis::ctrl->waitTrajectoryFinish();
    //
    // Grip::grip->toGripOutPose();
    // Grip::grip->waitForFinish();
    //
    // osDelay(1000);

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
