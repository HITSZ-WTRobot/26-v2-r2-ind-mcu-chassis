/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "can.h"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "chassis/LiftSide.hpp"
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

// 0 stop
// 1 vel
// 2 pos
size_t s_chassis = 0;

// 0 前后协同
// 1 控制前腿
// 2 控制后腿
size_t s_lift      = 0;
float  z_pos       = Chassis::Config::Lift::Position::Normal;
float  z_front_pos = z_pos;
float  z_rear_pos  = z_pos;

Chassis::Config::Limit lift_limit = Chassis::Config::Lift::DefaultLimit;

chassis::Posture  pos;
chassis::Velocity vel;

void TestTask(void* argument)
{
    chassis::Posture  _p  = pos;
    chassis::Velocity _v  = vel;
    float             _zp = z_pos, _zfp = z_front_pos, _zrp = z_rear_pos;
    for (;;)
    {
        // chassis controll
        if (s_chassis == 0)
        {
            Chassis::ctrl->stop();
        }
        else if (s_chassis == 1)
        {
            if (_v.vx != vel.vx || _v.vy != vel.vy || _v.wz != vel.wz)
            {
                _v = vel;
                Chassis::ctrl->setVelocityInBody(_v, false);
            }
        }
        else if (s_chassis == 2)
        {
            if (_p.x != pos.x || _p.y != pos.y || _p.yaw != pos.yaw)
            {
                _p = pos;
                Chassis::ctrl->setTargetPostureInWorld(_p);
            }
        }
        if (s_lift == 0)
        {
            if (_zp != z_pos)
            {
                _zp = z_pos, _zfp = z_pos, _zrp = z_pos;
                Chassis::motion->liftAllTo(z_pos, lift_limit);
            }
        }
        else if (s_lift == 1)
        {
            if (_zfp != z_front_pos)
            {
                _zfp = z_front_pos;
                Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front)
                        .to(z_front_pos, lift_limit);
            }
        }
        else if (s_lift == 2)
        {
            if (_zrp != z_rear_pos)
            {
                _zrp = z_rear_pos;
                Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear)
                        .to(z_rear_pos, lift_limit);
            }
        }
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

    // 初始化机构
    Chassis::enable();

    Chassis::motion->startCalibration();

    while (!Chassis::motion->isReady())
        osDelay(1);

    // 等待启动
    osDelay(1000);

    // 创建其他 tasks
    constexpr osThreadAttr_t test_attr{
        .name       = "test-task",
        .stack_size = 1024 * 4,
        .priority   = osPriorityNormal1,
    };

    osThreadNew(TestTask, nullptr, &test_attr);

    /* 初始化完成后退出线程 */
    osThreadExit();
}