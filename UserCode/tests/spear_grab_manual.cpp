/**
 * @file    spear_grab_manual.cpp
 * @brief   spear_grab Ozone 手动触发测试
 */
#include "tests.hpp"

#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "grip/actions/spear_grab.hpp"
#include "grip/grip.hpp"
#include "project_parts.hpp"

#include <cstdint>

namespace Tests::SpearGrabManual
{

// Ozone 触发变量：
// 0 none
// 1 grab
/// 单次触发命令；测试线程读到非 0 后会执行并自动清零。
volatile uint32_t trigger = 0U;

/// 夹取目标世界坐标 x，单位 m。
volatile float target_x   = 0.0f;
/// 夹取目标世界坐标 y，单位 m。
volatile float target_y   = 0.0f;
/// 夹取目标世界坐标 yaw，单位 deg。
volatile float target_yaw = 0.0f;
/// 动作完成后结束位姿 x，单位 m。
volatile float end_x      = 0.30f;
/// 动作完成后结束位姿 y，单位 m。
volatile float end_y      = 0.0f;
/// 动作完成后结束位姿 yaw，单位 deg。
volatile float end_yaw    = 0.0f;

/// 当前编译形态是否具备本地 spear grab 动作链能力。
volatile bool     capability_enabled{ false };
/// 当前运行时是否已满足 spear grab 执行条件：grip enabled，motion ready，loc/ctrl 已创建。
volatile bool     runtime_ready{ false };
/// 最近一次触发是否成功启动动作。
volatile bool     last_command_ok{ false };
/// 最近一次已消费的触发命令编号回显。
volatile uint32_t last_trigger{ 0U };

namespace
{

void TestTask(void* argument)
{
    (void)argument;

    auto& spear = Grip::Action::SpearGrab::inst();

    for (;;)
    {
        runtime_ready = ::Grip::grip != nullptr && ::Grip::grip->enabled() &&
                        Chassis::motion != nullptr && Chassis::motion->isReady() &&
                        Chassis::ctrl != nullptr && Chassis::loc != nullptr;

        const uint32_t cmd = trigger;
        if (cmd != 0U)
        {
            trigger         = 0U;
            last_trigger    = cmd;
            last_command_ok = false;

            if (runtime_ready && cmd == 1U && !spear.isRunning())
            {
                const chassis::Posture target{ .x = target_x, .y = target_y, .yaw = target_yaw };
                const chassis::Posture end{ .x = end_x, .y = end_y, .yaw = end_yaw };
                spear.grab(target, end);
                last_command_ok = spear.isRunning();
            }
        }

        osDelay(1);
    }
}

} // namespace

void init()
{
    if constexpr (!Tests::EnableSpearGrabManual)
        return;

    capability_enabled = ProjectParts::EnableSpearGrabWorkflow;
    if constexpr (!ProjectParts::EnableSpearGrabWorkflow)
        return;

    (void)Grip::Action::SpearGrab::inst();

    constexpr osThreadAttr_t attr{
        .name       = "test-spear",
        .stack_size = 512U * 4U,
        .priority   = osPriorityNormal,
    };

    osThreadNew(TestTask, nullptr, &attr);
}

} // namespace Tests::SpearGrabManual
