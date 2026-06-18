/**
 * @file    step_manual.cpp
 * @brief   台阶动作 Ozone 手动触发测试
 */
#include "tests.hpp"

#include "chassis/actions/Step.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "project_parts.hpp"

#include <cstdint>

namespace Tests::StepManual
{

// Ozone 触发变量：
// 0 none
// 1 step up 200
// 2 reserved
// 3 step down 200
// 4 step up 400
// 5 step down 400
/// 单次触发命令；测试线程读到非 0 后会执行并自动清零。
volatile uint32_t trigger = 0U;

// direction:
// 0 forward
// 1 backward
/// 台阶动作方向；`0` 前进方向，`1` 后退方向。
volatile uint32_t direction = 0U;

/// 起始时车体中心到台阶的距离，单位 m。
volatile float start_distance2step = 0.60f;
/// 动作结束时车体中心到台阶的距离，单位 m。
volatile float end_distance2step = 0.60f;
/// 台阶动作结束后的底盘高度；`0=Low`, `1=High`。
volatile uint32_t end_height = 0U;

/// 当前编译形态是否具备本地台阶动作链能力。
volatile bool capability_enabled{ false };
/// 当前运行时是否已满足台阶动作执行条件：motion ready 且 loc/ctrl 均已创建。
volatile bool runtime_ready{ false };
/// 最近一次触发是否成功被接受。
volatile bool last_command_ok{ false };
/// 最近一次已消费的触发命令编号回显。
volatile uint32_t last_trigger{ 0U };

namespace
{

bool decodeDirection(const uint32_t raw, Action::Step::Direction& dir)
{
    if (raw == 0U)
    {
        dir = Action::Step::Direction::Forward;
        return true;
    }

    if (raw == 1U)
    {
        dir = Action::Step::Direction::Backward;
        return true;
    }

    return false;
}

bool decodeEndHeight(const uint32_t raw, Action::Step::FinalHeight& height)
{
    if (raw == 0U)
    {
        height = Action::Step::FinalHeight::Low;
        return true;
    }

    if (raw == 1U)
    {
        height = Action::Step::FinalHeight::High;
        return true;
    }

    return false;
}

void TestTask(void* argument)
{
    (void)argument;

    auto& step = Action::Step::inst();

    for (;;)
    {
        runtime_ready = Chassis::motion != nullptr && Chassis::motion->isReady() &&
                        Chassis::ctrl != nullptr && Chassis::loc != nullptr;

        const uint32_t cmd = trigger;
        if (cmd != 0U)
        {
            trigger         = 0U;
            last_trigger    = cmd;
            last_command_ok = false;

            if (runtime_ready)
            {
                Action::Step::Direction   dir;
                Action::Step::FinalHeight final_height;
                switch (cmd)
                {
                case 1U:
                case 4U:
                    if (decodeDirection(direction, dir) &&
                        decodeEndHeight(end_height, final_height) && !step.isRunning())
                    {
                        const auto height = cmd == 4U ? Action::Step::Height::Step400
                                                      : Action::Step::Height::Step200;
                        step.up(start_distance2step, end_distance2step, dir, final_height, height);
                        last_command_ok = true;
                    }
                    break;
                case 2U:
                    break;
                case 3U:
                case 5U:
                    if (decodeDirection(direction, dir) &&
                        decodeEndHeight(end_height, final_height) && !step.isRunning())
                    {
                        const auto height = cmd == 5U ? Action::Step::Height::Step400
                                                      : Action::Step::Height::Step200;
                        step.down(
                                start_distance2step, end_distance2step, dir, final_height, height);
                        last_command_ok = true;
                    }
                    break;
                default:
                    break;
                }
            }
        }

        osDelay(1);
    }
}

} // namespace

void init()
{
    if constexpr (!Tests::EnableStepManual)
        return;

    capability_enabled = ProjectParts::EnableStepWorkflow;
    if constexpr (!ProjectParts::EnableStepWorkflow)
        return;

    (void)Action::Step::inst();

    constexpr osThreadAttr_t attr{
        .name       = "test-step",
        .stack_size = 1024 * 4U,
        .priority   = osPriorityNormal,
    };

    osThreadNew(TestTask, nullptr, &attr);
}

} // namespace Tests::StepManual
