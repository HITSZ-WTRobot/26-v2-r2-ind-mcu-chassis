/**
 * @file    step_manual.cpp
 * @brief   台阶动作 Ozone 手动触发测试
 */
#include "tests.hpp"

// 这个文件把 Step 动作拆成几个可单独触发的按钮，方便观察状态机阶段。
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
// 2 step up resume
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
/// `trigger == 1` 时是否在上台阶中途等待取件。
volatile bool will_take{ false };
/// `trigger == 3` 时下台阶完成后是否恢复到底盘正常高度。
volatile bool should_reset{ true };

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
    // Ozone 里只允许输入 0 / 1，其他值直接视为非法。
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
                Action::Step::Direction dir;
                switch (cmd)
                {
                case 1U:
                case 4U:
                    if (decodeDirection(direction, dir) && !step.isRunning())
                    {
                        const auto height = cmd == 4U ? Action::Step::Height::Step400
                                                      : Action::Step::Height::Step200;
                        step.up(start_distance2step, end_distance2step, dir, will_take, height);
                        last_command_ok = true;
                    }
                    break;
                case 2U:
                    if (step.isWaitingTake())
                    {
                        step.resume_up();
                        last_command_ok = true;
                    }
                    break;
                case 3U:
                case 5U:
                    if (decodeDirection(direction, dir) && !step.isRunning())
                    {
                        const auto height = cmd == 5U ? Action::Step::Height::Step400
                                                      : Action::Step::Height::Step200;
                        step.down(
                                start_distance2step, end_distance2step, dir, should_reset, height);
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
