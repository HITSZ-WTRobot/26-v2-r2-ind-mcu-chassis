/**
 * @file    auto_mapping.cpp
 * @brief   自动建图流程测试
 */
/**
 *      梅林坐标
 *      ---------------------------------
 *      |          |          |         |
 *      |   1.1    |   1.2    |   1.3   |
 *      |          |          |         |
 *      ---------------------------------
 *      |          |          |         |
 *      |   2.1    |   2.2    |   2.3   |
 *      |          |          |         |
 *      ---------------------------------
 *      |          |          |         |
 *      |   3.1    |   3.2    |   3.3   |
 *      |          |          |         |
 *      ---------------------------------
 *      |          |          |         |
 *      |   4.1    |   4.2    |   4.3   |
 *      |          |          |         |
 *      ---------------------------------
 */
#include "tests.hpp"

#include "chassis/actions/Step.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "project_parts.hpp"

#include <cstdint>

namespace Tests::AutoMapping
{

// Ozone 触发变量：
// 0 none
// 1 start automapping
/// 单次触发命令；测试线程读到非 0 后会执行并自动清零。
volatile uint32_t trigger = 0U;

/// 当前编译形态是否具备自动建图所需的本地底盘 + 台阶动作能力。
volatile bool capability_enabled{ false };
/// 当前运行时是否已满足自动建图执行条件：motion ready 且 loc/ctrl 均已创建。
volatile bool runtime_ready{ false };
/// 自动建图脚本当前是否正在执行。
volatile bool running{ false };
/// 最近一次脚本是否已完整执行到结束。
volatile bool finished{ false };
/// 最近一次触发是否成功启动脚本。
volatile bool last_command_ok{ false };
/// 最近一次已消费的触发命令编号回显。
volatile uint32_t last_trigger{ 0U };
/// 当前执行到的阶段编号，便于 Ozone 观察流程进度。
volatile uint32_t stage_index{ 0U };

namespace
{

constexpr float kFront = 0.0f;
constexpr float kRear  = 180.0f;
constexpr float kLeft  = 90.0f;
constexpr float kRight = -90.0f;

constexpr float turnRight = -90.0f;
constexpr float turnLeft  = 90.0f;
constexpr float turnStay  = 0.0f;

constexpr float    kStepDistanceToStep = 0.60f;
constexpr float    kStepDistanceAfter  = 0.60f;
constexpr uint32_t kSettleDelayMs      = 10U;

[[nodiscard]] bool canRunAutoMapping()
{
    return Chassis::motion != nullptr && Chassis::motion->isReady() && Chassis::ctrl != nullptr &&
           Chassis::loc != nullptr;
}

void settle()
{
    osDelay(kSettleDelayMs);
}

void toInWorld(const chassis::Posture& target)
{
    (void)Chassis::ctrl->setTargetPostureInWorld(
            target, Chassis::ChassisController::TrajectoryLinkMode::PreviousCurve);
    Chassis::ctrl->waitTrajectoryFinish();
    settle();
}

void turnInBody(const chassis::Posture& target)
{
    (void)Chassis::ctrl->setTargetPostureInBody(
            target, Chassis::ChassisController::TrajectoryLinkMode::PreviousCurve);
    Chassis::ctrl->waitTrajectoryFinish();
    settle();
}

void stepUp()
{
    auto& step = Action::Step::inst();
    step.up(kStepDistanceToStep, kStepDistanceAfter);
    step.waitForFinish();
    settle();
}

void stepDown()
{
    auto& step = Action::Step::inst();
    step.down(kStepDistanceToStep, kStepDistanceAfter);
    step.waitForFinish();
    settle();
}

bool runAutoMapping()
{
    if (!canRunAutoMapping())
        return false;

    stage_index = 1U;
    toInWorld({ 0.500f, 5.500f, kLeft });
    stage_index = 2U;
    toInWorld({ 8.730f, 5.400f, kFront });
    stage_index = 3U;
    toInWorld({ 8.730f, 0.600f, kRight });
    stage_index = 4U;
    toInWorld({ 0.600f, 0.600f, kRear });
    stage_index = 5U;
    toInWorld({ 0.600f, 5.400f, kLeft });

    stage_index = 6U;
    toInWorld({ 8.730f, 5.400f, kRight });
    stage_index = 7U;
    toInWorld({ 8.730f, 0.600f, kRear });
    stage_index = 8U;
    toInWorld({ 2.600f, 0.600f, kLeft });
    stage_index = 9U;
    toInWorld({ 2.600f, 5.400f, kFront });

    stage_index = 10U;
    toInWorld({ 2.600f, 3.000f, kFront });

    stage_index = 11U;
    stepUp();
    stage_index = 12U;
    turnInBody({ 0.0f, 0.0f, turnRight }); // 到达1.2

    stage_index = 13U;
    stepUp();
    stage_index = 14U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达1.1

    stage_index = 15U;
    stepUp();
    stage_index = 16U;
    turnInBody({ 0.0f, 0.0f, turnStay }); // 到达2.1

    stage_index = 17U;
    stepDown();
    stage_index = 18U;
    turnInBody({ 0.0f, 0.0f, turnStay }); // 到达3.1

    stage_index = 19U;
    stepDown();
    stage_index = 20U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达4.1

    stage_index = 21U;
    stepUp();
    stage_index = 22U;
    turnInBody({ 0.0f, 0.0f, turnStay }); // 到达4.2

    stage_index = 23U;
    stepDown();
    stage_index = 24U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达4.3

    stage_index = 25U;
    stepUp();
    stage_index = 26U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达3.3

    stage_index = 27U;
    stepUp();
    stage_index = 28U;
    turnInBody({ 0.0f, 0.0f, turnRight }); // 到达3.2

    stage_index = 29U;
    stepDown();
    stage_index = 30U;
    turnInBody({ 0.0f, 0.0f, turnRight }); // 到达2.2

    stage_index = 31U;
    stepDown();
    stage_index = 32U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达2.3

    stage_index = 33U;
    stepUp();
    stage_index = 34U;
    turnInBody({ 0.0f, 0.0f, turnLeft }); // 到达1.3

    stage_index = 35U;
    stepDown();
    stage_index = 36U;
    turnInBody({ 0.0f, 0.0f, turnRight }); // 到达1.2

    stage_index = 37U;
    stepDown();

    stage_index = 38U;
    toInWorld({ 0.400f, 1.400f, kFront });
    stage_index = 39U;

    return true;
}

void TestTask(void* argument)
{
    (void)argument;

    for (;;)
    {
        runtime_ready = canRunAutoMapping();

        const uint32_t cmd = trigger;
        if (cmd != 0U)
        {
            trigger         = 0U;
            last_trigger    = cmd;
            last_command_ok = false;

            if (cmd == 1U && runtime_ready && !running)
            {
                running     = true;
                finished    = false;
                stage_index = 0U;

                last_command_ok = runAutoMapping();
                finished        = last_command_ok;
                running         = false;
            }
        }

        osDelay(1);
    }
}

} // namespace

void init()
{
    if constexpr (!Tests::EnableAutoMapping)
        return;

    capability_enabled = ProjectParts::EnableStepWorkflow;
    if constexpr (!ProjectParts::EnableStepWorkflow)
        return;

    constexpr osThreadAttr_t attr{
        .name       = "test-map",
        .stack_size = 336U * 4U,
        .priority   = osPriorityNormal,
    };

    osThreadNew(TestTask, nullptr, &attr);
}

} // namespace Tests::AutoMapping
