/**
 * @file    grip_standalone.cpp
 * @brief   grip 独立 Ozone 调试测试
 */
#include "tests.hpp"

#include "cmsis_os2.h"
#include "grip/Config.hpp"
#include "grip/grip.hpp"
#include "project_parts.hpp"

#include <cstdint>

namespace Tests::GripStandalone
{

// Ozone 调试变量：
// mode:
// 0 stop
// 1 preset
// 2 custom
/// grip 独立测试模式。仅当模式或关联参数变化时，线程才重新下发一次命令。
volatile uint32_t mode = 0U;

// preset:
// 0 standby
// 1 prepare grab
// 2 grab
// 3 docking
// 4 kfs pickup
// 5 kfs store
// 6 kfs release
/// 预设姿态编号；仅在 `mode == 1` 时读取。
volatile uint32_t preset = 0U;

/// 自定义模式下的大臂目标角度，单位 deg。
volatile float custom_arm_pos  = Grip::Config::Poses::Standby.arm_pos;
/// 自定义模式下的转向目标角度，单位 deg。
volatile float custom_turn_pos = Grip::Config::Poses::Standby.turn_pos;
/// 自定义模式下的夹爪开合控制；`false` 张开，`true` 闭合。
volatile bool  custom_claw_close{ false };

/// 当前编译形态是否启用了 grip 能力；测试开启后可先观察此位再调参。
volatile bool     capability_enabled{ false };
/// 当前运行时是否已满足下发条件：grip 已创建且已 enable。
volatile bool     runtime_ready{ false };
/// 最近一次命令是否成功被接受。
volatile bool     last_command_ok{ false };
/// 最近一次真正执行的测试模式回显。
volatile uint32_t applied_mode{ 0U };
/// 最近一次真正执行的预设编号回显。
volatile uint32_t applied_preset{ 0U };
/// 最近一次真正执行的大臂角度回显，单位 deg。
volatile float    applied_arm_pos  = Grip::Config::Poses::Standby.arm_pos;
/// 最近一次真正执行的转向角度回显，单位 deg。
volatile float    applied_turn_pos = Grip::Config::Poses::Standby.turn_pos;
/// 最近一次真正执行的夹爪开合回显。
volatile bool     applied_claw_close{ false };

namespace
{

bool applyPreset(const uint32_t preset_id)
{
    if (::Grip::grip == nullptr)
        return false;

    switch (preset_id)
    {
    case 0U:
        return ::Grip::grip->toStandbyPose();
    case 1U:
        return ::Grip::grip->toPrepareGrabPose();
    case 2U:
        return ::Grip::grip->toGrabPose();
    case 3U:
        return ::Grip::grip->toDockingPose();
    case 4U:
        return ::Grip::grip->toKfsPickupPose();
    case 5U:
        return ::Grip::grip->toKfsStorePose();
    case 6U:
        return ::Grip::grip->toKfsReleasePose();
    default:
        return false;
    }
}

void applyCustom(const bool claw_close, const float arm_pos, const float turn_pos)
{
    if (claw_close)
        ::Grip::grip->closeClaw();
    else
        ::Grip::grip->openClaw();

    last_command_ok = ::Grip::grip->toJointPose({ arm_pos, turn_pos });
}

void TestTask(void* argument)
{
    (void)argument;

    uint32_t last_mode   = mode;
    uint32_t last_preset = preset;
    float    last_arm    = custom_arm_pos;
    float    last_turn   = custom_turn_pos;
    bool     last_claw   = custom_claw_close;
    bool     first_apply = true;

    for (;;)
    {
        runtime_ready = ::Grip::grip != nullptr && ::Grip::grip->enabled();

        const uint32_t desired_mode   = mode;
        const uint32_t desired_preset = preset;
        const float    desired_arm    = custom_arm_pos;
        const float    desired_turn   = custom_turn_pos;
        const bool     desired_claw   = custom_claw_close;

        const bool need_apply = first_apply || desired_mode != last_mode ||
                                desired_preset != last_preset || desired_arm != last_arm ||
                                desired_turn != last_turn || desired_claw != last_claw;

        if (runtime_ready && need_apply)
        {
            last_mode   = desired_mode;
            last_preset = desired_preset;
            last_arm    = desired_arm;
            last_turn   = desired_turn;
            last_claw   = desired_claw;
            first_apply = false;

            applied_mode       = desired_mode;
            applied_preset     = desired_preset;
            applied_arm_pos    = desired_arm;
            applied_turn_pos   = desired_turn;
            applied_claw_close = desired_claw;

            switch (desired_mode)
            {
            case 0U:
                ::Grip::grip->stop();
                last_command_ok = true;
                break;
            case 1U:
                last_command_ok = applyPreset(desired_preset);
                break;
            case 2U:
                applyCustom(desired_claw, desired_arm, desired_turn);
                break;
            default:
                last_command_ok = false;
                break;
            }
        }

        osDelay(1);
    }
}

} // namespace

void init()
{
    if constexpr (!Tests::EnableGripStandalone)
        return;

    capability_enabled = ProjectParts::EnableGrip;
    if constexpr (!ProjectParts::EnableGrip)
        return;

    constexpr osThreadAttr_t attr{
        .name       = "test-grip",
        .stack_size = 512U * 4U,
        .priority   = osPriorityNormal,
    };

    osThreadNew(TestTask, nullptr, &attr);
}

} // namespace Tests::GripStandalone
