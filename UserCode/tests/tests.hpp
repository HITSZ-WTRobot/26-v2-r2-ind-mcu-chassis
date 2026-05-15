/**
 * @file    tests.hpp
 * @brief   调试测试开关与统一入口
 */
#pragma once

// `tests/` 目录下每个 cpp 对应一个独立测试入口。
// 这样可以按文件启停，而不会把临时调试逻辑揉进正式业务代码。

/**
 * 一份 cpp 对应一项测试。
 *
 * `chassis_lift.cpp` 同时承载底盘与升降的联调测试，因此只保留一个总开关；
 * 文件内部再根据 `ProjectParts` 的启用状态，分别决定是否执行底盘 / 升降那部分逻辑。
 */
#ifndef TEST_ENABLE_CHASSIS_LIFT
#    define TEST_ENABLE_CHASSIS_LIFT 0
#endif

#ifndef TEST_ENABLE_GRIP_STANDALONE
#    define TEST_ENABLE_GRIP_STANDALONE 0
#endif

#ifndef TEST_ENABLE_STEP_MANUAL
#    define TEST_ENABLE_STEP_MANUAL 0
#endif

#ifndef TEST_ENABLE_SPEAR_GRAB_MANUAL
#    define TEST_ENABLE_SPEAR_GRAB_MANUAL 0
#endif

#ifndef TEST_ENABLE_AUTO_MAPPING
#    define TEST_ENABLE_AUTO_MAPPING 0
#endif

namespace Tests
{

// 宏开关转成编译期布尔值，便于业务代码里直接做 if constexpr。
inline constexpr bool EnableChassisLift     = TEST_ENABLE_CHASSIS_LIFT != 0;
inline constexpr bool EnableGripStandalone  = TEST_ENABLE_GRIP_STANDALONE != 0;
inline constexpr bool EnableStepManual      = TEST_ENABLE_STEP_MANUAL != 0;
inline constexpr bool EnableSpearGrabManual = TEST_ENABLE_SPEAR_GRAB_MANUAL != 0;
inline constexpr bool EnableAutoMapping     = TEST_ENABLE_AUTO_MAPPING != 0;

void init();

} // namespace Tests
