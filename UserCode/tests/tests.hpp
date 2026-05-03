/**
 * @file    tests.hpp
 * @brief   调试测试开关与统一入口
 */
#pragma once

/**
 * 一份 cpp 对应一项测试。
 *
 * `chassis_lift.cpp` 同时承载底盘与升降的联调测试，因此只保留一个总开关；
 * 文件内部再根据 `ProjectParts` 的启用状态，分别决定是否执行底盘 / 升降那部分逻辑。
 */
#ifndef TEST_ENABLE_CHASSIS_LIFT
#    define TEST_ENABLE_CHASSIS_LIFT 0
#endif

namespace Tests
{

inline constexpr bool EnableChassisLift = TEST_ENABLE_CHASSIS_LIFT != 0;

void init();

} // namespace Tests
