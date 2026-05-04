/**
 * @file    tests.cpp
 * @brief   调试测试入口聚合
 */
#include "tests.hpp"

namespace Tests::ChassisLift
{
void init();
}

namespace Tests::GripStandalone
{
void init();
}

namespace Tests::StepManual
{
void init();
}

namespace Tests::SpearGrabManual
{
void init();
}

namespace Tests
{

void init()
{
    ChassisLift::init();
    GripStandalone::init();
    StepManual::init();
    SpearGrabManual::init();
}

} // namespace Tests
