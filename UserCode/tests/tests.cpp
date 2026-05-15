/**
 * @file    tests.cpp
 * @brief   调试测试入口聚合
 */
#include "tests.hpp"

// 这里只负责聚合各测试入口的 init()，不承载任何具体测试逻辑。
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

namespace Tests::AutoMapping
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
    AutoMapping::init();
}

} // namespace Tests
