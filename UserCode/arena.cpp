/**
 * @file    arena.cpp
 * @author  syhanjin
 * @date    2026-02-04
 */
#include "static_arena.hpp"
#include <cstdio>

// 全局 new/delete 都走这个静态 arena。
// 设计目标是让所有动态对象在启动阶段一次性分配，避免嵌入式运行期堆碎片。
static StaticArena<72 * 1024> g_boot_arena;

namespace Arena
{
double get_usage_ratio()
{
    return g_boot_arena.usage_ratio();
}
} // namespace Arena

// 覆盖全局 operator new，项目中所有 `new` 都会落到这块静态内存。
void* operator new(const std::size_t size)
{
    void* ptr = g_boot_arena.allocate(size);
    if (!ptr)
    {
        // 分配失败说明 arena 太小或对象生命周期设计不合理。
        // 这里选择停机，便于调试器和看门狗暴露问题。
        while (true)
        { /* 触发看门狗或挂起 */
        }
    }
    return ptr;
}

// 必须实现对应的 delete，即使这个 arena 不支持释放单块内存。
void operator delete(void* p) noexcept
{
    (void)p;
}

// 数组版本复用同一块 arena。
void* operator new[](const std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void* p) noexcept
{
    ::operator delete(p);
}
