/**
 * @file    arena.cpp
 * @author  syhanjin
 * @date    2026-02-04
 */
#include "static_arena.hpp"
#include <cstdio>

// 按当前 full-feature + test 线程全开构建的静态高水位 30108 B 取 10% 余量后，
// 将 Arena 收敛到 33 KiB。
static StaticArena<33 * 1024> g_boot_arena;

namespace Arena
{
double get_usage_ratio()
{
    return g_boot_arena.usage_ratio();
}
} // namespace Arena

// 2. 覆盖全局 operator new
void* operator new(const std::size_t size)
{
    void* ptr = g_boot_arena.allocate(size);
    if (!ptr)
    {
        // 嵌入式调试：打印错误并死循环
        // printf("Error: Global Arena Out of Memory! Request: %zu bytes\n", size);
        while (true)
        { /* 触发看门狗或挂起 */
        }
    }
    return ptr;
}

// 必须实现对应的 delete (即便它什么都不做)
void operator delete(void* p) noexcept
{
    // 线性分配器无法回收单块内存，故此处留空
}

// 数组版本
void* operator new[](const std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void* p) noexcept
{
    ::operator delete(p);
}
