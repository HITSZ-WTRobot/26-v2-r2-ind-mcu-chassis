/**
 * @file    app.cpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#include "cmsis_os2.h"

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */

    /* 初始化完成后退出线程 */
    osThreadExit();
}