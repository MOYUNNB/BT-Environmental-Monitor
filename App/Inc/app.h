/**
 * @file    app.h
 * @brief   应用初始化入口
 * @note    提供 App_Init() 函数, 按顺序初始化各 BSP 驱动模块
 *          FreeRTOS 任务创建由 CubeMX 生成的 MX_FREERTOS_Init() 完成
 */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_data.h"
#include "app_conf.h"
#include <stdint.h>

/**
 * @brief   应用初始化: 初始化所有 BSP 驱动
 * @note    在 osKernelInitialize() 之前调用 (裸机阶段)
 *          初始化顺序: LCD → KEY → WS2812 → BLUETOOTH → TF
 */
void App_Init(void);

/**
 * @brief   获取 xSemaphore_SPI2 互斥锁句柄
 * @note    供 BSP 驱动传递信号量指针
 */
void *App_GetSPI2Mutex(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
