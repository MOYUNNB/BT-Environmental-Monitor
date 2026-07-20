/**
 * @file    app.c
 * @brief   应用初始化, 按顺序初始化各 BSP 驱动模块
 * @note    App_Init() 在进入 FreeRTOS 调度器之前调用
 *          各任务入口函数由 CubeMX 在 freertos.c 中通过 osThreadNew() 注册
 */
#include "app.h"
#include <stdio.h>
#include "lcd.h"
#include "key.h"
#include "ws2812.h"
#include "bluetooth.h"
#include "aht20.h"
#include "ina226.h"
#include "sd3078.h"
#include "icm42688.h"
#include "tf_card.h"

/* 外部 HAL 句柄 (在 main.c 中定义) */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern osMutexId_t xSemaphore_I2CHandle;
extern osMutexId_t xSemaphore_SPI2Handle;
extern osMutexId_t xSemaphore_SensorDataHandle;

/* 获取 SPI2 互斥锁句柄 */
void *App_GetSPI2Mutex(void)
{
    return (void *)&xSemaphore_SPI2Handle;
}

/**
 * @brief   初始化所有 BSP 驱动
 * @note    先初始化 I2C 外设, 再初始化 SPI/其他外设
 *          CubeMX 已初始化 HAL 外设, 此处只调用驱动 Init
 */
void App_Init(void)
{
    printf("\r\n========== System Init Start ==========\r\n");

    /* 1. LCD 初始化 */
    printf("[LCD] Init... ");
    LCD_Init();
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_DrawString(20, 140, "Initializing...", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);
    printf("OK\r\n");

    /* 2. 按键初始化 */
    printf("[KEY] Init... ");
    KEY_Init();
    printf("OK\r\n");

    /* 3. WS2812 初始化 */
    printf("[WS2812] Init... ");
    WS2812_Init();
    printf("OK\r\n");

    /* 4. 蓝牙初始化 */
    printf("[BT] Init... ");
    BLUETOOTH_Init(&huart2);
    printf("OK\r\n");

    /* 5. 传感器初始化 */
    printf("[AHT20] Init... ");
    if (AHT20_Init(&hi2c1, (void *)&xSemaphore_I2CHandle) == AHT20_OK)
        printf("OK\r\n");
    else
        printf("FAIL\r\n");

    printf("[INA226] Init... ");
    if (INA226_Init(&hi2c1, (void *)&xSemaphore_I2CHandle, 15.0f) == INA226_OK)
        printf("OK\r\n");
    else
        printf("FAIL\r\n");

    printf("[SD3078] Init... ");
    if (SD3078_Init(&hi2c1, (void *)&xSemaphore_I2CHandle, NULL) == SD3078_OK)
        printf("OK\r\n");
    else
        printf("FAIL\r\n");

    printf("[ICM42688] Init... ");
    if (ICM42688_Init(&hspi2, (void *)&xSemaphore_SPI2Handle) == ICM42688_OK)
        printf("OK\r\n");
    else
        printf("FAIL\r\n");

    /* 6. TF 卡初始化 */
    printf("[TF] Init... ");
    if (TF_Init())
        printf("OK\r\n");
    else
        printf("FAIL\r\n");

    printf("========== System Init Complete ==========\r\n");
}
