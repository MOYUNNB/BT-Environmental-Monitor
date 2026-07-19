/**
 * @file    aht20.h
 * @brief   AHT20 温湿度传感器驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址: 0x38, 挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 *          使用前需确保 I2C1 已初始化, 且 xSemaphore_I2C 互斥信号量已创建
 */

#ifndef __AHT20_H
#define __AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址 */
#define AHT20_ADDR                (0x38U << 1)  /* 8位地址, HAL 库需要左移1位 */

/* AHT20 命令定义 */
#define AHT20_CMD_INIT            0xBEU   /* 初始化命令 */
#define AHT20_CMD_MEASURE         0xACU   /* 触发测量命令 */
#define AHT20_CMD_MEASURE_ARG0    0x33U   /* 测量参数 byte0 */
#define AHT20_CMD_MEASURE_ARG1    0x00U   /* 测量参数 byte1 */

/* 状态寄存器 bit 定义 */
#define AHT20_STATUS_BUSY         (1U << 7)  /* bit7: 忙标志 */
#define AHT20_STATUS_CALIBRATED   (1U << 3)  /* bit3: 校准完成标志 */

/* 测量数据长度 */
#define AHT20_DATA_LEN            6U

/* 返回值定义 */
typedef enum {
    AHT20_OK = 0,
    AHT20_ERR_I2C,
    AHT20_ERR_NOT_CALIBRATED,
    AHT20_ERR_BUSY,
    AHT20_ERR_TIMEOUT
} AHT20_Status_t;

/**
 * @brief  AHT20 初始化
 * @param  hi2c: I2C 句柄指针 (CubeIDE 生成的 &hi2c1)
 * @param  pSemaphore: I2C 互斥信号量句柄, 传入 NULL 则不加锁 (裸机调试用)
 * @retval AHT20_OK: 成功, 其他: 失败
 */
AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);

/**
 * @brief  读取温湿度数据 (触发测量 + 读取, 一次调用完成)
 * @param  temperature: 输出温度值 (单位: °C, 精度 0.01)
 * @param  humidity: 输出湿度值 (单位: %RH, 精度 0.01)
 * @retval AHT20_OK: 成功, 其他: 失败
 */
AHT20_Status_t AHT20_ReadData(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* __AHT20_H */