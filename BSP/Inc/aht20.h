/**
 * @file    aht20.h
 * @brief   AHT20 温湿度传感器驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址 0x38, 挂在 I2C1 总线 (PCA9517 缓冲)
 */
#ifndef __AHT20_H
#define __AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址: HAL 的 I2C 地址是 8 位 (左移 1 位), 0x38→0x70(写)/0x71(读) */
#define AHT20_ADDR                (0x38U << 1)

/* 命令 */
#define AHT20_CMD_INIT            0xBEU   /* 初始化: 加载出厂校准系数 (上电后必须发) */
#define AHT20_CMD_MEASURE         0xACU   /* 触发测量 */
#define AHT20_CMD_MEASURE_ARG0    0x33U   /* 测量参数: 开启温湿度, 默认分辨率 */
#define AHT20_CMD_MEASURE_ARG1    0x00U   /* 保留, 固定 0x00 */

/* 状态寄存器 (0x71): bit7=忙, bit3=校准完成 */
#define AHT20_STATUS_BUSY         (1U << 7)   /* 1=测量中, 0=空闲 */
#define AHT20_STATUS_CALIBRATED   (1U << 3)   /* 1=校准系数已加载, 0=未校准 */

/* 测量数据长度: 一次返回 6 字节 (状态+温湿度原始值) */
#define AHT20_DATA_LEN            6U

/* 返回值 */
typedef enum {
    AHT20_OK = 0,
    AHT20_ERR_I2C,              /* I2C 通信失败 (NACK/超时/仲裁丢失) */
    AHT20_ERR_NOT_CALIBRATED,   /* 传感器校准未完成 (检查 bit3) */
    AHT20_ERR_BUSY,             /* 传感器忙 (上一轮测量未结束) */
    AHT20_ERR_TIMEOUT
} AHT20_Status_t;

/**
 * @brief  初始化 AHT20
 * @param  hi2c: I2C 句柄 (&hi2c1)
 * @param  pSemaphore: 互斥信号量, NULL 则不加锁 (裸机调试用)
 * @retval AHT20_OK 成功, 其他失败
 * @note   发送 0xBE 加载校准系数 → 等 40ms → 检查 bit3
 */
AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);

/**
 * @brief  触发测量并读取温湿度 (一次调用完成)
 * @param  temperature [out]: °C, 可传 NULL
 * @param  humidity    [out]: %RH, 可传 NULL
 * @retval AHT20_OK 成功
 * @note   发 0xAC → osDelay(80ms) → 读 6 字节 → 公式换算
 *         湿度: raw×100/2^20, 温度: raw×200/2^20−50
 */
AHT20_Status_t AHT20_ReadData(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* __AHT20_H */
