/**
 * @file    pca9555pw.h
 * @brief   PCA9555PW I/O 扩展芯片驱动 (I2C, 16 GPIO, 0x20)
 * @note    需要更多 GPIO 时通过 I2C 扩展, 挂在 I2C1 总线上
 */
#ifndef __PCA9555PW_H
#define __PCA9555PW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define PCA9555PW_ADDR          0x20U   /* I2C 地址 (7 位) */

/*
 * 寄存器映射:
 *   0x00 INPUT0  只读 | 0x01 INPUT1  只读 — 引脚当前电平
 *   0x02 OUTPUT0 R/W  | 0x03 OUTPUT1 R/W — 输出锁存
 *   0x04 POLARITY0     | 0x05 POLARITY1    — 输入极性反转 (0=正常 1=取反)
 *   0x06 CONFIG0       | 0x07 CONFIG1    — 方向 (0=输出 1=输入, 默认全输入)
 */
#define PCA9555PW_REG_INPUT0    0x00U
#define PCA9555PW_REG_INPUT1    0x01U
#define PCA9555PW_REG_OUTPUT0   0x02U
#define PCA9555PW_REG_OUTPUT1   0x03U
#define PCA9555PW_REG_POLARITY0 0x04U
#define PCA9555PW_REG_POLARITY1 0x05U
#define PCA9555PW_REG_CONFIG0   0x06U   /* bit=0 推挽输出, bit=1 高阻输入 */
#define PCA9555PW_REG_CONFIG1   0x07U

typedef enum {
    PCA9555PW_OK = 0,
    PCA9555PW_ERR_I2C
} PCA9555PW_Status_t;

/* 保存 I2C 句柄和信号量 */
void PCA9555PW_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);
/* 方向: bit=0 输出, bit=1 输入 (默认全输入) */
PCA9555PW_Status_t PCA9555PW_SetDirection(uint8_t config0, uint8_t config1);
/* 写两个输出锁存器 (port0 + port1) */
PCA9555PW_Status_t PCA9555PW_WriteOutput(uint8_t output0, uint8_t output1);
/* 读引脚实时电平 (port0 + port1) */
PCA9555PW_Status_t PCA9555PW_ReadInput(uint8_t *input0, uint8_t *input1);
/* 极性翻转: bit=1 输入取反, 常用于按键 (低效触发变高) */
PCA9555PW_Status_t PCA9555PW_SetPolarity(uint8_t polarity0, uint8_t polarity1);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9555PW_H */
