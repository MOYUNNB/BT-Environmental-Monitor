/**
 * @file    ina226.h
 * @brief   INA226 电源监控芯片驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址: 0x40, 挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 *          默认配置: 采样电阻 15mΩ, 电流 LSB = 1mA, 最大电流 = ±6.4A
 *          使用前需确保 I2C1 已初始化, 且 xSemaphore_I2C 互斥信号量已创建
 */

#ifndef __INA226_H
#define __INA226_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址 */
#define INA226_ADDR               (0x40U << 1)  /* 8位地址, HAL 库需要左移1位 */

/* 寄存器地址 */
#define INA226_REG_CONFIG         0x00U
#define INA226_REG_SHUNT_VOLTAGE  0x01U
#define INA226_REG_BUS_VOLTAGE    0x02U
#define INA226_REG_POWER          0x03U
#define INA226_REG_CURRENT        0x04U
#define INA226_REG_CALIBRATION    0x05U
#define INA226_REG_MASK_ENABLE    0x06U
#define INA226_REG_ALERT_LIMIT    0x07U
#define INA226_REG_MANUFACTURER   0xFEU
#define INA226_REG_DIE_ID         0xFFU

/* 配置寄存器常用值 (可根据需要修改) */
#define INA226_CONFIG_DEFAULT     0x4127U  /* 16次平均, 1.1ms转换, 连续模式 (shunt+bus) */
#define INA226_CONFIG_RESET       0x8000U  /* 复位所有寄存器到默认值 */

/* 返回值定义 */
typedef enum {
    INA226_OK = 0,
    INA226_ERR_I2C,
    INA226_ERR_NOT_FOUND
} INA226_Status_t;

/**
 * @brief  INA226 初始化 (复位 → 配置 → 写校准寄存器)
 * @param  hi2c: I2C 句柄指针 (CubeIDE 生成的 &hi2c1)
 * @param  pSemaphore: I2C 互斥信号量句柄, 传入 NULL 则不加锁 (裸机调试用)
 * @param  shuntResistance_mOhm: 采样电阻值 (毫欧), 通常为 15 (即 0.015Ω)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            float shuntResistance_mOhm);

/**
 * @brief  读取总线电压
 * @param  voltage: 输出电压值 (单位: V)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_ReadBusVoltage(float *voltage);

/**
 * @brief  读取电流
 * @param  current: 输出电流值 (单位: A)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_ReadCurrent(float *current);

/**
 * @brief  读取功率
 * @param  power: 输出功率值 (单位: W)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_ReadPower(float *power);

/**
 * @brief  读取采样电阻两端电压 (用于调试)
 * @param  shuntVoltage_mV: 输出采样电压 (单位: mV)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_ReadShuntVoltage(float *shuntVoltage_mV);

/**
 * @brief  一次性读取电压、电流、功率
 * @param  voltage: 输出总线电压 (V)
 * @param  current: 输出电流 (A)
 * @param  power: 输出功率 (W)
 * @retval INA226_OK: 成功, 其他: 失败
 */
INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power);

#ifdef __cplusplus
}
#endif

#endif /* __INA226_H */