/**
 * @file    ina226.h
 * @brief   INA226 电源监控芯片驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址 0x40, 挂在 I2C1 总线; 采样电阻 15mOhm, 电流 LSB=1mA
 */
#ifndef __INA226_H
#define __INA226_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define INA226_ADDR               (0x40U << 1)  /* 7位→8位: 0x40→0x80(写)/0x81(读) */

/*
 * 寄存器地址映射
 * 电压/电流寄存器是连续转换的结果 (由 Configuration.mode 控制)
 */
#define INA226_REG_CONFIG         0x00U  /* 配置: 平均次数 / 转换时间 / 模式 */
#define INA226_REG_SHUNT_VOLTAGE  0x01U  /* 采样电压, 有符号, LSB=2.5µV */
#define INA226_REG_BUS_VOLTAGE    0x02U  /* 总线电压, 无符号, LSB=1.25mV */
#define INA226_REG_POWER          0x03U  /* 功率, 无符号, LSB=25×Current_LSB */
#define INA226_REG_CURRENT        0x04U  /* 电流, 有符号, LSB=Current_LSB */
#define INA226_REG_CALIBRATION    0x05U  /* 校准: Cal=0.00512/(Current_LSB×R_shunt) */
#define INA226_REG_MASK_ENABLE    0x06U  /* 告警配置 (未使用) */
#define INA226_REG_ALERT_LIMIT    0x07U  /* 告警阈值 (未使用) */
#define INA226_REG_MANUFACTURER   0xFEU  /* 厂商 ID, 只读, 应返回 0x5449 ("TI") */
#define INA226_REG_DIE_ID         0xFFU  /* 版本号 */

/*
 * 配置寄存器 0x4127 的 bit 分解:
 *   bit15   0   = RST (0=不复位)
 *   bit14-13 01 = 平均次数=16  (电源纹波滤波, 但代价是 17.6ms 才一个结果)
 *   bit12-11 00 = Vbus 转换时间=1.1ms
 *   bit10-9  01 = Vshunt 转换时间=1.1ms
 *   bit8-7   00 = 保留
 *   bit6     1   = 保留 (必须写 1)
 *   bit5-4   00 = 保留
 *   bit3     1   = 保留 (必须写 1)
 *   bit2-0   111 = 连续 shunt+bus 测量
 *
 * 为什么选 16 次平均? 工频 50/60Hz 纹波下, 平均相当于低通滤波。
 * 为什么选 1.1ms? 140µs 噪声大, 8.2ms 太慢, 1.1ms 是平衡点。
 */
#define INA226_CONFIG_DEFAULT     0x4127U
#define INA226_CONFIG_RESET       0x8000U  /* 软复位, 所有寄存器回默认 */

/* 返回值 */
typedef enum {
    INA226_OK = 0,
    INA226_ERR_I2C,
    INA226_ERR_NOT_FOUND        /* 厂商 ID 不匹配 */
} INA226_Status_t;

/**
 * @brief  初始化 (复位→写校准→写配置→验证 ID)
 * @param  hi2c: I2C 句柄 (&hi2c1)
 * @param  pSemaphore: 互斥信号量, NULL 则不加锁
 * @param  shuntResistance_mOhm: 采样电阻值 (mOhm), 通常 15
 * @note   关键—Calibration 必须比 Configuration 先写!
 *         Configuration 写入后芯片立即开始转换,
 *         如果 Calibration 还没设好, 电流/功率会以错误比例输出。
 *
 *         Cal = 0.00512 / (Current_LSB × R_shunt)
 *         为什么是 0.00512 而不是 0.08192?
 *         INA226 内部将写入的 Cal 值除以 16 后再用,
 *         Cal_user = 0.08192/16 / (Current_LSB × R_shunt)
 */
INA226_Status_t INA226_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            float shuntResistance_mOhm);

/* 以下读取函数: LSB 为固定值 (见各函数说明) */

/**
 * @brief  读总线电压  V = reg × 1.25mV
 * @note   ADC 可测到 81.92V, 但芯片额定 36V, 本电路实际 3.3~5V
 */
INA226_Status_t INA226_ReadBusVoltage(float *voltage);

/**
 * @brief  读电流  I = (int16_t)reg × Current_LSB
 * @note   电流寄存器由 INA226 根据 shunt 电压和 Cal 内部计算,
 *         不是直接 ADC 读数, 所以必须写对 Cal 值
 */
INA226_Status_t INA226_ReadCurrent(float *current);

/**
 * @brief  读功率  P = reg × 25 × Current_LSB
 * @note   功率是 V_instant × I_instant (瞬时值乘积),
 *         不一定等于 ReadBusVoltage × ReadCurrent (两值可能不同时刻)
 */
INA226_Status_t INA226_ReadPower(float *power);

/**
 * @brief  读 shunt 采样电压 (调试用)  V_shunt = (int16_t)reg × 2.5µV
 * @note   配合 ReadCurrent 验证 V = I×R 是否成立
 */
INA226_Status_t INA226_ReadShuntVoltage(float *shuntVoltage_mV);

/**
 * @brief  一次加锁读三个寄存器 (电压+电流+功率), 减少总线竞争
 * @note   若分三次加锁读, 中间可能被其他任务中断,
 *         导致电压/电流/功率来自不同的转换周期。
 *         一次读保证三者是同一时刻的快照。
 */
INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power);

#ifdef __cplusplus
}
#endif

#endif /* __INA226_H */
