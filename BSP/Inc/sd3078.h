/**
 * @file    sd3078.h
 * @brief   SD3078 高精度 RTC 驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址 0x32, TCXO 温补晶振 ±3.8ppm, BCD 编码, 三级写保护
 */
#ifndef __SD3078_H
#define __SD3078_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C 地址 (7 位) */
#define SD3078_ADDR             0x32U

/* 返回值 */
typedef enum {
    SD3078_OK = 0,
    SD3078_ERR_I2C,
    SD3078_ERR_INVALID_PARAM
} SD3078_Status_t;

/*
 * 时间结构体 (公历, 无需 BCD 转换, 驱动内部自动转换)
 * RTC 芯片使用 BCD 码的原因:
 *   高 4 位=十位, 低 4 位=个位, 直接对应数码管送显
 *   BCD ↔ 十进制: ((val>>4)*10)+(val&0x0F) / ((val/10)<<4)|(val%10)
 */
typedef struct {
    uint16_t year;      /* 2000~2099 */
    uint8_t  month;     /* 1~12 */
    uint8_t  day;       /* 1~31 */
    uint8_t  weekday;   /* 0=周日, 1=周一 ... 6=周六 */
    uint8_t  hour;      /* 0~23 (24 小时制) */
    uint8_t  minute;    /* 0~59 */
    uint8_t  second;    /* 0~59 */
} SD3078_Time_t;

/* 充电配置 */
typedef struct {
    uint8_t enable_charge;  /* 0=不配置, 非0=使能 */
    uint8_t charge_value;   /* 充电电阻: 0x82=2KΩ (默认) */
} SD3078_Config_t;

/**
 * @brief  初始化 SD3078
 * @param  hi2c: I2C 句柄
 * @param  pSemaphore: 互斥信号量, NULL 不加锁
 * @param  config: 充电配置, NULL 则跳过充电配置
 * @note   清除首次上电 OSF (晶振停振) 标志; 可选配置备份电池充电
 */
SD3078_Status_t SD3078_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            const SD3078_Config_t *config);

/*
 * 以下函数: 一次 I2C 突发读/写 7 个连续寄存器 (0x00~0x06),
 * 实现原子性 — 避免两次独立读之间时间进位 (比如先读了秒,下一秒才读分)
 */
SD3078_Status_t SD3078_GetTime(SD3078_Time_t *time);
SD3078_Status_t SD3078_SetTime(const SD3078_Time_t *time);

bool SD3078_TimeIsValid(const SD3078_Time_t *time);

/*
 * 读取芯片内置温度传感器值 (用于 TCXO 自动补偿, 用户读它=监控环境)
 * TCXO: 内置温度传感器 → ADC → 查补偿表 → 调负载电容 → 频率稳定
 * 精度 ±3.8ppm 全温范围 vs 普通 RTC ±20~50ppm
 */
SD3078_Status_t SD3078_ReadTemperature(int8_t *temperature);

/*
 * 备份电池电压 (9 位 ADC, LSB=10mV):
 * CONTROL1.bit3=高1位, 0x1B 寄存器=低8位
 * raw=300 → 3.0V
 */
SD3078_Status_t SD3078_ReadBattery(uint16_t *battery_mv);

#ifdef __cplusplus
}
#endif

#endif /* __SD3078_H */
