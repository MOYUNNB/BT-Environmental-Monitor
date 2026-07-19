/**
 * @file    at24c02.h
 * @brief   AT24C02 EEPROM 驱动 (I2C, 256 字节, 8 位地址, 页写入 8 字节)
 * @note    配置参数/校准系数存储, 字节级修改无需预擦除 (vs Flash 需擦扇区)
 */
#ifndef __AT24C02_H
#define __AT24C02_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define AT24C02_ADDR            0x50U   /* 7 位 I2C 地址 (A2=A1=A0=GND) */

/* 容量 & 页大小 */
#define AT24C02_SIZE            256U    /* 2Kb = 256 字节 */
#define AT24C02_PAGE_SIZE       8U      /* 页写入最多 8 字节 (越界会回绕到页首) */

typedef enum {
    AT24C02_OK = 0,
    AT24C02_ERR_I2C,
    AT24C02_ERR_INVALID_ADDR    /* 地址 ≥256 */
} AT24C02_Status_t;

/* EEPROM 无需初始化命令, Init 只保存 I2C 句柄和信号量 */
void AT24C02_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);
/* 单字节读: 直接读, 无需等待 (组合逻辑寻址) */
AT24C02_Status_t AT24C02_ReadByte(uint8_t addr, uint8_t *data);
/* 单字节写: 写后等 5ms 内部编程 (页缓冲→存储单元), 期间不响应 I2C */
AT24C02_Status_t AT24C02_WriteByte(uint8_t addr, uint8_t data);
/* 连续读: 一次 I2C 帧完成, 地址自动递增 */
AT24C02_Status_t AT24C02_ReadBuffer(uint8_t addr, uint8_t *data, uint16_t len);

/*
 * 连续写入: 自动按页分割 (跨页计算: page_remain = 8 - addr%8)
 * 每写一页后等 5ms 内部编程完成
 */
AT24C02_Status_t AT24C02_WriteBuffer(uint8_t addr, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AT24C02_H */
