/**
 * @file    w25q128.h
 * @brief   W25Q128 SPI Flash 驱动 (基于 HAL 硬件 SPI)
 * @note    SPI2, CS=PE4, 与 ICM-42688 共享 SPI2 总线
 *          128 Mbit = 16 MB, SPI NOR Flash, 扇区 4KB, 页 256B
 */
#ifndef __W25Q128_H
#define __W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* 容量: 128 Mbit = 16 MB */
#define W25Q128_PAGE_SIZE       256U   /* 页: 最小写入单位 */
#define W25Q128_SECTOR_SIZE     4096U  /* 扇区: 最小擦除单位 (4KB) */
#define W25Q128_FLASH_SIZE      (16U * 1024U * 1024U)

/*
 * JEDEC ID: 0xEF4018
 *   0xEF = Winbond
 *   0x40 = W25Q 系列 (Quad SPI NOR)
 *   0x18 = 128 Mbit
 */
#define W25Q128_JEDEC_ID        0xEF4018UL

/* 返回值 */
typedef enum {
    W25Q128_OK = 0,
    W25Q128_ERR_SPI,
    W25Q128_ERR_TIMEOUT,
    W25Q128_ERR_INVALID_ARG
} W25Q128_Status_t;

/* 保存 SPI 句柄 + 退掉电, 使用前先调一次 */
void W25Q128_Init(SPI_HandleTypeDef *hspi, void *pSemaphore);

/* JEDEC ID: 发 0x9F → 回 3 字节 (制造商+类型+容量), W25Q128 应返回 0xEF4018 */
W25Q128_Status_t W25Q128_ReadJedecId(uint32_t *jedec_id);
/* 轮询 SR1.BUSY, 擦除/写入后必须等芯片内部操作完成 */
W25Q128_Status_t W25Q128_WaitReady(uint32_t timeout_ms);
/* 扇区擦除 (4KB), 地址自动对齐扇区边界, 擦除后全 0xFF */
W25Q128_Status_t W25Q128_EraseSector(uint32_t address);
/* 读任意长度: 0x03 + 3 字节地址 + 连续收, length>0xFFFF 自动分段 */
W25Q128_Status_t W25Q128_Read(uint32_t address, uint8_t *data, size_t length);

/*
 * 写入: 自动跨页分割 (页边界 256B, 地址低 8 位=0)
 * 调用者负责: 写入前目标扇区必须先擦除
 *   (Flash 只能 1→0, 不能 0→1; 擦除将整个扇区置 1)
 */
W25Q128_Status_t W25Q128_Write(uint32_t address, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H */
