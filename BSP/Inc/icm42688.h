/**
 * @file    icm42688.h
 * @brief   ICM-42688 六轴 IMU 驱动 (SPI2, CS=PE7, 与 W25Q128 共享总线)
 * @note    SPI 读写函数已实现, Init/ReadAccel/ReadGyro/ReadTemp 需补充
 */
#ifndef __ICM42688_H
#define __ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* WHO_AM_I 固定值 0x47 */
#define ICM42688_WHO_AM_I_VALUE     0x47U

/*
 * 关键寄存器 (Bank 0):
 *   0x75 WHO_AM_I (只读) | 0x76 BANK_SEL | 0x11 DEVICE_CONFIG(软复位)
 *   0x4E PWR_MGMT0     | 0x4F GYRO_CONFIG0 | 0x50 ACCEL_CONFIG0
 *   0x1D~0x2A 14 字节数据 (温度+加速度+陀螺仪, Big Endian)
 */

/* 加速度量程 (FSR) — 值写入 ACCEL_CONFIG0 bit[7:5] */
typedef enum {
    ICM42688_ACCEL_FSR_16G = 0x00,   /* ±16G,  ~2048 LSB/g */
    ICM42688_ACCEL_FSR_8G  = 0x01,   /* ±8G,   ~4096 LSB/g */
    ICM42688_ACCEL_FSR_4G  = 0x02,   /* ±4G,   ~8192 LSB/g */
    ICM42688_ACCEL_FSR_2G  = 0x03,   /* ±2G,   ~16384 LSB/g */
} ICM42688_AccelFSR_t;

/* 陀螺仪量程 — 值写入 GYRO_CONFIG0 bit[7:5] */
typedef enum {
    ICM42688_GYRO_FSR_2000DPS = 0x00, /* ±2000 °/s, ~16.4 LSB/dps */
    ICM42688_GYRO_FSR_1000DPS = 0x01,
    ICM42688_GYRO_FSR_500DPS  = 0x02,
    ICM42688_GYRO_FSR_250DPS  = 0x03,
    ICM42688_GYRO_FSR_125DPS  = 0x04,
    ICM42688_GYRO_FSR_62DPS   = 0x05,
    ICM42688_GYRO_FSR_31DPS   = 0x06,
    ICM42688_GYRO_FSR_15DPS   = 0x07,
} ICM42688_GyroFSR_t;

/* 返回值 */
typedef enum {
    ICM42688_OK = 0,
    ICM42688_ERR_SPI,
    ICM42688_ERR_TIMEOUT,
    ICM42688_ERR_NOT_FOUND,     /* WHO_AM_I 不匹配 */
    ICM42688_ERR_INVALID_ARG
} ICM42688_Status_t;

/**
 * @brief  初始化 (WHO_AM_I → 软复位 → 电源模式 → 量程)
 * @note   顺序不能错: 软复位后才切到已知状态; 校验通过后才开始配置
 *         PWR_MGMT0=0x0F(加速度+陀螺仪 低噪声), 等 50ms 稳定后再设量程
 */
ICM42688_Status_t ICM42688_Init(SPI_HandleTypeDef *hspi, void *pSemaphore);

/*
 * 以下读取函数换算公式:
 *   加速度(g) = (int16_t)raw / s_accel_lsb
 *   陀螺仪(dps)= (int16_t)raw / s_gyro_lsb
 *   温度(°C)  = (float)raw / 132.48f + 25.0f
 *
 * 14 字节突发读取 (0x1D): 温度2B + 加速度6B + 陀螺仪6B, 全 Big Endian
 * 建议 ReadAccel 和 ReadGyro 共用一次 14 字节读取, 保证数据同帧
 */
ICM42688_Status_t ICM42688_ReadAccel(float *x, float *y, float *z);
ICM42688_Status_t ICM42688_ReadGyro(float *x, float *y, float *z);
ICM42688_Status_t ICM42688_ReadTemp(float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H */
