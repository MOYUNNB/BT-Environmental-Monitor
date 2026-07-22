/**
 * @file    icm42688.c
 * @brief   ICM-42688 六轴 IMU 驱动实现
 * @note    SPI2, CS=PE7, 与 W25Q128 共享 SPI2 总线 (互斥锁保护)
 *          突发读取 14 字节 (TEMP + ACCEL + GYRO) 一次完成
 */
#include "icm42688.h"
#include "cmsis_os.h"

/* CS = PE7 */
#define IMU_CS_PORT             GPIOE
#define IMU_CS_PIN              GPIO_PIN_7

/* 模块内部变量 */
static SPI_HandleTypeDef *s_hspi = NULL;
static void              *s_sem  = NULL;
static float              s_accel_lsb = 2048.0f;  /* ±16G */
static float              s_gyro_lsb  = 16.4f;    /* ±2000dps */

/* ---- SPI 总线锁 ---- */
static void spi_lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void spi_unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/* ---- CS 控制 ---- */
static void cs_select(void)   { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static void cs_deselect(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET); }

/**
 * @brief  SPI 写寄存器
 */
static ICM42688_Status_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, tx, 2U, 100U);
    cs_deselect();
    return (hal == HAL_OK) ? ICM42688_OK : ICM42688_ERR_SPI;
}

/**
 * @brief  SPI 突发读多个寄存器 (CS 全程低电平)
 */
static ICM42688_Status_t icm42688_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80U);
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &addr, 1U, 100U);
    if (hal == HAL_OK)
        hal = HAL_SPI_Receive(s_hspi, data, len, 100U);
    cs_deselect();
    return (hal == HAL_OK) ? ICM42688_OK : ICM42688_ERR_SPI;
}

/* ========== 对外接口 ========== */

ICM42688_Status_t ICM42688_Init(SPI_HandleTypeDef *hspi, void *pSemaphore)
{
    s_hspi = hspi;
    s_sem  = pSemaphore;
    cs_deselect();
    HAL_Delay(1U);

    uint8_t whoami = 0;
    ICM42688_Status_t ret;

    spi_lock();

    /* 1. 切到 Bank 0 */
    ret = icm42688_write_reg(0x76, 0x00);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }

    /* 2. 软复位 */
    ret = icm42688_write_reg(0x11, 0x01);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    HAL_Delay(10);

    /* 3. 读取 WHO_AM_I 校验 (应返回 0x47) */
    ret = icm42688_read_regs(0x75, &whoami, 1);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    if (whoami != ICM42688_WHO_AM_I_VALUE) { spi_unlock(); return ICM42688_ERR_NOT_FOUND; }

    /* 4. 加速度+陀螺仪低噪声模式 */
    ret = icm42688_write_reg(0x4E, 0x0F);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    HAL_Delay(50);

    /* 5. 加速度计 ±16G, 1kHz ODR */
    ret = icm42688_write_reg(0x50, (ICM42688_ACCEL_FSR_16G << 5) | 0x07);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    s_accel_lsb = 2048.0f;

    /* 6. 陀螺仪 ±2000dps, 1kHz ODR */
    ret = icm42688_write_reg(0x4F, (ICM42688_GYRO_FSR_2000DPS << 5) | 0x07);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    s_gyro_lsb = 16.4f;

    spi_unlock();
    return ICM42688_OK;
}

/*
 * ICM42688_ReadAll — 一次 SPI 突发读 14 字节 (0x1D~0x2A)
 *   偏移 0-1:  TEMP_DATA      偏移 2-3:  ACCEL_DATA_X
 *   偏移 4-5:  ACCEL_DATA_Y   偏移 6-7:  ACCEL_DATA_Z
 *   偏移 8-9:  GYRO_DATA_X    偏移 10-11: GYRO_DATA_Y
 *   偏移 12-13: GYRO_DATA_Z
 */
ICM42688_Status_t ICM42688_ReadAll(float *accel_x, float *accel_y, float *accel_z,
                                   float *gyro_x,  float *gyro_y,  float *gyro_z,
                                   float *temp_c)
{
    uint8_t buf[14];

    spi_lock();
    ICM42688_Status_t ret = icm42688_read_regs(0x1D, buf, 14);
    spi_unlock();
    if (ret != ICM42688_OK) return ret;

    /* 温度 */
    if (temp_c != NULL) {
        int16_t raw_t = (int16_t)((uint16_t)(buf[0]) << 8U | buf[1]);
        *temp_c = (float)raw_t / 132.48f + 25.0f;
    }

    /* 加速度 */
    if (accel_x != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[2]) << 8U | buf[3]);
        *accel_x = (float)raw / s_accel_lsb;
    }
    if (accel_y != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[4]) << 8U | buf[5]);
        *accel_y = (float)raw / s_accel_lsb;
    }
    if (accel_z != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[6]) << 8U | buf[7]);
        *accel_z = (float)raw / s_accel_lsb;
    }

    /* 陀螺仪 */
    if (gyro_x != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[8])  << 8U | buf[9]);
        *gyro_x = (float)raw / s_gyro_lsb;
    }
    if (gyro_y != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[10]) << 8U | buf[11]);
        *gyro_y = (float)raw / s_gyro_lsb;
    }
    if (gyro_z != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[12]) << 8U | buf[13]);
        *gyro_z = (float)raw / s_gyro_lsb;
    }

    return ICM42688_OK;
}

/*
 * 以下函数保留向后兼容, 内部均委托给 ICM42688_ReadAll。
 * 新代码请直接使用 ICM42688_ReadAll (一次 SPI 突发更高效且数据同帧)。
 */
ICM42688_Status_t ICM42688_ReadAccel(float *x, float *y, float *z)
{
    return ICM42688_ReadAll(x, y, z, NULL, NULL, NULL, NULL);
}

ICM42688_Status_t ICM42688_ReadGyro(float *x, float *y, float *z)
{
    return ICM42688_ReadAll(NULL, NULL, NULL, x, y, z, NULL);
}

ICM42688_Status_t ICM42688_ReadTemp(float *temp_c)
{
    return ICM42688_ReadAll(NULL, NULL, NULL, NULL, NULL, NULL, temp_c);
}
