/**
 * @file    aht20.c
 * @brief   AHT20 温湿度传感器驱动实现
 * @note    从嘉立创 fdb 示例工程提取, 改为 HAL 硬件 I2C, 增加信号量保护
 *
 * 数据转换公式 (来自 AHT20 数据手册):
 *   湿度 = (raw_humi * 100) / 2^20
 *   温度 = (raw_temp * 200) / 2^20 - 50
 */

#include "aht20.h"
#include "cmsis_os.h"   /* FreeRTOS 信号量, 若不用 RTOS 可删除此行和锁相关代码 */

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c   = NULL;   /* I2C 句柄 */
static void              *s_sem    = NULL;   /* 互斥信号量 */

/* ---- 内部辅助函数 ---- */

static void i2c_lock(void)
{
    if (s_sem != NULL) {
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
    }
}

static void i2c_unlock(void)
{
    if (s_sem != NULL) {
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
    }
}

/**
 * @brief  读取 AHT20 状态寄存器 (0x71)
 */
static uint8_t aht20_read_status(void)
{
    uint8_t status = 0xFFU;
    /* 状态寄存器地址 0x71, 读 1 字节 */
    HAL_I2C_Mem_Read(s_hi2c, AHT20_ADDR, 0x71U, I2C_MEMADD_SIZE_8BIT,
                     &status, 1U, 100U);
    return status;
}

/* ---- 对外接口 ---- */

AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    uint8_t cmd[3];
    uint8_t status;

    s_hi2c = hi2c;
    s_sem  = pSemaphore;

    /* 发送初始化命令: 0xBE, 0x08, 0x00 */
    cmd[0] = AHT20_CMD_INIT;
    cmd[1] = 0x08U;
    cmd[2] = 0x00U;

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /* 等待校准完成 (数据手册要求 10ms, 给 40ms 余量) */
    osDelay(40U);

    /* 检查校准标志 */
    i2c_lock();
    status = aht20_read_status();
    i2c_unlock();

    if (!(status & AHT20_STATUS_CALIBRATED)) {
        return AHT20_ERR_NOT_CALIBRATED;
    }

    return AHT20_OK;
}

AHT20_Status_t AHT20_ReadData(float *temperature, float *humidity)
{
    uint8_t  cmd[3];
    uint8_t  data[AHT20_DATA_LEN];
    uint32_t raw_humi;
    uint32_t raw_temp;

    /* 1. 发送触发测量命令 */
    cmd[0] = AHT20_CMD_MEASURE;
    cmd[1] = AHT20_CMD_MEASURE_ARG0;
    cmd[2] = AHT20_CMD_MEASURE_ARG1;

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /* 2. 等待测量完成 (数据手册典型 80ms) */
    osDelay(80U);

    /* 3. 读取 6 字节数据 */
    i2c_lock();
    if (HAL_I2C_Master_Receive(s_hi2c, AHT20_ADDR, data, AHT20_DATA_LEN, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /* 4. 解析数据 */
    raw_humi = ((uint32_t)data[1] << 12U)
             | ((uint32_t)data[2] << 4U)
             | ((uint32_t)data[3] >> 4U);

    raw_temp = (((uint32_t)data[3] & 0x0FU) << 16U)
             | ((uint32_t)data[4] << 8U)
             |  (uint32_t)data[5];

    /* 5. 换算为工程值 (公式来源: AHT20 datasheet) */
    if (humidity != NULL) {
        *humidity = (float)raw_humi * 100.0f / 1048576.0f;       /* 2^20 = 1048576 */
    }
    if (temperature != NULL) {
        *temperature = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;
    }

    return AHT20_OK;
}