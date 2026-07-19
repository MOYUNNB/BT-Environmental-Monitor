/**
 * @file    pca9555pw.c
 * @brief   PCA9555PW I/O 扩展芯片驱动实现 (HAL I2C, 16 位 GPIO)
 * @note    存储映射寄存器: 输入/输出/极性/方向, 通过 I2C Mem_Read/Write 访问
 */
#include "pca9555pw.h"
#include "cmsis_os.h"

static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

static void lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/*
 * Mem_Write 时序: S + 设备地址(W) + 寄存器地址 + 数据 + P
 * Mem_Read 时序:  S + 设备地址(W) + 寄存器地址 + Sr + 设备地址(R) + 数据 + N + P
 * 关键: 先用 Repeated START (Sr) 而非 STOP+START, 保证事务原子性
 */
static PCA9555PW_Status_t write_reg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(PCA9555PW_ADDR << 1),
                                               (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                                               &value, 1U, 100U);
    return (hal == HAL_OK) ? PCA9555PW_OK : PCA9555PW_ERR_I2C;
}

static PCA9555PW_Status_t read_reg(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(PCA9555PW_ADDR << 1),
                                              (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                                              value, 1U, 100U);
    return (hal == HAL_OK) ? PCA9555PW_OK : PCA9555PW_ERR_I2C;
}

/* 保存 I2C 句柄和信号量, 外部调用前需先 Init */
void PCA9555PW_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    s_hi2c = hi2c;
    s_sem  = pSemaphore;
}

/* config: bit=0 推挽输出, bit=1 高阻输入 (默认全输入) */
PCA9555PW_Status_t PCA9555PW_SetDirection(uint8_t config0, uint8_t config1)
{
    PCA9555PW_Status_t ret;
    lock();
    ret = write_reg(PCA9555PW_REG_CONFIG0, config0);
    if (ret == PCA9555PW_OK) ret = write_reg(PCA9555PW_REG_CONFIG1, config1);
    unlock();
    return ret;
}

/* 写两个输出寄存器 (0-out + 1-out), 一个失败则整个操作回滚 */
PCA9555PW_Status_t PCA9555PW_WriteOutput(uint8_t output0, uint8_t output1)
{
    PCA9555PW_Status_t ret;
    lock();
    ret = write_reg(PCA9555PW_REG_OUTPUT0, output0);
    if (ret == PCA9555PW_OK) ret = write_reg(PCA9555PW_REG_OUTPUT1, output1);
    unlock();
    return ret;
}

/* 读两个输入寄存器, 一次锁内完成, 保证两个端口来自同一采样时刻 */
PCA9555PW_Status_t PCA9555PW_ReadInput(uint8_t *input0, uint8_t *input1)
{
    PCA9555PW_Status_t ret;
    lock();
    ret = read_reg(PCA9555PW_REG_INPUT0, input0);
    if (ret == PCA9555PW_OK) ret = read_reg(PCA9555PW_REG_INPUT1, input1);
    unlock();
    return ret;
}

/* 极性翻转: bit=1 时输入取反, 常用于按键检测 (低电平触发反提高)
   polarity0 = PORT0 极性, polarity1 = PORT1 极性 */
PCA9555PW_Status_t PCA9555PW_SetPolarity(uint8_t polarity0, uint8_t polarity1)
{
    PCA9555PW_Status_t ret;
    lock();
    ret = write_reg(PCA9555PW_REG_POLARITY0, polarity0);
    if (ret == PCA9555PW_OK) ret = write_reg(PCA9555PW_REG_POLARITY1, polarity1);
    unlock();
    return ret;
}
