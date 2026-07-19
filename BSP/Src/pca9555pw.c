/**
 * @file    pca9555pw.c
 * @brief   PCA9555PW I/O 扩展芯片驱动实现
 * @note    HAL 硬件 I2C, 16 位 GPIO 扩展。
 *          从嘉立创 fdb 示例工程参考, 将 soft_i2c 改为 HAL 硬件 I2C。
 *
 * PCA9555PW 寄存器:
 *   0x00/0x01: 输入寄存器 (只读)
 *   0x02/0x03: 输出寄存器
 *   0x04/0x05: 极性反转寄存器
 *   0x06/0x07: 配置寄存器 (0=输出, 1=输入)
 *
 * ============================================================
 * HAL_I2C_Mem_Read / Mem_Write 模式详解
 * ============================================================
 * PCA9555PW 使用 I2C 存储映射 (Memory-Mapped) 接口,
 * 即通过 I2C 总线发送一个"内部寄存器地址"来指定要操作哪个寄存器。
 * 这不同于某些只响应一个地址的"命令式"外设 (如只响应 0x00 的传感器)。
 *
 * HAL_I2C_Mem_Write 的 I2C 总线时序:
 *   S | 设备地址(W) | A | 寄存器地址 | A | 数据字节0 | A | ... | 数据字节N | A | P
 *   ^ START       ^ACK                    ^ACK                    ^ACK        ^STOP
 *
 * HAL_I2C_Mem_Read 的 I2C 总线时序:
 *   S | 设备地址(W) | A | 寄存器地址 | A | Sr | 设备地址(R) | A | 数据字节0 | N | P
 *   ^ START       ^ACK                    ^重新起始
 *
 * 关键点:
 *   Mem_Read 会在发完寄存器地址后发送一个 **重新起始 (Repeated START, Sr)**,
 *   而不是 STOP + START。这是 I2C 协议的优化:
 *   - 如果 STOP + START, 中间可能被其他主机抢占总线
 *   - 用 Sr 可以原子地完成"指定地址 + 读取数据", 保证事务连续
 *
 * 为什么寄存器地址是 8 位?
 *   因为 PCA9555PW 的寄存器地址范围是 0x00~0x07 (共 8 个寄存器),
 *   8 位地址足够。I2C_MEMADD_SIZE_8BIT 告诉 HAL:
 *   在 Mem_Read/Mem_Write 时发送 1 个字节作为寄存器地址。
 *
 * 为什么 AT24C02 (EEPROM) 需要 16 位地址?
 *   因为 AT24C02 有 256 字节存储空间, 需要 8 位地址;
 *   而 AT24C64 有 8192 字节, 需要 16 位地址。
 *   所以地址位数完全取决于芯片内部存储空间大小。
 *   PCA9555PW 只有 8 个寄存器, 8 位足够 (0x00~0x07)。
 */
#include "pca9555pw.h"
#include "cmsis_os.h"

static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

static void lock(void)
{
    if (s_sem != NULL)
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}

static void unlock(void)
{
    if (s_sem != NULL)
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
}

/**
 * @brief  向 PCA9555PW 指定寄存器写入一个字节
 * @param  reg: 寄存器地址 (0x00~0x07)
 * @param  value: 要写入的值
 * @retval PCA9555PW_OK 或 PCA9555PW_ERR_I2C
 *
 * 使用 HAL_I2C_Mem_Write 发送:
 *   - 设备地址: PCA9555PW_ADDR << 1 (左移 1 位腾出 R/W 位)
 *   - 寄存器地址: reg (8 位, 由 I2C_MEMADD_SIZE_8BIT 指定)
 *   - 数据: 1 字节
 *   - 超时: 100 ms
 *
 * I2C_MEMADD_SIZE_8BIT vs I2C_MEMADD_SIZE_16BIT:
 *   告诉 HAL 在发送寄存器地址时发 1 字节还是 2 字节。
 *   PCA9555PW 的内部寄存器地址只有 8 位, 所以用 8BIT。
 *   如果误用 16BIT, HAL 会多发一个字节 (0x00),
 *   PCA9555PW 可能会将 0x00 视为寄存器地址, 导致操作错误。
 */
static PCA9555PW_Status_t write_reg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(PCA9555PW_ADDR << 1),
                                               (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                                               &value, 1U, 100U);
    return (hal == HAL_OK) ? PCA9555PW_OK : PCA9555PW_ERR_I2C;
}

/**
 * @brief  从 PCA9555PW 指定寄存器读取一个字节
 * @param  reg: 寄存器地址 (0x00~0x07)
 * @param  value: 输出读取的值
 * @retval PCA9555PW_OK 或 PCA9555PW_ERR_I2C
 *
 * 使用 HAL_I2C_Mem_Read, 内部时序:
 *   S + 设备地址(W) + 寄存器地址 + Sr + 设备地址(R) + 数据 + N + P
 *
 * 注意:
 *   读取输入寄存器 (0x00/0x01) 时, 读取到的是调用瞬间的引脚电平快照,
 *   不是持续采样。如果需要监视引脚变化, 需要定时轮询或使用 PCA9555PW
 *   的中断输出引脚 (INT, 当输入引脚状态变化时拉低)。
 */
static PCA9555PW_Status_t read_reg(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(PCA9555PW_ADDR << 1),
                                              (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                                              value, 1U, 100U);
    return (hal == HAL_OK) ? PCA9555PW_OK : PCA9555PW_ERR_I2C;
}

void PCA9555PW_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    s_hi2c = hi2c;
    s_sem  = pSemaphore;
}

PCA9555PW_Status_t PCA9555PW_SetDirection(uint8_t config0, uint8_t config1)
{
    PCA9555PW_Status_t ret;

    lock();
    ret = write_reg(PCA9555PW_REG_CONFIG0, config0);
    if (ret == PCA9555PW_OK)
        ret = write_reg(PCA9555PW_REG_CONFIG1, config1);
    unlock();

    return ret;
}

PCA9555PW_Status_t PCA9555PW_WriteOutput(uint8_t output0, uint8_t output1)
{
    PCA9555PW_Status_t ret;

    lock();
    ret = write_reg(PCA9555PW_REG_OUTPUT0, output0);
    if (ret == PCA9555PW_OK)
        ret = write_reg(PCA9555PW_REG_OUTPUT1, output1);
    unlock();

    return ret;
}

PCA9555PW_Status_t PCA9555PW_ReadInput(uint8_t *input0, uint8_t *input1)
{
    PCA9555PW_Status_t ret;

    lock();
    ret = read_reg(PCA9555PW_REG_INPUT0, input0);
    if (ret == PCA9555PW_OK)
        ret = read_reg(PCA9555PW_REG_INPUT1, input1);
    unlock();

    return ret;
}

PCA9555PW_Status_t PCA9555PW_SetPolarity(uint8_t polarity0, uint8_t polarity1)
{
    PCA9555PW_Status_t ret;

    lock();
    ret = write_reg(PCA9555PW_REG_POLARITY0, polarity0);
    if (ret == PCA9555PW_OK)
        ret = write_reg(PCA9555PW_REG_POLARITY1, polarity1);
    unlock();

    return ret;
}
