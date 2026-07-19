/**
 * @file    at24c02.c
 * @brief   AT24C02 EEPROM 驱动实现 (HAL I2C, 页写入自动跨页分割, 5ms 内部编程)
 */
#include "at24c02.h"
#include "cmsis_os.h"

static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

static void lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/* 保存 I2C 句柄和信号量, 之后所有读写操作都走这两个静态变量 */
void AT24C02_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    s_hi2c = hi2c;
    s_sem  = pSemaphore;
}

/* 单字节读: 一次 Mem_Read 完成, EEPROM 地址自动递增, 无需跨页处理 */
AT24C02_Status_t AT24C02_ReadByte(uint8_t addr, uint8_t *data)
{
    if (addr >= AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;
    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                              (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                              data, 1U, 100U);
    unlock();
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

/* 单字节写: 等内部编程完成 (5ms) 再发下一帧, 否则 EEPROM NACK */
AT24C02_Status_t AT24C02_WriteByte(uint8_t addr, uint8_t data)
{
    if (addr >= AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;
    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                               (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                               &data, 1U, 100U);
    unlock();
    /* 写入后芯片开始 5ms 内部编程, 此间 I2C 不响应 (NACK) */
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

/*
 * 连续读: 一次 I2C 帧完成 (组合逻辑寻址, 无需等待)
 * 地址自动递增, 从 addr 读到 addr+len-1
 */
AT24C02_Status_t AT24C02_ReadBuffer(uint8_t addr, uint8_t *data, uint16_t len)
{
    if (addr + len > AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;
    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                              (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                              data, len, 100U);
    unlock();
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

/*
 * 连续写入: 自动按页分割
 *
 * EEPROM 页缓冲区地址计数器是 3 位 — 越界会回绕到页首!
 * 例如地址 7 写 3 字节: 写到地址 7→8(实际是0)→1, 覆盖开头!
 *
 * 所以必须分页: 每页最多写 page_remain 字节, 然后等 5ms 内部编程
 *
 * 也可以改用 ACK 轮询 (HAL_I2C_IsDeviceReady) 替代固定 5ms,
 * 最快 ~2ms 就能继续, 但代码复杂度略增
 */
AT24C02_Status_t AT24C02_WriteBuffer(uint8_t addr, const uint8_t *data, uint16_t len)
{
    if (addr + len > AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;

    while (len > 0U) {
        uint16_t page_remain = AT24C02_PAGE_SIZE - (addr % AT24C02_PAGE_SIZE);
        uint16_t chunk = (len < page_remain) ? len : page_remain;

        lock();
        HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                                   (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                                   (uint8_t *)data, chunk, 100U);
        unlock();

        if (hal != HAL_OK) return AT24C02_ERR_I2C;

        addr += (uint8_t)chunk;
        data += chunk;
        len  -= chunk;

        /* 等内部编程完成 */
        HAL_Delay(5U);
    }

    return AT24C02_OK;
}
