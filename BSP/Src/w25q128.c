/**
 * @file    w25q128.c
 * @brief   W25Q128 SPI Flash 驱动实现
 * @note    从嘉立创 fdb 示例工程移植, 增加 SPI 总线互斥锁保护
 *          CS=PE4 (硬编码, 与 ICM-42688 共享 SPI2)
 */
#include "w25q128.h"
#include "cmsis_os.h"

#define FLASH_CS_PORT           GPIOE
#define FLASH_CS_PIN            GPIO_PIN_4

/*
 * 常用指令:
 *   0x06 写使能 | 0x05 读状态 | 0x02 页写入(256B) | 0x03 读数据
 *   0x20 扇区擦除(4K) | 0x9F JEDEC ID | 0xAB 退出掉电
 */
#define CMD_WRITE_ENABLE        0x06U
#define CMD_READ_STATUS1        0x05U
#define CMD_PAGE_PROGRAM        0x02U
#define CMD_READ_DATA           0x03U
#define CMD_SECTOR_ERASE        0x20U
#define CMD_JEDEC_ID            0x9FU
#define CMD_RELEASE_PWR_DOWN    0xABU

/* 状态寄存器 bit */
#define SR1_BUSY                0x01U   /* bit0=忙, 擦除/写入期间=1 */
#define SR1_WEL                 0x02U   /* bit1=写使能锁存器 WEL */

#define DEFAULT_TIMEOUT         1000U
#define ERASE_TIMEOUT           5000U   /* 扇区擦除最长 ~400ms, 留余量 */

static SPI_HandleTypeDef *s_hspi = NULL;
static void              *s_sem  = NULL;

static void spi_lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void spi_unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/* CS 低电平有效 */
static void cs_select(void)   { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET); }
static void cs_deselect(void) { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET); }

static W25Q128_Status_t from_hal(HAL_StatusTypeDef hal)
{
    if (hal == HAL_OK)      return W25Q128_OK;
    if (hal == HAL_TIMEOUT) return W25Q128_ERR_TIMEOUT;
    return W25Q128_ERR_SPI;
}

/* 发单字节命令 (CS 包裹) */
static W25Q128_Status_t send_cmd(uint8_t cmd)
{
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    cs_deselect();
    return from_hal(hal);
}

/*
 * 写使能 (0x06): SPI Flash 内部 WEL 写使能锁存器
 *   上电/写入/擦除完成后 WEL 自动清零
 *   任何写入/擦除前必须置 WEL=1, 否则命令被忽略
 *   这是防误写机制: EMI 干扰很难精确模拟 0x06 指令序列
 */
static W25Q128_Status_t write_enable(void)
{
    W25Q128_Status_t ret = send_cmd(CMD_WRITE_ENABLE);
    if (ret != W25Q128_OK) return ret;

    /* 读状态寄存器验证 WEL 位是否置 1 */
    uint8_t sr;
    ret = from_hal(HAL_SPI_Receive(s_hspi, &sr, 1U, DEFAULT_TIMEOUT));
    if (ret != W25Q128_OK) return ret;
    cs_select();
    uint8_t cmd = CMD_READ_STATUS1;
    HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    HAL_SPI_Receive(s_hspi, &sr, 1U, DEFAULT_TIMEOUT);
    cs_deselect();
    return ((sr & SR1_WEL) != 0U) ? W25Q128_OK : W25Q128_ERR_SPI;
}

/*
 * 发 "命令 + 3 字节地址" (24 位地址, 寻址 16MB, 大端序)
 *   buf[0]=cmd, buf[1]=addr[23:16], buf[2]=addr[15:8], buf[3]=addr[7:0]
 */
static W25Q128_Status_t send_addr_cmd(uint8_t cmd, uint32_t addr)
{
    uint8_t buf[4] = {cmd,
                      (uint8_t)(addr >> 16),
                      (uint8_t)(addr >> 8),
                      (uint8_t)addr};
    return from_hal(HAL_SPI_Transmit(s_hspi, buf, 4U, DEFAULT_TIMEOUT));
}

/* ---- 对外接口 ---- */

void W25Q128_Init(SPI_HandleTypeDef *hspi, void *pSemaphore)
{
    s_hspi = hspi;
    s_sem  = pSemaphore;
    cs_deselect();

    /* 退出掉电模式 (0xAB), 芯片可能处于低功耗状态 */
    (void)send_cmd(CMD_RELEASE_PWR_DOWN);
    HAL_Delay(1U);
}

W25Q128_Status_t W25Q128_ReadJedecId(uint32_t *jedec_id)
{
    if (jedec_id == NULL) return W25Q128_ERR_INVALID_ARG;

    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t id[3];

    /*
     * 发 0x9F → 收 3 字节 (不需要地址, Flash 内部 ID 寄存器自动输出)
     * SPI 全双工: 接收时 MOSI 需同时发哑字节提供时钟
     *   (HAL_SPI_Receive 内部发 0x00)
     */
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    if (hal == HAL_OK)
        hal = HAL_SPI_Receive(s_hspi, id, 3U, DEFAULT_TIMEOUT);
    cs_deselect();

    if (hal != HAL_OK) return from_hal(hal);

    /*
     * 合并: id[0]<<16 | id[1]<<8 | id[2]
     * W25Q128 应返回 0xEF4018
     * 如果返回 0xFFFFFF → SPI 通信失败或芯片损坏
     */
    *jedec_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    return W25Q128_OK;
}

/*
 * 轮询状态寄存器 SR1_BUSY: 擦除(150ms)/写入(~3ms) 后必须等待完成
 * 因为 SPI 传输快 (微秒级), Flash 内部操作慢 (毫秒级)
 * 不等待就发下一条命令会被忽略
 */
W25Q128_Status_t W25Q128_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t sr;
    uint8_t cmd = CMD_READ_STATUS1;

    do {
        cs_select();
        HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
        if (hal == HAL_OK)
            hal = HAL_SPI_Receive(s_hspi, &sr, 1U, DEFAULT_TIMEOUT);
        cs_deselect();

        if (hal != HAL_OK) return from_hal(hal);
        if ((sr & SR1_BUSY) == 0U) return W25Q128_OK;
    } while ((HAL_GetTick() - start) < timeout_ms);

    return W25Q128_ERR_TIMEOUT;
}

/*
 * 扇区擦除: 写使能 → 0x20 + 地址 → 等忙结束
 * 地址自动对齐到 4KB 边界 (address & ~(4095))
 */
W25Q128_Status_t W25Q128_EraseSector(uint32_t address)
{
    if (address >= W25Q128_FLASH_SIZE) return W25Q128_ERR_INVALID_ARG;

    uint32_t sector_addr = address & ~(W25Q128_SECTOR_SIZE - 1U);

    spi_lock();
    W25Q128_Status_t ret = W25Q128_WaitReady(DEFAULT_TIMEOUT);
    if (ret == W25Q128_OK) ret = write_enable();
    if (ret == W25Q128_OK) {
        cs_select();
        ret = send_addr_cmd(CMD_SECTOR_ERASE, sector_addr);
        cs_deselect();
    }
    if (ret == W25Q128_OK)
        ret = W25Q128_WaitReady(ERASE_TIMEOUT);
    spi_unlock();

    return ret;
}

/*
 * 读数据: 0x03 + 3 字节地址 → 收 N 字节
 * 读前也 WaitReady (擦除中读数据可能读到未定义值)
 * 分段处理: length>0xFFFF 时拆多次 (HAL SPI_Receive 用 uint16_t)
 */
W25Q128_Status_t W25Q128_Read(uint32_t address, uint8_t *data, size_t length)
{
    if (data == NULL && length > 0U) return W25Q128_ERR_INVALID_ARG;
    if (address + length > W25Q128_FLASH_SIZE) return W25Q128_ERR_INVALID_ARG;
    if (length == 0U) return W25Q128_OK;

    spi_lock();
    W25Q128_Status_t ret = W25Q128_WaitReady(DEFAULT_TIMEOUT);
    if (ret == W25Q128_OK) {
        cs_select();
        ret = send_addr_cmd(CMD_READ_DATA, address);
        while (ret == W25Q128_OK && length > 0U) {
            uint16_t chunk = (length > 0xFFFFU) ? 0xFFFFU : (uint16_t)length;
            ret = from_hal(HAL_SPI_Receive(s_hspi, data, chunk, DEFAULT_TIMEOUT));
            data += chunk;
            length -= chunk;
        }
        cs_deselect();
    }
    spi_unlock();

    return ret;
}

/*
 * 单页写入 (<256 字节, 不跨页)
 * 调用者负责: 目标扇区必须先擦除 (Flash 只能 1→0)
 */
static W25Q128_Status_t page_program(uint32_t address, const uint8_t *data, size_t length)
{
    W25Q128_Status_t ret = W25Q128_WaitReady(DEFAULT_TIMEOUT);
    if (ret != W25Q128_OK) return ret;

    ret = write_enable();
    if (ret != W25Q128_OK) return ret;

    cs_select();
    ret = send_addr_cmd(CMD_PAGE_PROGRAM, address);
    if (ret == W25Q128_OK)
        ret = from_hal(HAL_SPI_Transmit(s_hspi, (uint8_t *)data, (uint16_t)length, DEFAULT_TIMEOUT));
    cs_deselect();

    if (ret != W25Q128_OK) return ret;
    return W25Q128_WaitReady(PROGRAM_TIMEOUT);
}

/*
 * 写入: 自动跨页分割
 *
 * 页边界限制: 地址低 8 位=0 为页起始 (256 对齐)
 * 如果写入跨越页边界, Flash 的地址指针会回绕到页首, 覆盖已有数据!
 * 所以必须分段: 先算当前页剩余, chunk = min(length, page_remain)
 */
W25Q128_Status_t W25Q128_Write(uint32_t address, const uint8_t *data, size_t length)
{
    if (data == NULL && length > 0U) return W25Q128_ERR_INVALID_ARG;
    if (address + length > W25Q128_FLASH_SIZE) return W25Q128_ERR_INVALID_ARG;
    if (length == 0U) return W25Q128_OK;

    spi_lock();
    while (length > 0U) {
        size_t page_remain = W25Q128_PAGE_SIZE - (address % W25Q128_PAGE_SIZE);
        size_t chunk = (length < page_remain) ? length : page_remain;

        W25Q128_Status_t ret = page_program(address, data, chunk);
        if (ret != W25Q128_OK) {
            spi_unlock();
            return ret;
        }

        address += (uint32_t)chunk;
        data += chunk;
        length -= chunk;
    }
    spi_unlock();

    return W25Q128_OK;
}
