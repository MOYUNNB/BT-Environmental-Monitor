/**
 * @file    w25q128.c
 * @brief   W25Q128 SPI Flash 驱动实现
 * @note    从嘉立创 fdb 示例工程移植, 增加 SPI 总线互斥信号量保护。
 *          CS 引脚 (PE4) 硬编码以匹配本项目硬件设计。
 *
 * W25Q128 常用指令:
 *   0x06: 写使能        0x05: 读状态寄存器 1
 *   0x02: 页写入(256B)  0x03: 读数据
 *   0x20: 扇区擦除(4K)  0x9F: JEDEC ID
 *   0xAB: 退出掉电
 *
 * ============================================================
 * SPI Flash 操作时序总览
 * ============================================================
 * SPI Flash 的所有操作遵循统一的命令协议:
 *
 *   1. CS 拉低 (选中芯片)
 *   2. 发送命令字节 (如 0x03 读, 0x02 写, 0x20 擦除)
 *   3. [可选] 发送 3 字节地址 (MSB first, 24 位地址, 最大 16MB)
 *   4. [可选] 发送/接收数据字节
 *   5. CS 拉高 (释放芯片)
 *
 * CS 拉高是关键 — Flash 芯片检测到 CS 上升沿时,
 * 认为当前命令结束, 准备接收下一条命令。
 * 如果 CS 拉高不及时, Flash 可能处于未定义状态。
 *
 * CS = GPIO PE4, 与其他 SPI 设备共享同一 SPI 总线:
 *   任何操作前必须通过信号量获取总线独占权,
 *   操作完成后立即释放, 避免阻塞其他 SPI 设备。
 */
#include "w25q128.h"
#include "cmsis_os.h"

/* CS 引脚: PE4 (硬编码, 与项目中 SPI2 总线定义一致) */
#define FLASH_CS_PORT           GPIOE
#define FLASH_CS_PIN            GPIO_PIN_4

/* SPI 命令 */
#define CMD_WRITE_ENABLE        0x06U   /* 写使能 */
#define CMD_READ_STATUS1        0x05U   /* 读状态寄存器 1 */
#define CMD_PAGE_PROGRAM        0x02U   /* 页写入 (最多 256 字节) */
#define CMD_READ_DATA           0x03U   /* 读数据 (任意长度) */
#define CMD_SECTOR_ERASE        0x20U   /* 扇区擦除 (4 KB) */
#define CMD_JEDEC_ID            0x9FU   /* 读 JEDEC ID */
#define CMD_RELEASE_PWR_DOWN    0xABU   /* 释放掉电模式 / 读设备 ID */

/* 状态寄存器 bit */
#define SR1_BUSY                0x01U   /* bit0: 忙标志 (1=忙, 0=空闲) */
#define SR1_WEL                 0x02U   /* bit1: 写使能锁存器 (1=使能) */

#define DEFAULT_TIMEOUT         1000U
#define ERASE_TIMEOUT           5000U   /* 扇区擦除最多约 400ms, 预留余量 */
#define PROGRAM_TIMEOUT         1000U   /* 页写入最多约 3ms */

static SPI_HandleTypeDef *s_hspi = NULL;
static void              *s_sem  = NULL;

static void spi_lock(void)
{
    if (s_sem != NULL)
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}

static void spi_unlock(void)
{
    if (s_sem != NULL)
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
}

/**
 * @brief  选中 Flash (CS 拉低)
 *
 * SPI 设备的 CS (片选) 是低电平有效:
 *   拉低 = 芯片被选中, 响应 SPI 时钟和数据
 *   拉高 = 芯片取消选中, 数据线进入高阻态
 *
 * GPIO_PIN_RESET = 拉低 GPIO 输出寄存器
 *   (STM32 的 GPIO 输出: RESET=0=低电平, SET=1=高电平)
 */
static void cs_select(void)
{
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);
}

static void cs_deselect(void)
{
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
}

static W25Q128_Status_t from_hal(HAL_StatusTypeDef hal)
{
    if (hal == HAL_OK)      return W25Q128_OK;
    if (hal == HAL_TIMEOUT) return W25Q128_ERR_TIMEOUT;
    return W25Q128_ERR_SPI;
}

/**
 * @brief  发送单字节命令 (CS 包裹)
 *
 * SPI 通信步骤:
 *   1. cs_select(): CS 拉低, Flash 进入命令接收状态
 *   2. HAL_SPI_Transmit: MOSI 线发送命令字节,
 *      同时 MISO 线会收到垃圾数据 (因为 SPI 是全双工, 收发同步)
 *   3. cs_deselect(): CS 拉高, 命令执行完成
 *
 * 注意:
 *   纯命令 (如写使能) 不需要地址和数据,
 *   某些命令 (如读状态) 则在发命令后继续发/收数据,
 *   CS 在整个过程中保持低电平。
 */
static W25Q128_Status_t send_cmd(uint8_t cmd)
{
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    cs_deselect();
    return from_hal(hal);
}

/**
 * @brief  写使能 (发送 0x06) + 验证 WEL 位
 *
 * ============================================================
 * 为什么 SPI Flash 需要写使能 (Write Enable) 步骤?
 * ============================================================
 * W25Q128 内部有一个 **写使能锁存器 (Write Enable Latch, WEL)**:
 *   - 每次上电复位、写入完成、擦除完成后, WEL 自动清零
 *   - 执行 0x06 (写使能) 后, WEL 置 1, 允许一次写入/擦除
 *   - 任何写入/擦除完成后 WEL 自动清零
 *
 * 这套机制的目的是 **防误写**:
 *   (a) 上电后 SPI 总线可能有不确定的电平跳变, 但 WEL=0, 不会触发写入
 *   (b) 程序跑飞时, 如果恰好执行了页写入命令 (0x02),
 *       但此前没有执行写使能, Flash 会忽略写入
 *   (c) EMI 干扰导致 SPI 总线上出现类似命令的模式,
 *       但 WEL 必须由精确的 0x06 命令序列设置, EMI 很难模拟
 *
 * 状态寄存器 SR1 bit1 = WEL:
 *   写入 0x06 后, 读 SR1 检查 WEL 位是否置 1,
 *   如果未置 1, 说明 Flash 可能处于保护状态或芯片异常,
 *   返回错误而非继续往下写, 避免静默失败。
 */
static W25Q128_Status_t write_enable(void)
{
    W25Q128_Status_t ret = send_cmd(CMD_WRITE_ENABLE);
    if (ret != W25Q128_OK) return ret;

    uint8_t sr;
    ret = from_hal(HAL_SPI_Receive(s_hspi, &sr, 1U, DEFAULT_TIMEOUT));
    if (ret != W25Q128_OK) return ret;
    /* 读状态寄存器需要先发命令再收 */
    cs_select();
    uint8_t cmd = CMD_READ_STATUS1;
    HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    HAL_SPI_Receive(s_hspi, &sr, 1U, DEFAULT_TIMEOUT);
    cs_deselect();
    return ((sr & SR1_WEL) != 0U) ? W25Q128_OK : W25Q128_ERR_SPI;
}

/**
 * @brief  发送 "命令 + 3 字节地址" (无数据阶段)
 *
 * W25Q128 的地址是 24 位:
 *   位 [23:0] 寻址 16 MB 空间 (2^24 = 16,777,216)
 *
 * 为什么是 24 位?
 *   128 Mbit = 16 MB = 16,777,216 字节
 *   16,777,216 < 2^24 = 16,777,216
 *   刚好 24 位地址线。更大容量的 Flash (如 256 Mbit)
 *   需要 32 位地址。
 *
 * 字节序: Big Endian (大端, MSB first)
 *   buf[1] = addr[23:16] (最高 8 位)
 *   buf[2] = addr[15:8]  (中间 8 位)
 *   buf[3] = addr[7:0]   (最低 8 位)
 *
 * 计算: (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr
 *   右移 16 位取高 8 位, 右移 8 位取中 8 位, 不移动取低 8 位
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

    /**
     * 发送退出掉电命令 (0xAB):
     *   W25Q128 在上电后可能处于 "掉电模式" (Power-down),
     *   这是低功耗状态, 不响应读写命令。
     *   0xAB 命令让芯片恢复正常工作模式。
     *
     * 如果芯片已在正常模式 (复位后默认), 0xAB 命令无副作用,
     * 所以这里无条件发送是安全的。
     */
    (void)send_cmd(CMD_RELEASE_PWR_DOWN);
    HAL_Delay(1U);
}

W25Q128_Status_t W25Q128_ReadJedecId(uint32_t *jedec_id)
{
    if (jedec_id == NULL) return W25Q128_ERR_INVALID_ARG;

    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t id[3];

    /**
     * JEDEC ID 读取时序:
     *   CS 拉低 -> 发 0x9F 命令 -> 接收 3 字节 ID -> CS 拉高
     *
     * 注意: JEDEC ID 命令不需要发送地址!
     *   Flash 有一个内部 ID 寄存器, 收到 0x9F 后
     *   在 MISO 线上依次移出 3 个 ID 字节。
     *
     * SPI 是全双工协议:
     *   在接收 3 字节 ID 时, MOSI 线上需要同时发送虚拟字节
     *   (任意值, HAL_SPI_Receive 内部发 0x00)
     *   因为 SPI 主设备必须提供时钟, 而从设备只在有时钟时移出数据。
     */
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &cmd, 1U, DEFAULT_TIMEOUT);
    if (hal == HAL_OK)
        hal = HAL_SPI_Receive(s_hspi, id, 3U, DEFAULT_TIMEOUT);
    cs_deselect();

    if (hal != HAL_OK) return from_hal(hal);

    /**
     * 将 3 个 ID 字节合并为 32 位整数:
     *   (id[0] << 16) | (id[1] << 8) | id[2]
     *
     *   id[0] = 制造商 (0xEF)
     *   id[1] = 类型     (0x40)
     *   id[2] = 容量     (0x18)
     *
     *   结果: 0x00EF4018
     */
    *jedec_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    return W25Q128_OK;
}

/**
 * @brief  等待 Flash 空闲 (轮询 SR1_BUSY 位)
 *
 * ============================================================
 * 为什么需要检查忙标志?
 * ============================================================
 * Flash 的擦除和写入操作比 SPI 通信慢得多:
 *   - SPI 传输 256 字节: ~12 μs (21 MHz 时钟)
 *   - Flash 编程 256 字节: ~0.64 ms (快者 ~0.4 ms, 慢者 ~3 ms)
 *   - 扇区擦除: ~150 ms (典型值), 最长 ~400 ms
 *
 * 所以每次发送擦除或写入命令后, 必须等待 Flash 内部操作完成,
 * 否则下次命令会被忽略。
 *
 * 轮询方式: 发送 0x05 (读状态寄存器 1) 命令, 接收 1 字节,
 *   检查 bit0 (BUSY):
 *     1 = 芯片忙, 继续轮询
 *     0 = 芯片空闲, 可以接收下一条命令
 *
 * 超时保护:
 *   如果芯片损坏或死锁, BUSY 永远为 1,
 *   用 HAL_GetTick() 计时, 超过 timeout_ms 后返回超时错误,
 *   防止系统卡死在轮询循环中。
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

/**
 * @brief  擦除一个扇区 (4 KB)
 *
 * ============================================================
 * 擦除操作步骤 (为什么需要写使能 + 等待忙结束):
 * ============================================================
 * Flash 的扇区擦除流程是严格顺序的:
 *
 *   1. W25Q128_WaitReady() -- 确保上次操作已完成
 *      如果上次擦除还没结束就发新命令, 新命令会被忽略
 *
 *   2. write_enable()      -- 置 WEL=1
 *      任何修改 Flash 内容的操作 (写/擦除) 都必须先写使能
 *
 *   3. CMD_SECTOR_ERASE (0x20) + 3 字节地址
 *      地址无需对齐到扇区起始, Flash 会自动将地址
 *      向下舍入到最近的扇区边界 (见下面 address & ~4095)
 *      Flash 内部将整个 4KB 扇区全部置 1
 *
 *   4. W25Q128_WaitReady() -- 等待擦除完成
 *      扇区擦除最慢约 400ms, 这里给 5000ms 超时
 *
 * 地址对齐:
 *   sector_addr = address & ~(W25Q128_SECTOR_SIZE - 1U)
 *   4096 = 0x1000, 减去 1 得 0x0FFF
 *   取反: 0xFFFFF000 (低 12 位清零)
 *   所以无论输入什么地址, 都会得到 4KB 对齐的扇区基址
 *   例如 0x00151234 & ~0x0FFF = 0x00151000
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

/**
 * @brief  读数据 (任意地址, 任意长度)
 *
 * 读数据使用 0x03 命令, 时序:
 *   CS 低 -> 发 0x03 -> 发 3 字节地址 -> 接收 N 字节数据 -> CS 高
 *
 * 读操作不需要写使能! 也不会改变 Flash 内容,
 * 所以不需要等待忙状态 (但为了与写/擦除的 SPI 总线互斥保持一致,
 * 这里还是做了 W25Q128_WaitReady)。
 *
 * 为什么读取也要先等不忙?
 *   如果在擦除过程中读 Flash, 读到的数据是未定义的,
 *   而且某些 Flash 在忙时可能不响应读命令。
 *   因此标准做法是在任何操作前先检查总线状态。
 *
 * 分段读取处理:
 *   while 循环 + chunk = min(length, 0xFFFF)
 *   这是因为 HAL_SPI_Receive 的 length 参数是 uint16_t,
 *   最大 65535 字节。如果要读超过 64KB, 需要分段。
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

/**
 * @brief  页写入 (单页, 不超过 256 字节)
 *
 * 这是内部函数, 对外接口 W25Q128_Write 负责跨页分割。
 *
 * 页写入时序:
 *   CS 低 -> 发 0x02 -> 发 3 字节地址 -> 发 N 字节数据 -> CS 高
 *
 * ============================================================
 * 为什么页写入不能跨 256 字节边界?
 * ============================================================
 * W25Q128 内部有一个 256 字节的 **页缓冲 (Page Buffer, SRAM)**:
 *
 *   1. 收到 0x02 命令和地址后, Flash 加载页缓冲地址指针
 *   2. 后续收到的数据依次填入页缓冲
 *   3. CS 拉高后, Flash 开始将页缓冲内容写入存储阵列
 *
 * 关键限制:
 *   地址的低 8 位 (A7~A0) 是页内偏移, 超出 256 后自动回绕!
 *   什么意思? 如果从地址 0x000100 开始写入 300 字节:
 *     前 256 字节写入地址 0x000100~0x0001FF (页 0x01)
 *     后 44 字节会回绕到地址 0x000100~0x00012B (覆盖了开头!)
 *   这不是错误, 而是 Flash 的硬件行为。
 *
 * 所以软件必须在外部按页边界分割写入:
 *   页边界 = N * 256, 即地址低 8 位为 0
 *   计算当前页剩余: 256 - (address % 256)
 *
 * 示例: 写地址 0x0103, 长度 400 字节
 *   第一页: address=0x0103, 剩余=256-3=253, 写 253 字节
 *   第二页: address=0x0200, 剩余=256, 写 256 字节
 *   完成 (写了 253+256=509 > 400, 实际写 253+147)
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

/**
 * @brief  写入数据 (自动跨页分割)
 *
 * 对外接口: 自动处理:
 *   1. 跨页边界分割 (调用 page_program 多次)
 *   2. 每页写入前的等待忙 + 写使能
 *   3. SPI 总线互斥信号量保护
 *
 * 写入前注意事项 (调用者负责):
 *   - 目标扇区必须先擦除 (Flash 只能将 1 变 0, 不能将 0 变 1)
 *   - 如果需要追加写入已有数据, 需先读→修改→擦除→写回
 *
 * 为什么不能在驱动内部自动擦除?
 *   擦除粒度是扇区 (4KB), 如果每次写都自动擦除整个扇区,
 *   扇区内其他数据会丢失。擦除是"破坏性"操作, 必须由上层管理。
 *
 * 跨页计算:
 *   page_remain = W25Q128_PAGE_SIZE - (address % W25Q128_PAGE_SIZE)
 *   例如 address = 0x0103, 0x0103 % 256 = 3
 *   当前页剩余 = 256 - 3 = 253 字节
 *   然后 chunk = min(length, page_remain)
 *   分多次 page_program 调用完成整个写入。
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
