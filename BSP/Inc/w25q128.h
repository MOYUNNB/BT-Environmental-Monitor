/**
 * @file    w25q128.h
 * @brief   W25Q128 SPI Flash 驱动 (基于 HAL 硬件 SPI)
 *
 * ============================================================
 * 1. SPI Flash vs EEPROM 的区别
 * ============================================================
 * 两者都是非易失性存储器, 但设计理念和适用场景完全不同:
 *
 * ┌──────────────┬───────────────────────┬───────────────────────┐
 * │ 特性         │ SPI Flash (W25Q128)   │ EEPROM (AT24C02)      │
 * ├──────────────┼───────────────────────┼───────────────────────┤
 * │ 容量         │ 大 (Mb~Gb级) 128Mb   │ 小 (Kb级) 2Kb~256Kb  │
 * │ 擦除单位     │ 扇区 (4KB) 或整片    │ 字节级                │
 * │ 写入单位     │ 页 (256B)            │ 字节级                │
 * │ 写入前操作   │ 必须先擦除           │ 可直接写 (0→1 需擦除) │
 * │ 擦除寿命     │ ~10万次              │ ~100万次               │
 * │ 读速度       │ 快 (SPI 时钟频率)    │ 快                    │
 * │ 写速度       │ 快 (页写入)          │ 慢 (每字节 ~5ms)      │
 * │ 典型成本/MB  │ 极低                 │ 高                    │
 * │ 典型用途     │ 固件存储, 文件系统   │ 配置参数, 校准数据    │
 * └──────────────┴───────────────────────┴───────────────────────┘
 *
 * W25Q128 为 128 Mbit (16 MB), 适合存储: 中文字库、图片、录音文件、数据日志。
 *
 * ============================================================
 * 2. JEDEC ID 的含义
 * ============================================================
 * JEDEC ID 是 JEDEC 标准化组织定义的 Flash 芯片标识码,
 * 由 3 个字节组成, 通过 SPI 命令 0x9F 读取:
 *
 *   字节 1 = 制造商 ID
 *     Winbond (华邦) = 0xEF
 *     Macronix (旺宏) = 0xC2
 *     Micron (美光) = 0x2C
 *     ISSI  = 0x9D
 *     GigaDevice = 0xC8
 *
 *   字节 2 = 存储器类型
 *     W25Q 系列 = 0x40 (Quad I/O SPI NOR Flash)
 *
 *   字节 3 = 容量
 *     0x18 = 128 Mbit (即 W25Q128)
 *     0x17 = 64 Mbit  (W25Q64)
 *     0x19 = 256 Mbit (W25Q256)
 *
 * 所以 W25Q128 的 JEDEC ID = 0xEF4018
 *
 * 为什么需要 JEDEC ID?
 *   初始化时读取 JEDEC ID 验证:
 *   - 确认 SPI 通信正常 (时序、极性、相位正确)
 *   - 确认连接的芯片符合预期型号
 *   - 如果返回 0xFFFFFF 或全 0, 说明 SPI 通信失败或芯片损坏
 *
 * ============================================================
 * 3. 扇区 (Sector) / 页 (Page) / 字节 (Byte) 层级关系
 * ============================================================
 * W25Q128 的存储层次:
 *
 *   整片 (Chip) = 16 MB = 128 Mbit
 *     └─ 块 (Block, 64 KB) x256 个
 *          └─ 扇区 (Sector, 4 KB) x4096 个
 *               └─ 页 (Page, 256 B) x65536 个
 *                    └─ 字节 (Byte, 1 B) x16,777,216 个
 *
 * 擦除操作的最小单位: 扇区 (4 KB)
 *   也可以擦除 32KB 块、64KB 块或整片
 *   但擦除扇区最常用, 因为粒度适中:
 *     - 整片擦除 (4s) vs 扇区擦除 (150ms)
 *     - 如果只改几字节, 擦除整片太慢
 *
 * 写入操作的最小单位: 页 (256 B)
 *   虽然可以写 1 字节, 但页写入 256 字节是最快的,
 *   因为 Flash 会先把数据缓存到内部 256B SRAM 页缓冲,
 *   然后一次性写入存储阵列。
 *
 * 读取操作的最小单位: 字节
 *   可以任意地址、任意长度连续读取, 无限制。
 *
 * 为何 Flash 不能字节级覆盖写?
 *   Flash 的存储单元是浮栅晶体管 (Floating Gate MOSFET):
 *     擦除 = 将电子从浮栅中移出 → 单元变为 1 (高阻)
 *     写入 = 将电子注入浮栅     → 单元变为 0 (导通)
 *   电子注入是逐位的 (1→0 可独立), 但移出电子需要大电压擦除,
 *   且擦除的最小单位是扇区 (因为浮栅共享擦除电路)。
 *   所以: 擦除将扇区全部置 1 → 写入将需要的位从 1 变 0。
 *
 * SPI2 接口, CS = PE4, 与 ICM-42688 共享 SPI2 总线
 * 使用前需确保 SPI2 已初始化, 且 xSemaphore_SPI2 互斥信号量已创建
 *
 * === 移植说明 ===
 * 从嘉立创 fdb 示例工程移植, 保持 HAL SPI 接口不变,
 * 增加信号量保护以适配 FreeRTOS + SPI 总线共享场景。
 */
#ifndef __W25Q128_H
#define __W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* 容量定义 */
#define W25Q128_PAGE_SIZE       256U   /* 页大小: 256 字节 */
#define W25Q128_SECTOR_SIZE     4096U  /* 扇区大小: 4 KB   */
#define W25Q128_FLASH_SIZE      (16U * 1024U * 1024U)  /* 128 Mbit = 16 MB */

/**
 * JEDEC ID (华邦)
 *   0xEF = Winbond 制造商 ID
 *   0x40 = 存储器类型 (W25Q 系列 Quad SPI)
 *   0x18 = 128 Mbit 容量
 */
#define W25Q128_JEDEC_ID        0xEF4018UL

/* 返回值定义 */
typedef enum {
    W25Q128_OK = 0,
    W25Q128_ERR_SPI,
    W25Q128_ERR_TIMEOUT,
    W25Q128_ERR_INVALID_ARG
} W25Q128_Status_t;

/**
 * @brief  初始化 W25Q128
 * @param  hspi: SPI 句柄指针 (&hspi2)
 * @param  pSemaphore: SPI 互斥信号量, NULL 则不加锁
 */
void W25Q128_Init(SPI_HandleTypeDef *hspi, void *pSemaphore);

/**
 * @brief  读取 JEDEC ID
 * @param  jedec_id: 输出 JEDEC ID (W25Q128: 0xEF4018)
 * @retval W25Q128_OK: 成功
 */
W25Q128_Status_t W25Q128_ReadJedecId(uint32_t *jedec_id);

/**
 * @brief  等待 Flash 空闲
 * @param  timeout_ms: 超时时间 (ms)
 * @retval W25Q128_OK: 成功
 */
W25Q128_Status_t W25Q128_WaitReady(uint32_t timeout_ms);

/**
 * @brief  擦除一个扇区 (4KB)
 * @param  address: 扇区内任意地址
 * @retval W25Q128_OK: 成功
 */
W25Q128_Status_t W25Q128_EraseSector(uint32_t address);

/**
 * @brief  读取数据
 * @param  address: 读取起始地址 (0 ~ 16MB-1)
 * @param  data: 输出缓冲区
 * @param  length: 读取长度
 * @retval W25Q128_OK: 成功
 */
W25Q128_Status_t W25Q128_Read(uint32_t address, uint8_t *data, size_t length);

/**
 * @brief  写入数据 (跨页自动分割)
 * @param  address: 写入起始地址
 * @param  data: 要写入的数据
 * @param  length: 写入长度
 * @note   写入前需先擦除对应扇区
 * @retval W25Q128_OK: 成功
 */
W25Q128_Status_t W25Q128_Write(uint32_t address, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H */
