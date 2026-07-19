/**
 * @file    at24c02.h
 * @brief   AT24C02 EEPROM 驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址: 0x50, 256 字节 (8 位地址寻址)
 *          挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 *          标准页写入: 一次最多 8 字节
 *
 *          === 移植说明 ===
 *          直接使用 HAL 硬件 I2C, 无特殊算法, 可直接移植使用。
 *
 * ===== EEPROM vs Flash 区别 =====
 *
 *   特性         | EEPROM (AT24C02)          | NOR Flash (W25Q128)
 *   -------------|---------------------------|----------------------------
 *   擦除单位     | 字节级 (无需预擦除)        | 扇区/块 (4KB/64KB)
 *   写入单位     | 字节或页 (8/16/32/64 字节) | 页 (256 字节)
 *   写入前擦除   | 不需要                     | 需要 (擦除 = 所有位写 1)
 *   擦除寿命     | 100 万次                   | 10 万次
 *   读取速度     | 慢 (I2C, 400kHz)           | 快 (SPI, 几十 MHz)
 *   写入速度     | 慢 (~5ms/byte)             | 快 (~1ms/page)
 *   数据保持     | 40 年                      | 20 年
 *   容量范围     | 1Kb ~ 1Mb                  | 1Mb ~ 512Mb
 *   成本/Byte    | 高                         | 低
 *
 *   AT24C02 的优势:
 *   - 可以单字节修改, 不需要整块擦除
 *   - 适合存储配置参数、校准数据 (少量、频繁修改)
 *   - 掉电不丢失
 *
 *   Flash 的优势:
 *   - 大容量、高速
 *   - 适合存储固件、字库、数据记录
 *
 *   【存储层次在本系统中的应用】
 *   AT24C02 (256B): 系统参数、校准系数、WiFi/BLE 配置、用户偏好
 *   W25Q128 (128Mb): 字库、图片、音频、历史记录
 *   TF 卡 (FATFS): 日志文件、数据导出
 *
 * ===== AT24C02 页写入限制原理 =====
 *
 *   【为什么有页写入限制?】
 *   1. EEPROM 内部有一个"页缓冲区" (Page Buffer), 大小 8 字节
 *   2. 页写入时, 数据先暂存在页缓冲区, 然后触发内部编程
 *   3. 页缓冲区的地址计数器是 8 字节的模计数器
 *   4. 当地址到达页边界时, 计数器会回绕到页内基地址
 *
 *   【页边界现象】
 *   假设页大小为 8:
 *   - 地址 0~7 属于第 0 页, 地址 8~15 属于第 1 页
 *   - 从地址 7 开始写 3 个字节: 数据写入 7, 然后回绕到 0, 覆盖了 0, 1 地址的内容
 *   - 这就是"页回绕" (Page Rollover) 问题
 *
 *   【解决方案】
 *   每次写入前检查是否跨越页边界
 *   如果跨越, 分成两次写入
 *   详见 at24c02.c 中 AT24C02_WriteBuffer 的实现
 *
 * ===== I2C Mem_Read 和普通读的区别 =====
 *
 *   HAL_I2C_Mem_Read 专门用于"先写子地址, 再读数据"的场景:
 *
 *   【Mem_Read 时序】
 *   START + DEV_ADDR(W) + ACK + MEM_ADDR + ACK
 *     → RESTART
 *     → START + DEV_ADDR(R) + ACK + DATA... + STOP
 *
 *   【普通 Master_Receive 时序】
 *   START + DEV_ADDR(R) + ACK + DATA... + STOP
 *
 *   区别: Mem_Read 多了"写子地址"这一步
 *
 *   EEPROM 为什么需要 Mem_Read?
 *   - EEPROM 内部有多个存储单元, 需要"地址"来指定读哪个位置
 *   - 普通读只能从当前内部指针位置读
 *   - Mem_Read 先设置内部指针, 再读取
 *   - AT24C02 只有 256 字节, 所以 MEM_ADDR 是 8 位 (1 字节)
 *   - 如果是更大的 EEPROM (如 AT24C256, 32KB), MEM_ADDR 是 16 位 (2 字节)
 *     这时需要将 I2C_MEMADD_SIZE_8BIT 改为 I2C_MEMADD_SIZE_16BIT
 */
#ifndef __AT24C02_H
#define __AT24C02_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C 设备地址 (7 位) */
#define AT24C02_ADDR            0x50U
                                        /* 注意: AT24C02 的 A2/A1/A0 引脚决定地址  */
                                        /* A2=A1=A0=GND → 7位地址 = 0x50           */
                                        /* 8位地址 = 0xA0 (写) / 0xA1 (读)          */
                                        /* 如果用 HAL 的 Mem_Read:                   */
                                        /*   传入 7 位地址左移 1 位 (HAL 要求)       */

/**
 * @brief  存储容量
 *   AT24C02 = 2Kb (Kilo-bit) = 256 × 8 字节
 *   因为容量小, 使用 8 位地址即可寻址全部空间
 *   地址范围: 0x00 ~ 0xFF (0 ~ 255)
 */
#define AT24C02_SIZE            256U    /* 256 字节 */

/**
 * @brief  页大小 (Page Size)
 *   AT24C02 的页写入缓冲区大小是 8 字节
 *
 *   不同型号 EEPROM 的页大小:
 *   AT24C01 (1Kb)   = 8 字节
 *   AT24C02 (2Kb)   = 8 字节 (当前型号)
 *   AT24C04 (4Kb)   = 16 字节
 *   AT24C08 (8Kb)   = 16 字节
 *   AT24C16 (16Kb)  = 16 字节
 *   AT24C32 (32Kb)  = 32 字节
 *   AT24C64 (64Kb)  = 32 字节
 *   AT24C128 (128Kb)= 64 字节
 *   AT24C256 (256Kb)= 64 字节
 *
 *   页大小与容量成正比, 容量越大页越大
 */
#define AT24C02_PAGE_SIZE       8U      /* 页写入最大 8 字节 */

/* 返回值定义 */
typedef enum {
    AT24C02_OK = 0,             /* 操作成功 */
    AT24C02_ERR_I2C,            /* I2C 通信失败 (NACK/超时/仲裁丢失) */
    AT24C02_ERR_INVALID_ADDR    /* 地址超出范围 (≥256) */
} AT24C02_Status_t;

/**
 * @brief  初始化 AT24C02 (仅保存 I2C 句柄和信号量)
 * @param  hi2c: I2C 句柄指针
 * @param  pSemaphore: I2C 互斥信号量, NULL 则不加锁
 *
 * @note   AT24C02 不需要发送初始化命令 (不同于 AHT20/INA226)
 *         EEPROM 上电即可读写, 所以 Init 只保存指针
 *         这样可以节省 I2C 总线的一次不必要访问
 */
void AT24C02_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);

/**
 * @brief  从 EEPROM 读取一字节
 * @param  addr: 字节地址 (0 ~ 255)
 * @param  data: 输出数据
 * @retval AT24C02_OK: 成功
 *
 * @note   读取不需要等待, EEPROM 的读操作是组合逻辑完成的
 *         (内部地址译码器直接输出数据到 I2C 移位寄存器)
 *         写入才需要等待 5ms 内部编程时间
 */
AT24C02_Status_t AT24C02_ReadByte(uint8_t addr, uint8_t *data);

/**
 * @brief  向 EEPROM 写入一字节
 * @param  addr: 字节地址 (0 ~ 255)
 * @param  data: 要写入的数据
 * @retval AT24C02_OK: 成功
 * @note   写入后需 5ms 等待内部编程完成
 *
 *         内部流程: 字节加载到页缓冲区 → 内部编程(5ms) → 数据写入存储单元
 *         在此期间 I2C 接口不响应任何命令 (必须 5ms 后才能再次访问)
 */
AT24C02_Status_t AT24C02_WriteByte(uint8_t addr, uint8_t data);

/**
 * @brief  连续读取多字节 (跨页自动处理)
 * @param  addr: 起始地址
 * @param  data: 输出缓冲区
 * @param  len: 读取长度
 * @retval AT24C02_OK: 成功
 *
 * @note   EEPROM 的读操作支持"顺序读取" (Sequential Read):
 *         每读一个字节, 内部地址自动加 1
 *         到达 0xFF 后回绕到 0x00 (地址计数器回绕)
 *         不需要像写操作那样分页处理, 因为读没有页缓冲区限制
 *
 *         注意: addr + len 不能超过 256, 否则返回 AT24C02_ERR_INVALID_ADDR
 */
AT24C02_Status_t AT24C02_ReadBuffer(uint8_t addr, uint8_t *data, uint16_t len);

/**
 * @brief  页写入多字节 (跨页自动分割)
 * @param  addr: 起始地址
 * @param  data: 要写入的数据
 * @param  len: 写入长度
 * @retval AT24C02_OK: 成功
 *
 * @note   写入流程:
 *         1. 计算当前页的剩余字节数: page_remain = PAGE_SIZE - (addr % PAGE_SIZE)
 *         2. 本次写入 chunk = min(len, page_remain) 字节
 *         3. 发送 chunk 数据 (使用 I2C Mem_Write)
 *         4. 等待 5ms 内部编程完成
 *         5. 更新 addr, data, len
 *         6. 如果 len > 0, 回到步骤 1
 *
 *         为什么需要跨页分割? 见文件顶部页写入限制原理说明
 */
AT24C02_Status_t AT24C02_WriteBuffer(uint8_t addr, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AT24C02_H */
