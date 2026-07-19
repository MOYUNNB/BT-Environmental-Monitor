/**
 * @file    aht20.h
 * @brief   AHT20 温湿度传感器驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址: 0x38, 挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 *          使用前需确保 I2C1 已初始化, 且 xSemaphore_I2C 互斥信号量已创建
 *
 * ===== AHT20 工作原理 =====
 * AHT20 是一款 MEMS 电容式温湿度传感器, 内部结构包含:
 *
 *   【湿度测量 - 电容式原理】
 *   1. 传感器内部有一个高分子聚合物介电层, 上下各有一个电极
 *   2. 环境中的水分子透过上电极微孔进入聚合物层
 *   3. 水分子被聚合物层吸附 → 介电常数发生变化 → 电容值改变
 *   4. 电容值变化被内部 CVC (电容-电压转换器) 转为电压信号
 *   5. ADC 采样后得到原始的 20 位湿度数字值
 *   6. 注意: 电容式湿度传感器在 <20%RH 和 >80%RH 时非线性增大
 *      AHT20 内部已做非线性校准, 输出值已基本线性化
 *
 *   【温度测量】
 *   1. 采用带隙温度传感器 (band-gap) 原理
 *   2. 利用 PN 结正向压降随温度变化的特性 (约 -2mV/°C)
 *   3. 经过内部 ADC (也是 20 位) 转换为数字量
 *   4. 温度检测范围: -40°C ~ +85°C, 精度 ±0.3°C (典型值)
 *
 *   【数据校准】
 *   - 出厂前每个传感器经过单独标定
 *   - 校准系数存储在传感器内部的 OTP (一次可编程) 存储器中
 *   - 上电后需要发送 0xBE 初始化命令, 触发校准系数加载到工作寄存器
 *   - 初始化后 bit3 (校准状态位) 变为 1 表示校准完成
 *
 * ===== I2C 通信时序 =====
 * AHT20 的 I2C 地址是 0x38 (7 位地址, 不含 R/W 位)
 *
 *   【写操作 (主机→传感器)】
 *   START + 0x38(W) + ACK + CMD + ACK + [ARG0 + ACK + ARG1 + ACK] + STOP
 *   - 初始化命令: 0xBE 0x08 0x00
 *   - 触发测量命令: 0xAC 0x33 0x00
 *
 *   【读操作 (主机←传感器)】
 *   START + 0x38(R) + ACK + DATA0 + ACK + ... + DATAN + NACK + STOP
 *   - 读取状态: 发 0x71 子地址, 再读 1 字节
 *   - 读取测量结果: 直接读 6 字节 (需先触发测量并等待 80ms)
 *
 *   【HAL_I2C_Mem_Read vs Master_Transmit/Receive 的选择】
 *   - 读状态寄存器: 用 Mem_Read (先写子地址 0x71, 再读)
 *   - 发命令: 用 Master_Transmit (不带子地址, 直接发命令字节)
 *   - 读数据: 用 Master_Receive (读 6 字节, 此时传感器内部指针已在测量完成后自动复位到数据起始位置)
 *
 * ===== 初始化流程 =====
 *   上电 (≥100ms) → 发送 0xBE 初始化 → 等待 10ms → 检查 bit3=1? → 就绪
 *
 * ===== 测量流程 =====
 *   发送 0xAC (触发测量) → 等待 80ms → 读取 6 字节 → 解析 → 换算
 */
#ifndef __AHT20_H
#define __AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址 */
#define AHT20_ADDR                (0x38U << 1)  /* 8位地址, HAL 库需要左移1位 */
                                                /* 0x38(7位) → 0x70(8位写) / 0x71(8位读) */

/* AHT20 命令定义 */
#define AHT20_CMD_INIT            0xBEU   /* 初始化命令 (加载校准系数) */
#define AHT20_CMD_MEASURE         0xACU   /* 触发测量命令 (唤醒传感器开始测量) */
#define AHT20_CMD_MEASURE_ARG0    0x33U   /* 测量参数 byte0: 开启温度+湿度测量 */
#define AHT20_CMD_MEASURE_ARG1    0x00U   /* 测量参数 byte1: 保留, 固定为 0x00 */

/**
 * @brief  状态寄存器 bit 解释 (地址: 0x71, 只读)
 *
 *   bit7: 忙标志 (1=忙, 0=空闲)
 *         测量期间为 1, 结束后自动清 0
 *   bit6: 保留
 *   bit5: 保留
 *   bit4: 保留
 *   bit3: 校准标志 (1=校准完成, 0=未校准)
 *         上电后需发送初始化命令触发校准完成
 *         如果校准未完成就触发测量, 结果将无效
 *   bit2: 保留
 *   bit1: 保留
 *   bit0: 保留
 */
#define AHT20_STATUS_BUSY         (1U << 7)  /* bit7: 忙标志 */
#define AHT20_STATUS_CALIBRATED   (1U << 3)  /* bit3: 校准完成标志 */

/**
 * @brief  测量数据长度
 *   数据手册规定一次测量返回 6 字节:
 *   byte[0]: 状态寄存器 (同 0x71 读取的值)
 *   byte[1-3]: 湿度数据 (20 位, 分布在 3 个字节中)
 *   byte[3-5]: 温度数据 (20 位, 分布在 3 个字节中, 与湿度共享 byte[3])
 */
#define AHT20_DATA_LEN            6U

/* 返回值定义 */
typedef enum {
    AHT20_OK = 0,               /* 操作成功 */
    AHT20_ERR_I2C,              /* I2C 通信失败 (NACK/超时/仲裁丢失) */
    AHT20_ERR_NOT_CALIBRATED,   /* 传感器校准未完成 (检查 bit3) */
    AHT20_ERR_BUSY,             /* 传感器忙 (上一轮测量未结束) */
    AHT20_ERR_TIMEOUT           /* 等待超时 */
} AHT20_Status_t;

/**
 * @brief  AHT20 初始化
 * @param  hi2c: I2C 句柄指针 (CubeIDE 生成的 &hi2c1)
 * @param  pSemaphore: I2C 互斥信号量句柄, 传入 NULL 则不加锁 (裸机调试用)
 * @retval AHT20_OK: 成功, 其他: 失败
 *
 * @note   内部操作:
 *         1. 保存 I2C 句柄和信号量到模块内部静态变量
 *         2. 发送 0xBE 0x08 0x00 初始化命令, 触发加载出厂校准系数
 *         3. 等待 40ms (数据手册要求 10ms, 加余量)
 *         4. 读取状态寄存器, 确认 bit3(校准标志)=1
 *         5. 若校准未完成返回 AHT20_ERR_NOT_CALIBRATED
 */
AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);

/**
 * @brief  读取温湿度数据 (触发测量 + 读取, 一次调用完成)
 * @param  temperature: 输出温度值 (单位: °C, 精度 0.01)
 * @param  humidity: 输出湿度值 (单位: %RH, 精度 0.01)
 * @retval AHT20_OK: 成功, 其他: 失败
 *
 * @note   内部流程:
 *         1. 发送 0xAC 0x33 0x00 触发测量
 *         2. osDelay(80ms) 等待传感器完成测量
 *         3. 读取 6 字节原始数据
 *         4. 按公式换算为工程值
 *         5. 可通过传入 NULL 选择不读取某一项
 */
AHT20_Status_t AHT20_ReadData(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* __AHT20_H */
