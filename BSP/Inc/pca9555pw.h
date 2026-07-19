/**
 * @file    pca9555pw.h
 * @brief   PCA9555PW I/O 扩展芯片驱动 (基于 HAL 硬件 I2C)
 *
 * ============================================================
 * 1. IO 扩展芯片的作用 -- 为什么需要它?
 * ============================================================
 * STM32F407VET6 有 82 个 GPIO, 但本项目已占用大量引脚:
 *   - SPI1: LCD (SCK, MOSI, CS, DC, RST, BL)
 *   - SPI2: Flash + IMU (SCK, MOSI, MISO, CSx2)
 *   - SDIO: TF 卡 (CLK, CMD, D0~D3)
 *   - I2C1: 6 个 I2C 设备 (SCL, SDA)
 *   - USART1: 调试串口
 *   - USART2: 蓝牙模块
 *   - TIM5: WS2812
 *   - 其他: 按键, LED, 电池检测等
 *
 * 当系统需要控制或读取更多外设 (如继电器、指示灯、拨码开关、电平检测) 时,
 * GPIO 数量不足。PCA9555PW 通过 I2C 总线扩展出 16 个 GPIO, 只需 2 个引脚。
 *
 * 对比软件模拟 GPIO:
 *   - 软件模拟: 用其他外设的未用引脚, 但需要改 PCB 布线
 *   - I2C 扩展: 共享现有 I2C 总线, 只需添加芯片, 无需改 PCB
 *   - 74HC595 (串转并): 单向, 只能输出, 需要 3 个独立引脚
 *   - PCA9555PW: 双向, 可配置输入/输出, 只需 2 个引脚 (挂在 I2C 总线上)
 *
 * ============================================================
 * 2. PCA9555PW 寄存器地址映射
 * ============================================================
 * 地址 | 名称       | 类型 | 说明
 * ------+------------+------+-------------------------------
 * 0x00 | INPUT0     | 只读 | IO0 端口输入电平 (读引脚)
 * 0x01 | INPUT1     | 只读 | IO1 端口输入电平
 * 0x02 | OUTPUT0    | R/W  | IO0 端口输出电平 (写引脚)
 * 0x03 | OUTPUT1    | R/W  | IO1 端口输出电平
 * 0x04 | POLARITY0  | R/W  | IO0 极性反转 (0=正常, 1=取反)
 * 0x05 | POLARITY1  | R/W  | IO1 极性反转
 * 0x06 | CONFIG0    | R/W  | IO0 方向 (0=输出, 1=输入)
 * 0x07 | CONFIG1    | R/W  | IO1 方向
 *
 * 每个寄存器对应一个 8 位端口, 每个位对应一个引脚:
 *   bit0 = P0/P8, bit1 = P1/P9, ..., bit7 = P7/P15
 *
 * I2C 地址: 0x20, 16 位 GPIO (IO0/IO1), 挂在 I2C1 总线上
 * 通过 PCA9517 缓冲器。配置寄存器 0x06 (IO0) / 0x07 (IO1),
 * 输出寄存器 0x02 (IO0) / 0x03 (IO1)
 *
 * === 移植说明 ===
 * 直接使用 HAL 硬件 I2C 封装寄存器读写, 无特殊算法。
 */
#ifndef __PCA9555PW_H
#define __PCA9555PW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址 (7 位) */
#define PCA9555PW_ADDR          0x20U

/**
 * 寄存器地址
 *
 * PA9555PW 内部寄存器地址映射:
 *
 * 0x00 / 0x01 -- 输入寄存器 (只读)
 *   读取时返回引脚当前电平: 0=低电平, 1=高电平
 *   对于未配置为输入的引脚, 读取的是输出锁存器的值, 而非引脚实际电平。
 *   所以一定要先配置好方向。
 *
 * 0x02 / 0x03 -- 输出寄存器
 *   写入值会锁存在内部寄存器, 当对应引脚配置为输出时输出该电平。
 *   上电默认值 = 0xFF (所有引脚高电平, 即高阻态/内部上拉无驱动),
 *   这是 I/O 扩展芯片的常见安全行为: 上电时不主动驱动任何引脚。
 *
 * 0x04 / 0x05 -- 极性反转寄存器 (高级功能)
 *   设置对应位的输入极性反转。例如设置 bit0 = 1, 则 INPUT0 的 bit0 读取时取反。
 *   这可用于消除外部反相器的逻辑, 也可以在软件中处理, 但硬件处理更快。
 *
 * 0x06 / 0x07 -- 配置寄存器 (方向控制)
 *   0 = 输出, 1 = 输入 (上电默认值 = 0xFF, 即所有引脚为输入)
 */
#define PCA9555PW_REG_INPUT0    0x00U   /* IO0 输入寄存器, 只读 */
#define PCA9555PW_REG_INPUT1    0x01U   /* IO1 输入寄存器, 只读 */
#define PCA9555PW_REG_OUTPUT0   0x02U   /* IO0 输出寄存器 */
#define PCA9555PW_REG_OUTPUT1   0x03U   /* IO1 输出寄存器 */
#define PCA9555PW_REG_POLARITY0 0x04U   /* IO0 极性反转 */
#define PCA9555PW_REG_POLARITY1 0x05U   /* IO1 极性反转 */
#define PCA9555PW_REG_CONFIG0   0x06U   /* IO0 方向: 0=输出, 1=输入 */
#define PCA9555PW_REG_CONFIG1   0x07U   /* IO1 方向: 0=输出, 1=输入 */

/**
 * 方向配置详解:
 *
 * 0 = 输出:
 *   引脚驱动方式为推挽输出 (push-pull)
 *   内部锁存器 OUTPUT 寄存器的值被输出到引脚
 *   可以直接驱动 LED、继电器等负载
 *
 * 1 = 输入:
 *   引脚为高阻输入 (高阻态 = 3-state / Hi-Z)
 *   内部锁存器 OUTPUT 与引脚断开
 *   外部信号通过输入缓冲器进入 INPUT 寄存器
 *   (PCA9555PW 内部无上拉/下拉, 输入引脚需外部上拉或由驱动源驱动)
 *
 * 默认所有引脚为输入 (高阻), 这是安全的:
 *   MCU 复位或未初始化时, 不会意外驱动外部电路。
 *
 * 配置示例:
 *   IO0.0~IO0.3 = 输出 (低 4 位), IO0.4~IO0.7 = 输入 (高 4 位)
 *   config0 = 0xF0 (0b1111 0000, 高位输入, 低位输出)
 */

/* 返回值定义 */
typedef enum {
    PCA9555PW_OK = 0,
    PCA9555PW_ERR_I2C
} PCA9555PW_Status_t;

/**
 * @brief  初始化 PCA9555PW (保存 I2C 句柄和信号量)
 * @param  hi2c: I2C 句柄指针
 * @param  pSemaphore: I2C 互斥信号量, NULL 则不加锁
 */
void PCA9555PW_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore);

/**
 * @brief  设置 IO0/IO1 方向 (逐位)
 * @param  config0: IO0 方向, 0=输出, 1=输入
 * @param  config1: IO1 方向, 0=输出, 1=输入
 * @retval PCA9555PW_OK: 成功
 */
PCA9555PW_Status_t PCA9555PW_SetDirection(uint8_t config0, uint8_t config1);

/**
 * @brief  写 IO0/IO1 输出寄存器
 * @param  output0: IO0 输出值 (仅配置为输出的位有效)
 * @param  output1: IO1 输出值
 * @retval PCA9555PW_OK: 成功
 */
PCA9555PW_Status_t PCA9555PW_WriteOutput(uint8_t output0, uint8_t output1);

/**
 * @brief  读 IO0/IO1 输入寄存器
 * @param  input0: 输出 IO0 输入值
 * @param  input1: 输出 IO1 输入值
 * @retval PCA9555PW_OK: 成功
 */
PCA9555PW_Status_t PCA9555PW_ReadInput(uint8_t *input0, uint8_t *input1);

/**
 * @brief  设置极性反转 (逐位)
 * @param  polarity0: IO0 极性, 0=正常, 1=反转
 * @param  polarity1: IO1 极性, 0=正常, 1=反转
 * @retval PCA9555PW_OK: 成功
 */
PCA9555PW_Status_t PCA9555PW_SetPolarity(uint8_t polarity0, uint8_t polarity1);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9555PW_H */
