/**
 * @file    sd3078.h
 * @brief   SD3078 高精度 RTC 驱动 (基于 HAL 硬件 I2C)
 *
 * ============================================================
 * 1. RTC (实时时钟) 工作原理
 * ============================================================
 * RTC 的核心是一个 **32.768 kHz 晶振驱动分频链**:
 *   32.768 kHz --> 15 级二分频 (÷32768) --> 1 Hz (每秒进位)
 * 选择 32.768 kHz 的理由:
 *   - 2^15 = 32768, 即 15 级二分频恰好得到 1 Hz
 *   - 32768 是 2 的整数次幂, 可用纯数字分频器实现, 无需锁相环
 *   - 频率较低, 晶振功耗小, 适合电池备份
 *
 * SD3078 使用的 **温度补偿晶振 (TCXO)** 内置温度传感器和补偿电路,
 * 在 -40~+85°C 范围内将频率偏差控制在 ±3.8ppm 以内
 * (普通 RTC 晶振无补偿, 典型偏差 ±20~±50ppm)。
 *
 * ============================================================
 * 2. BCD (Binary-Coded Decimal) 编码
 * ============================================================
 * RTC 芯片普遍采用 BCD 码而非纯二进制存储时间, 原因:
 *
 *   (a) 人类可读性: BCD 每个字节的高 4 位是十位, 低 4 位是个位,
 *       直接对应数码管/段码 LCD 的显示, 无需进制转换。
 *       例如 0x59 = 59 秒, 直接分解为 5 和 9 送显。
 *
 *   (b) 兼容性: 从早期分立式 RTC (如 MC146818) 开始就使用 BCD,
 *       行业延续至今, 许多传统 RTC 芯片 (DS1307, PCF8563, SD3078)
 *       均沿用此标准, 便于替换。
 *
 *   (c) 避免 60/24 进位混淆: 秒/分的范围是 0~59, 时 0~23,
 *       用 BCD 可以直观检查合法性 (BCD 的 0x59 后正确进位到 0x60?
 *       实际上 BCD 0x5A~0x5F 是非法值, 固件校验更容易)。
 *       如果用纯二进制, 59 = 0x3B, 完全看不出是 59 秒。
 *
 *   BCD 与二进制的相互转换 (见 sd3078.c 中的 bcd_to_dec / dec_to_bcd):
 *     BCD 转十进制: ((val >> 4) * 10) + (val & 0x0F)
 *     十进制转 BCD: ((val / 10) << 4) | (val % 10)
 *
 * ============================================================
 * 3. SD3078 温度补偿晶振 (TCXO) 原理
 * ============================================================
 * 普通石英晶振的频率-温度曲线呈二次抛物线, 在 25°C 附近最准,
 * 极端温度下偏差可达 -0.04 ppm/°C^2 (即 -100°C 偏差 ~160ppm)。
 *
 * SD3078 内置温度传感器 + 数字补偿电路:
 *   温度传感器 --> ADC 采样 --> 查补偿表 --> 调整晶振负载电容
 * 补偿后的精度:
 *   - 常温 (25°C): ±1 ppm (≈ 每月 2.6 秒误差)
 *   - 全温范围 (-40~85°C): ±3.8 ppm (≈ 每月 10 秒误差)
 * 相比无补偿的 DS1307 (±2 ppm/°C, 极端温度每月差数分钟) 优势显著。
 *
 * SD3078_ReadTemperature() 读取的就是这个内置温度传感器的值,
 * 它同时用于自动补偿, 用户读取它主要是监控环境温度。
 *
 * ============================================================
 * 4. 写保护设计思路
 * ============================================================
 * RTC 的时间寄存器是系统关键数据, 误写可能导致:
 *   - 时间错乱, 影响日志时间戳
 *   - 闹钟异常, 影响定时触发
 *   - 电池充电配置错误, 损坏备份电池
 *
 * SD3078 采用 **三级写保护** (见 sd3078.c 中 sd3078_enable_write):
 *   CTRL2.WRTC1 (bit7) = 1
 *   CTRL1.WRTC2 (bit2) = 1
 *   CTRL1.WRTC3 (bit7) = 1
 * 必须三个位同时置 1 才能写入, 缺一不可。
 * 这是一个 **"三把钥匙开一把锁"** 的安全设计:
 *   即便 I2C 总线受到 EMI 干扰导致某一位翻转, 也不会误触发写入。
 *   同时, 这三个位分散在两个不同的寄存器中,
 *   进一步降低了程序中单点故障导致误写的概率。
 *
 * I2C 地址: 0x32, 挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 * 系统上电首次调用 SD3078_Init() 会自动清除首次上电标志
 * BCD 编码格式, 支持 2000~2099 年
 *
 * === 移植说明 ===
 * 从嘉立创 fdb 示例工程移植, 将底层从 soft_i2c 改为 HAL 硬件 I2C,
 * 增加信号量保护以适配 FreeRTOS 多任务环境。
 */
#ifndef __SD3078_H
#define __SD3078_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C 设备地址 (7 位) */
#define SD3078_ADDR             0x32U

/* 返回值定义 */
typedef enum {
    SD3078_OK = 0,
    SD3078_ERR_I2C,
    SD3078_ERR_INVALID_PARAM
} SD3078_Status_t;

/* 时间结构体 (公历, 无需 BCD 转换) */
typedef struct {
    uint16_t year;      /* 2000 ~ 2099 */
    uint8_t  month;     /* 1 ~ 12 */
    uint8_t  day;       /* 1 ~ 31 */
    uint8_t  weekday;   /* 0 = 周日, 1 = 周一 ... 6 = 周六 */
    uint8_t  hour;      /* 0 ~ 23 (24 小时制) */
    uint8_t  minute;    /* 0 ~ 59 */
    uint8_t  second;    /* 0 ~ 59 */
} SD3078_Time_t;

/* 初始化配置 */
typedef struct {
    uint8_t enable_charge;  /* 0: 不配置充电, 非 0: 使能充电 */
    uint8_t charge_value;   /* 充电电阻值: 0x82 = 2KΩ (默认), 其他见数据手册 */
} SD3078_Config_t;

/**
 * @brief  初始化 SD3078
 * @param  hi2c: I2C 句柄指针
 * @param  pSemaphore: I2C 互斥信号量, NULL 则不加锁
 * @param  config: 初始化配置 (可选, 传 NULL 使用默认值)
 * @retval SD3078_OK: 成功
 */
SD3078_Status_t SD3078_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            const SD3078_Config_t *config);

/**
 * @brief  读取 RTC 时间
 * @param  time: 输出时间结构体
 * @retval SD3078_OK: 成功
 */
SD3078_Status_t SD3078_GetTime(SD3078_Time_t *time);

/**
 * @brief  设置 RTC 时间
 * @param  time: 要写入的时间 (需经 SD3078_TimeIsValid 校验)
 * @retval SD3078_OK: 成功
 */
SD3078_Status_t SD3078_SetTime(const SD3078_Time_t *time);

/**
 * @brief  检查时间合法性
 * @retval true: 合法, false: 非法
 */
bool SD3078_TimeIsValid(const SD3078_Time_t *time);

/**
 * @brief  读取芯片内置温度 (±3.8ppm 温补晶振温度)
 * @param  temperature: 输出温度值 (°C)
 * @retval SD3078_OK: 成功
 */
SD3078_Status_t SD3078_ReadTemperature(int8_t *temperature);

/**
 * @brief  读取备份电池电压
 * @param  battery_mv: 输出电池电压 (mV)
 * @retval SD3078_OK: 成功
 */
SD3078_Status_t SD3078_ReadBattery(uint16_t *battery_mv);

#ifdef __cplusplus
}
#endif

#endif /* __SD3078_H */
