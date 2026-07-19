/**
 * @file    ina226.h
 * @brief   INA226 电源监控芯片驱动 (基于 HAL 硬件 I2C)
 * @note    I2C 地址: 0x40, 挂在 I2C1 总线上, 通过 PCA9517 缓冲器
 *          默认配置: 采样电阻 15mOhm, 电流 LSB = 1mA, 最大电流 = ±6.4A
 *          使用前需确保 I2C1 已初始化, 且 xSemaphore_I2C 互斥信号量已创建
 *
 * ===== INA226 工作原理 =====
 *
 *   【电流测量 - 电流分流 (Shunt) 原理】
 *   1. 在电源回路中串联一个低阻值精密电阻 (R_shunt, 本设计 = 15mOhm)
 *   2. 根据欧姆定律: I = V_shunt / R_shunt
 *      当 1A 电流流过 15mOhm 电阻时, 两端产生 15mV 压降
 *   3. INA226 内部有一个高精度 ΔΣ (Delta-Sigma) ADC
 *      测量 shunt 电阻两端的差分电压 V_shunt
 *   4. ADC 的输入范围: ±81.92mV (可编程, 默认)
 *      对应的有效输入信号: 差分电压
 *   5. ADC 分辨率:
 *      - Shunt 电压 LSB = 2.5uV (固定, 参考数据手册)
 *      - 16 位有符号: -32768 ~ +32767 × 2.5uV = ±81.92mV
 *   6. 为什么用差分输入? 因为负载电流可能在电源正端或负端,
 *      差分 ADC 可以测量双向电流 (充电/放电)
 *
 *   【电压测量 - 总线电压】
 *   1. INA226 同时对 Vbus (电源端对地电压) 进行测量
 *   2. 内部有一个分压电阻网络, 将高电压衰减到 ADC 可接受范围
 *   3. 总线电压 LSB = 1.25mV (固定, 参考数据手册)
 *   4. 16 位无符号: 0 ~ 65535 × 1.25mV = 0V ~ 81.92V
 *   5. 虽然可以测量到 81.92V, 但芯片绝对最大额定值是 36V
 *      本电路设计在 3.3V ~ 5V 范围
 *
 *   【功率计算】
 *   1. INA226 内部有硬件乘法器: Power = Bus_Voltage × Current
 *   2. 功率寄存器是 16 位无符号
 *   3. 功率 LSB = 25 × Current_LSB
 *   4. 这节省了主控的计算资源, 且不受软件中断延迟影响
 *   5. 但是: 校准值必须正确写入, 否则功率值按默认 LSB 算会错误
 *
 * ===== 寄存器表格 =====
 *
 *   地址  | 寄存器名       | 类型 | 说明
 *   ------|---------------|------|--------------------
 *   0x00  | Configuration | R/W  | 配置: 平均次数/转换时间/运行模式
 *   0x01  | Shunt Voltage | R    | shunt 采样电压 (有符号 16 位)
 *   0x02  | Bus Voltage   | R    | 总线电压 (无符号 16 位)
 *   0x03  | Power         | R    | 功率值 (无符号 16 位)
 *   0x04  | Current       | R    | 电流值 (有符号 16 位)
 *   0x05  | Calibration   | R/W  | 校准寄存器 (关键!)
 *   0x06  | Mask/Enable   | R/W  | 告警配置 - 本驱动未使用
 *   0x07  | Alert Limit   | R/W  | 告警阈值 - 本驱动未使用
 *   0xFE  | Manufacturer  | R    | 厂商 ID = 0x5449 ("TI" ASCII)
 *   0xFF  | Die ID        | R    | 芯片版本号
 *
 *   注意: 电压电流等结果寄存器的有效数据取决于配置寄存器的 mode 位
 *         mode = 111 表示连续测量 shunt+bus (本文默认配置)
 *
 * ===== 校准算法原理 =====
 *
 *   INA226 内部的电流计算结果为:
 *     Current_Register = (V_shunt × Calibration) / 32768
 *   其中 V_shunt 是以 2.5uV 为单位的整数 (即 ADC 原始读数)
 *
 *   我们希望: Current_Register × Current_LSB = 真实电流 (A)
 *   即: Current_Register = 真实电流 / Current_LSB
 *
 *   由欧姆定律:
 *     真实电流 (A) = V_shunt (V) / R_shunt (Ohm)
 *     真实电流 (A) = V_shunt_raw × 2.5e-6 / R_shunt
 *   (V_shunt_raw 是 shunt 电压 ADC 原始值)
 *
 *   已知 Current_Register = (V_shunt_raw × Cal) / 32768
 *   代入: (V_shunt_raw × Cal) / 32768 = V_shunt_raw × 2.5e-6 / R_shunt / Current_LSB
 *   化简: Cal / 32768 = 2.5e-6 / (R_shunt × Current_LSB)
 *   变形: Cal = 32768 × 2.5e-6 / (R_shunt × Current_LSB)
 *   32768 × 2.5e-6 = 0.08192
 *   注意: 数据手册给出的是 0.00512, 比 0.08192 小 16 倍
 *   这是因为数据手册的公式考虑了一个隐含的 16 倍因子:
 *   Cal = 0.00512 / (Current_LSB × R_shunt)
 *   这个 0.00512 = 0.08192 / 16 (相当于内部还有 4 位移位)
 *
 *   【关键结论】
 *   校准值 Cal 的作用是让 INA226 内部的原始 ADC 读数
 *   与用户设定的 Current_LSB 相匹配, 使 Current_Register × Current_LSB = 真实电流
 *
 * ===== Current_LSB 选择依据 =====
 *
 *   Current_LSB 的选择影响:
 *   1. 电流测量分辨率: LSB 越小, 分辨率越高
 *   2. 最大可测电流: Max_Current = Current_LSB × 32767
 *   3. 校准值 Cal: LSB 越小, Cal 越大, 但 Cal 是 16 位寄存器 (最大 65535)
 *
 *   选择过程:
 *   1. 估算最大负载电流: 本设备最大约 3A
 *   2. 计算公式建议值: Current_LSB = Max_I / 2^15 = 3 / 32768 ≈ 0.0000915A
 *   3. 向上取整到 1mA: 方便计算, 且 Cal = 0.00512 / (0.001 × 0.015) = 341 (在 0~65535 内)
 *   4. 如果选 0.1mA: Max_Current = 0.0001 × 32767 = 3.27A (够用), 但 Cal = 3413 (也可行)
 *   5. 选 1mA 的原因: 精度够 (1mA 对于电池供电设备来说已经精细), Cal 值适中
 *
 * ===== 0x5449 厂商 ID 解释 =====
 *   INA226 的 MANUFACTURER_ID 寄存器 (0xFE) 固定返回 0x5449
 *   这是 "TI" 两个字母的 ASCII 码:
 *     'T' = 0x54, 'I' = 0x49
 *   使用大端序存储所以组合为 0x5449
 *   不同厂商的设备 ID 不同, 用于验证硬件连接正确
 */
#ifndef __INA226_H
#define __INA226_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C 设备地址 */
#define INA226_ADDR               (0x40U << 1)  /* 8位地址, HAL 库需要左移1位 */
                                                /* 0x40(7位) → 0x80(8位写) / 0x81(8位读) */

/**
 * @brief  INA226 寄存器地址映射
 *
 *   INA226 内部有 16 个 16 位寄存器, 通过 8 位地址索引
 *   读写操作:
 *     写: START + DEV_ADDR(W) + REG_ADDR + DATA_H + DATA_L + STOP
 *     读: START + DEV_ADDR(W) + REG_ADDR + RESTART + DEV_ADDR(R) + DATA_H + DATA_L + STOP
 *          (先写寄存器地址, 再重新启动进行读取)
 */
#define INA226_REG_CONFIG         0x00U  /* 配置寄存器: 控制测量模式和转换参数 */
#define INA226_REG_SHUNT_VOLTAGE  0x01U  /* Shunt 电压寄存器: 只读, 有符号 */
#define INA226_REG_BUS_VOLTAGE    0x02U  /* 总线电压寄存器: 只读, 无符号 */
#define INA226_REG_POWER          0x03U  /* 功率寄存器: 只读, 无符号 */
#define INA226_REG_CURRENT        0x04U  /* 电流寄存器: 只读, 有符号 */
#define INA226_REG_CALIBRATION    0x05U  /* 校准寄存器: 校准电流和功率 */
#define INA226_REG_MASK_ENABLE    0x06U  /* 掩码/使能寄存器: 告警配置 (未使用) */
#define INA226_REG_ALERT_LIMIT    0x07U  /* 告警限制寄存器: 阈值设置 (未使用) */
#define INA226_REG_MANUFACTURER   0xFEU  /* 厂商 ID 寄存器: 只读, 应返回 0x5449 ("TI") */
#define INA226_REG_DIE_ID         0xFFU  /* 芯片 ID 寄存器: 只读, 版本号 */

/**
 * @brief  配置寄存器 (0x00) 常用值
 *
 *   INA226_CONFIG_DEFAULT = 0x4127 的 bit 分解:
 *   bit15:    0     - RST (复位位, 1=复位, 此处为 0 不复位)
 *   bit14-13: 01    - 平均次数 = 16 (4 种模式: 00=1, 01=4, 10=8, 11=16, 这里选最高)
 *   bit12-11: 00    - Vbus 转换时间 = 1.1ms
 *   bit10-9:  01    - Vshunt 转换时间 = 1.1ms (与 Vbus 同步)
 *   bit8-7:   00    - 模式 = 连续 shunt+bus 测量
 *   bit6:     1     - 保留位 (必须写 1, 数据手册规定)
 *   bit5-4:   00    - 保留
 *   bit3:     1     - 保留位 (必须写 1, 数据手册规定)
 *   bit2-0:   111   - 运行模式 = 7 (连续测量 shunt 和 bus, 见数据手册 Table 7)
 *
 *   为什么选 16 次平均?
 *   当 ADC 转换噪声较大时, 多次平均可以滤除工频干扰
 *   电源电压在 50Hz/60Hz 下有纹波, 16 次平均相当于做了低通滤波
 *   代价是测量速度: 16 × 1.1ms ≈ 17.6ms 才能得到一个结果
 *   本系统温湿度采样是 100ms 级别, 17.6ms 完全够用
 *
 *   为什么选 1.1ms 转换时间?
 *   INA226 支持 140us / 204us / 332us / 588us / 1.1ms / 2.1ms / 4.1ms / 8.2ms
 *   1.1ms 在精度和速度之间是平衡选择
 *   更快的转换时间 (140us) 会增大量化噪声
 *   更慢的转换时间 (8.2ms) 提供更好的噪声性能, 但 +16 次平均 = 131ms 才能出一个结果
 *
 *   INA226_CONFIG_RESET = 0x8000
 *   写 0x8000 到配置寄存器会复位所有寄存器到默认值
 *   bit15 = 1 触发复位, 硬件自动清除此位
 */
#define INA226_CONFIG_DEFAULT     0x4127U  /* 16次平均, 1.1ms转换, 连续模式 (shunt+bus) */
#define INA226_CONFIG_RESET       0x8000U  /* 复位所有寄存器到默认值 */

/* 返回值定义 */
typedef enum {
    INA226_OK = 0,              /* 操作成功 */
    INA226_ERR_I2C,             /* I2C 通信失败 */
    INA226_ERR_NOT_FOUND        /* 未检测到芯片 (厂商 ID 不匹配) */
} INA226_Status_t;

/**
 * @brief  INA226 初始化 (复位 → 配置 → 写校准寄存器)
 * @param  hi2c: I2C 句柄指针 (CubeIDE 生成的 &hi2c1)
 * @param  pSemaphore: I2C 互斥信号量句柄, 传入 NULL 则不加锁 (裸机调试用)
 * @param  shuntResistance_mOhm: 采样电阻值 (毫欧), 通常为 15 (即 0.015 Ohm)
 * @retval INA226_OK: 成功, 其他: 失败
 *
 * @note   初始化序列:
 *         1. 复位: 写 0x8000 到 Configuration 寄存器
 *         2. 写 Calibration 寄存器: 计算 Cal = 0.00512 / (Current_LSB × R_shunt)
 *         3. 写 Configuration 寄存器: 配置平均次数和转换模式
 *         4. 验证: 读 Manufacturer ID 寄存器, 确认值为 0x5449
 *
 *         Calibration 寄存器的写入顺序必须在 Configuration 寄存器之前!
 *         因为 INA226 在 Configuration 写入后立即开始转换,
 *         如果 Calibration 尚未设置, 电流和功率输出将以错误比例输出
 */
INA226_Status_t INA226_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            float shuntResistance_mOhm);

/**
 * @brief  读取总线电压
 * @param  voltage: 输出电压值 (单位: V)
 * @retval INA226_OK: 成功, 其他: 失败
 * @note   总线电压 = Bus_Voltage_Register × 1.25mV (分压衰减已在芯片内部补偿)
 */
INA226_Status_t INA226_ReadBusVoltage(float *voltage);

/**
 * @brief  读取电流
 * @param  current: 输出电流值 (单位: A)
 * @retval INA226_OK: 成功, 其他: 失败
 * @note   电流 = Current_Register × Current_LSB (有符号)
 *         电流寄存器实际是 INA226 根据 shunt 电压和校准值内部计算的结果
 */
INA226_Status_t INA226_ReadCurrent(float *current);

/**
 * @brief  读取功率
 * @param  power: 输出功率值 (单位: W)
 * @retval INA226_OK: 成功, 其他: 失败
 * @note   功率 = Power_Register × 25 × Current_LSB
 *         注意: 功率不能直接等于电压 × 电流!
 *         INA226 内部计算功率时是取瞬时值, 而电压和电流可能不是同一时刻采样
 *         所以 V_reg × I_reg 可能与 Power_reg 有细微差异
 */
INA226_Status_t INA226_ReadPower(float *power);

/**
 * @brief  读取采样电阻两端电压 (用于调试)
 * @param  shuntVoltage_mV: 输出采样电压 (单位: mV)
 * @retval INA226_OK: 成功, 其他: 失败
 * @note   采样电压 = Shunt_Voltage_Register × 2.5uV (有符号)
 *         正值为电流正向流动, 负值为反向/充电
 *         主要用在实际观测 INA226 的原始差分 ADC 输出
 */
INA226_Status_t INA226_ReadShuntVoltage(float *shuntVoltage_mV);

/**
 * @brief  一次性读取电压、电流、功率
 * @param  voltage: 输出总线电压 (V)
 * @param  current: 输出电流 (A)
 * @param  power: 输出功率 (W)
 * @retval INA226_OK: 成功, 其他: 失败
 *
 * @note   为什么提供这个函数而不是分别调用三个?
 *         1. 减少 I2C 总线竞争 (一次加锁读完)
 *         2. 三个量是同一时刻的快照 (连续读取间隔仅几微秒)
 *         3. 如果分别调用, 中间被其他任务中断, 读取的电压和电流可能不是同一个瞬时的
 */
INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power);

#ifdef __cplusplus
}
#endif

#endif /* __INA226_H */
