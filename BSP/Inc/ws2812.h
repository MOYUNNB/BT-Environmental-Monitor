/**
 * @file    ws2812.h
 * @brief   WS2812 RGB LED 驱动 (TIM5_CH4 PWM + DMA)
 * @note    800KHz PWM, 3 颗板载 LED, GRB 格式
 *          使用 TIM5_CH4 PWM + DMA 发送颜色数据, 非软件 bit-bang
 *
 *          === 需要你自己实现 ===
 *          ws2812.c 中已提供 DMA 缓冲区和 PWM 启动辅助函数,
 *          你需要补充:
 *          1. WS2812_Init()    — 清空缓冲区 + 关灯
 *          2. WS2812_SetPixel()— 设置单颗灯颜色 (GRB 编码)
 *          3. WS2812_SetAll()  — 设置所有灯同色
 *          4. WS2812_Update()  — 启动 DMA 发送
 *          5. HAL_TIM_PWM_PulseFinishedCallback — DMA 完成后停 PWM
 *
 *          参考: 嘉立创 fdb-master/0_example/RGB/ws2812-onboard-pa3-project
 *
 *          ⚠ TIM5 当前 Period=209 (400KHz), WS2812 需要 800KHz。
 *          可能需要调整 TIM5 Period 或在 T0H/T1H 中补偿。
 *          CubeMX 中修改 TIM5 Period=104, 或在 main.ioc 中调整。
 */
#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * ============================================================
 *  学习笔记: WS2812 单总线协议详解
 * ============================================================
 *
 * WS2812 使用"单线归零码"通信协议:
 *   只有一根数据线 (DIN), 通过高低脉冲的宽度来区分 0 和 1。
 *
 * 时序规格 (来自 WS2812B 数据手册, 典型值):
 *
 *   T0H = 0.35 μs:  码 0 的高电平宽度
 *   T0L = 0.90 μs:  码 0 的低电平宽度
 *   T1H = 0.70 μs:  码 1 的高电平宽度
 *   T1L = 0.55 μs:  码 1 的低电平宽度
 *
 *   一个 bit 的周期: T0H+T0L = T1H+T1L ≈ 1.25 μs
 *   数据率: 1 / 1.25 μs = 800 Kbps
 *
 *   每个 LED 需要 24 bit 数据 (GRB):
 *     24 bit × 1.25 μs = 30 μs/灯
 *     3 颗灯: 3 × 30 μs = 90 μs
 *
 *   复位信号: DIN 保持低电平 > 50 μs
 *     WS2812 在 DIN 低电平超过 50 μs 后, 判断为复位信号,
 *     将当前收到的 24 bit 数据锁存到 PWM 输出寄存器。
 *     然后等待下一帧数据。
 *
 * 时序图 (一个 bit):
 *
 *   码 0:   ┌┐    └┘────────────────
 *           T0H   T0L
 *          0.35μs 0.90μs
 *
 *   码 1:   ┌──────┐    └────────────
 *           T1H   T1L
 *          0.70μs 0.55μs
 *
 * 注意: 时序要求非常严格!
 *   WS2812 的时序容差约为 ±150ns。
 *   STM32F407 主频 168MHz ≈ 5.95ns/指令周期。
 *   如果用软件延时 (NOP 循环), 中断响应、分支预测等
 *   会导致时序不稳定。
 *
 *   使用 PWM+DMA 的好处:
 *   - 硬件产生精确的 PWM 波形, 不受中断影响
 *   - DMA 自动从内存搬运 CCR 值到 TIM 寄存器
 *   - CPU 零占用
 *
 * ============================================================
 *  学习笔记: GRB 编码 (不是 RGB!)
 * ============================================================
 *
 * WS2812 的颜色数据使用 GRB (Green-Red-Blue) 顺序发送,
 * 而不是常规的 RGB 顺序。这是 WS2812 的硬件特性。
 *
 * 24 位数据格式 (高位在前, MSB first):
 *
 *   bit [23:16] = Green  (8 bit, 0~255)
 *   bit [15:8]  = Red    (8 bit, 0~255)
 *   bit [7:0]   = Blue   (8 bit, 0~255)
 *
 * 颜色值构造:
 *   uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
 *
 * 示例:
 *   纯红色: R=255, G=0, B=0  → GRB = 0x00FF00
 *   纯绿色: R=0,   G=255, B=0 → GRB = 0xFF0000
 *   纯蓝色: R=0,   G=0,   B=255 → GRB = 0x0000FF
 *   白色:   R=255, G=255, B=255 → GRB = 0xFFFFFF
 *   黄色:   R=255, G=255, B=0   → GRB = 0xFFFF00 (红+绿, 但 GRB 中 G 占高位)
 *
 * 常见错误: 直接使用 RGB565 的颜色值发送给 WS2812,
 * 颜色会完全错乱。
 * 正确做法: 分别传入 R/G/B 分量, 拼接为 GRB 格式。
 *
 * ============================================================
 *  学习笔记: PWM + DMA 驱动原理
 * ============================================================
 *
 * 核心思路: 用 TIM5 的 PWM 输出, 通过改变 CCR 值来改变
 * 占空比, 从而产生不同宽度的脉冲。
 *
 * 硬件链路:
 *   TIM5_CH4 (PA3) → WS2812 DIN
 *
 * PWM 配置:
 *   频率: 800 KHz (一个 bit 的周期)
 *   占空比: 通过 CCR 值控制
 *
 *   PWM 周期: T = (ARR + 1) × (PSC + 1) / TIM_CLK
 *   其中 ARR = Period, PSC = Prescaler
 *
 *   STM32F407: TIM5 挂在 APB1 上, 最大时钟 84 MHz
 *   但 TIM5 有一个倍频器: 如果 APB1 prescaler ≠ 1,
 *   则 TIM 时钟 = APB1 时钟 × 2
 *
 *   通常配置: TIM 时钟 = 84 MHz × 2 = 168 MHz (如果 APB1 预分频 = 4)
 *   或者直接 = 84 MHz (取决于 CubeMX 配置)
 *
 *   800 KHz PWM:
 *     如果 TIM 时钟 = 168 MHz:
 *       Period = 168 MHz / 800 KHz - 1 = 210 - 1 = 209
 *       此时 T0H = 25 个 tick = 25 / 168 MHz ≈ 0.149 μs ❌ 太短!
 *
 *     如果 TIM 时钟 = 84 MHz:
 *       Period = 84 MHz / 800 KHz - 1 = 105 - 1 = 104
 *       此时 T0H = 25 个 tick = 25 / 84 MHz ≈ 0.298 μs ❌ 还是偏短
 *
 *   → 需要根据实际 TIM 时钟频率计算正确的 Period。
 *   → T0H/T1H 的值也需要根据实际时钟频率校准。
 *
 * 校准方法 (示波器实测):
 *   1. 设置 Period=209 (假设 168MHz TIM 时钟)
 *   2. 设置 T0H=28, 用示波器看高电平宽度
 *   3. 调整直到 T0H≈0.35μs
 *   4. 同理调整 T1H≈0.70μs
 *
 * DMA 传输过程:
 *   1. 在内存中准备 s_buffer[], 每个元素是 TIM CCR 值
 *   2. 对于 bit=0: 写 WS2812_T0H (小占空比)
 *   3. 对于 bit=1: 写 WS2812_T1H (大占空比)
 *   4. 调用 HAL_TIM_PWM_Start_DMA(), DMA 从 s_buffer
 *      自动搬运到 TIM5->CCR4
 *   5. TIM5 自动比较 ARR 和 CCR4, 输出对应占空比的 PWM
 *   6. 传输完成后, HAL_TIM_PWM_PulseFinishedCallback 被调用
 *
 * 缓冲区到波形的映射:
 *   s_buffer[n] = 25 → PWM 占空比 = 25/209 ≈ 12%
 *   s_buffer[n] = 50 → PWM 占空比 = 50/209 ≈ 24%
 *   12% 占空比 → 高电平约 0.35μs → 码 0
 *   24% 占空比 → 高电平约 0.70μs → 码 1
 *
 * ============================================================
 *  学习笔记: 复位信号 (>50μs 低电平)
 * ============================================================
 *
 * WS2812 的帧结束信号:
 *   当 DIN 保持低电平超过 50μs 时, WS2812 判定当前帧结束:
 *   1. 将接收到的 24 bit 数据锁存到输出寄存器
 *   2. LED 更新为新的颜色
 *   3. 准备接收下一帧
 *
 * 在 PWM+DMA 方案中, 复位信号用连续的低电平脉冲实现:
 *   s_buffer 的前后各有一段全 0 的区域 (WS2812_RESET_LEN 个元素)。
 *   这些元素对应的 PWM 占空比为 0 (CCR=0),
 *   所以输出恒低电平。
 *
 *   时间 = RESET_LEN × PWM 周期 = 500 × 1.25μs ≈ 625μs
 *   这远大于 50μs 的要求, 所以没问题 (留了余量)。
 *
 *   为什么前后都有复位?
 *     前复位: 确保 WS2812 在上次传输完成后被正确复位
 *     后复位: 确保本次传输的数据被锁存
 *
 * ============================================================
 *  学习笔记: 多颗灯级联原理
 * ============================================================
 *
 * WS2812 支持级联 (Daisy Chain):
 *
 *   MCU (PA3) ──→ LED0 DIN → LED0 DOUT ──→ LED1 DIN → LED1 DOUT ──→ LED2 DIN
 *
 * 数据传输过程:
 *   1. MCU 发送 24bit × 3 = 72bit 数据
 *   2. LED0 收到前 24bit: 保存为自己颜色, 其余 48bit 从 DOUT 转发
 *   3. LED1 收到 48bit: 保存前 24bit, 转发后 24bit
 *   4. LED2 收到最后 24bit: 保存为自己颜色
 *   5. 所有 LED 等待复位信号后更新颜色
 *
 * 每个 WS2812 芯片相当于一个移位寄存器:
 *   - 数据从 DIN 移入
 *   - 内部计数器计数: 0~23 是自己的数据
 *   - 24~47: 从 DOUT 输出
 *   - 遇到复位信号 (>50μs 低): 锁存数据, 计数器归零
 *
 * 在这个项目中, 板载 3 颗 WS2812:
 *   WS2812_NUM = 3
 *   总共需要发送 3 × 24 = 72 bit 数据。
 *   缓冲区长度 = 复位(500) + 72 + 复位(500) ≈ 1072 元素
 *
 * 关于 LED 索引:
 *   索引 0 = 离 MCU 最近的一颗
 *   索引 1 = 中间一颗
 *   索引 2 = 最远的一颗
 *   (实际布局以 PCB 为准, 可能需要调整)
 *
 * ============================================================
 *  学习笔记: TIM 频率计算公式
 * ============================================================
 *
 * TIM 输出频率 = TIM_CLK / (Period + 1) / (Prescaler + 1)
 *
 * 其中:
 *   TIM_CLK: TIM 模块的输入时钟
 *   Period:  TIM 自动重装载寄存器 (ARR)
 *   Prescaler: TIM 预分频器 (PSC)
 *
 * STM32F407 时钟树:
 *   HSE (8MHz) → PLL → SYSCLK (168MHz)
 *   APB1 时钟 = 42MHz (AHB/4), 但 TIM 挂在 APB1 上时:
 *     如果 APB1 prescaler ≠ 1, TIM 时钟 = APB1 时钟 × 2
 *     所以 TIM5 时钟 = 42MHz × 2 = 84MHz
 *   APB2 时钟 = 84MHz (AHB/2), TIM1/TIM8 挂在 APB2 上
 *
 * 以 TIM5 (APB1) 为例:
 *   假设 TIM_CLK = 84MHz, 目标 PWM = 800KHz:
 *     84000000 / 800000 = 105
 *     Period = 105 - 1 = 104
 *     Prescaler = 0 (不分频)
 *
 * 这个 Period=104 需要在 CubeMX 中设置:
 *   TIM5 → Parameter Settings → Counter Period = 104
 *
 * 如果 CubeMX 中当前 Period=209:
 *   PWM = 84000000 / (209+1) / 1 ≈ 400 KHz
 *   这是 WS2812 时序要求 (800KHz) 的一半, 需要调整!
 *
 * 调整方法:
 *   1. CubeMX 中修改 TIM5 Period 为 104
 *   2. 或代码中修改 __HAL_TIM_SET_AUTORELOAD(&htim5, 104)
 *   3. 然后校准 T0H/T1H
 */

#define WS2812_NUM              3U      /* 板载 3 颗 LED */

/*
 * PWM 时序参数 (需要在 TIM5 Period 下校准)
 *
 * 这些值是在假设 TIM_CLK=84MHz, Period=209 前提下的经验值。
 * 如果你在 CubeMX 中修改了 Period, 需要重新计算。
 *
 * 计算方式:
 *   T0H 的脉冲宽度 = T0H / (Period+1) × PWM周期
 *   PWM周期 = 1/800KHz = 1.25μs (如果配置正确)
 *
 * 例如 Period=209:
 *   码 0 高电平: T0H=25 → 25/(209+1) × 1.25μs ≈ 0.149μs ❌
 *   需要调大 T0H, 如 T0H=58 → 58/210 × 1.25μs ≈ 0.345μs ✓
 *
 * 建议: 先用示波器观察波形, 调整到符合规格:
 *   T0H ≈ 0.35μs (±150ns)
 *   T1H ≈ 0.70μs (±150ns)
 */
#define WS2812_T0H              25U     /* 0 码高电平占空比 */
#define WS2812_T1H              50U     /* 1 码高电平占空比 */
#define WS2812_RESET_LEN        500U    /* 复位脉冲长度 (低电平) */

/*
 * 缓冲区结构:
 *   [RESET×500] [LED0×24] [LED1×24] [LED2×24] [RESET×500]
 *   总共: 500 + 72 + 500 = 1072 个 uint32_t
 *
 * 每个元素是 TIM CCR 值:
 *   0 = 低电平 (占空比 0%)
 *   WS2812_T0H = 码 0 (约 12% 占空比, 如果 Period=209)
 *   WS2812_T1H = 码 1 (约 24% 占空比, 如果 Period=209)
 *
 * 为什么是 uint32_t?
 *   TIM5->CCR4 是 32 位寄存器, DMA 传输单元大小需要和 CCR 匹配。
 *   如果 DMA 配置为半字 (16-bit), 可以用 uint16_t 节省一半内存。
 *   但为了通用性 (TIM 支持不同的数据宽度), 这里用 uint32_t。
 */

/* TIM 和 DMA 通道 (与 main.c 配置一致) */
#define WS2812_TIM               (&htim5)
#define WS2812_CHANNEL           TIM_CHANNEL_4

extern TIM_HandleTypeDef htim5;

/**
 * @brief  初始化 WS2812 (关灯)
 * @note   TODO: 清空缓冲区 → 调 Update 关灯
 *
 * 实现步骤:
 *   1. 清空 s_buffer 全部为 0 (或 WS2812_T0H)
 *   2. 调用 WS2812_Update() 发送数据, 让所有灯熄灭
 *   3. 注意: 需要等待传输完成 (WS2812_WaitReady)
 */
void WS2812_Init(void);

/**
 * @brief  设置单颗灯颜色
 * @param  index: LED 索引 (0~2)
 * @param  r,g,b: RGB 颜色分量 (0~255)
 * @note   TODO: GRB 格式编码到 PWM 缓冲区
 *
 * 实现步骤:
 *   1. 构造 GRB 颜色: uint32_t grb = (g<<16) | (r<<8) | b;
 *   2. 计算缓冲区偏移: buf_ptr = &s_buffer[RESET_LEN + index*24]
 *   3. 遍历 24 bit (高位到低位, bit 23→0):
 *      if (grb & (1UL << bit)) → *buf_ptr++ = WS2812_T1H
 *      else                    → *buf_ptr++ = WS2812_T0H
 *   4. 注意: 这只是修改缓冲区, 需要调用 WS2812_Update
 *      才能把数据发送到 WS2812。
 *
 *   为什么从高位到低位?
 *     WS2812 期望 MSB first (高位在前)。
 *     bit 23 (G 的最高位) 先发送, bit 0 (B 的最低位) 最后发送。
 */
void WS2812_SetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  设置所有灯同色
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  启动 DMA 发送到 WS2812
 * @note   需在 HAL_TIM_PWM_PulseFinishedCallback 中停 DMA
 *
 * 实现步骤:
 *   1. 等待前一次传输完成 (WS2812_WaitReady)
 *   2. 设置忙标志 (s_busy = 1)
 *   3. 启动 PWM DMA 传输:
 *      HAL_TIM_PWM_Start_DMA(WS2812_TIM, WS2812_CHANNEL,
 *                            (uint32_t*)s_buffer, WS2812_BUF_LEN)
 *   4. DMA 传输完成后, 在回调函数中停止 PWM 并清除忙标志
 *
 * 为什么不能在前一次传输进行中再启动一次?
 *   因为 DMA 和 TIM 还在处理上一次的缓冲区,
 *   此时修改缓冲区会导致数据错乱。
 *   所以必须 WaitReady 才能启动下一次。
 */
void WS2812_Update(void);

/**
 * @brief  等待上次 DMA 发送完成
 */
void WS2812_WaitReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */
