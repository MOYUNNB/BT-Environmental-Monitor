/**
 * @file    ws2812.c
 * @brief   WS2812 RGB LED 驱动实现 (TIM5_CH4 PWM + DMA)
 * @note    PWM+DMA 发送, CPU 零占用.
 *
 * 工作原理:
 *   WS2812 使用单线归零码, 每 bit 由高电平宽度表示 0/1:
 *     T0H ≈ 0.35μs (高), T0L ≈ 0.9μs  (低) → 码 0
 *     T1H ≈ 0.7μs  (高), T1L ≈ 0.55μs (低) → 码 1
 *   用 TIM5_CH4 PWM, 通过调整 CCR4 值产生不同占空比,
 *   DMA 从缓冲区连续搬移 CCR 值, 实现 24bit × N 颗灯的数据发送。
 *   发送完后需 >50μs 低电平复位。
 *
 * 缓冲区结构:
 *   [RESET脉冲] [LED0_24bit] [LED1_24bit] [LED2_24bit] [RESET脉冲]
 *
 * 参考: 嘉立创 fdb-master/0_example/RGB/ws2812-onboard-pa3-project
 */
#include "ws2812.h"
#include <stdio.h>

/*
 * ============================================================
 *  学习笔记: WS2812 驱动整体设计
 * ============================================================
 *
 * 为什么用 PWM + DMA 而不是软件延时?
 *
 * 软件延时法 (不推荐):
 *   for (int bit = 23; bit >= 0; bit--) {
 *       if (color & (1UL << bit)) {
 *           DIN_HIGH(); delay_ns(700);  // T1H
 *           DIN_LOW();  delay_ns(550);  // T1L
 *       } else {
 *           DIN_HIGH(); delay_ns(350);  // T0H
 *           DIN_LOW();  delay_ns(900);  // T0L
 *       }
 *   }
 *
 * 软件延时的问题:
 *   1. CPU 被完全占用, 无法响应中断或其他任务
 *   2. 延时精度受中断响应影响 (如果中断来了, 时序会歪)
 *   3. 不同编译器优化级别下延时不同
 *   4. 频繁 IO 操作效率低
 *   5. 如果要更新多颗灯, CPU 要忙几百微秒
 *
 * PWM + DMA 法的优势:
 *   1. CPU 零占用 — 启动 DMA 后 CPU 可以去干别的事
 *   2. 时序精确 — 由硬件 TIM 和 DMA 保证
 *   3. 不受中断影响 — TIM 和 DMA 硬件独立运行
 *   4. 容易控制多颗灯 — 只需增加缓冲区长度
 *   5. 可以在 RTOS 中使用 — 不会阻塞高优先级任务
 *
 * 缺点:
 *   1. 占用一个 TIM 资源和一个 DMA 通道
 *   2. 初始化配置较复杂 (需要理解 TIM/DMA 配置)
 *   3. 需要较大的缓冲区 (~4KB for 3 LEDs, uint32_t)
 *   4. 修改颜色后需要重新触发 DMA 传输
 *
 * ============================================================
 *  学习笔记: TIM 配置详解 (来自 main.c / CubeMX)
 * ============================================================
 *
 * TIM5 配置:
 *   Clock Source: Internal Clock
 *   Channel 4: PWM Generation CH4
 *   Prescaler (PSC): 0 (不分频)
 *   Counter Mode: Up
 *   Counter Period (ARR): 104 (如已调整到 800KHz)
 *   Auto-reload preload: Enable
 *   CH4 Polarity: High
 *
 * 时钟计算:
 *   TIM5_CLK = APB1_TIM_CLK
 *   如果 APB1 prescaler = 4 (42MHz), TIM5 自动 ×2 = 84MHz
 *   PWM = 84MHz / (104+1) / (0+1) = 800 KHz ✅
 *
 * DMA 配置:
 *   DMA2 Stream x (查手册: TIM5_CH4 使用哪个 DMA 通道/流)
 *   Direction: Memory to Peripheral
 *   Peripheral: TIM5->CCR4 (地址由 HAL 处理)
 *   Memory: s_buffer (uint32_t 数组)
 *   Increment Peripheral: No  (每次都写同一个 CCR 寄存器)
 *   Increment Memory: Yes     (缓冲区地址递增)
 *   Data Width: Word (32-bit, 匹配 TIM5->CCR4 宽度)
 *   Mode: Normal (不是循环模式, 传输一次就停)
 *
 * ============================================================
 *  学习笔记: DMA 缓冲区格式详解
 * ============================================================
 *
 * s_buffer 的每一个元素不是"bit", 而是"PWM 周期"。
 * 每个 PWM 周期产生一个脉冲, 脉冲的宽度由 CCR 值决定。
 *
 * 码 0 的波形:
 *   ┌──┐
 *   │  │
 *   └──┴──────────────────┘
 *   ↑ CCR=T0H    ↑ 周期结束, 自动归零
 *
 * 码 1 的波形:
 *   ┌──────────┐
 *   │          │
 *   └──────────┴──────────┘
 *   ↑ CCR=T1H    ↑ 周期结束, 自动归零
 *
 * 所以:
 *   每个 bit = 1 个 PWM 周期 = 1 个 s_buffer 元素
 *   24 bits = 24 个 s_buffer 元素
 *   N 颗灯 = N × 24 个 s_buffer 元素
 *
 * 缓冲区索引分配:
 *   [0..RESET_LEN-1]:         前复位 (全 T0H 或 0)
 *   [RESET_LEN..RESET_LEN+23]: LED0 的 24 bit
 *   [RESET_LEN+24..+47]:      LED1 的 24 bit
 *   [RESET_LEN+48..+71]:      LED2 的 24 bit
 *   [RESET_LEN+72..+RESET_LEN+72+RESET_LEN-1]: 后复位
 *
 * 为什么复位用 T0H 而不是 0?
 * 有些 WS2812 版本对 T0H 和纯低电平的反应不同。
 * 为了保险, 复位脉冲统一用 T0H (码 0 的占空比)。
 * 但更常见的是直接写 0 (CCR=0, 占空比=0%)。
 * 因为 WS2812 的复位条件是: DIN 低 >50μs, 占空比 0% 就是恒低。
 *
 * 实际上, 方法有两种:
 * 方案 A: 复位段填 0 (CCR=0) — 最常用
 * 方案 B: 复位段填 T0H — 保持最后一个 bit 是码 0 的波形
 * 本驱动采用方案 A (复位段填 0)。
 */

/* 缓冲区总长 = 复位 + N×24 + 复位 */
#define WS2812_BUF_LEN  (WS2812_RESET_LEN + WS2812_NUM * 24U + WS2812_RESET_LEN)
/*
 * 缓冲区大小计算:
 *   WS2812_RESET_LEN = 500
 *   WS2812_NUM = 3
 *   WS2812_BUF_LEN = 500 + 3×24 + 500 = 1072
 *
 * 每个元素是 uint32_t (4 字节)
 * 总内存: 1072 × 4 = 4288 字节 ≈ 4.2 KB
 *
 * 这 4.2 KB 是静态分配的 (编译时分配), 不占用堆空间。
 * 如果使用 uint16_t (DMA 半字传输), 可减半为 2.1 KB。
 * 但需要确保 DMA 配置的数据宽度也是半字 (Half Word)。
 *
 * 注意: 本项目 FreeRTOS 堆约 94KB, 任务栈约 9KB,
 * 全局变量约 2KB, 所以 4.2KB 是完全可以接受的。
 */

/* DMA 缓冲区 (每个元素是 TIM CCR 值) */
static uint32_t s_buffer[WS2812_BUF_LEN];
/*
 * s_buffer 是全局/静态变量, 存储在 .bss 段 (未初始化).
 * 上电时 .bss 段由启动代码自动清零。
 * 所以首次使用时 s_buffer 全为 0, 即所有灯灭。
 *
 * 但在 WS2812_Init() 中显式清空缓冲区是一个好习惯,
 * 保证了代码的可读性和可移植性。
 */

static volatile uint8_t s_busy = 0;
/*
 * s_busy 标志:
 *   volatile 关键字表示:
 *     - 这个变量可能在中断处理函数中被修改
 *     - 编译器不能对这个变量的访问做优化
 *     - 每次使用都必须从内存读取, 不能使用寄存器缓存的值
 *
 * 如果不加 volatile, 在以下场景会出现 bug:
 *   while (s_busy != 0) {}  // 等待
 *   编译器可能优化为: 读取一次 s_busy,
 *   如果为 0 就跳过循环, 否则死循环。
 *   但实际上 s_busy 可能在中断中被改为 0,
 *   编译器不知道这一点, 就会出问题。
 */

void WS2812_WaitReady(void)
{
    /*
     * 等待前一次 DMA 传输完成。
     * s_busy 在传输开始时被设为 1,
     * 在传输完成回调中被设为 0。
     *
     * 这是一个"忙等待" (busy-waiting) 循环。
     * 在 FreeRTOS 中, 如果等待时间较长,
     * 可以考虑在这里加 osDelay(1) 让出 CPU。
     *
     * 但对于 WS2812, 传输时间很短:
     *   3 颗灯: ~110μs (含复位时间)
     *   1050 个 PWM 周期 × 1.25μs ≈ 1.3ms
     * 所以忙等待是可以接受的。
     *
     * 但如果灯的数量很多 (如 100 颗), 传输时间可能 > 3ms,
     * 这时应该考虑用信号量阻塞等待, 而非忙等。
     */
    while (s_busy != 0U) { }
}

/*
 * ============================================================
 *  学习笔记: WS2812_Init 实现指南
 * ============================================================
 *
 * 初始化 WS2812 就是确保所有灯熄灭:
 *   1. 清空 s_buffer (全写 0)
 *   2. 调用 WS2812_Update() 发送数据
 *   3. 等待传输完成
 *
 * 注意: 初始化时 TIM5 应已由 CubeMX 初始化完成。
 * 如果 HAL_TIM_PWM_Start_DMA 返回 HAL_ERROR,
 * 检查 TIM5 和 DMA 的配置。
 */
void WS2812_Init(void)
{
    printf("[WS2812dbg] step1: clear buf... ");
    s_busy = 0U;
    for (uint16_t i = 0; i < WS2812_BUF_LEN; i++) {
        s_buffer[i] = 0;
    }
    printf("OK\r\n");

    printf("[WS2812dbg] step2: Update... ");
    WS2812_Update();
    printf("OK (busy=%d)\r\n", (int)s_busy);

    printf("[WS2812dbg] step3: WaitReady... ");
    WS2812_WaitReady();
    printf("OK\r\n");
}

/*
 * ============================================================
 *  学习笔记: WS2812_SetPixel 实现指南
 * ============================================================
 *
 * 将 R/G/B 颜色编码为 PWM 缓冲区中的 CCR 值。
 *
 * 编码过程:
 *   1. 拼接 GRB: g<<16 | r<<8 | b
 *   2. 遍历 24 bit (高位到低位)
 *   3. 封装每个 bit 为 CCR 值
 *
 * 缓冲区索引计算:
 *   偏移 = 前导复位长度 + LED索引 × 24
 *   uint32_t *p = &s_buffer[WS2812_RESET_LEN + index * 24U];
 *
 * 遍历 24 bit:
 *   for (int8_t bit = 23; bit >= 0; bit--) {
 *       if (grb & (1UL << bit))
 *           *p++ = WS2812_T1H;  // bit=1 → 宽脉冲
 *       else
 *           *p++ = WS2812_T0H;  // bit=0 → 窄脉冲
 *   }
 *
 * 注意:
 *   1. 索引越界检查: if (index >= WS2812_NUM) return;
 *   2. 颜色值范围: 0~255
 *   3. 这只是准备缓冲区, 不触发传输
 *   4. 多次 SetPixel 后调用 Update 一次性发送
 */
void WS2812_SetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_NUM) return;

    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;  /* GRB 格式 */
    uint32_t *buf = &s_buffer[WS2812_RESET_LEN + (uint32_t)index * 24U];
    for (int8_t bit = 23; bit >= 0; bit--) {
        *buf++ = (color & (1UL << (uint32_t)bit)) ? WS2812_T1H : WS2812_T0H;
    }
}

/*
 * ============================================================
 *  学习笔记: WS2812_SetAll 实现指南
 * ============================================================
 *
 * 最简单的实现: for 循环, 每颗灯调用 SetPixel。
 *
 * void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
 *     for (uint8_t i = 0; i < WS2812_NUM; i++) {
 *         WS2812_SetPixel(i, r, g, b);
 *     }
 * }
 *
 * 这样可以实现:
 *   WS2812_SetAll(255, 0, 0)   → 所有灯红色
 *   WS2812_SetAll(0, 255, 0)   → 所有灯绿色
 *   WS2812_SetAll(0, 0, 255)   → 所有灯蓝色
 *   WS2812_SetAll(255, 255, 255) → 所有灯白色
 *   WS2812_SetAll(0, 0, 0)     → 所有灯灭
 *
 * 如果要实现不同的颜色或动画效果:
 *   1. 单独调 SetPixel(index, r, g, b)
 *   2. 然后调 Update()
 */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < WS2812_NUM; i++) {
        WS2812_SetPixel(i, r, g, b);
    }
}

/*
 * ============================================================
 *  学习笔记: WS2812_Update 实现指南
 * ============================================================
 *
 * 启动 DMA 传输, 将缓冲区中的数据发送到 WS2812。
 *
 * 实现:
 *   WS2812_WaitReady();  // 等待前一次完成
 *   s_busy = 1;           // 设置忙标志
 *   HAL_TIM_PWM_Start_DMA(WS2812_TIM, WS2812_CHANNEL,
 *                         (uint32_t*)s_buffer, WS2812_BUF_LEN);
 *    // 启动后, DMA 自动从 s_buffer 搬运数据到 TIM5->CCR4
 *    // CPU 可以去做其他事情
 *
 * HAL_TIM_PWM_Start_DMA 内部做了什么:
 *   1. 使能 TIM5_CH4 的 PWM 输出
 *   2. 配置 DMA 从 s_buffer 搬运到 &TIM5->CCR4
 *   3. 启动 DMA
 *   4. 每次 DMA 传输完成, TIM 更新 CCR, 输出新的占空比
 *   5. 全部传输完成 → 触发 HAL_TIM_PWM_PulseFinishedCallback
 *
 * 一个常见问题: 在调用 HAL_TIM_PWM_Start_DMA 前,
 * 确保 TIM5 的 PWM 输出已经配置正确 (在 CubeMX 中配置好了)。
 * 如果返回 HAL_ERROR, 检查:
 *   - DMA 是否已初始化
 *   - DMA 流是否被其他外设占用
 *   - TIM5 是否已使能
 */
void WS2812_Update(void)
{
    WS2812_WaitReady();
    s_busy = 1;
    if (HAL_TIM_PWM_Start_DMA(WS2812_TIM, WS2812_CHANNEL,
                               (uint32_t*)s_buffer, WS2812_BUF_LEN) != HAL_OK) {
        /* DMA 未配置或配置错误, 清除忙标志避免死锁 */
        s_busy = 0;
    }
}

/*
 * PWM 传输完成回调: 停止 PWM + 清除忙标志
 * HAL 弱函数, 定义在这里自动覆盖默认实现
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        HAL_TIM_PWM_Stop_DMA(WS2812_TIM, WS2812_CHANNEL);
        s_busy = 0;
    }
}

/*
 * ============================================================
 *  学习笔记: PWM 传输完成回调 — HAL_TIM_PWM_PulseFinishedCallback
 * ============================================================
 *
 * 当 DMA 传输完 s_buffer 的所有元素后:
 *   1. TIM5 输出最后一个 PWM 脉冲
 *   2. DMA 传输完成中断触发
 *   3. HAL 库调用 HAL_TIM_PWM_PulseFinishedCallback
 *   4. 在这个回调中: 停止 PWM 输出, 清除忙标志
 *
 * 这个回调函数应该在 stm32f4xx_it.c 中实现,
 * 或者直接写在 ws2812.c 中 (HAL 弱函数重写)。
 *
 * 注意: 这个回调在中断上下文中执行!
 *   所以代码必须简洁, 不能有 HAL_Delay 等阻塞操作。
 *   如果需要, 可以用信号量通知任务, 而非在中断中处理。
 *
 * 实现:
 *
 * void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 * {
 *     // 检查是否是 TIM5
 *     if (htim->Instance == TIM5) {
 *         // 停止 PWM 输出
 *         HAL_TIM_PWM_Stop_DMA(WS2812_TIM, WS2812_CHANNEL);
 *         // 清除忙标志, 允许下一帧传输
 *         s_busy = 0;
 *         // 可选: 释放信号量通知任务 "LED 更新完成"
 *         // osSemaphoreRelease(s_led_done_sem);
 *     }
 * }
 *
 * 为什么需要停止 PWM?
 *   如果不停止 PWM, TIM5 会继续以最后一个 CCR 值输出 PWM,
 *   WS2812 会认为数据还在传输, 不会锁存颜色。
 *   停止 PWM 后, TIM5 输出恢复为默认状态 (高/低取决于配置),
 *   再经过复位区间的低电平, WS2812 就知道帧结束了。
 *
 * 另一种实现方式:
 *   如果 DMA 配置了完成中断回调, 也可以使用:
 *   HAL_DMA_RegisterCallback / HAL_DMA_IRQHandler
 *   但 HAL_TIM_PWM_PulseFinishedCallback 更简洁。
 *
 * 注意: 总线锁的考虑
 *   本项目中 TIM5 是专用于 WS2812 的, 不与其他设备共享。
 *   所以不需要互斥锁保护。
 */
