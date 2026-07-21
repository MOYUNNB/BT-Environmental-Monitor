/**
 * @file    ws2812.h
 * @brief   WS2812 RGB LED 驱动 (TIM5_CH4 PWM + DMA)
 * @note    800KHz PWM, 3 颗板载 LED, GRB 格式, CPU 零占用
 *          === 需要你自己实现 ===
 */
#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define WS2812_NUM              3U      /* 板载 3 颗 LED 级联 */

/*
 * PWM 时序参数 (已校准 @ 84MHz TIM5, ARR=104 → 800KHz):
 *   单周期 1.25µs, 每 tick = 11.9ns
 *   码 0: T0H≈393ns (33 tick), 码 1: T1H≈797ns (67 tick)
 *   WS2812 规格: T0H=350±150ns, T1H=700±150ns, 均在容差范围内
 */
#define WS2812_T0H              33U     /* 0 码高电平 CCR (393ns) */
#define WS2812_T1H              67U     /* 1 码高电平 CCR (797ns) */
#define WS2812_RESET_LEN        100U    /* 复位低电平 >50µs, 100×1.25µs=125µs */

/*
 * 缓冲区结构: [RESET×500] [LED0×24] [LED1×24] [LED2×24] [RESET×500]
 * 每个元素是 TIM CCR 值: 0=低电平, T0H=码0, T1H=码1
 */
#define WS2812_TIM               (&htim5)
#define WS2812_CHANNEL           TIM_CHANNEL_4

extern TIM_HandleTypeDef htim5;

/* 配置 TIM5_CH4 PWM + DMA, 初始化 WS2812 缓冲区全 0 */
void WS2812_Init(void);

/*
 * GRB 编码 (不是 RGB!):
 *   32-bit: (g<<16) | (r<<8) | b
 *   遍历 24 bit (MSB first), 为 1 写 T1H, 为 0 写 T0H
 */
void WS2812_SetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/*
 * 启动 DMA 发送: HAL_TIM_PWM_Start_DMA
 * 完成后回调里停 PWM + 清除忙标志
 */
void WS2812_Update(void);
void WS2812_WaitReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */
