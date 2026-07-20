/**
 * @file    backlight.h
 * @brief   LCD 背光控制驱动
 * @note    支持两种模式:
 *          - BACKLIGHT_USE_GPIO: GPIO 简单开关 (默认)
 *          - BACKLIGHT_USE_PWM:  PWM 调光 (需在 CubeMX 中配置 TIM10)
 */
#ifndef __BACKLIGHT_H
#define __BACKLIGHT_H

/* 背光控制模式: PWM (TIM10_CH1 / PB8), 与嘉立创参考代码一致 */
#define BACKLIGHT_USE_PWM 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 编译时选择背光控制模式 */
#ifndef BACKLIGHT_USE_PWM
#define BACKLIGHT_USE_GPIO 1   /* 默认使用 GPIO 开关 */
#endif

#if BACKLIGHT_USE_PWM
/* PWM 模式: 亮度范围 0-100 */
#define BACKLIGHT_MAX_BRIGHTNESS    100
#define BACKLIGHT_DEFAULT_BRIGHTNESS 50

/**
 * @brief   初始化背光 PWM (TIM10_CH1 / PB8)
 */
void Backlight_Init(void);

/**
 * @brief   设置屏幕亮度
 * @param   brightness: 0-100 (0=关闭, 100=最亮)
 */
void Backlight_Set(uint8_t brightness);
#else
/* GPIO 模式: 引脚定义 (使用 CubeMX 配置的 LCD_BL 宏) */
#define BACKLIGHT_GPIO_Port  LCD_BL_GPIO_Port
#define BACKLIGHT_GPIO_Pin   LCD_BL_Pin

/**
 * @brief   初始化背光 GPIO
 */
void Backlight_Init(void);
#endif

/**
 * @brief   打开背光
 */
void Backlight_On(void);

/**
 * @brief   关闭背光
 */
void Backlight_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* __BACKLIGHT_H */