/**
 * @file    backlight.c
 * @brief   LCD 背光控制实现
 * @note    支持两种模式:
 *          - GPIO 模式: 简单开/关 (默认)
 *          - PWM 模式:  可调亮度 (需配置 TIM10)
 *
 * 硬件连接:
 *   - GPIO 模式: PB0 (LCD_BL, 推挽输出)
 *   - PWM 模式:  PB0 (TIM10_CH1, AF3), 10kHz PWM
 */
#include "backlight.h"
#include "stm32f4xx_hal.h"
#include "main.h"  /* 包含 CubeMX 生成的引脚宏定义 */

#if BACKLIGHT_USE_PWM
/* ===== PWM 模式实现 ===== */

/* TIM10 句柄 (extern, 由 CubeMX 生成) */
extern TIM_HandleTypeDef htim10;

/* 当前亮度记录 */
static uint8_t s_current_brightness = BACKLIGHT_DEFAULT_BRIGHTNESS;

void Backlight_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. 开启时钟 */
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 2. 配置 GPIO (PB8 -> AF3_TIM10) */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       /* 复用推挽 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速，利于 PWM 波形 */
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;   /* 映射到 TIM10 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 3. 配置定时器基础参数 */
    /* Clock = 168MHz, Target = 10kHz */
    /* PSC = 167, ARR = 99 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 167;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 99;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        /* Error Handler */
    }

    /* 4. 配置 PWM 通道 */
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
    {
        /* Error Handler */
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;                    /* PWM 模式1：CNT < CCR 时有效 */
    sConfigOC.Pulse = s_current_brightness;               /* 初始占空比 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;            /* 有效电平为低电平 */
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        /* Error Handler */
    }

    /* 5. 启动 PWM */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void Backlight_Set(uint8_t brightness)
{
    if (brightness > BACKLIGHT_MAX_BRIGHTNESS)
    {
        brightness = BACKLIGHT_MAX_BRIGHTNESS;
    }

    s_current_brightness = brightness;

    /* 修改 CCR1 寄存器改变占空比 */
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, brightness);
}

void Backlight_On(void)
{
    Backlight_Set(BACKLIGHT_MAX_BRIGHTNESS);
}

void Backlight_Off(void)
{
    Backlight_Set(0);
}

#else /* !BACKLIGHT_USE_PWM */
/* ===== GPIO 模式实现 (默认) ===== */

void Backlight_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 开启 GPIOB 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置 PB0 (LCD_BL) 为推挽输出 */
    GPIO_InitStruct.Pin = BACKLIGHT_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BACKLIGHT_GPIO_Port, &GPIO_InitStruct);

    /* 初始状态：关闭 */
    Backlight_Off();
}

void Backlight_On(void)
{
    /* 拉低 PB0 打开背光 (参考嘉立创: 低电平有效) */
    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_GPIO_Pin, GPIO_PIN_RESET);
}

void Backlight_Off(void)
{
    /* 拉高 PB0 关闭背光 */
    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_GPIO_Pin, GPIO_PIN_SET);
}

#endif /* BACKLIGHT_USE_PWM */