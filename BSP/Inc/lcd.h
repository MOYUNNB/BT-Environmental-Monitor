/**
 * @file    lcd.h
 * @brief   ST7789V2 LCD 驱动 (SPI1, 21MHz, 240×320, CS=PE14/DC=PD14/RST=PE1)
 * @note    SPI 发送辅助函数已实现, Init/Clear/DrawPixel/DrawString 需补充
 */
#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 屏幕尺寸 */
#define LCD_WIDTH               240U
#define LCD_HEIGHT              320U

/*
 * RGB565 颜色格式: RRRRR GGGGGG BBBBB (16 位)
 * 构造: ((r>>3)<<11) | ((g>>2)<<5) | (b>>3)
 */
#define LCD_COLOR_WHITE         0xFFFFU
#define LCD_COLOR_BLACK         0x0000U
#define LCD_COLOR_RED           0xF800U
#define LCD_COLOR_GREEN         0x07E0U
#define LCD_COLOR_BLUE          0x001FU
#define LCD_COLOR_CYAN          0x07FFU
#define LCD_COLOR_MAGENTA       0xF81FU
#define LCD_COLOR_YELLOW        0xFFE0U
#define LCD_COLOR_GRAY          0x8430U
#define LCD_COLOR_ORANGE        0xFD20U
#define LCD_COLOR_DARKBLUE      0x01CFU

/* 显示方向 — 通过 MADCTL(0x36) 的 MV/MX/MY 位控制 */
typedef enum {
    LCD_DIR_PORTRAIT = 0,       /* 0°   — 240×320 */
    LCD_DIR_LANDSCAPE = 1,      /* 90°  — 320×240, MADCTL=0x60(MV|MX) */
    LCD_DIR_PORTRAIT_180 = 2,   /* 180° — MADCTL=0xC0(MX|MY) */
    LCD_DIR_LANDSCAPE_270 = 3,  /* 270° — MADCTL=0xA0(MV|MY) */
} LCD_Direction_t;

/*
 * SPI1 独占 (不与任何设备共享), 因此不需要互斥锁
 * 数据量大时 (如全屏填充 153600 字节), 考虑 DMA 传输
 */

/* 上电初始化: 硬件复位 → 发初始化序列 (sleep_out + 时序参数 + 开显示) */
void LCD_Init(void);
/* 显示方向: CASET/RASET 交换 + MADCTL.MV/MX/MY 位 */
void LCD_SetDirection(LCD_Direction_t dir);

/*
 * 画窗模式: CASET(0x2A)+RASET(0x2B) 设置矩形区域 → RAMWR(0x2C) 发送像素
 * 连续像素自动填入窗口 (从左到右, 从上到下)
 */
void LCD_Clear(uint16_t color);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/*
 * ASCII 字符串 — 需要实现 5×7 字库表 (0x20~0x7E, ~475 字节)
 * scale=放大倍数 (1~4), 每个像素变 scale×scale 方块
 */
void LCD_DrawString(uint16_t x, uint16_t y, const char *str,
                    uint16_t color, uint16_t bg, uint8_t scale);

/* 整数/浮点包装: snprintf → DrawString */
void LCD_DrawInt(uint16_t x, uint16_t y, int32_t value,
                 uint16_t color, uint16_t bg, uint8_t scale);
void LCD_DrawFloat(uint16_t x, uint16_t y, float value, uint8_t decimals,
                   uint16_t color, uint16_t bg, uint8_t scale);

/* 进度条: 基于 FillRect 实现 */
void LCD_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         uint8_t percent, uint16_t color_fg, uint16_t color_bg);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */
