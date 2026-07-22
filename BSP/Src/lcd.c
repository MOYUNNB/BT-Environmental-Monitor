/**
 * @file    lcd.c
 * @brief   ST7789V2 LCD 驱动 (参考嘉立创 st7789.c)
 * @note    SPI1, CS=PE14, DC=PD14, RST=PE1, 240×320
 *
 * 参考: 嘉立创 fdb-master/0_example/LCD/lcd-2-0-display-project/BSP/lcd/st7789.c
 */
#include "lcd.h"
#include "backlight.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* 引脚定义 (硬编码, 与 main.h 一致) */
#define LCD_CS_PORT             GPIOE
#define LCD_CS_PIN              GPIO_PIN_14
#define LCD_DC_PORT             GPIOD
#define LCD_DC_PIN              GPIO_PIN_14
#define LCD_RST_PORT            GPIOE
#define LCD_RST_PIN             GPIO_PIN_1

/* SPI 句柄 (extern, main.c 中定义) */
extern SPI_HandleTypeDef hspi1;

static LCD_Direction_t s_dir = LCD_DIR_PORTRAIT;

/*
 * 颜色字节序修正:
 * MCU 是小端, SPI MSB 先发。直接发送 uint16_t 先发低字节,
 * 但 ST7789 先收高字节。这里手动组合为 {高字节, 低字节}。
 */
#define COLOR_HI(c)  ((uint8_t)((c) >> 8))
#define COLOR_LO(c)  ((uint8_t)((c) & 0xFF))

/*========== 底层 GPIO / SPI 操作 ==========*/
static inline void cs_sel(void)  { HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_des(void)  { HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET); }
static inline void dc_cmd(void)  { HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET); }
static inline void dc_dat(void)  { HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET); }

/* 单字节命令: CS↓ → DC=0 → SPI发 → DC=1 → CS↑
 * 注意: 发送完命令后 DC 回到数据模式 (与参考代码一致) */
static void lcd_write_cmd(uint8_t cmd)
{
    cs_sel();
    dc_cmd();
    HAL_SPI_Transmit(&hspi1, &cmd, 1U, HAL_MAX_DELAY);
    dc_dat();
    cs_des();
}

/* 单字节数据 */
static void lcd_write_data8(uint8_t data)
{
    cs_sel();
    dc_dat();
    HAL_SPI_Transmit(&hspi1, &data, 1U, HAL_MAX_DELAY);
    cs_des();
}

/* 连续 N 字节数据 (CS 全程保持低电平) */
static void lcd_write_datas(uint8_t *data, size_t len)
{
    if (len == 0U) return;
    cs_sel();
    dc_dat();
    HAL_SPI_Transmit(&hspi1, data, (uint16_t)len, HAL_MAX_DELAY);
    cs_des();
}

/*========== 窗口设置 (CS 全程保持低, 减少 GPIO 翻转) ==========*/
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    cs_sel();

    dc_cmd(); { uint8_t cmd = 0x2A; HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); }
    dc_dat(); { uint8_t seq[4] = { (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
                                   (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF) };
                 HAL_SPI_Transmit(&hspi1, seq, 4, HAL_MAX_DELAY); }

    dc_cmd(); { uint8_t cmd = 0x2B; HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); }
    dc_dat(); { uint8_t seq[4] = { (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
                                   (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF) };
                 HAL_SPI_Transmit(&hspi1, seq, 4, HAL_MAX_DELAY); }

    dc_cmd(); { uint8_t cmd = 0x2C; HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); }
    dc_dat();

    cs_des();
}

/*========== 显示方向 ==========*/
void LCD_SetDirection(LCD_Direction_t dir)
{
    s_dir = dir;
    uint8_t madctl = 0;
    switch (dir) {
        case LCD_DIR_PORTRAIT:      madctl = 0x00; break;
        case LCD_DIR_LANDSCAPE:     madctl = 0xC0; break;  /* MV|MX */
        case LCD_DIR_PORTRAIT_180:  madctl = 0x70; break;  /* MX|MY|ML */
        case LCD_DIR_LANDSCAPE_270: madctl = 0xA0; break;  /* MV|MY */
    }
    lcd_write_cmd(0x36);
    lcd_write_data8(madctl);
}

/*========== 填充 / 清屏 ==========*/
void LCD_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x0 > x1 || y0 > y1) return;
    if (x1 >= LCD_WIDTH)  x1 = LCD_WIDTH  - 1U;
    if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1U;

    lcd_set_window(x0, y0, x1, y1);

    uint32_t pixels = (uint32_t)(x1 - x0 + 1U) * (uint32_t)(y1 - y0 + 1U);

    /* 预填充 1KB 缓冲区 (512 像素, 颜色已交换字节序) */
    uint8_t buf[1024];
    uint8_t hi = COLOR_HI(color);
    uint8_t lo = COLOR_LO(color);
    for (uint16_t i = 0; i < 1024; i += 2) {
        buf[i]     = hi;
        buf[i + 1] = lo;
    }

    uint32_t total_bytes = pixels * 2U;
    uint32_t sent = 0;
    while (sent < total_bytes) {
        uint32_t chunk = total_bytes - sent;
        if (chunk > 1024) chunk = 1024;
        lcd_write_datas(buf, (size_t)chunk);
        sent += (uint32_t)chunk;
    }
}

void LCD_Clear(uint16_t color)
{
    LCD_FillRect(0, 0, LCD_WIDTH - 1U, LCD_HEIGHT - 1U, color);
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    lcd_set_window(x, y, x, y);
    lcd_write_data8(COLOR_HI(color));
    lcd_write_data8(COLOR_LO(color));
}

/*========== 初始化寄存器序列 ==========*/
static void lcd_reg_init(void)
{
    /* Interface Pixel Format: 16bit */
    lcd_write_cmd(0x3A);
    lcd_write_data8(0x05);

    /* Porch control */
    lcd_write_cmd(0xB2);
    {
        uint8_t seq[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        lcd_write_datas(seq, sizeof(seq));
    }

    /* Gate control */
    lcd_write_cmd(0xB7); lcd_write_data8(0x35);

    /* VCOM */
    lcd_write_cmd(0xBB); lcd_write_data8(0x32);

    /* VDVVRHEN */
    lcd_write_cmd(0xC2); lcd_write_data8(0x01);

    /* VRH */
    lcd_write_cmd(0xC3); lcd_write_data8(0x15);

    /* VDV */
    lcd_write_cmd(0xC4); lcd_write_data8(0x20);

    /* FRCTRL2 */
    lcd_write_cmd(0xC6); lcd_write_data8(0x0F);

    /* Power control */
    lcd_write_cmd(0xD0);
    {
        uint8_t seq[] = {0xA4, 0xA1};
        lcd_write_datas(seq, sizeof(seq));
    }

    /* Gamma 正极 */
    lcd_write_cmd(0xE0);
    {
        uint8_t seq[] = {0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34};
        lcd_write_datas(seq, sizeof(seq));
    }

    /* Gamma 负极 */
    lcd_write_cmd(0xE1);
    {
        uint8_t seq[] = {0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34};
        lcd_write_datas(seq, sizeof(seq));
    }

    /* 显示反转 ON */
    lcd_write_cmd(0x21);

    /* 开显示 */
    lcd_write_cmd(0x29);

    /* ※ 与参考代码一致: 发 RAMWR, 让 LCD 进入内存写模式 */
    lcd_write_cmd(0x2C);
}

/*========== 主初始化 ==========*/
void LCD_Init(void)
{
    printf("[LCD] Backlight init...\r\n");
    Backlight_Init();
    Backlight_Off();

    /* 1. 硬件复位: RST 低 10ms → 高 120ms */
    printf("[LCD] HW Reset...\r\n");
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    /* 2. 退出睡眠 */
    printf("[LCD] SLPOUT...\r\n");
    lcd_write_cmd(0x11);
    HAL_Delay(200);

    /* 3. 设置方向 (MADCTL) — 与参考一致: 在 COLMOD 前设置 */
    printf("[LCD] MADCTL...\r\n");
    LCD_SetDirection(LCD_DIR_PORTRAIT);

    /* 4. 寄存器初始化 (含 COLMOD + gamma + INVON + DISPON + RAMWR) */
    printf("[LCD] Reg init...\r\n");
    lcd_reg_init();

    /* 5. 清屏黑色 */
    printf("[LCD] Clear...\r\n");
    LCD_Clear(LCD_COLOR_BLACK);

    /* 6. 开背光 */
    printf("[LCD] Backlight ON...\r\n");
    Backlight_On();

    printf("[LCD] Init Done!\r\n");
}

/*========== 字符串绘制 ==========*/

/* 行缓冲区大小 (LCD_DrawString 用): 足够容纳最长字符串一行像素 (RGB565) */
#define DS_BUF_SIZE 1024U

/* 5x7 ASCII 字库 (0x20~0x7E, 95 字符) */
static const uint8_t s_font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x80,0x80,0x80,0x80,0x80}, /* '_' */
    {0x00,0x03,0x07,0x00,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x02,0x01,0x02,0x04,0x02}, /* '~' */
};

void LCD_DrawString(uint16_t x, uint16_t y, const char *str,
                    uint16_t color, uint16_t bg, uint8_t scale)
{
    if (scale > 4 || scale == 0) scale = 1;

    size_t len = strlen(str);
    if (len == 0) return;

    /* 每字符宽度 = 5 列字体 + 1 列间距, 乘缩放 */
    uint16_t char_step = (5U + 1U) * (uint16_t)scale;
    uint16_t total_w   = (uint16_t)(len * char_step);
    uint16_t total_h   = 7U * (uint16_t)scale;

    /* 一次窗口覆盖整个字符串 (替代原逐像素 FillRect 方式) */
    lcd_set_window(x, y, x + total_w - 1U, y + total_h - 1U);

    uint8_t buf[DS_BUF_SIZE];
    uint16_t row_bytes = total_w * 2U;
    if (row_bytes > DS_BUF_SIZE) row_bytes = DS_BUF_SIZE;

    /* 逐字体行 (7 行) */
    for (uint8_t row = 0; row < 7U; row++) {
        /* 构建该行所有字符的像素数据 */
        uint16_t idx = 0;
        const char *cp = str;
        while (*cp && idx + 2U <= row_bytes) {
            char c = *cp++;
            if (c < 0x20 || c > 0x7E) c = ' ';
            const uint8_t *glyph = s_font5x7[c - 0x20];

            /* 5 列字体, 水平缩放 */
            for (uint8_t col = 0; col < 5U; col++) {
                uint16_t pixel = (glyph[col] & (1U << row)) ? color : bg;
                uint8_t hi = COLOR_HI(pixel);
                uint8_t lo = COLOR_LO(pixel);
                for (uint8_t sc = 0; sc < scale; sc++) {
                    if (idx + 2U <= row_bytes) { buf[idx++] = hi; buf[idx++] = lo; }
                }
            }
            /* 间距列 */
            for (uint8_t sc = 0; sc < scale; sc++) {
                if (idx + 2U <= row_bytes) { buf[idx++] = COLOR_HI(bg); buf[idx++] = COLOR_LO(bg); }
            }
        }

        /* 垂直缩放: 同一行像素发送 scale 次 */
        for (uint8_t sr = 0; sr < scale; sr++) {
            lcd_write_datas(buf, idx);
        }
    }
}

void LCD_DrawInt(uint16_t x, uint16_t y, int32_t value,
                 uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)value);
    LCD_DrawString(x, y, buf, color, bg, scale);
}

void LCD_DrawFloat(uint16_t x, uint16_t y, float value, uint8_t decimals,
                   uint16_t color, uint16_t bg, uint8_t scale)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    LCD_DrawString(x, y, buf, color, bg, scale);
}

void LCD_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         uint8_t percent, uint16_t color_fg, uint16_t color_bg)
{
    if (percent > 100U) percent = 100U;
    LCD_FillRect(x, y, x + w - 1, y + h - 1, color_bg);
    uint16_t fill_w = (uint16_t)((uint32_t)w * percent / 100U);
    if (fill_w > 0U)
        LCD_FillRect(x, y, x + fill_w - 1, y + h - 1, color_fg);
}
