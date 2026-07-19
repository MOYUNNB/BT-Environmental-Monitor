/**
 * @file    lcd.c
 * @brief   ST7789V2 LCD 驱动实现
 * @note    SPI1, CS=PE14, DC=PD14, RST=PE1
 *          SPI 辅助发送函数已实现, TODO 部分需要你来写!
 *
 * ST7789V2 初始化流程:
 *   1. 硬件复位: RST 拉低 10ms → 拉高 120ms
 *   2. 发初始化指令序列 (SLPOUT、COLMOD、MADCTL、DISPON 等)
 *   3. 开背光
 *
 * SPI 写命令: CS拉低 → DC拉低 → SPI发送命令字节 → DC拉高 → 发数据 → CS拉高
 * 连续写数据: CS拉低 → DC拉高 → SPI发送 → CS拉高
 *
 * 参考: 嘉立创 fdb-master/0_example/LCD/lcd-2-0-display-project
 *       BSP/lcd/st7789.c + st7789_port.c (约 500 行, 可直接搬运)
 */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/*
 * ============================================================
 *  学习笔记: 引脚定义与 GPIO 配置
 * ============================================================
 *
 * LCD 使用 SPI1 (PA5=SCLK, PA7=MOSI), 加上 3 个 GPIO:
 *
 *   CS (Chip Select):   PE14
 *     拉低选中 LCD, 拉高释放。
 *     因为 SPI1 只接 LCD, 理论上可以一直拉低 (上电即可)。
 *     但保持标准的 CS 控制是良好习惯。
 *
 *   DC (Data/Command): PD14
 *     这是 ST7789V2 的关键引脚:
 *       低电平 = 发送命令
 *       高电平 = 发送数据
 *     其他 SPI 设备 (如 Flash) 通常没有这个引脚。
 *     因为 ST7789V2 通过同一 SPI 总线既接收命令也接收数据,
 *     所以需要 DC 来区分。
 *
 *   RST (Reset): PE1
 *     低电平拉至少 10μs 触发硬件复位。
 *     硬件复位比软件复位 (SWRESET 命令) 更彻底。
 *     初始化开始时先做硬件复位, 再做软件复位。
 *
 * 引脚配置 (在 CubeMX 中完成):
 *   PE14: 推挽输出, 初始高电平
 *   PD14: 推挽输出, 初始高电平
 *   PE1:  推挽输出, 初始高电平
 *   速度和上拉/下拉: 默认即可, 因为 SPI 速度较高 (21MHz),
 *   建议将输出速度设为 HIGH。
 *
 * 为什么不用 CS 一直拉低?
 *   虽然 SPI1 只用于 LCD, 但如果 CS 一直拉低:
 *   1. ST7789V2 会一直处于选中状态, 增加功耗
 *   2. 如果其他 SPI 设备后续挂到 SPI1, CS 拉低会冲突
 *   3. 在 SPI 空闲时拉高 CS, 可以防止噪声误触发
 */

/* 引脚定义 (硬编码, 与 main.h 一致) */
#define LCD_CS_PORT             GPIOE
#define LCD_CS_PIN              GPIO_PIN_14
#define LCD_DC_PORT             GPIOD
#define LCD_DC_PIN              GPIO_PIN_14
#define LCD_RST_PORT            GPIOE
#define LCD_RST_PIN             GPIO_PIN_1

/* SPI 句柄 (extern, main.c 中定义) */
extern SPI_HandleTypeDef hspi1;

/* 当前方向 */
static LCD_Direction_t s_dir = LCD_DIR_PORTRAIT;

/*
 * ============================================================
 *  学习笔记: SPI 辅助发送函数设计思路
 * ============================================================
 *
 * 下面 4 个辅助函数封装了 SPI 通信的底层细节:
 *
 * cs_select/cs_deselect: CS 引脚控制, 选中/释放 LCD
 * dc_cmd/dc_data:       DC 引脚控制, 命令/数据模式
 *
 * lcd_write_cmd:  写 1 字节命令
 *   用于发送 SWRESET、SLPOUT 等单字节命令
 *   时序: CS↓ → DC=0 → SPI发 → CS↑
 *
 * lcd_write_data: 写 1 字节数据
 *   用于发送 COLMOD 的参数、CASET/RASET 的参数等
 *   时序: CS↓ → DC=1 → SPI发 → CS↑
 *
 * lcd_write_buf:  连续写 N 字节数据
 *   用于批量发送像素数据 (画窗模式)
 *   时序: CS↓ → DC=1 → SPI发 N字节 → CS↑
 *
 * 注意: 这些函数每次都会拉高拉低 CS。
 * 对于 CASET + RASET + RAMWR 这样的命令序列,
 * 更好的做法是让 CS 在整个序列中保持低电平:
 *
 *   CS↓
 *   DC=0, SPI发 CASET(0x2A)
 *   DC=1, SPI发 XS, XS, XE, XE (4字节参数)
 *   DC=0, SPI发 RASET(0x2B)
 *   DC=1, SPI发 YS, YS, YE, YE (4字节参数)
 *   DC=0, SPI发 RAMWR(0x2C)
 *   DC=1, SPI发 像素数据... (连续 N 字节)
 *   CS↑
 *
 * 但为了简化代码, 下面的辅助函数是"每事务一 CS"。
 * 如果你追求效率, 可以自己实现一个 lcd_send_cmd_data() 函数
 * 保持 CS 不变跨越多字节。
 */

/* ---- SPI 辅助发送 (已实现) ---- */

static void cs_select(void) { HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET); }
static void cs_deselect(void) { HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET); }
static void dc_cmd(void) { HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET); }
static void dc_data(void) { HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET); }

static void lcd_write_cmd(uint8_t cmd)
{
    /*
     * 发命令: CS选中 → DC命令模式 → SPI发送 → CS释放
     * 参数 cmd 是 ST7789V2 的命令码, 如 0x01(SWRESET)
     */
    cs_select();
    dc_cmd();
    HAL_SPI_Transmit(&hspi1, &cmd, 1U, 100U);
    cs_deselect();
}

static void lcd_write_data(uint8_t data)
{
    /*
     * 发 1 字节数据: CS选中 → DC数据模式 → SPI发送 → CS释放
     * 通常在发送命令后调用, 用于发送命令参数。
     * 例如: COLMOD(0x3A) 后发 0x55 表示 RGB565。
     */
    cs_select();
    dc_data();
    HAL_SPI_Transmit(&hspi1, &data, 1U, 100U);
    cs_deselect();
}

static void lcd_write_buf(const uint8_t *buf, uint16_t len)
{
    /*
     * 批量发送数据: 用于大量像素数据的传输。
     * DC 在批量传输过程中保持高电平 (数据模式),
     * CS 全程保持低电平。
     *
     * 这里为了简化, 使用阻塞式 HAL_SPI_Transmit。
     * 对于大尺寸图片, 建议改用:
     *   HAL_SPI_Transmit_DMA(&hspi1, buf, len);
     * 这样 CPU 在 SPI 传输期间可以去处理其他任务。
     *
     * 但如果使用 DMA, 需要注意:
     * 1. DMA 传输期间 buf 不能被修改或释放
     * 2. 需要通过回调函数在传输完成后拉高 CS
     * 3. DMA 传输完成前不能再发起新的 SPI 传输
     */
    cs_select();
    dc_data();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 100U);
    cs_deselect();
}

/* ========== TODO: 以下函数需要你自己实现 ========== */

/*
 * ============================================================
 *  学习笔记: LCD_Init 实现指南
 * ============================================================
 *
 * 初始化顺序是关键:
 *
 *   Step 1: 硬件复位
 *     RST 拉低 → 延时 10ms → RST 拉高 → 延时 120ms
 *
 *   Step 2: 软件复位 + 退出睡眠
 *     lcd_write_cmd(0x01);  // SWRESET
 *     HAL_Delay(150);
 *     lcd_write_cmd(0x11);  // SLPOUT
 *     HAL_Delay(200);
 *
 *   Step 3: 配置像素格式 (RGB565)
 *     lcd_write_cmd(0x3A);  // COLMOD
 *     lcd_write_data(0x55); // 16-bit RGB565
 *
 *   Step 4: 配置显示方向 (竖屏/横屏)
 *     LCD_SetDirection(LCD_DIR_PORTRAIT);
 *
 *   Step 5: 开显示
 *     lcd_write_cmd(0x29);  // DISPON
 *     HAL_Delay(50);
 *
 *   遇到黑屏怎么办:
 *     1. 检查 RST 引脚的复位时序是否正常
 *     2. 检查 DC 引脚电平是否正确 (命令/数据切换)
 *     3. 尝试在初始化序列前加更长的延时 (LCD 上电需要时间)
 *     4. 检查背光引脚是否使能
 *     5. 用逻辑分析仪抓 SPI 信号
 *
 *   常见错误:
 *     - 没有延时或延时不够 (LCD 内部振荡器需要时间稳定)
 *     - COLMOD 和实际发送的像素格式不匹配
 *     - MADCTL 方向设置不对 (屏幕显示内容反了)
 */
void LCD_Init(void)
{
    /*
     * TODO: 实现初始化流程
     *
     * // 硬件复位
     * HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
     * HAL_Delay(10);
     * HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
     * HAL_Delay(120);
     *
     * // 软件复位
     * lcd_write_cmd(0x01);  // SWRESET
     * HAL_Delay(150);
     *
     * // 退出睡眠模式
     * lcd_write_cmd(0x11);  // SLPOUT
     * HAL_Delay(200);
     *
     * // 配置像素格式
     * lcd_write_cmd(0x3A);  // COLMOD
     * lcd_write_data(0x55); // RGB565
     *
     * // 设置初始方向
     * LCD_SetDirection(LCD_DIR_PORTRAIT);
     *
     * // 开显示
     * lcd_write_cmd(0x29);  // DISPON
     * HAL_Delay(50);
     *
     * // 清屏
     * LCD_Clear(LCD_COLOR_BLACK);
     *
     * // 如果需要, 在这里使能背光:
     * // 如果背光由 GPIO 控制:
     * //     HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_SET);
     * // 如果由 PWM 控制:
     * //     HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_x);
     * //     __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_x, 100);
     */
}

/*
 * ============================================================
 *  学习笔记: LCD_SetDirection 实现指南
 * ============================================================
 *
 * MADCTL (0x36) 是内存访问控制寄存器:
 *   通过修改 MX/MY/MV 位来改变显示方向。
 *
 * 各 bit 含义:
 *   bit7 (MY):  行地址顺序, 0=从上到下, 1=从下到上
 *   bit6 (MX):  列地址顺序, 0=从左到右, 1=从右到左
 *   bit5 (MV):  行列交换, 0=正常, 1=交换 (实现横屏)
 *   bit4 (ML):  行输出模式 (垂直刷新方向)
 *   bit3 (BGR): 颜色顺序, 0=RGB, 1=BGR
 *
 * 各方向 MADCTL 值:
 *   Portrait      (0°):   0x00
 *   Landscape     (90°):  0x60  (MV|MX)
 *   Portrait_180  (180°): 0xC0  (MX|MY)
 *   Landscape_270 (270°): 0xA0  (MV|MY)
 *
 * 注意: 改变方向后, 屏幕的"宽"和"高"会交换。
 * 例如: 竖屏时 240x320, 横屏时变成 320x240。
 * 在定义 LCD_WIDTH/LCD_HEIGHT 时需要根据方向决定。
 *
 * 处理方式:
 *   简单方式: 固定竖屏, 不使用横屏
 *   高级方式: 根据 s_dir 动态交换 width/height
 */
void LCD_SetDirection(LCD_Direction_t dir)
{
    s_dir = dir;
    /*
     * TODO: 修改 MADCTL (0x36) 参数
     *
     * uint8_t madctl = 0;
     * switch (dir) {
     *     case LCD_DIR_PORTRAIT:      madctl = 0x00; break;
     *     case LCD_DIR_LANDSCAPE:     madctl = 0x60; break;  // MV|MX
     *     case LCD_DIR_PORTRAIT_180:  madctl = 0xC0; break;  // MX|MY
     *     case LCD_DIR_LANDSCAPE_270: madctl = 0xA0; break;  // MV|MY
     * }
     * lcd_write_cmd(0x36);  // MADCTL
     * lcd_write_data(madctl);
     */
}

/*
 * ============================================================
 *  学习笔记: LCD_Clear 实现指南
 * ============================================================
 *
 * 清屏就是填充整个 GRAM 为一种颜色:
 *   1. CASET(0, 239) — 所有列
 *   2. RASET(0, 319) — 所有行
 *   3. RAMWR — 发送 76800 次颜色值
 *
 * 这需要发送 240 × 320 × 2 = 153600 字节。
 * 在 21MHz SPI 下大约需要 58ms (理论值)。
 *
 * 实现方案对比:
 *
 * 方案 A: 逐像素 for 循环 (最简单)
 *   for (uint32_t i = 0; i < 76800; i++) {
 *       lcd_write_data(color >> 8);  // 高字节
 *       lcd_write_data(color & 0xFF); // 低字节
 *   }
 *   -> 非常慢! 每次数据传输都有脚本开销。
 *
 * 方案 B: 小缓冲区循环 (推荐入门)
 *   uint8_t buf[512];  // 256 像素
 *   for (size = 0; size < 512; size += 2) {
 *       buf[size]   = color >> 8;
 *       buf[size+1] = color & 0xFF;
 *   }
 *   for (uint32_t i = 0; i < 76800 / 256; i++) {
 *       lcd_write_buf(buf, 512);
 *   }
 *   -> 效率高很多, 且 RAM 占用小
 *
 * 方案 C: DMA + 大缓冲区 (最高效)
 *   准备 153600 字节缓冲区 (需要外部 SRAM 或 SDRAM)
 *   HAL_SPI_Transmit_DMA(&hspi1, buf, 153600);
 *   -> CPU 零占用, 传输完成后在回调中操作
 *   -> 但需要外部存储, 本项目可能没有足够 RAM
 */
void LCD_Clear(uint16_t color)
{
    /*
     * TODO: 设置窗口为全屏 → 批量写 color 数据
     *
     * // 设置全屏窗口
     * lcd_write_cmd(0x2A);              // CASET
     * lcd_write_data(0x00); lcd_write_data(0x00);  // XS = 0
     * lcd_write_data(0x00); lcd_write_data(0xEF);  // XE = 239
     * lcd_write_cmd(0x2B);              // RASET
     * lcd_write_data(0x00); lcd_write_data(0x00);  // YS = 0
     * lcd_write_data(0x01); lcd_write_data(0x3F);  // YE = 319
     *
     * // 开始写像素
     * lcd_write_cmd(0x2C);              // RAMWR
     *
     * // 填充所有像素 (76800 个)
     * uint8_t hi = (color >> 8) & 0xFF;
     * uint8_t lo = color & 0xFF;
     * for (uint32_t i = 0; i < (240 * 320); i++) {
     *     lcd_write_data(hi);
     *     lcd_write_data(lo);
     * }
     *
     * 提示: 如果屏幕中间有条纹, 说明 SPI 时钟频率太高
     * (信号质量不佳), 可以降低 SPI 波特率分频试试。
     */
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    (void)x; (void)y; (void)color;
    /*
     * TODO: 设置 CASET(0x2A) + RASET(0x2B) → RAMWR(0x2C) 写 2 字节颜色
     *
     * // 设置单像素窗口
     * lcd_write_cmd(0x2A);              // CASET
     * lcd_write_data(x >> 8); lcd_write_data(x & 0xFF);
     * lcd_write_data(x >> 8); lcd_write_data(x & 0xFF);
     * lcd_write_cmd(0x2B);              // RASET
     * lcd_write_data(y >> 8); lcd_write_data(y & 0xFF);
     * lcd_write_data(y >> 8); lcd_write_data(y & 0xFF);
     *
     * // 写入颜色
     * lcd_write_cmd(0x2C);              // RAMWR
     * lcd_write_data((color >> 8) & 0xFF);  // 高字节
     * lcd_write_data(color & 0xFF);         // 低字节
     *
     * 注意: CASET/RASET 参数是 16 位, 所以需要拆为高低字节,
     * 且 ST7789V2 使用 Big Endian 格式。
     * x > 255 时 x>>8 不为 0, 所以不能直接写 0x00。
     * 正确写法: 总是拆为高低字节发送。
     */
}

void LCD_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    (void)x0; (void)y0; (void)x1; (void)y1; (void)color;
    /*
     * TODO: 设置窗口 → 循环或 DMA 写 color
     *
     * 实现和 LCD_Clear 类似, 但窗口大小是 (x1-x0+1) × (y1-y0+1):
     *
     * uint32_t pixels = (uint32_t)(x1 - x0 + 1) * (y1 - y0 + 1);
     *
     * // 设置窗口
     * lcd_write_cmd(0x2A);   // CASET
     * lcd_write_data(x0 >> 8); lcd_write_data(x0 & 0xFF);
     * lcd_write_data(x1 >> 8); lcd_write_data(x1 & 0xFF);
     * lcd_write_cmd(0x2B);   // RASET
     * lcd_write_data(y0 >> 8); lcd_write_data(y0 & 0xFF);
     * lcd_write_data(y1 >> 8); lcd_write_data(y1 & 0xFF);
     *
     * // 填充
     * lcd_write_cmd(0x2C);   // RAMWR
     * uint8_t hi = (uint8_t)(color >> 8);
     * uint8_t lo = (uint8_t)color;
     * for (uint32_t i = 0; i < pixels; i++) {
     *     lcd_write_data(hi);
     *     lcd_write_data(lo);
     * }
     *
     * 优化: 对于大量像素, 先构建缓冲区再一次发送。
     * 就像 LCD_Clear 中方案 B 描述的那样。
     */
}

void LCD_DrawString(uint16_t x, uint16_t y, const char *str,
                    uint16_t color, uint16_t bg, uint8_t scale)
{
    (void)x; (void)y; (void)str; (void)color; (void)bg; (void)scale;
    /*
     * TODO: 遍历字符串每个字符 → 查 5x7 字库 → 画点
     *
     * 实现步骤:
     *
     * // Step 1: 定义 5x7 ASCII 字库 (只包含可打印字符 0x20~0x7E)
     * // 每个字符 5 字节, 共 95 个字符
     * static const uint8_t s_font5x7[95][5] = {
     *     // 空格 ' ' (0x20)
     *     {0x00, 0x00, 0x00, 0x00, 0x00},
     *     // '!' (0x21)
     *     {0x00, 0x00, 0x5F, 0x00, 0x00},
     *     // '"' (0x22)
     *     {0x00, 0x07, 0x00, 0x07, 0x00},
     *     // ... 需要完整 95 个字符的定义
     *     // 可以从开源项目 (Adafruit-GFX, u8g2) 复制
     * };
     *
     * // Step 2: 检查放大倍数上限
     * if (scale > 4) scale = 4;
     * if (scale == 0) scale = 1;
     *
     * // Step 3: 遍历字符串
     * uint16_t cur_x = x;
     * while (*str) {
     *     char c = *str++;
     *     if (c < 0x20 || c > 0x7E) c = ' ';  // 非可打印字符转空格
     *     const uint8_t *glyph = s_font5x7[c - 0x20];
     *
     *     for (uint8_t col = 0; col < 5; col++) {
     *         for (uint8_t row = 0; row < 7; row++) {
     *             uint16_t pixel_color = (glyph[col] & (1 << row)) ? color : bg;
     *             // 画放大的方块: scale × scale 像素
     *             LCD_FillRect(cur_x + col * scale,
     *                          y + row * scale,
     *                          cur_x + col * scale + scale - 1,
     *                          y + row * scale + scale - 1,
     *                          pixel_color);
     *         }
     *     }
     *     cur_x += (5 + 1) * scale;  // 5 列 + 1 列间距
     *
     *     // 如果超出屏幕宽度, 换行 (可选)
     * }
     *
     * 注意: 每次调用 DrawString 都会产生大量 SPI 事务。
     * 如果频繁更新文字 (如实时温度), 建议:
     * 1. 只更新变化的部分 (不要全屏清)
     * 2. 先画一个背景色矩形覆盖旧文字, 再写新文字
     */
}

/*
 * ============================================================
 *  学习笔记: LCD_DrawInt / LCD_DrawFloat — 包装函数
 * ============================================================
 *
 * 这两个函数是对 LCD_DrawString 的简单包装:
 *
 *   LCD_DrawInt(10, 20, 12345, WHITE, BLACK, 2)
 *     1. snprintf(buf, "12345") — 整数转字符串
 *     2. LCD_DrawString(10, 20, buf, WHITE, BLACK, 2)
 *
 *   LCD_DrawFloat(10, 50, 3.1415, 2, WHITE, BLACK, 1)
 *     1. snprintf(buf, "3.14") — 浮点数转字符串, 保留 2 位小数
 *     2. LCD_DrawString(10, 50, buf, WHITE, BLACK, 1)
 *
 * 注意: printf 族函数 (包括 snprintf) 会显著增加固件体积。
 * 对于 printf 中的 %f 支持, ARM GCC 需要链接选项:
 *   -u _printf_float
 * 否则浮点数格式化会显示为空字符串。
 *
 * 如果不想用 printf, 可以手动实现整数/浮点数转字符串:
 *   void itoa(int32_t value, char *buf);      // 手写整数转换
 *   void ftoa(float value, char *buf, int dec); // 手写浮点转换
 */

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

/*
 * ============================================================
 *  学习笔记: LCD_DrawProgressBar — 进度条实现
 * ============================================================
 *
 * 一个简单的水平进度条, 基于 LCD_FillRect 实现。
 * 已经实现完毕, 不需要修改。
 *
 * 视觉效果:
 *   ┌─────────────────────┐
 *   │███████████████░░░░░░│  ← 75% 进度
 *   └─────────────────────┘
 *
 * 实现原理:
 *   1. 限制 percent 在 0~100 范围
 *   2. 画整个矩形作为背景 (color_bg)
 *   3. 计算填充宽度: fill_w = w * percent / 100
 *   4. 画前景矩形 (color_fg) 覆盖左侧 fill_w 部分
 *
 * 扩展想法:
 *   - 可以添加边框
 *   - 可以添加百分比文字 (调用 DrawInt)
 *   - 可以改为垂直进度条
 *   - 可以添加渐变色效果
 */
void LCD_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         uint8_t percent, uint16_t color_fg, uint16_t color_bg)
{
    if (percent > 100U) percent = 100U;
    LCD_FillRect(x, y, x + w - 1, y + h - 1, color_bg);
    uint16_t fill_w = (uint16_t)((uint32_t)w * percent / 100U);
    if (fill_w > 0U)
        LCD_FillRect(x, y, x + fill_w - 1, y + h - 1, color_fg);
}
