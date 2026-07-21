/**
 * @file    lcd_page.c
 * @brief   三个 LCD 显示页面 — 全屏布局, 参考嘉立创 st7789_demo() 的多字号+多彩色风格
 * @note    使用 scale=1/2/3 混排填充 240×320 屏幕
 */
#include "lcd_page.h"
#include "lcd.h"
#include "app_conf.h"
#include <stdio.h>
#include <string.h>

PageID_t g_current_page = PAGE_DATA;
static PageID_t s_last_page = PAGE_DATA;

/* ---- 辅助: 温度颜色 ---- */
static uint16_t temp_color(float t)
{
    if (t < 10.0f) return LCD_COLOR_BLUE;
    if (t < 20.0f) return LCD_COLOR_CYAN;
    if (t < 30.0f) return LCD_COLOR_GREEN;
    if (t < 40.0f) return LCD_COLOR_YELLOW;
    return LCD_COLOR_RED;
}

/* ---- 辅助: 水平分隔线 ---- */
static void draw_sep(uint16_t y, uint16_t color)
{
    LCD_FillRect(4, y, LCD_WIDTH - 5, y + 1, color);
}

/* ---- 辅助: 状态栏 (底部) ---- */
static void draw_status_bar(const SensorData_t *data)
{
    uint16_t y = LCD_HEIGHT - 14;
    LCD_FillRect(0, y, LCD_WIDTH - 1, LCD_HEIGHT - 1, LCD_COLOR_DARKBLUE);

    char buf[64];
    snprintf(buf, sizeof(buf), "BT:OK  SD:OK  LOG:#%lu",
             (unsigned long)data->seq_num);
    LCD_DrawString(2, y + 2, buf, LCD_COLOR_WHITE, LCD_COLOR_DARKBLUE, 1);
}

/* ---- 辅助: 传感器状态指示灯 (4px 方块) ---- */
static void draw_sensor_led(uint16_t x, uint16_t y, uint8_t ok)
{
    LCD_FillRect(x, y, x + 5, y + 5, ok ? LCD_COLOR_GREEN : LCD_COLOR_RED);
}

/*=================================================================
 *  页面 1: 数据页 (全屏布局)
 *=================================================================*/
static void page_data_draw(const SensorData_t *data)
{
    char buf[32];
    uint16_t y = 0;

    /* ── 标题栏 (scale=2, 蓝底白字) ── */
    LCD_FillRect(0, 0, LCD_WIDTH - 1, 20, LCD_COLOR_BLUE);
    LCD_DrawString(4, 3, "ENV MONITOR", LCD_COLOR_WHITE, LCD_COLOR_BLUE, 2);

    /* ── RTC 时间 ── */
    y = 24;
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
             (unsigned)data->timestamp.year, (unsigned)data->timestamp.month,
             (unsigned)data->timestamp.day,
             (unsigned)data->timestamp.hour, (unsigned)data->timestamp.minute,
             (unsigned)data->timestamp.second);
    LCD_DrawString(4, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);

    draw_sep(44, LCD_COLOR_GRAY);

    /* ── 温湿度 (AHT20) ── */
    y = 50;
    LCD_DrawString(4, y, "TEMP", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    LCD_DrawString(80, y, "HUMI", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    draw_sensor_led(40, y + 3, !!(data->sensors_ok & SENSOR_OK_AHT20));

    y = 62;
    uint16_t tc = temp_color(data->env.temperature);
    snprintf(buf, sizeof(buf), "%.1f C", (double)data->env.temperature);
    LCD_DrawString(4, y, buf, tc, LCD_COLOR_BLACK, 3);

    snprintf(buf, sizeof(buf), "%.1f %%", (double)data->env.humidity);
    LCD_DrawString(80, y, buf, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 3);

    /* ── 温度进度条 ── */
    {
        float pct = (data->env.temperature - 0.0f) / 50.0f * 100.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        LCD_DrawProgressBar(4, 100, LCD_WIDTH - 8, 8, (uint8_t)pct, tc, LCD_COLOR_GRAY);
    }

    draw_sep(116, LCD_COLOR_GRAY);

    /* ── 电源 (INA226) ── */
    y = 122;
    LCD_DrawString(4, y, "POWER", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    draw_sensor_led(48, y + 3, !!(data->sensors_ok & SENSOR_OK_INA226));

    y = 134;
    snprintf(buf, sizeof(buf), "%.2f V", (double)data->power.bus_voltage);
    LCD_DrawString(4, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);

    snprintf(buf, sizeof(buf), "%.3f A", (double)data->power.current);
    LCD_DrawString(120, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);

    y = 156;
    snprintf(buf, sizeof(buf), "%.2f W", (double)data->power.power);
    LCD_DrawString(4, y, buf, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 3);

    draw_sep(188, LCD_COLOR_GRAY);

    /* ── IMU 简明 (ICM42688) ── */
    y = 194;
    LCD_DrawString(4, y, "ACCEL", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    LCD_DrawString(120, y, "GYRO", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    draw_sensor_led(48, y + 3, !!(data->sensors_ok & SENSOR_OK_ICM42688));

    y = 208;
    snprintf(buf, sizeof(buf), "X:%.1f", (double)data->imu.accel_x);
    LCD_DrawString(4, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Y:%.1f", (double)data->imu.accel_y);
    LCD_DrawString(60, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Z:%.1f", (double)data->imu.accel_z);
    LCD_DrawString(116, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    y = 224;
    snprintf(buf, sizeof(buf), "X:%.1f", (double)data->imu.gyro_x);
    LCD_DrawString(120, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Y:%.1f", (double)data->imu.gyro_y);
    LCD_DrawString(176, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Z:%.1f", (double)data->imu.gyro_z);
    LCD_DrawString(120, y + 14, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    /* ── 底部空白区: 温度柱状图 ── */
    {
        float t = data->env.temperature;
        int16_t bar_h = (int16_t)(t * 4.0f);
        if (bar_h > 50) bar_h = 50;
        if (bar_h < 0) bar_h = 0;
        LCD_FillRect(4, 292 - (uint16_t)bar_h, 16, 291, tc);
        LCD_DrawString(24, 275, "T", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    }

    draw_status_bar(data);
}

static void page_data_update(const SensorData_t *data)
{
    page_data_draw(data);
}

/*=================================================================
 *  页面 2: IMU 图表页
 *=================================================================*/
static void page_imu_draw(const SensorData_t *data)
{
    char buf[32];
    uint16_t y;

    /* ── 标题栏 ── */
    LCD_FillRect(0, 0, LCD_WIDTH - 1, 20, LCD_COLOR_BLUE);
    LCD_DrawString(4, 3, "IMU DATA", LCD_COLOR_WHITE, LCD_COLOR_BLUE, 2);

    /* ── 加速度 ── */
    y = 26;
    LCD_DrawString(4, y, "Accelerometer (g)", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);

    y = 40;
    float ax = data->imu.accel_x, ay = data->imu.accel_y, az = data->imu.accel_z;

    snprintf(buf, sizeof(buf), "X: %+.2f", (double)ax);
    LCD_DrawString(4, y, buf, LCD_COLOR_RED, LCD_COLOR_BLACK, 2);
    snprintf(buf, sizeof(buf), "Y: %+.2f", (double)ay);
    LCD_DrawString(4, y + 22, buf, LCD_COLOR_GREEN, LCD_COLOR_BLACK, 2);
    snprintf(buf, sizeof(buf), "Z: %+.2f", (double)az);
    LCD_DrawString(4, y + 44, buf, LCD_COLOR_BLUE, LCD_COLOR_BLACK, 2);

    /* ── 加速度柱状图 ── */
    {
        uint16_t bar_y = y + 2;
        uint16_t bar_base = bar_y + 60;
        int16_t bx = (int16_t)(ax * 30.0f);
        int16_t by_ = (int16_t)(ay * 30.0f);
        int16_t bz = (int16_t)(az * 30.0f);
        if (bx > 55) { bx = 55; } else if (bx < -55) { bx = -55; }
        if (by_ > 55) { by_ = 55; } else if (by_ < -55) { by_ = -55; }
        if (bz > 55) { bz = 55; } else if (bz < -55) { bz = -55; }

        /* 零线 */
        LCD_FillRect(160, bar_base, 235, bar_base + 1, LCD_COLOR_GRAY);
        /* X bar */
        if (bx >= 0) LCD_FillRect(172, bar_base - (uint16_t)bx, 188, bar_base - 1, LCD_COLOR_RED);
        else         LCD_FillRect(172, bar_base, 188, bar_base + (uint16_t)(-bx) - 1, LCD_COLOR_RED);
        /* Y bar */
        if (by_ >= 0) LCD_FillRect(192, bar_base - (uint16_t)by_, 208, bar_base - 1, LCD_COLOR_GREEN);
        else          LCD_FillRect(192, bar_base, 208, bar_base + (uint16_t)(-by_) - 1, LCD_COLOR_GREEN);
        /* Z bar */
        if (bz >= 0) LCD_FillRect(212, bar_base - (uint16_t)bz, 228, bar_base - 1, LCD_COLOR_BLUE);
        else         LCD_FillRect(212, bar_base, 228, bar_base + (uint16_t)(-bz) - 1, LCD_COLOR_BLUE);

        LCD_DrawString(172, bar_base + 4, "X", LCD_COLOR_RED,   LCD_COLOR_BLACK, 1);
        LCD_DrawString(192, bar_base + 4, "Y", LCD_COLOR_GREEN, LCD_COLOR_BLACK, 1);
        LCD_DrawString(212, bar_base + 4, "Z", LCD_COLOR_BLUE,  LCD_COLOR_BLACK, 1);
    }

    draw_sep(128, LCD_COLOR_GRAY);

    /* ── 陀螺仪 ── */
    y = 134;
    LCD_DrawString(4, y, "Gyroscope (dps)", LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);

    y = 148;
    snprintf(buf, sizeof(buf), "X: %+.1f", (double)data->imu.gyro_x);
    LCD_DrawString(4, y, buf, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 2);
    snprintf(buf, sizeof(buf), "Y: %+.1f", (double)data->imu.gyro_y);
    LCD_DrawString(4, y + 22, buf, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 2);
    snprintf(buf, sizeof(buf), "Z: %+.1f", (double)data->imu.gyro_z);
    LCD_DrawString(4, y + 44, buf, LCD_COLOR_MAGENTA, LCD_COLOR_BLACK, 2);

    /* ── 陀螺仪柱状图 ── */
    {
        uint16_t gy = y + 2;
        uint16_t gbase = gy + 60;
        int16_t gx = (int16_t)(data->imu.gyro_x / 4.0f);
        int16_t gy_v = (int16_t)(data->imu.gyro_y / 4.0f);
        int16_t gz = (int16_t)(data->imu.gyro_z / 4.0f);
        if (gx > 55) { gx = 55; } else if (gx < -55) { gx = -55; }
        if (gy_v > 55) { gy_v = 55; } else if (gy_v < -55) { gy_v = -55; }
        if (gz > 55) { gz = 55; } else if (gz < -55) { gz = -55; }

        LCD_FillRect(160, gbase, 235, gbase + 1, LCD_COLOR_GRAY);
        if (gx >= 0) LCD_FillRect(172, gbase - (uint16_t)gx, 188, gbase - 1, LCD_COLOR_YELLOW);
        else         LCD_FillRect(172, gbase, 188, gbase + (uint16_t)(-gx) - 1, LCD_COLOR_YELLOW);
        if (gy_v >= 0) LCD_FillRect(192, gbase - (uint16_t)gy_v, 208, gbase - 1, LCD_COLOR_CYAN);
        else           LCD_FillRect(192, gbase, 208, gbase + (uint16_t)(-gy_v) - 1, LCD_COLOR_CYAN);
        if (gz >= 0) LCD_FillRect(212, gbase - (uint16_t)gz, 228, gbase - 1, LCD_COLOR_MAGENTA);
        else         LCD_FillRect(212, gbase, 228, gbase + (uint16_t)(-gz) - 1, LCD_COLOR_MAGENTA);

        LCD_DrawString(172, gbase + 4, "X", LCD_COLOR_YELLOW,  LCD_COLOR_BLACK, 1);
        LCD_DrawString(192, gbase + 4, "Y", LCD_COLOR_CYAN,    LCD_COLOR_BLACK, 1);
        LCD_DrawString(212, gbase + 4, "Z", LCD_COLOR_MAGENTA, LCD_COLOR_BLACK, 1);
    }

    /* ── IMU 温度 ── */
    {
        uint16_t ty = 256;
        snprintf(buf, sizeof(buf), "IMU Temp: %.1f C", (double)data->imu.temp_c);
        LCD_DrawString(4, ty, buf, LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
    }

    draw_status_bar(data);
}

static void page_imu_update(const SensorData_t *data)
{
    page_imu_draw(data);
}

/*=================================================================
 *  页面 3: 状态页
 *=================================================================*/
static void page_status_draw(const SensorData_t *data)
{
    char buf[64];
    uint16_t y;

    LCD_FillRect(0, 0, LCD_WIDTH - 1, 20, LCD_COLOR_BLUE);
    LCD_DrawString(4, 3, "STATUS", LCD_COLOR_WHITE, LCD_COLOR_BLUE, 2);

    /* ── 序列号 ── */
    y = 30;
    snprintf(buf, sizeof(buf), "Seq #%lu", (unsigned long)data->seq_num);
    LCD_DrawString(4, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2);

    draw_sep(52, LCD_COLOR_GRAY);

    /* ── 传感器状态 (大字+色块) ── */
    struct { const char *name; uint8_t mask; uint16_t y; } sensors[] = {
        {"AHT20",    SENSOR_OK_AHT20,    64},
        {"INA226",   SENSOR_OK_INA226,   108},
        {"ICM42688", SENSOR_OK_ICM42688, 152},
        {"SD3078",   SENSOR_OK_SD3078,   196},
    };

    for (int i = 0; i < 4; i++) {
        uint8_t ok = !!(data->sensors_ok & sensors[i].mask);
        uint16_t sy = sensors[i].y;
        uint16_t sc = ok ? LCD_COLOR_GREEN : LCD_COLOR_RED;

        /* 状态色块 */
        LCD_FillRect(8, sy, 56, sy + 24, sc);

        /* 传感器名称 (白色/黑色前景, 取决于底色) */
        LCD_DrawString(12, sy + 6, sensors[i].name, LCD_COLOR_BLACK, sc, 2);

        /* 状态文字 */
        LCD_DrawString(68, sy + 6, ok ? "OK" : "ERROR", sc, LCD_COLOR_BLACK, 2);
    }

    /* ── 系统时间 ── */
    y = 248;
    snprintf(buf, sizeof(buf), "RTC: %02u:%02u:%02u",
             (unsigned)data->timestamp.hour,
             (unsigned)data->timestamp.minute,
             (unsigned)data->timestamp.second);
    LCD_DrawString(4, y, buf, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 2);

    /* ── 温度概览 ── */
    y = 274;
    uint16_t tc = temp_color(data->env.temperature);
    snprintf(buf, sizeof(buf), "Temp: %.1f C", (double)data->env.temperature);
    LCD_DrawString(4, y, buf, tc, LCD_COLOR_BLACK, 2);

    draw_status_bar(data);
}

static void page_status_update(const SensorData_t *data)
{
    page_status_draw(data);
}

/* ---- 公开接口 ---- */

void LCD_Page_Init(void)
{
    g_current_page = PAGE_DATA;
    s_last_page = PAGE_DATA;
}

void LCD_Page_Switch(PageID_t page)
{
    if (page >= PAGE_COUNT) return;
    g_current_page = page;
    s_last_page = page;
}

/*
 * LCD_Page_Refresh — 页面刷新入口, 每 200ms 由 StartLCDUpdate 任务调用。
 *
 * _draw vs _update 分工:
 *   _draw:   全屏绘制整个页面, 清除所有旧内容。仅在页面切换时执行一次。
 *   _update: 只更新变化的数值区域 (时间、温湿度等), 保留静态元素。
 *            目前 _update 调用 _draw 做了全屏重绘, 后续可优化为增量刷新。
 *
 * 竞态处理:
 *   KeyScan 任务 (优先级 12) 高于 LCDUpdate (优先级 10), 可能在绘制途中
 *   抢占并修改 g_current_page。这里在函数入口拍摄页号快照,
 *   确保一次 Refresh 始终绘制同一个页面, 不会中途串页。
 */
void LCD_Page_Refresh(const SensorData_t *data)
{
    if (data == NULL) return;

    PageID_t page = g_current_page;             /* 快照: 锁定当前页号 */

    if (page != s_last_page) {                  /* 页号变了 → 全屏重绘 */
        s_last_page = page;
        switch (page) {
            case PAGE_DATA:      page_data_draw(data);   break;
            case PAGE_IMU_CHART: page_imu_draw(data);    break;
            case PAGE_STATUS:    page_status_draw(data);  break;
            default: break;
        }
    } else {                                     /* 页号没变 → 数值更新 */
        switch (page) {
            case PAGE_DATA:      page_data_update(data);   break;
            case PAGE_IMU_CHART: page_imu_update(data);    break;
            case PAGE_STATUS:    page_status_update(data);  break;
            default: break;
        }
    }
}

void LCD_Page_Redraw(const SensorData_t *data)
{
    s_last_page = PAGE_COUNT;
    LCD_Page_Refresh(data);
}
