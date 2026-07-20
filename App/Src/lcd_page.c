/**
 * @file    lcd_page.c
 * @brief   三个 LCD 显示页面实现
 * @note    数据页 (温湿度+电压+时间)、IMU 图表页 (加速度/陀螺仪)、状态页 (系统信息)
 *          全屏重绘在页面切换时触发, 刷新只更新数值区域
 */
#include "lcd_page.h"
#include "lcd.h"
#include "app_conf.h"
#include <stdio.h>
#include <string.h>

PageID_t g_current_page = PAGE_DATA;
static PageID_t s_last_page = PAGE_DATA;

/* ---- 辅助: 绘制状态栏 (底部) ---- */
static void draw_status_bar(uint16_t y)
{
    LCD_FillRect(0, y, LCD_WIDTH - 1, LCD_HEIGHT - 1, LCD_COLOR_BLACK);
    LCD_DrawString(2, y + 2, "BT:OK  SD:OK  LOG:ON",
                   LCD_COLOR_GRAY, LCD_COLOR_BLACK, 1);
}

/* ---- 页面 1: 数据页 ---- */
static void page_data_draw(const SensorData_t *data)
{
    char buf[32];
    uint16_t y = 0;

    /* 标题栏 */
    LCD_FillRect(0, 0, LCD_WIDTH - 1, 16, LCD_COLOR_BLUE);
    LCD_DrawString(4, 2, "Env Monitor v1.0", LCD_COLOR_WHITE, LCD_COLOR_BLUE, 1);

    y = 22;
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
             (unsigned)data->timestamp.year, (unsigned)data->timestamp.month,
             (unsigned)data->timestamp.day,
             (unsigned)data->timestamp.hour, (unsigned)data->timestamp.minute,
             (unsigned)data->timestamp.second);
    LCD_DrawString(4, y, buf, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 1);
    y += 18;

    /* 环境数据 (AHT20) */
    LCD_DrawString(4, y, "--- AHT20 ---", LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Temp: %.1f C", (double)data->env.temperature);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Humi: %.1f %%", (double)data->env.humidity);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 20;

    /* 电源数据 (INA226) */
    LCD_DrawString(4, y, "--- INA226 ---", LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Volt: %.2f V", (double)data->power.bus_voltage);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Curr: %.3f A", (double)data->power.current);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Pwr:  %.2f W", (double)data->power.power);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 20;

    /* IMU 数据 (ICM42688) */
    LCD_DrawString(4, y, "--- ICM42688 ---", LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Accel: %.2f %.2f %.2f g",
             (double)data->imu.accel_x, (double)data->imu.accel_y, (double)data->imu.accel_z);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    y += 14;
    snprintf(buf, sizeof(buf), "Gyro:  %.1f %.1f %.1f dps",
             (double)data->imu.gyro_x, (double)data->imu.gyro_y, (double)data->imu.gyro_z);
    LCD_DrawString(8, y, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    draw_status_bar(y + 20);
}

static void page_data_update(const SensorData_t *data)
{
    page_data_draw(data);  /* 简单方案: 直接全屏重绘 */
}

/* ---- 页面 2: IMU 图表页 ---- */
static void page_imu_draw(const SensorData_t *data)
{
    char buf[32];

    LCD_Clear(LCD_COLOR_BLACK);
    LCD_DrawString(4, 2, "IMU Data", LCD_COLOR_BLUE, LCD_COLOR_BLACK, 2);

    snprintf(buf, sizeof(buf), "Accel X: %.2f g", (double)data->imu.accel_x);
    LCD_DrawString(4, 30, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Accel Y: %.2f g", (double)data->imu.accel_y);
    LCD_DrawString(4, 48, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Accel Z: %.2f g", (double)data->imu.accel_z);
    LCD_DrawString(4, 66, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    /* 垂直柱状图: Accel Z ±3g 范围 */
    int16_t bar_h = (int16_t)(data->imu.accel_z * 30.0f);
    if (bar_h > 140) bar_h = 140;
    if (bar_h < -140) bar_h = -140;

    uint16_t bar_x = 160;
    uint16_t bar_center = 200;
    if (bar_h >= 0) {
        LCD_FillRect(bar_x, bar_center - (uint16_t)bar_h, bar_x + 40, bar_center - 1, LCD_COLOR_GREEN);
    } else {
        LCD_FillRect(bar_x, bar_center, bar_x + 40, bar_center + (uint16_t)(-bar_h) - 1, LCD_COLOR_RED);
    }

    snprintf(buf, sizeof(buf), "Gyro X: %.1f", (double)data->imu.gyro_x);
    LCD_DrawString(4, 100, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Gyro Y: %.1f", (double)data->imu.gyro_y);
    LCD_DrawString(4, 118, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    snprintf(buf, sizeof(buf), "Gyro Z: %.1f", (double)data->imu.gyro_z);
    LCD_DrawString(4, 136, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    draw_status_bar(300);
}

static void page_imu_update(const SensorData_t *data)
{
    page_imu_draw(data);
}

/* ---- 页面 3: 状态页 ---- */
static void page_status_draw(const SensorData_t *data)
{
    char buf[64];

    LCD_Clear(LCD_COLOR_BLACK);
    LCD_DrawString(4, 2, "System Status", LCD_COLOR_BLUE, LCD_COLOR_BLACK, 2);

    snprintf(buf, sizeof(buf), "Seq: %lu", (unsigned long)data->seq_num);
    LCD_DrawString(4, 30, buf, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);

    snprintf(buf, sizeof(buf), "AHT20:  %s", (data->sensors_ok & SENSOR_OK_AHT20) ? "OK" : "ERR");
    LCD_DrawString(4, 48, buf, (data->sensors_ok & SENSOR_OK_AHT20) ? LCD_COLOR_GREEN : LCD_COLOR_RED, LCD_COLOR_BLACK, 1);

    snprintf(buf, sizeof(buf), "INA226: %s", (data->sensors_ok & SENSOR_OK_INA226) ? "OK" : "ERR");
    LCD_DrawString(4, 66, buf, (data->sensors_ok & SENSOR_OK_INA226) ? LCD_COLOR_GREEN : LCD_COLOR_RED, LCD_COLOR_BLACK, 1);

    snprintf(buf, sizeof(buf), "ICM42688: %s", (data->sensors_ok & SENSOR_OK_ICM42688) ? "OK" : "ERR");
    LCD_DrawString(4, 84, buf, (data->sensors_ok & SENSOR_OK_ICM42688) ? LCD_COLOR_GREEN : LCD_COLOR_RED, LCD_COLOR_BLACK, 1);

    snprintf(buf, sizeof(buf), "SD3078: %s", (data->sensors_ok & SENSOR_OK_SD3078) ? "OK" : "ERR");
    LCD_DrawString(4, 102, buf, (data->sensors_ok & SENSOR_OK_SD3078) ? LCD_COLOR_GREEN : LCD_COLOR_RED, LCD_COLOR_BLACK, 1);

    draw_status_bar(300);
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

void LCD_Page_Refresh(const SensorData_t *data)
{
    if (data == NULL) return;

    if (g_current_page != s_last_page) {
        /* 页面切换: 全屏重绘 */
        s_last_page = g_current_page;
        switch (g_current_page) {
            case PAGE_DATA:      page_data_draw(data);   break;
            case PAGE_IMU_CHART: page_imu_draw(data);    break;
            case PAGE_STATUS:    page_status_draw(data);  break;
            default: break;
        }
    } else {
        /* 同页面: 增量更新 */
        switch (g_current_page) {
            case PAGE_DATA:      page_data_update(data);   break;
            case PAGE_IMU_CHART: page_imu_update(data);    break;
            case PAGE_STATUS:    page_status_update(data);  break;
            default: break;
        }
    }
}

void LCD_Page_Redraw(const SensorData_t *data)
{
    s_last_page = PAGE_COUNT;  /* 强制全屏重绘 */
    LCD_Page_Refresh(data);
}
