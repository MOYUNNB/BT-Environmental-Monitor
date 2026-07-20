/**
 * @file    lcd_page.h
 * @brief   多页显示框架
 * @note    定义三个显示页面: 数据页、IMU 图表页、状态页, 以及页面切换逻辑
 *          由 EC11 编码器切换页面, KEY1 返回首页
 */
#ifndef __LCD_PAGE_H
#define __LCD_PAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_data.h"
#include <stdint.h>

/* 页面枚举 */
typedef enum {
    PAGE_DATA = 0,      /* 数据页: 温湿度+电压+时间 */
    PAGE_IMU_CHART,     /* IMU 图表页: 加速度/陀螺仪实时曲线 */
    PAGE_STATUS,        /* 状态页: 系统信息 */
    PAGE_COUNT
} PageID_t;

/* 当前页面编号 (由按键/EC11 任务修改, LCD 任务读取) */
extern PageID_t g_current_page;

/**
 * @brief   初始化页面系统
 */
void LCD_Page_Init(void);

/**
 * @brief   切换到指定页面 (全屏重绘)
 */
void LCD_Page_Switch(PageID_t page);

/**
 * @brief   刷新当前页面 (增量更新数据)
 * @param   data: 最新传感器数据
 */
void LCD_Page_Refresh(const SensorData_t *data);

/**
 * @brief   强制重绘当前页面 (全屏)
 * @param   data: 传感器数据
 */
void LCD_Page_Redraw(const SensorData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_PAGE_H */
