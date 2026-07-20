/**
 * @file    app_conf.h
 * @brief   应用层配置宏
 * @note    集中管理采样周期、阈值、引脚映射等参数
 */
#ifndef __APP_CONF_H
#define __APP_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 采样周期 (ms) ---- */
#define CFG_SENSOR_PERIOD_MS        100U     /* 传感器采集周期 */
#define CFG_LCD_PERIOD_MS           200U     /* LCD 刷新周期 */
#define CFG_TF_LOG_PERIOD_MS        1000U    /* TF 卡记录周期 */
#define CFG_BT_SEND_PERIOD_MS       1000U    /* 蓝牙推送周期 */
#define CFG_KEY_SCAN_PERIOD_MS      10U      /* 按键扫描周期 */
#define CFG_WS2812_PERIOD_MS        1000U    /* WS2812 更新周期 */

/* ---- LCD 页面 ---- */
#define CFG_PAGE_DATA               0U       /* 数据页 */
#define CFG_PAGE_IMU                1U       /* IMU 图表页 */
#define CFG_PAGE_STATUS             2U       /* 状态页 */
#define CFG_PAGE_COUNT              3U       /* 页面总数 */

/* ---- 温度 LED 颜色阈值 ---- */
#define CFG_TEMP_COLD               10.0f    /* <10°C 蓝色 */
#define CFG_TEMP_COOL               20.0f    /* 10~20°C 青色 */
#define CFG_TEMP_WARM               30.0f    /* 20~30°C 绿色 */
#define CFG_TEMP_HOT                40.0f    /* 30~40°C 黄色, >40°C 红色 */

/* ---- 传感器有效范围 ---- */
#define CFG_TEMP_MIN                -40.0f
#define CFG_TEMP_MAX                85.0f
#define CFG_HUMI_MIN                0.0f
#define CFG_HUMI_MAX                100.0f

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONF_H */
