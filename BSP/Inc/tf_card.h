/**
 * @file    tf_card.h
 * @brief   TF 卡 FATFS 文件系统 + CSV 日志 (SDIO 4-bit, FATFS, 按天分割)
 * @note    CubeMX 已生成 diskio, 应用层需补充 Init/LogSensor/Flush/Deinit
 */
#ifndef __TF_CARD_H
#define __TF_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "fatfs.h"

/*
 * CSV 表头 — 首行, 与数据格式的列数必须严格一致
 * 共 13 列: 时间戳 + 温湿 + 电压电流功率 + 加速度3 + 陀螺仪3
 */
#define TF_CSV_HEADER \
    "Timestamp,Temp(C),Humidity(%),Voltage(V),Current(A),Power(W)," \
    "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ\r\n"

/*
 * 日志策略:
 *   初始化时 f_open 打开当日文件 (追加模式)
 *   Task_TF_Log 每秒调 f_sync 刷缓冲区 (防掉电丢 <1s 数据)
 *   文件按天分割: 2026-07-18.csv 等, 避免单文件过大
 *   优先级最低: FATFS 写操作慢 (10~50ms), 不阻塞实时任务
 */
bool TF_Init(void);                                                         /* 挂载 FATFS, 创建当日 CSV, 写表头 */
bool TF_LogSensor(const char *timestamp,                                    /* 写一行传感器数据 (f_write → f_sync) */
                  float temp, float humi,
                  float volt, float curr, float pwr,
                  const float accel_g[3], const float gyro_dps[3]);
void TF_Flush(void);                                                        /* 强制刷缓冲到 SD 卡 (每秒调一次防掉电) */
void TF_Deinit(void);                                                       /* 关闭文件 + 卸载 FATFS */

#ifdef __cplusplus
}
#endif

#endif /* __TF_CARD_H */
