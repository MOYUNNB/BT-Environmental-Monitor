/**
 * @file    sensor_data.h
 * @brief   全局传感器数据结构 + 任务间通信 (队列 + 互斥信号量)
 * @note    定义 SensorData_t 结构体, 以及 xQueue_SensorData / xSemaphore_SensorData
 *          所有 FreeRTOS 任务通过此结构体交换传感器数据
 */
#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* 传感器状态位掩码 */
#define SENSOR_OK_AHT20         (1U << 0)
#define SENSOR_OK_INA226        (1U << 1)
#define SENSOR_OK_ICM42688      (1U << 2)
#define SENSOR_OK_SD3078        (1U << 3)

/* ---- 子数据结构 ---- */

/* RTC 时间 */
typedef struct {
    uint16_t year;      /* 2000~2099 */
    uint8_t  month;     /* 1~12 */
    uint8_t  day;       /* 1~31 */
    uint8_t  hour;      /* 0~23 */
    uint8_t  minute;    /* 0~59 */
    uint8_t  second;    /* 0~59 */
} SensorTime_t;

/* 环境数据 (AHT20) */
typedef struct {
    float temperature;  /* °C */
    float humidity;     /* %RH */
} EnvData_t;

/* 电源数据 (INA226) */
typedef struct {
    float bus_voltage;  /* V */
    float current;      /* A */
    float power;        /* W */
} PowerData_t;

/* IMU 数据 (ICM42688) */
typedef struct {
    float accel_x, accel_y, accel_z;  /* g */
    float gyro_x,  gyro_y,  gyro_z;   /* °/s */
    float temp_c;                      /* 芯片温度 */
} IMUData_t;

/* ---- 主数据结构 ---- */

/* 完整传感器快照 */
typedef struct {
    SensorTime_t  timestamp;   /* 采样时间戳 (RTC) */
    EnvData_t     env;         /* AHT20 温湿度 */
    PowerData_t   power;       /* INA226 电压电流功率 */
    IMUData_t     imu;         /* ICM42688 加速度/陀螺仪 */
    uint32_t      seq_num;     /* 序列号 */
    uint8_t       sensors_ok;  /* 传感器状态位掩码 */
} SensorData_t;

/* ---- IPC 接口 ---- */

/* 队列句柄 (在 main.c 中定义) */
extern osMessageQueueId_t xQueue_SensorDataHandle;
extern osMutexId_t        xSemaphore_SensorDataHandle;

/**
 * @brief   用互斥锁保护写入全局传感器数据
 * @param   data: 要写入的快照指针
 */
void SensorData_Update(const SensorData_t *data);

/**
 * @brief   用互斥锁保护读取全局传感器数据
 * @param   data: 输出缓冲区指针
 */
void SensorData_Read(SensorData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_DATA_H */
