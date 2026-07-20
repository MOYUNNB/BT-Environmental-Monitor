/**
 * @file    sensor_data.c
 * @brief   全局传感器数据管理与同步
 * @note    实现 SensorData_t 的读写操作, 互斥信号量保护 + 队列发布
 */
#include "sensor_data.h"

/* 全局传感器数据快照 (互斥锁保护) */
static SensorData_t s_global_data = {0};

void SensorData_Update(const SensorData_t *data)
{
    if (data == NULL) return;

    /* 获取互斥锁 */
    if (xSemaphore_SensorDataHandle != NULL) {
        osMutexAcquire(xSemaphore_SensorDataHandle, osWaitForever);
    }

    s_global_data = *data;

    /* 释放互斥锁 */
    if (xSemaphore_SensorDataHandle != NULL) {
        osMutexRelease(xSemaphore_SensorDataHandle);
    }

    /* 同时推入队列, 供等待的任务消费 */
    if (xQueue_SensorDataHandle != NULL) {
        osMessageQueuePut(xQueue_SensorDataHandle, data, 0U, 0U);
    }
}

void SensorData_Read(SensorData_t *data)
{
    if (data == NULL) return;

    if (xSemaphore_SensorDataHandle != NULL) {
        osMutexAcquire(xSemaphore_SensorDataHandle, osWaitForever);
    }

    *data = s_global_data;

    if (xSemaphore_SensorDataHandle != NULL) {
        osMutexRelease(xSemaphore_SensorDataHandle);
    }
}
