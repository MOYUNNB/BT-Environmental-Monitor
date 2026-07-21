/**
 * @file    key.h
 * @brief   按键扫描 (3 按键, 10ms 周期)
 * @note    按键消抖: 状态机多级采样 (连续 3 次一致才确认)
 *          在 KEY_Scan() 中实现, 由 StartKeyScan 任务每 10ms 调用
 */
#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 按键 ID */
typedef enum {
    KEY_NONE = 0,
    KEY_1,          /* PA0 */
    KEY_2,          /* PE8 */
    KEY_3,          /* PC13 (板载) */
} KeyID_t;

/*
 * 按键消抖 — 状态机:
 *   每次采样比较当前电平和稳定电平:
 *     相同 → count++; count≥3 则确认状态变化
 *     不同 → count=0, 记录新采样值
 *   KEY_GetPressed 读后自动清除 (防重复触发)
 */
void KEY_Init(void);              /* 初始化 GPIO, 初始状态设为释放 */
void KEY_Scan(void);              /* 10ms 周期: 采样 → 消抖 → 记录状态变化 */
KeyID_t KEY_GetPressed(void);     /* 读已按下的键 (读后自动清除, 防重复触发) */

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
