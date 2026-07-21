/**
 * @file    key.h
 * @brief   按键扫描 + EC11 旋转编码器 (3 按键 + EC11, 10ms 周期)
 * @note    按键消抖: 状态机多级采样 (连续 3 次一致才确认)
 *          EC11: 查表法 (4-bit 索引 16 项, 根据 AB 相位差判正反转)
 *          均在 KEY_Scan() 中实现, 由 StartKeyScan 任务每 10ms 调用
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

/* EC11 方向 */
typedef enum {
    EC11_NONE = 0,
    EC11_CW,        /* 顺时针 */
    EC11_CCW,       /* 逆时针 */
} EC11_Dir_t;

/*
 * 按键消抖 — 状态机:
 *   每次采样比较当前电平和稳定电平:
 *     相同 → count++; count≥3 则确认状态变化
 *     不同 → count=0, 记录新采样值
 *   KEY_GetPressed 读后自动清除 (防重复触发)
 *
 * EC11 — 查表法:
 *   idx = (last_AB << 2) | current_AB
 *   查表 ec11_table[16]: 0=无变化, +1=CW, -1=CCW, 0=非法
 *   非法状态: 两个 bit 同时变化 (采样间隔太长)
 */
void KEY_Init(void);              /* 初始化 GPIO + 定时器, 配好按键引脚 */
void KEY_Scan(void);              /* 10ms 周期: 采样 → 消抖 → 记录状态变化 */
KeyID_t KEY_GetPressed(void);     /* 读已按下的键 (读后自动清除, 防重复触发) */
int8_t EC11_GetDelta(void);       /* 读 EC11 旋转增量: +1=CW, -1=CCW, 0=没动 */

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
