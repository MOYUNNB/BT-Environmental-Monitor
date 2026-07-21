/**
 * @file    key.c
 * @brief   按键扫描驱动实现
 * @note    由 Task_Key_Scan (10ms 周期) 驱动。
 *
 * 按键消抖: 3 次连续采样一致才确认按下/释放。
 *
 * 参考:
 *   GPIO EXTI: fdb-master/0_example/GPIO/gpio-key-exti-project
 */
#include "key.h"

/*
 * ============================================================
 *  学习笔记: 引脚定义
 * ============================================================
 *
 * 3 个按键:
 *
 *   按键 1 (KEY1): PA0
 *     普通 GPIO 输入, 外部上拉 (或内部上拉)。
 *     不受任何外设复用影响 (AF=0)。
 *     按下时: GPIO 读取为 0 (低电平)
 *     释放时: GPIO 读取为 1 (高电平)
 *
 *   按键 2 (KEY2): PE8
 *     同 KEY1, 普通 GPIO 输入。
 *
 *   按键 3 (KEY3): PC13
 *     这是板载按键 (通常接在 PC13),
 *     注意: PC13 也是 RTC 校准输出和 TAMPER 引脚,
 *     如果用作普通 GPIO, 需要在 CubeMX 中禁用其他功能。
 *
 * 内部上拉 vs 外部上拉:
 *   内部上拉: 约 30~50kΩ (STM32F407 典型值)
 *             优点: 省 PCB 面积, 少一个电阻
 *             缺点: 电阻较大, 抗干扰能力稍弱
 *   外部上拉: 4.7~10kΩ
 *             优点: 更强的抗干扰, 更快的上升沿
 *             缺点: 需要额外电阻
 *
 * 注意: 按键引脚在 CubeMX 中配置为:
 *   GPIO Mode: Input mode
 *   GPIO Pull-up/Pull-down: Pull-up (如果无外部上拉)
 *   不要开 GPIO 的 Alternate Function!
 */

/* 引脚定义 (与 main.h 一致) */
#define KEY1_PORT       GPIOA
#define KEY1_PIN        GPIO_PIN_0
#define KEY2_PORT       GPIOE
#define KEY2_PIN        GPIO_PIN_8
#define KEY3_PORT       GPIOC
#define KEY3_PIN        GPIO_PIN_13

/*
 * 消抖参数说明:
 *   采样周期 = 10ms (由 KEY_Scan 的调用周期决定)
 *   采样次数 = 3
 *   总消抖时间 = 10ms × 3 = 30ms
 *
 *   这个时间大于典型按键的抖动时间 (5~20ms),
 *   所以可以有效消抖。
 *
 *   如果按键质量很差 (抖动 > 30ms),
 *   可以增加 KEY_SAMPLE_CNT 到 4 或 5。
 *   但注意: 增加采样次数会增加按键响应延迟。
 *   3 次采样 + 10ms 周期 = 30ms 延迟, 用户基本感觉不到。
 *   如果延迟 > 100ms, 用户会感觉"按键不跟手"。
 */

/* 消抖计数 */
#define KEY_DEBOUNCE_MS 20U     /* 总消抖时间 (多次采样) */
#define KEY_SAMPLE_CNT  3U      /* 连续一致次数 */

/*
 * ============================================================
 *  学习笔记: 按键状态机数据结构
 * ============================================================
 *
 * typedef struct {
 *     uint8_t  raw;       // 当前原始电平 (每次采样更新)
 *     uint8_t  stable;    // 稳定状态 (0=按下, 1=释放)
 *     uint8_t  count;     // 连续一致计数
 * } KeyState_t;
 *
 * 每个按键有一个 KeyState_t 实例:
 *   s_keys[0] → KEY1 (PA0)
 *   s_keys[1] → KEY2 (PE8)
 *   s_keys[2] → KEY3 (PC13)
 *
 * 状态机工作过程:
 *
 *   初始: raw=1, stable=1, count=0 (假设初始为释放)
 *
 *   第 1 次采样 (10ms):
 *     读 GPIO = 0 (用户刚好按下, 正在抖动)
 *     比较: raw=1 ≠ 0 → count=0, raw=0
 *     状态: raw=0, stable=1, count=0
 *
 *   第 2 次采样 (20ms):
 *     读 GPIO = 0 (抖动结束, 稳定在低电平)
 *     比较: raw=0 == 0 → count=1 (一致, 计数+1)
 *     状态: raw=0, stable=1, count=1
 *
 *   第 3 次采样 (30ms):
 *     读 GPIO = 0
 *     比较: raw=0 == 0 → count=2
 *     状态: raw=0, stable=1, count=2
 *
 *   第 4 次采样 (40ms): (这里假设 count≥3 时触发)
 *     读 GPIO = 0
 *     比较: raw=0 == 0 → count=3
 *     count >= 3 → 更新 stable=0, 记录按键按下
 *     状态: raw=0, stable=0, count=0 (或保持 3)
 *
 *   第 5 次采样 (50ms):
 *     读 GPIO = 0 (用户一直按着)
 *     比较: raw=0 == stable=0 → count=1
 *     状态: raw=0, stable=0, count=1
 *     stale=0 是按下状态, 但 count 还没到 3, 不触发
 *
 *   第 6~N 次: 一直按着, count 一直在增加
 *   但 stable=0 已经确认按下, 所以不会再触发第二次。
 *
 * 释放过程 (用户松手):
 *   读 GPIO = 1 → 状态机类似从稳定 0 切换到稳定 1
 *   count ≥ 3 后 stable 更新为 1 (释放状态)
 *   释放时不需要记录到 s_pressed (只记录按下事件)
 */

/* 按键状态机 */
typedef struct {
    uint8_t  raw;           /* 当前原始电平 */
    uint8_t  stable;        /* 稳定状态 (0=按下, 1=释放) */
    uint8_t  count;         /* 连续一致计数 */
} KeyState_t;

static KeyState_t s_keys[3];
static KeyID_t    s_pressed = KEY_NONE;

/* ---- 读引脚电平 ---- */

/*
 * 读取三个按键的原始电平:
 *   HAL_GPIO_ReadPin 返回 GPIO_PinState:
 *     GPIO_PIN_RESET = 0 (低电平, 通常=按键按下)
 *     GPIO_PIN_SET   = 1 (高电平, 通常=按键释放)
 *
 * 注意: 这取决于按键的硬件接法:
 *   图中是"按下=低电平" (按键接 GND, 上拉电阻到 VCC)
 *   如果是"按下=高电平" (按键接 VCC, 下拉电阻到 GND),
 *   则返回值的含义相反。
 *
 * 本驱动假设: 按下 = 0, 释放 = 1 (标准的上拉输入接法)
 */
static uint8_t key_read_raw(uint8_t idx)
{
    switch (idx) {
        case 0: return (uint8_t)HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
        case 1: return (uint8_t)HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);
        case 2: return (uint8_t)HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN);
        default: return 1U;
    }
}

/* ---- 对外接口 ---- */

/*
 * ============================================================
 *  学习笔记: KEY_Init 实现指南
 * ============================================================
 *
 * 初始化所有状态变量到"释放"状态:
 *
 *   for (i = 0; i < 3; i++) {
 *       s_keys[i].raw    = 1;  // 假设初始为高电平 (释放)
 *       s_keys[i].stable = 1;  // 初始稳定状态为释放
 *       s_keys[i].count  = 0;  // 一致计数器清零
 *   }
 *   s_pressed = KEY_NONE;       // 清除按下的按键记录
 *
 * 为什么不直接读 GPIO 来初始化 raw/stable?
 *   因为首次调用 KEY_Init 时, GPIO 可能已经初始化完毕,
 *   可以直接读。但更保险的做法是默认设为 1 (释放)。
 *   如果上电时按键刚好被按下, 下一次 KEY_Scan 会
 *   检测到这个变化并记录。
 *
 * 已实现, 无需修改。
 */
void KEY_Init(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        s_keys[i].raw    = 1U;
        s_keys[i].stable = 1U;
        s_keys[i].count  = 0U;
    }
    s_pressed = KEY_NONE;
}

/*
 * ============================================================
 *  学习笔记: KEY_Scan 实现指南 (这是最关键的函数!)
 * ============================================================
 *
 * KEY_Scan 由 StartKeyScan 任务每 10ms 调用一次。
 * 完成按键消抖状态机。
 *
 * 按键消抖部分伪代码:
 *
 *   for (i = 0; i < 3; i++) {
 *       uint8_t level = key_read_raw(i);       // 读 GPIO
 *       if (level == s_keys[i].stable) {        // 和稳定状态一致?
 *           s_keys[i].count++;
 *           if (s_keys[i].count >= KEY_SAMPLE_CNT) {
 *               // 已经连续 N 次一致, 确认状态稳定
 *               if (s_keys[i].stable == 0) {
 *                   // 稳定为按下! 记录按键
 *                   s_pressed = (KeyID_t)(KEY_1 + i);
 *                   // 注意: KEY_1 = 1, KEY_2 = 2, KEY_3 = 3
 *               }
 *               // 计数器保持, 防止再次触发
 *           }
 *       } else {
 *           // 不一致! 可能是抖动, 也可能是真正变化
 *           s_keys[i].raw = level;              // 更新原始值
 *           s_keys[i].count = 0;                // 计数器清零
 *       }
 *   }
 *
 * 这段逻辑需要自己实现。注意以下几点:
 *
 *   1. 什么时候记录按键?
 *      当稳定状态变为 0 (按下) 时记录。
 *      而不是在 count 初次达到 3 时记录。
 *      如果记录条件写在 "level != stable" 分支里,
 *      会导致按键释放也被记录!
 *
 *   2. 如何防止长按重复触发?
 *      在确认 stable=0 后, 不再继续记录。
 *      因为当 stable 已经是 0 时,
 *      level == stable 分支不会触发 s_pressed 更新。
 *      只有 stable 从 1 变成 0 (按下瞬间) 才会触发。
 *
 *   3. count 需要重置吗?
 *      在确认稳定后, 可以让 count 保持 KEY_SAMPLE_CNT。
 *      这样下次变化时 (level != stable),
 *      count=0 重新计数, 状态机正常工作。
 */
void KEY_Scan(void)
{
    /* === 按键消抖状态机 === */
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t level = key_read_raw(i);
        if (level == s_keys[i].stable) {
            /* 和稳定状态一致, 计数值递增 */
            if (s_keys[i].count < KEY_SAMPLE_CNT) {
                s_keys[i].count++;
            }
            /* 连续采样一致 → 确认是稳定状态 */
            if (s_keys[i].count >= KEY_SAMPLE_CNT) {
                /* 只在从释放(1)变为按下(0)时触发, 防止长按重复 */
                if (s_keys[i].stable == 0 && s_keys[i].count == KEY_SAMPLE_CNT) {
                    s_pressed = (KeyID_t)(KEY_1 + i);
                }
                /* 保持 count 为 KEY_SAMPLE_CNT, 下次变化时会清零重新开始 */
            }
        } else {
            /* 不一致 → 可能是抖动或真正变化, 更新采样值, 计数器清零 */
            s_keys[i].raw = level;
            s_keys[i].count = 0;
            /* 更新稳定状态 (连续一致才会确认变化) */
            s_keys[i].stable = level;
        }
    }

    /* === 注意: EC11 已移除, 纯按键驱动 === */
}

/*
 * ============================================================
 *  学习笔记: KEY_GetPressed — 原子读清模式
 * ============================================================
 *
 * 已经在 key.h 的学习笔记中详细解释过, 这里不再重复。
 *
 * 注意:
 *   因为这是在任务上下文中调用 (非中断),
 *   所以不需要关中断来保证原子性。
 *   但如果 KEY_Scan 在中断中调用, 则需要:
 *
 *   KeyID_t KEY_GetPressed(void) {
 *       taskENTER_CRITICAL();
 *       KeyID_t k = s_pressed;
 *       s_pressed = KEY_NONE;
 *       taskEXIT_CRITICAL();
 *       return k;
 *   }
 *
 * 临界区 (Critical Section) 保证:
 *   1. s_pressed 的读取和清除之间不会被中断打断
 *   2. 不会丢失按键事件
 *
 * 在 FreeRTOS 中也可以使用:
 *   portDISABLE_INTERRUPTS() / portENABLE_INTERRUPTS()
 *   或使用队列 (xQueueReceive) 替代全局变量
 *
 * 本驱动中 KEY_Scan 在任务中执行, 不是中断,
 * 所以不需要临界区保护。
 */
KeyID_t KEY_GetPressed(void)
{
    KeyID_t k = s_pressed;
    s_pressed = KEY_NONE;
    return k;
}

/*
 * ============================================================
 *  总结: 按键数据流
 * ============================================================
 *
 *    ┌──────────────────────────────────────────────────┐
 *    │                   10ms 定时器                     │
 *    │               (FreeRTOS Tick 驱动)                │
 *    └──────────────────────┬───────────────────────────┘
 *                           │
 *                           ▼
 *    ┌──────────────────────────────────────────────────┐
 *    │          StartKeyScan 任务 (10ms 周期)            │
 *    │                                                  │
 *    │  1. 调用 KEY_Scan()                              │
 *    │     ├── key_read_raw(0) → 消抖状态机 → s_pressed │
 *    │     ├── key_read_raw(1) → 消抖状态机 → s_pressed │
 *    │     └── key_read_raw(2) → 消抖状态机 → s_pressed │
 *    │                                                  │
 *    │  2. osDelay(10) 等待下一周期                     │
 *    └──────────────────────┬───────────────────────────┘
 *                           │
 *             ┌─────────────┼──────────────┐
 *             ▼             ▼              ▼
 *    ┌────────────┐  ┌──────────┐  ┌─────────────┐
 *    │LCD 更新任务 │  │蓝牙任务   │  │WS2812 任务   │
 *    │ 200ms 周期  │  │事件驱动   │  │ 50ms 周期   │
 *    │            │  │          │  │             │
 *    │读取按键:   │  │读取按键:  │  │ (不使用按键) │
 *    │GetPressed()│  │GetPressed│  │             │
 *    │→ 切换页面  │  │→ 发指令  │  │             │
 *    └────────────┘  └──────────┘  └─────────────┘
 *
 * 注意: KEY_GetPressed 使用读后清, 所以多个任务同时
 * 读时, 只有一个任务能读到按键事件, 其他读到 KEY_NONE。
 * 如果多个任务都需要按键事件, 可以考虑使用消息队列
 * (xQueue_SensorData 类似的 xQueue_KeyEvent) 来广播。
 */
