/**
 * @file    bluetooth.h
 * @brief   蓝牙模块驱动 + JSON 数据协议 (XW040, USART2, 9600bps)
 * @note    接收: DMA Circular + IDLE 中断不定长; 发送: 阻塞式
 *          环形缓冲区 + JSON 解析, 支持 get_temp/get_all/led 命令
 */
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * DMA 接收缓冲区 (256 字节):
 *   最大 JSON 包约 75 字节, 256 留有余量
 *   若缓冲区 < 最大包长, 数据可能被 DMA 覆盖
 */
#define BT_RX_BUF_SIZE          256U

/*
 * 环形队列 (512 字节, 2 的幂):
 *   缓存 ISR→Task 之间的数据, 容量 512, head/tail 用 &(size-1) 快速取模
 *   ISR 只写 head, Task 只读 tail, 单生产者+单消费者无需锁
 */
#define BT_RING_BUF_SIZE        512U

/* 命令枚举 */
typedef enum {
    BT_CMD_NONE = 0,            /* 无命令 */
    BT_CMD_GET_TEMP,            /* {"cmd":"get_temp"} */
    BT_CMD_GET_ALL,             /* {"cmd":"get_all"} */
    BT_CMD_SET_LED,             /* {"cmd":"led","r":xxx,"g":xxx,"b":xxx} */
    BT_CMD_UNKNOWN,
} BT_Cmd_t;

/* 解析结果包 */
typedef struct {
    BT_Cmd_t cmd;
    int32_t  r, g, b;           /* LED 参数 (BT_CMD_SET_LED) */
} BT_CmdPacket_t;

/* 初始化 UART + DMA 循环接收, 注册 IDLE 中断 */
void BLUETOOTH_Init(UART_HandleTypeDef *huart);

/*
 * 阻塞发送: 低频场景 (1 包/秒) 简单可靠
 * 9600bps 下 150 字节约 156ms; 若超时需检查蓝牙是否连接
 */
void BLUETOOTH_Send(const char *json_str);

/*
 * 构造 JSON 并发送 (含 IMU):
 *   {"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.250,"pwr":3.01,
 *    "accel":{"x":0.01,"y":0.02,"z":9.81},"gyro":{"x":0.1,"y":0.2,"z":0.0},
 *    "angle":{"x":12,"y":-5,"z":90}}\r\n
 * 尾部 \r\n: 手机端 App 以换行分隔数据包
 */
void BLUETOOTH_SendSensorData(float temp, float humi,
                              float volt, float curr, float pwr,
                              float accel_x, float accel_y, float accel_z,
                              float gyro_x, float gyro_y, float gyro_z,
                              float angle_x, float angle_y, float angle_z);

/*
 * 非阻塞获取命令:
 *   从环形队列读一行 → strstr 简易 JSON 解析 (不引入 cJSON)
 *   返回 1=有新命令, 0=无
 *   调用者每 10~50ms 查询一次
 */
uint8_t BLUETOOTH_GetCmd(BT_CmdPacket_t *pkt);

/*
 * IDLE 中断处理 (在 stm32f4xx_it.c 的 USART2_IRQHandler 中调用):
 *   只做数据搬移 (DMA buf → 环形队列), 不做协议解析
 *   必须极快 (<10µs): 不能 printf/malloc/信号量普通 API
 */
void BLUETOOTH_IDLE_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H */
