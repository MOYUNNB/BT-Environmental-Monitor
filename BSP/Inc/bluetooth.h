/**
 * @file    bluetooth.h
 * @brief   蓝牙模块驱动 + JSON 数据协议
 * @note    XW040 模块, USART2 (PD5/PD6), 9600bps
 *          接收: DMA Circular + 空闲中断 (IDLE) 实现不定长接收
 *          发送: 阻塞式 HAL_UART_Transmit
 *
 *          === 需要你自己实现 ===
 *          这是 README 中面试亮点的核心模块!
 *          bluetooth.c 中已提供 DMA 缓冲区和环形队列, 你需要:
 *          1. BLUETOOTH_Init() — 启动 DMA 接收 + 使能 IDLE 中断
 *          2. BLUETOOTH_SendJSON() — 构造 JSON 字符串并发送
 *          3. BLUETOOTH_GetCmd() — 从环形队列取一条完整指令
 *          4. USART2_IRQHandler — IDLE 中断处理 (在 stm32f4xx_it.c 中注册)
 *
 *          参考: README 中「DMA + 空闲中断实现不定长接收」章节
 *
 * ============================================================
 * ==                    蓝牙模块工作原理                      ==
 * ============================================================
 *
 * 【XW040 蓝牙模块】
 *   XW040 是一款基于 HCI (Host Controller Interface) 协议的 SPP (Serial Port
 *   Profile) 蓝牙从模块。对 MCU 来说, XW040 就是一个 UART 透传设备:
 *
 *   MCU (USART2) <--UART--> XW040 <--蓝牙 2.4GHz--> 手机/PC
 *
 *   MCU 往 USART2 发什么, 手机就收到什么; 手机发什么, MCU 的中断就能收到。
 *   也就是说, 蓝牙模块把有线 UART 变成了无线 UART —— 这就是"透传"的含义。
 *
 *   关键特性:
 *   - 工作模式: SPP (Serial Port Profile), 类似有线串口
 *   - 波特率: 9600 bps (与 USART2 一致)
 *   - 默认配对: 模块上电即广播, 手机搜索 "XW040" 即可连接
 *   - 功耗: 广播状态约 10mA, 连接后约 5mA
 *
 * 【SPP 协议】
 *   SPP 是蓝牙经典(BR/EDR)协议中的一个 Profile, 定义了如何在蓝牙链路上
 *   模拟串口通信。RFCOMM 协议位于 L2CAP 之上, 提供串口仿真。
 *   简单说: SPP = 无线版 UART。
 *
 * 【DMA + IDLE 不定长接收原理 (面试亮点!)】
 *   传统 UART 接收方式: 中断每字节进一次 ISR, CPU 开销大, 高波特率下容易丢数据。
 *   本项目采用 DMA + IDLE 方式, CPU 几乎零开销完成接收:
 *
 *   ○ DMA Circular Mode (循环模式):
 *     - DMA 配置为"循环模式"(Circular), 通道连接到 USART2 RX 请求
 *     - 每收到 1 字节, DMA 自动从 USART DR 寄存器搬运到 s_dma_rx_buf
 *     - DMA 内部有 Current Memory Address 指针, 每搬运一字节递增
 *     - 到达缓冲区末尾 (s_dma_rx_buf + BT_RX_BUF_SIZE) 时, 自动回卷到开头
 *       ┌─────────────────────────────────────┐
 *       │ s_dma_rx_buf[0..BT_RX_BUF_SIZE-1]   │  ← 环形缓冲区
 *       │ ↑DMA写入               ↑上次读取位置 │
 *       └─────────────────────────────────────┘
 *     - DMA 全程不需要 CPU 参与, 数据传输由 DMA 控制器硬件完成
 *
 *   ○ IDLE Interrupt (空闲中断):
 *     - USART 外设内置空闲检测: RX 线空闲超过 1 字节传输时间时,
 *       硬件自动置位 USART_SR 寄存器的 IDLE 标志位
 *     - 9600bps 下 1 字节 ≈ 1.04ms (1 起始位 + 8 数据位 + 1 停止位)
 *     - 所以: 超过约 1ms 没有新数据 → 认为一帧结束 → 触发 IDLE 中断
 *     - 这特别适合不定长数据包场景 —— 不需要预先知道一包有多长!
 *
 *   ○ NDTR 计算接收长度:
 *     - DMA 的 NDTR (Number of Data Transfer) 寄存器保存"还剩多少字节没搬"
 *     - DMA 启动时 NDTR = BT_RX_BUF_SIZE (256)
 *     - 每搬运 1 字节, NDTR 递减 1
 *     - 到达末尾回卷时, NDTR 重新变成 BT_RX_BUF_SIZE
 *     - 本次接收长度 = (上次 NDTR - 当前 NDTR) & (BT_RX_BUF_SIZE - 1)
 *     - & (size-1) 是处理回卷的经典技巧: 256=2^8, BT_RX_BUF_SIZE-1=0xFF
 *     - 这相当于一个简单的模运算, 比 % 快得多
 *
 *   ○ 环形队列解耦 ISR 和 Task:
 *     - ISR (生产者): IDLE 中断 → 计算 NDTR → 数据入 s_ring
 *     - Task (消费者): BLUETOOTH_GetCmd() → 从 s_ring 取数据 → JSON 解析
 *     - 环形队列头尾指针 head/tail, 当 head != tail 时有数据
 *     - 容量为 2 的幂 (512), 用 &(size-1) 实现快速取模
 *     - ISR 中只做入队操作, 不进行 JSON 解析 —— 中断服务要尽量快!
 *
 *   ★ 关键原则: 绝不能停 DMA!
 *     有些教程在 IDLE 中断里调用 HAL_UART_DMAStop, 处理完再重启。
 *     这会丢失停 DMA 期间到达的字节! 本项目用环形队列, DMA 全程运行,
 *     ISR 只负责"告诉消费者有新数据到了", 由消费者任务在空闲时处理。
 *
 * 【为什么 HAL 不支持 IDLE 中断?】
 *   STM32 HAL 库提供了 HAL_UART_Receive_DMA() 启动 DMA 接收, 但 HAL 的
 *   DMA 接收是定长模式 —— HAL 期望你提前知道接收多少字节, 接收完后调用
 *   回调函数 HAL_UARTEx_RxEventCallback()。但蓝牙数据包长度不定,
 *   所以我们必须手动使能 IDLE 中断, 在中断服务函数中自行处理:
 *
 *   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // HAL 没有封装这个!
 *
 * 【IDLE 中断处理完整流程】
 *   在 USART2_IRQHandler() 中:
 *
 *   1. 检查标志:
 *      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) == RESET) return;
 *      // UART_FLAG_IDLE = USART_SR 寄存器的 bit4
 *      // RESET = 0, 表示 IDLE 标志未置位, 不是空闲中断, 直接返回
 *
 *   2. 清除标志 (STM32 标准操作):
 *      __HAL_UART_CLEAR_IDLEFLAG(&huart2);
 *      // 等效操作: (void)huart2.Instance->SR; (void)huart2.Instance->DR;
 *      // 为什么是读 SR 再读 DR? 这是 STM32 手册规定的清除 IDLE 标志标准时序
 *      // 注意: 不能只用 CLEAR_IDLEFLAG 宏, 它实际是置位 USART_SR 的 IDLE 位
 *      // 正确做法: 读 SR 后再读 DR, 硬件自动清除 IDLE 标志
 *
 *   3. 读 DMA 计数器:
 *      uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
 *      // 获取 DMA 当前还剩多少字节没搬
 *      // hdma_usart2_rx 由 CubeMX 生成, 在 main.h 或 stm32f4xx_hal_msp.c 中定义
 *
 *   4. 计算本次接收数据量:
 *      uint16_t received = (uint16_t)((s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1));
 *      s_last_ndtr = ndtr;
 *      // s_last_ndtr 保存上次中断时的 NDTR 值
 *      // 差值 & (size-1) 自动处理 DMA 回绕
 *      // 例子: s_last_ndtr=200, ndtr=180 → received=20 (正常)
 *      // 例子: s_last_ndtr=10, ndtr=250 (回绕了) → (10-250)&0xFF = 16 (正确!)
 *
 *   5. 数据入环形队列:
 *      // 从 s_dma_rx_buf 中读取 received 字节
 *      // 注意: 数据在 DMA 缓冲区中可能绕回! DMA 可能写满尾部后回到了开头
 *      // 所以要分段拷贝:
 *      //   第一段: 从读取位置到缓冲区末尾
 *      //   第二段: (如果有回绕) 从缓冲区开头到 DMA 当前位置
 *      // 每字节调用 ring_put() 入队
 *
 * 【JSON 协议设计】
 *   为什么选择 JSON 而非自定义二进制协议?
 *   - 人类可读: 调试时一眼就能看出数据内容和格式
 *   - 调试方便: 手机端 App 直接显示文本, 不需要协议解析库
 *   - 可扩展: 增加字段不需要修改协议版本号或协商过程
 *   - 缺点: 长度冗余, 但 9600bps 下每包约 150 字节, 可以接受
 *
 *   发送格式:
 *     {"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.25,"pwr":3.01}
 *   接收格式 (手机 → MCU 命令):
 *     {"cmd":"get_temp"}               ← 获取温度
 *     {"cmd":"get_all"}                ← 获取全部传感器数据
 *     {"cmd":"led","r":255,"g":0,"b":0} ← 设置 LED 颜色
 *
 * 【蓝牙发送速率限制】
 *   9600 bps = 960 字节/秒 (8N1 格式: 每字节 10 bit)
 *   每个 JSON 包 ≈ 150 字节 (含 \r\n 结尾)
 *   960 ÷ 150 ≈ 6.4 包/秒, 取整限制: 最多 6 包/秒
 *   实际设计中, Task_SensorRead 每 100ms 采集一次, 但通过判断数据是否
 *   有明显变化来决定是否发送, 避免蓝牙信道拥塞。
 *
 * 【波特率为何选 9600?】
 *   1. XW040 模块默认波特率 9600
 *   2. 低速增加传输距离和稳定性 (蓝牙无线环境更容易受干扰)
 *   3. 温湿度传感器数据变化缓慢, 1-2 秒更新一次完全足够
 *   4. 如果将来需要高速传输 (如 OTA), 可以协商切换到 115200
 *
 * 【AT 指令配置 XW040】
 *   部分蓝牙模块支持 AT 指令 (如 HC-05/HC-06), XW040 通过特定引脚进入
 *   AT 模式可配置:
 *   - AT+NAME=XW040-001  // 改名字
 *   - AT+BAUD=4          // 改波特率 (4=9600)
 *   本项目假设模块已预配置, 不发送 AT 指令。
 *   如需配置, 在 BLUETOOTH_Init 中先进入 AT 模式, 发送指令后切回数据模式。
 */
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * DMA 接收缓冲区大小 —— 必须足够容纳最大 JSON 包
 * 256 字节的选择理由:
 *   最大 JSON 包 ≈ {"temp":-40.0,"humi":0.0,"volt":0.00,"curr":0.000,"pwr":0.00}
 *   长度约 75 字节。加上 \r\n 和各种可能的扩展字段, 128 也够。设为 256 留有余量。
 *   注意: DMA 使用本缓冲区做循环搬运, 如果缓冲区太小 (< 最大包长),
 *   数据可能被覆盖。经验法则: 缓冲区至少为最大包长的 2 倍。
 */
#define BT_RX_BUF_SIZE          256U

/*
 * 环形队列大小 —— 512 字节
 * 作用: 缓存 ISR 和任务处理之间的数据
 * 容量计算:
 *   IDLE 中断最多产生 BT_RX_BUF_SIZE=256 字节数据
 *   如果任务来不及处理, 最多缓存 512 字节
 *   512 = 2^9, 所以取模可以用 &(512-1) = &0x1FF, 极快
 *   选择 2 的幂不是巧合 —— 为了用位运算代替慢速除法/模运算
 */
#define BT_RING_BUF_SIZE        512U

/*
 * 支持的命令枚举
 * 枚举值从 0 开始 (BT_CMD_NONE), BT_CMD_UNKNOWN 放在最后
 * 手机通过 JSON 格式发送命令, MCU 解析后转换成枚举值
 */
typedef enum {
    BT_CMD_NONE = 0,            /* 无命令 / 空 */
    BT_CMD_GET_TEMP,            /* {"cmd":"get_temp"} — 请求当前温度 */
    BT_CMD_GET_ALL,             /* {"cmd":"get_all"}  — 请求全部传感器数据 */
    BT_CMD_SET_LED,             /* {"cmd":"led","r":255,"g":0,"b":0} — 设置 WS2812 LED */
    BT_CMD_UNKNOWN,             /* 无法识别的命令 */
} BT_Cmd_t;

/*
 * 解析后的指令数据包结构体
 * 用于 BLUETOOTH_GetCmd() 返回给调用者
 * cmd 字段指示命令类型, r/g/b 仅在 BT_CMD_SET_LED 时有效
 * 其他命令类型可以扩展此结构体 (如加入 dimmer 亮度字段)
 */
typedef struct {
    BT_Cmd_t cmd;               /* 命令类型 */
    int32_t  r, g, b;           /* LED 参数 (BT_CMD_SET_LED 时有效), 范围 0-255 */
} BT_CmdPacket_t;

/**
 * @brief  初始化蓝牙模块 (启动 DMA 接收 + IDLE 中断)
 * @param  huart: USART 句柄指针 (&huart2)
 * @note   TODO: 启动 DMA 循环接收 → 使能 IDLE 中断
 *
 * 实现指引:
 *   1. 保存 huart 到 s_huart
 *   2. 初始化环形队列: ring_init(&s_ring)
 *   3. 启动 DMA 循环接收:
 *      HAL_UART_Receive_DMA(s_huart, s_dma_rx_buf, BT_RX_BUF_SIZE);
 *      这行调用会让 DMA 将 USART2 RX 数据持续搬运到 s_dma_rx_buf
 *   4. 手动使能 USART2 IDLE 中断:
 *      __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
 *      为什么要手动? 因为 HAL 不支持 IDLE 中断, 它只支持"固定长度接收完成"中断
 *   5. 初始化 s_last_ndtr:
 *      s_last_ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
 *      保存初始 NDTR 值, 供 IDLE 中断中计算接收长度
 *   6. 注意: 不要在 Init 里使能 USART2_IRQHandler —— USART2 全局中断
 *      由 NVIC 控制, CubeMX 已在 stm32f4xx_it.c 中生成 IRQHandler。
 *      你需要做的是在 IRQHandler 中判断 IDLE 标志并调用 BLUETOOTH_IDLE_IRQHandler()
 *
 * 常见错误:
 *   - DMA 改成 Normal 模式而不是 Circular → 收满一次就停了
 *   - 忘记使能 IDLE 中断 → 永远不会触发接收处理
 *   - 多次调用 Init 导致 DMA 重复启动 → HAL 返回 HAL_BUSY
 */
void BLUETOOTH_Init(UART_HandleTypeDef *huart);

/**
 * @brief  发送 JSON 数据 (阻塞式)
 * @param  json_str: JSON 字符串, 需以 \r\n 结尾
 * @note   TODO: HAL_UART_Transmit
 *
 * 实现指引:
 *   HAL_UART_Transmit(s_huart, (uint8_t *)json_str, strlen(json_str), 超时时间);
 *   阻塞发送, 等待发送完成或超时返回。
 *
 * 为什么用阻塞式而不是 DMA?
 *   本项目发送数据频率低 (每秒 1 次), 阻塞等待对系统影响很小。
 *   9600bps 下 150 字节约 150/960*1000 ≈ 156ms 发送时间。
 *   Task_LCD_Update 每 200ms 执行一次, 蓝牙发送会阻塞 156ms,
 *   虽然 LCD 更新延迟了 156ms, 但人类肉眼看不出来。
 *   如果换成 115200bps, 150 字节只需约 13ms, 就更不是问题了。
 *
 * 为什么不使用中断/DMA 发送?
 *   HAL_UART_Transmit_IT 需要维护发送完成标志, 代码复杂。
 *   对于低频发送场景, 阻塞式简单可靠。
 *   但如果将来需要同时发送大量数据 (如 OTA 固件升级),
 *   建议改用 DMA 发送, 避免阻塞其他任务。
 *
 * 超时时间设置:
 *   9600bps 下 256 字节最大传输 ≈ 267ms, 所以设 1000ms 是安全的。
 *   如果返回 HAL_TIMEOUT, 需要检查蓝牙模块是否连接/波特率是否正确。
 */
void BLUETOOTH_Send(const char *json_str);

/**
 * @brief  发送传感器 JSON 数据 (构造后再发)
 * @param  temp: 温度 (°C)
 * @param  humi: 湿度 (%RH)
 * @param  volt: 电压 (V)
 * @param  curr: 电流 (A)
 * @param  pwr:  功率 (W)
 * @note   TODO: snprintf → BLUETOOTH_Send
 *
 * 数据格式:
 *   {"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.25,"pwr":3.01}\r\n
 *
 * 实现指引:
 *   1. 在栈上申请一个 char[200] 的缓冲区 (200 字节足够, 不要用 malloc)
 *   2. 调用 snprintf 按 JSON 格式组装:
 *      snprintf(buf, sizeof(buf),
 *          "{\"temp\":%.1f,\"humi\":%.1f,\"volt\":%.2f,\"curr\":%.3f,\"pwr\":%.2f}\r\n",
 *          temp, humi, volt, curr, pwr);
 *   3. 调用 BLUETOOTH_Send(buf) 发送
 *
 * 为什么用 snprintf 而非 sprintf?
 *   snprintf 会限制最大写入长度, 防止缓冲区溢出。
 *   sprintf 不检查长度 —— 如果温度显示 "-40.00" 这种长格式,
 *   加上其他字段可能刚好超过缓冲区大小, snprintf 会安全截断。
 *
 * 浮点数格式化说明:
 *   %.1f — 小数点后 1 位: 温度显示到 0.1°C 精度 (AHT20 精度 ±0.3°C)
 *   %.2f — 小数点后 2 位: 电压显示到 0.01V (INA226 分辨率 1.25mV)
 *   %.3f — 小数点后 3 位: 电流显示到 1mA
 *   数据精度和数据来源的硬件精度匹配, 不显示虚假的"高精度"。
 *
 * \r\n 结尾的重要性:
 *   手机端接收程序通常以换行符分割数据包。\r\n 是标准的行结束符
 *   (CR+LF, 回车换行), 兼容性和可读性最好。
 *
 * 注意:
 *   C 标准库的 sprintf 系函数体积较大 (~12KB), 可能影响固件大小。
 *   如果空间紧张, 可以自己实现简单的 float_to_str() 替代。
 */
void BLUETOOTH_SendSensorData(float temp, float humi, float volt, float curr, float pwr);

/**
 * @brief  获取一条蓝牙指令 (非阻塞)
 * @param  pkt: 输出解析后的指令包
 * @retval 1: 有新指令, 0: 无指令
 * @note   TODO: 从环形队列取数据 → JSON 解析
 *
 * 实现指引:
 *   这是蓝牙接收的核心函数, 由任务循环调用。
 *   由于是非阻塞函数, 调用者可以每 10-50ms 调用一次,
 *   没有命令时立即返回 0, 不影响其他任务运行。
 *
 *   步骤:
 *   1. 定义一个局部缓冲区 line[128] (接收一行命令)
 *   2. 循环调用 ring_get(&s_ring, &ch), 从环形队列取一个字节:
 *      - 如果 ring_get 返回 -1 → 队列空, 返回 0
 *      - 如果 ch == '\n' → 一行结束, 开始解析
 *      - 否则将 ch 追加到 line 缓冲区
 *      - 注意: 忽略 '\r' (回车符, 和 \n 一起出现)
 *   3. 行解析 (简易 JSON 解析器, 不需要引入 cJSON 库):
 *      - 用 strstr() 搜索关键字段:
 *        if (strstr(line, "\"get_temp\""))  → pkt->cmd = BT_CMD_GET_TEMP
 *        if (strstr(line, "\"get_all\""))   → pkt->cmd = BT_CMD_GET_ALL
 *        if (strstr(line, "\"led\""))       → pkt->cmd = BT_CMD_SET_LED
 *      - 对于 SET_LED, 需要提取 r/g/b 值:
 *        char *p = strstr(line, "\"r\":");
 *        if (p) pkt->r = atoi(p + 4);
 *        // 注意 atoi 从 p+4 位置开始解析数字, 遇到非数字字符停止
 *        // 同理解析 g 和 b
 *   4. 返回 1 表示有新命令
 *
 * 为什么不直接使用 cJSON 库?
 *   cJSON 功能强大, 但体积大 (~3KB 代码 + 动态内存分配)。
 *   本项目的命令格式简单固定, strstr + atoi 足够。
 *   这就是"够用原则"——不要为了炫技引入不必要的依赖。
 *
 * 安全注意事项:
 *   限定 line 缓冲区最大 128 字节, 防止恶意的超长 JSON 导致栈溢出。
 *   atoi 对非数字输入返回 0, 所以如果手机端发送 {"cmd":"led"} 不带 RGB,
 *   解析结果也是安全的 (r=g=b=0, 关灯)。
 *   这是"防御性编程": 不要相信任何来自外部的数据, 包括自己手机发的。
 *
 * 调试技巧:
 *   如果发现命令解析不对, 可以在解析前用 printf 打印 line 的内容:
 *   printf("BT raw: %s\r\n", line);
 *   常见问题: line 中包含了 \r 导致 strstr 匹配不上。
 */
uint8_t BLUETOOTH_GetCmd(BT_CmdPacket_t *pkt);

/**
 * @brief  空闲中断回调 (在 stm32f4xx_it.c 的 USART2_IRQHandler 中调用)
 * @note   TODO: 判断 IDLE 标志 → 读 NDTR → 将数据推入环形队列
 *
 * 实现指引:
 *   此函数在中断上下文中调用, 必须执行得尽可能快!
 *   "快"的意思是: 几微秒内完成, 不能有 printf/snprintf/malloc 等耗时操作。
 *   这个函数只做一件事: 把 DMA 缓冲区里的新数据搬到环形队列。
 *
 *   步骤:
 *   1. 检查 IDLE 标志:
 *      if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET) return;
 *   2. 清除 IDLE 标志 (标准读 SR 再读 DR 方式):
 *      __HAL_UART_CLEAR_IDLEFLAG(s_huart);
 *      或手动: (void)s_huart->Instance->SR; (void)s_huart->Instance->DR;
 *   3. 获取当前 DMA NDTR:
 *      uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
 *      // ndtr 是 DMA 还剩多少字节没搬运, 值范围 0 ~ BT_RX_BUF_SIZE
 *   4. 计算接收字节数:
 *      uint16_t received = (uint16_t)((s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1));
 *      s_last_ndtr = ndtr;
 *      // received 可能为 0 (DMA 还没来得及搬运数据),
 *      // 如果为 0 直接 return, 不处理
 *   5. 计算读取起始位置:
 *      uint16_t start_pos = (ndtr) & (BT_RX_BUF_SIZE - 1);
 *      // DMA 当前停在"还没搬运的第一个字节",
 *      // 公式: start_pos = (BT_RX_BUF_SIZE - ndtr) & (BT_RX_BUF_SIZE - 1)?
 *      // 不对! 让我们重新思考:
 *      // DMA 在 Circular 模式下的指针行为:
 *      //   初始: DMA 指针指向 s_dma_rx_buf[0]
 *      //   收 1 字节: DMA 指针 → s_dma_rx_buf[1], NDTR 从 256→255
 *      //   收 N 字节后: DMA 指针→ s_dma_rx_buf[N], NDTR = 256-N
 *      // 所以: 有效数据起始位置 = (BT_RX_BUF_SIZE - ndtr) & (BT_RX_BUF_SIZE - 1)
 *      //      有效数据结束位置 = 前一次的起始位置 = s_last_read_pos
 *      // 实践中更简单的做法: 从 "上次读到的位置" 读出 received 字节
 *      // 即 start = (s_last_read_pos + 1) & (BT_RX_BUF_SIZE - 1) ...
 *      // 要理解复杂, 先实现简单: 用一个 pos 变量追踪读取位置
 *
 * 简化版实现 (如果不想绕晕):
 *   用两个变量: s_last_ndtr (上次 NDTR) 和 s_dma_read_pos (上次读取位置)
 *   每次 IDLE 中断:
 *     received = (s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1);
 *     从 s_dma_read_pos 循环读取 received 字节入队列;
 *     s_dma_read_pos = (s_dma_read_pos + received) & (BT_RX_BUF_SIZE - 1);
 *     s_last_ndtr = ndtr;
 *
 *   回绕处理示例:
 *     缓冲区图: [0---100---200---255]
 *     如果 DMA 写到 250, 又收了 10 字节回绕到开头:
 *       上一帧: 数据在 0-9, 起始于 0, received=10
 *       这帧: 数据在 10-29, 起始于 10, received=20
 *     DMA 缓冲区的逻辑是循环的, 你的读取逻辑也必须循环。
 *     用 ring_put() 逐个字节入队, 它会自己处理回绕。
 *
 *   ⚠️ 关键注意事项:
 *     1. 不要在中断中解析 JSON —— 那会阻塞系统
 *     2. 不要在中断中 printf/串口输出 —— 可能引发重入问题
 *     3. 如果 received > BT_RX_BUF_SIZE/2, 说明可能数据错位
 *        (DMA 设置问题或缓冲区溢出), 可以考虑丢弃整帧
 *     4. 确保 s_dma_rx_buf 和 s_ring 全局可见 —— 中断可以访问全局变量
 *
 * 【面试官可能会问的深入问题】
 *   Q: 为什么不用 HAL_UARTEx_RxEventCallback?
 *   A: HAL 的 RxEventCallback 是 DMA 传输完成后回调, 需要指定接收长度。
 *      我们的数据包是"不定长"的, 所以不能用 —— 这就是要手动用 IDLE 的原因。
 *
 *   Q: DMA 回绕时数据处理有什么坑?
 *   A: 如果 received 计算用了无符号减法或模运算错了, 可能导致读数据错位。
 *      关键: 用 &(size-1) 而不是 %, 即使 NDTR 从 1 变成 253 (回绕),
 *      (1 - 253)&0xFF = 4, 正确!
 *
 *   Q: 如果两个数据包背靠背到达 (没有空闲时间), 会怎样?
 *   A: 它们会合并成一个大的"一帧", 因为中间没有 IDLE。
 *      这是这种方案的固有限制。解决: 在数据内容中加帧头帧尾标记,
 *      或者在数据包之间插入延时。
 */
void BLUETOOTH_IDLE_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H */
