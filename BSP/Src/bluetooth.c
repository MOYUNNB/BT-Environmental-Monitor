/**
 * @file    bluetooth.c
 * @brief   蓝牙模块驱动 + JSON 数据协议实现
 * @note    USART2 DMA Circular + IDLE 中断不定长接收。
 *          这是 README 面试亮点的核心 — 面试官必问!
 *
 * DMA + IDLE 接收原理:
 *   1. DMA 设置为循环模式 (Circular), 连到 USART2 RX
 *   2. 每收到一字节, DMA 自动搬移到 s_dma_rx_buf
 *   3. 总线空闲超过 1 字节时间 → USART 置位 IDLE 标志 → IDLE 中断
 *   4. 在 IDLE 中断中: 读 USART2->SR → 读 USART2->DR 清标志
 *   5. 计算接收长度: len = BT_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(usart2_dma)
 *      或者使用差值法: received = (prev_ndtr - cur_ndtr) & (BUF_SIZE - 1)
 *   6. 将数据推入环形队列 s_ring_buf, 供解析任务消费
 *
 * 关键: 不能停 DMA! 停了会丢数据。
 * 环形队列解耦了接收 (ISR, 生产者) 和处理 (Task, 消费者) 的时序。
 *
 * 数据格式 (JSON):
 *   发送: {"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.25,"pwr":3.01}
 *   接收: {"cmd":"get_temp"} / {"cmd":"led","r":255,"g":0,"b":0}
 *
 * ============================================================
 * ==                XW040 蓝牙模块硬件连接                    ==
 * ============================================================
 *
 *   XW040 模块通过 USART2 与 STM32F407 通信:
 *     STM32 PD5 (USART2_TX)  → XW040 RX (模块接收)
 *     STM32 PD6 (USART2_RX)  → XW040 TX (模块发送)
 *     STM32 GND               → XW040 GND
 *     STM32 3.3V              → XW040 VCC
 *
 *   注意:
 *   - XW040 工作电压 3.3V, 不能接 5V
 *   - 蓝牙天线部分不要铺地或走线, 否则会减少通信距离
 *   - 如果使用 AT 指令配置, 需要额外连接一个 GPIO 到模块的 AT 引脚
 *
 * ============================================================
 * ==         DMA + IDLE 不定长接收的详细工作机制              ==
 * ============================================================
 *
 * 【DMA Circular Mode 深入理解】
 *
 *   DMA (Direct Memory Access, 直接存储器访问) 是 Cortex-M 内核外的一个
 *   硬件模块, 专门负责数据传输。它有自己的总线矩阵, 不占用 CPU 的指令执行
 *   带宽。本项目中, DMA 从 USART2 的 DR 寄存器读取接收到的字节, 写入内存
 *   数组 s_dma_rx_buf。
 *
 *   启动步骤:
 *   HAL_UART_Receive_DMA(&huart2, s_dma_rx_buf, BT_RX_BUF_SIZE);
 *
 *   这行 HAL 调用内部做了:
 *   1. 设置 DMA 源地址 = USART2->DR 的地址 (USART2 RX 数据寄存器)
 *   2. 设置 DMA 目标地址 = s_dma_rx_buf 的地址
 *   3. 设置传输长度 NDTR = BT_RX_BUF_SIZE (256)
 *   4. 使能 DMA 通道 CIRC 位 (循环模式)
 *   5. 使能 DMA 通道 (EN 位置 1)
 *   6. USART2 CR3 寄存器的 DMAR (DMA Receiver) 位置 1 (使能 DMA 接收请求)
 *   此后, 每收到一字节, DMA 自动从 USART2->DR 读取并写入 s_dma_rx_buf[i],
 *   然后 i++, NDTR--, 到末尾后 i=0, NDTR=BT_RX_BUF_SIZE, 继续循环。
 *
 *   DMA 有三个关键寄存器:
 *   - CNDTR (Current NDTR): 当前剩余传输字节数, 可随时读取
 *   - CMAR  (Current Memory Address Register): 当前内存地址
 *   - CPAR  (Current Peripheral Address Register): 外设地址 (不变)
 *   我们主要用 CNDTR 来计算接收了多少数据。
 *
 * 【IDLE 中断详解】
 *
 *   USART 外设的空闲检测电路:
 *   - 每收到一字节, RX 线上有电平变化, 空闲检测计数器复位
 *   - RX 线连续保持高电平 (空闲状态) 超过 1 个字节时间 (9600bps ≈ 1.04ms),
 *     空闲检测计数溢出, 置位 USART_SR 寄存器的 IDLE 标志 (bit 4)
 *   - 如果 USART_CR1 的 IDLEIE 位为 1, 则产生 USART 全局中断
 *
 *   中断响应链:
 *   USART2 RX 空闲 → IDLE 标志 = 1 → USART 全局中断请求 → NVIC → CPU 执行
 *   USART2_IRQHandler() 函数 (在 stm32f4xx_it.c 中) → 判断 IDLE 标志 →
 *   调用 BLUETOOTH_IDLE_IRQHandler()
 *
 *   标准清 IDLE 标志时序 (来自 STM32F4 参考手册 RM0090):
 *   1. 读 USART_SR 寄存器 (读 SR)
 *   2. 读 USART_DR 寄存器 (读 DR)
 *   这个读序列让硬件自动清除 IDLE 标志。
 *   HAL 宏 __HAL_UART_CLEAR_IDLEFLAG 也是这么做的, 但为了保险,
 *   也可以手动:
 *     (void)s_huart->Instance->SR;
 *     (void)s_huart->Instance->DR;
 *   (void) 转换防止编译器警告"未使用的值"。
 *
 *   有些人会问: 读 SR 再读 DR, DR 里的数据会不会丢失?
 *   不会! DR 是 USART 接收数据寄存器, 在 DMA 模式下, 数据已经被 DMA
 *   搬运到 s_dma_rx_buf 了。读 DR 不会影响 DMA 正在进行的操作。
 *   实际上, DMA 和 CPU 可以同时访问 USART, 一个读走数据, 一个读状态。
 *
 * 【NDTR 差值与 DMA 回绕的数学】
 *
 *   设 B = BT_RX_BUF_SIZE = 256 = 2^8
 *   mask = B - 1 = 0xFF
 *
 *   初始状态:
 *     DMA CNDTR = B, DMA 内存地址 = s_dma_rx_buf[0]
 *
 *   接收 10 字节后:
 *     CNDTR = B - 10 = 246, DMA 内存地址 = s_dma_rx_buf[10]
 *     已接收 = (初始 NDTR - 当前 NDTR) & mask = (256 - 246) & 0xFF = 10 ✓
 *
 *   接收 256 字节, 回绕到开头:
 *     一开始: NDTR = B - 255 = 1 (只剩 1 字节没搬)
 *     再收 1 字节: NDTR 从 1 变成 B (计数重置!)
 *     实际上 DMA 内部: 当 NDTR 减到 0 时, 硬件自动重新加载 CNDTR = B,
 *     并将内存地址重置为 s_dma_rx_buf[0]
 *     所以: 回绕后 NDTR = B
 *
 *   接收 260 字节后 (回绕后又收了 4 字节):
 *     CNDTR = B - 260 + B = 256 - 4 = 252
 *     已接收 = (256 - 252) & 0xFF = 4 ✓
 *
 *   通用公式:
 *     received = (prev_ndtr - cur_ndtr) & (B - 1)
 *
 *   证明:
 *     当没有回绕: prev > cur, (prev - cur) = 正数
 *     当回绕一次: prev < cur, (prev - cur) 为负数
 *       在 C 中 uint16_t 减法会回绕 (0 - 1 = 65535)
 *       但 &(B-1) 截断了高位, 等效于 (prev - cur + B) % B
 *       所以即使回绕, 结果也正确!
 *
 *   限制: 如果接收的字节数超过 B, 这个公式会出错。
 *   所以我们的缓冲区必须足够大 (256 字节 > 最大 JSON 包长)。
 *
 * 【环形队列实现细节】
 *
 *   环形队列 (Ring Buffer) 是一个 FIFO (先进先出) 数据结构:
 *   - 固定的数组 + 头尾指针
 *   - 头指针 (head): 指向下一个写入位置
 *   - 尾指针 (tail): 指向下一个读取位置
 *   - 当 head == tail → 队列空
 *   - 当 (head + 1) % size == tail → 队列满
 *   - 容量 = size - 1 (故意留一个空位, 用于区分满和空)
 *
 *   为什么容量是 size - 1 而不是 size?
 *     如果队列满也允许 head == tail, 那就无法区分"满"和"空"。
 *     所以: 约定 head == tail 表示空, 满的条件是 head 再走一步就追上 tail。
 *
 *   为什么 ring_avail 用 &(size-1) 而不是 % size?
 *     取模运算 % 在 ARM Cortex-M4 上需要 4-6 个时钟周期 (除法指令),
 *     而 &(size-1) 仅需 1 个时钟周期。
 *     ring_avail 在 ISR 中调用, 必须是极快的。
 *     条件: size 必须为 2 的幂。BT_RING_BUF_SIZE = 512 = 2^9, 可以。
 *
 *   对于 ring_put、ring_get:
 *     同样用 &(size-1) 实现回绕。
 *     next = (r->head + 1) & (BT_RING_BUF_SIZE - 1);
 *     这等效于 next = (r->head + 1) % BT_RING_BUF_SIZE;
 *     但汇编代码少得多。
 *
 * 【USART2_IRQHandler 注册位置】
 *   CubeMX 自动生成的中断服务函数在 Core/Src/stm32f4xx_it.c 中:
 *
 *     void USART2_IRQHandler(void)
 *     {
 *         // USER CODE BEGIN USART2_IRQn 0
 *         BLUETOOTH_IDLE_IRQHandler();  // ← 你在这里添加
 *         // USER CODE END USART2_IRQn 0
 *         HAL_UART_IRQHandler(&huart2); // HAL 处理其他 USART 中断
 *         // USER CODE BEGIN USART2_IRQn 1
 *         // USER CODE END USART2_IRQn 1
 *     }
 *
 *   注意调用顺序: 先处理 IDLE 中断 (自定义), 再调用 HAL_UART_IRQHandler
 *   (处理 HAL 注册的其他中断)。如果调换顺序, IDLE 标志可能被 HAL 内部处理
 *   时清除, 导致我们的处理函数读不到数据。
 *
 * 【面试高频题】
 *
 *   Q: DMA + IDLE 方式和每字节中断相比有什么优势?
 *   A: 每字节中断: 9600bps 下一个 CPU 要进 960 次中断/秒, 每次约 1μs,
 *      每秒中断开销约 960μs, 占 0.096% CPU。看似不多, 但如果波特率升到
 *      115200, 每秒 11520 次中断 → 1.15% CPU 占用。
 *      DMA + IDLE: 9600bps 下每秒只进 1-6 次 IDLE 中断 (取决于数据包多少),
 *      CPU 几乎零开销。关键是: DMA 在后台搬运数据, CPU 可以处理其他任务。
 *
 *   Q: 为什么不用串口 FIFO (USART 自带 FIFO)?
 *   A: STM32F407 不支持 USART FIFO (F0/F3/G4/L4 系列部分支持)。
 *      我们的方案是自己用内存实现了 FIFO, 且容量更大 (256 字节 VS 硬件 16 字节)。
 *
 *   Q: 数据包之间没有空闲间隔怎么办?
 *   A: IDLE 在两个包之间不会触发。解决方案: 在解析层检测固定长度的帧头/
 *      帧尾, 或用超时定时器: 收到数据后启动定时器, 若定时器溢出时没有新数据,
 *      则认为一帧结束。这就是 "超时帧检测" 方案, 比纯 IDLE 更可靠。
 *      本项目用简单的 IDLE + \n 换行判断, 满足蓝牙通信需求。
 *
 *   Q: 如果 DMA 缓冲区被覆盖 (还来不及处理就满了) 怎么办?
 *   A: BT_RX_BUF_SIZE 是最大 JSON 包的 3 倍以上 (256 VS ~75),
 *      理论上不会出现。如果真出现, 有两种方案:
 *      a) 增大缓冲区到 512 或 1024
 *      b) 在 BLUETOOTH_IDLE_IRQHandler 中检测 received == BT_RX_BUF_SIZE
 *         (缓冲区满了 → 新旧数据开始重叠) → 丢弃整帧数据, 打印告警
 *
 * ============================================================
 * ==             JSON 协议设计详解                           ==
 * ============================================================
 *
 * 为什么选 JSON?
 *   1. 人类可读: 手机端调试 App 直接显示 "{"temp":25.3}" 无需解码
 *   2. 调试方便: 用串口助手就能模拟蓝牙发送, 不需要专门的协议调试工具
 *   3. 跨语言: Python/JavaScript/C 都能轻易处理
 *   4. 可扩展: 要增加字段 (如 PM2.5) 直接加进 JSON, 旧解析器忽略新字段
 *   5. 自描述: 不用查文档就知道每个值的含义
 *
 * 为什么不选二进制协议?
 *   二进制协议 (如 Modbus RTU, CANOpen) 效率高但调试困难:
 *   - 0x01 0x03 0x00 0x01 0x00 0x01 0xD5 0xCA 是什么? 没人一眼看懂
 *   - 增加字段需要修改协议版本
 *   - 发送端和接收端必须用同一个编解码表
 *   - 对于 9600bps 的低速链路, 协议开销不是主要矛盾
 *
 * 额外福利: JSON 包可以直接用 MQTT 转发到云端
 *   如果将来要上物联网云平台 (阿里云 IoT / AWS IoT),
 *   现在的 JSON 格式直接就是 MQTT payload, 无需格式转换。
 *
 * 协议设计约定:
 *   上行 (STM32 → 手机): 只发传感器数据, 不发日志/调试信息
 *   下行 (手机 → STM32): 命令格式, 支持 get_temp / get_all / led
 *   每帧以 \r\n 结尾, 接收方按行读取
 */
#include "bluetooth.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- 模块内部全局变量 ---- */

/*
 * s_huart: 保存 USART 句柄指针
 * 由 BLUETOOTH_Init 传入并保存, 供整个模块使用。
 * 为什么不用全局的 &huart2?
 *   - 降低模块间耦合: BSP 模块不引用 main.c 中定义的变量
 *   - 可测试性: 单元测试时可以传入 mock 句柄
 *   - 可移植: 如果换到 USART1, 只需改 Init 的入参
 */
static UART_HandleTypeDef *s_huart = NULL;

/*
 * hdma_usart2_rx: DMA 句柄, 由 CubeMX 在 main.h 或 stm32f4xx_hal_msp.c 中声明
 * 使用 extern 引用, 因为这是 CubeMX 生成的全局变量。
 * 我们需要它来读取 DMA 的 CNDTR 寄存器计算接收数据量。
 */
extern DMA_HandleTypeDef hdma_usart2_rx;

/*
 * s_dma_rx_buf: DMA 循环接收缓冲区
 *
 * 数据流向:
 *   USART2 RX 引脚 → USART2 DR 寄存器 (硬件接收移位寄存器)
 *   → DMA 通道 (自动读取 DR, 写入内存)
 *   → s_dma_rx_buf[] (DMA 循环写入)
 *   → (IDLE 中断中) 推入 s_ring 环形队列
 *   → (Task 中) BLUETOOTH_GetCmd 取出解析
 *
 * 注意: s_dma_rx_buf 既是 DMA 缓冲区, 也是我们读取数据的地方。
 * DMA 一直在往这个缓冲区写, 我们在 IDLE 中断中读 —— 这实际上是一个
 * 多生产者 (DMA), 单消费者 (IDLE ISR) 的场景。
 * 由于 DMA 写和 CPU 读访问的是不同位置 (DMA 写最新数据, CPU 读已写好的旧数据),
 * 且 CPU 读操作是原子的 (单字节), 所以不存在竞争条件。
 */
static uint8_t s_dma_rx_buf[BT_RX_BUF_SIZE];

/*
 * 环形队列结构体
 *
 * buf[size]:  存储数据的缓冲区
 * head:       下次写入位置 (也就是下一个可用空位)
 * tail:       下次读取位置 (也就是下一个有效数据)
 *
 * 初始化: head = tail = 0
 *   → 队列空
 * 写入一个字节: 放入 buf[head], head++
 *   → 如果 head 到达 size, 回绕到 0
 * 读取一个字节: 从 buf[tail] 取出, tail++
 *   → 如果 tail 到达 size, 回绕到 0
 * 满的条件: (head + 1) % size == tail
 *   → 因为我们留了一个空位, 所以实际可用容量 = size - 1
 *   为什么留一个空位? 否则空和满都是 head == tail, 无法区分。
 *
 * 队列大小 BT_RING_BUF_SIZE = 512 = 2^9
 * head 和 tail 都是 uint16_t, 但只在 0~511 范围内循环。
 * 用 &(size-1) 而不是 %size 实现快速取模。
 */
typedef struct {
    uint8_t  buf[BT_RING_BUF_SIZE];
    uint16_t head;      /* 写指针: 下一个写入位置 */
    uint16_t tail;      /* 读指针: 下一个读取位置 */
} RingBuf_t;

/*
 * s_ring: 全局环形队列实例
 * s_last_ndtr: 上一次 IDLE 中断时记录的 DMA CNDTR 值,
 *              用于计算两次中断之间的接收数据量。
 */
static RingBuf_t s_ring;
static uint16_t  s_last_ndtr = 0;

/*
 * s_dma_read_pos: 用于追踪在 s_dma_rx_buf 中应该从哪个位置读取数据。
 * 这是另一种实现方式 —— 不用 NDTR 算起始位置, 直接维护一个读取指针。
 * 每次 IDLE 中断: 从 s_dma_read_pos 读取 received 个字节, 然后
 * s_dma_read_pos = (s_dma_read_pos + received) & (BT_RX_BUF_SIZE - 1)
 *
 * 这个变量当前未被使用, 留作参考。
 * 你可以选择用 s_last_ndtr 或 s_dma_read_pos, 两者逻辑等价。
 */
/* static uint16_t s_dma_read_pos = 0; */

/* ---- 环形队列操作函数 (内联, 仅本文件可见) ---- */

/**
 * @brief  初始化环形队列: 头尾指针归零
 * @param  r: 环形队列指针
 *
 * 初始化后: head == tail == 0, 表示队列空。
 * 不需要清空 buf 数组, 因为 head/tail 控制访问范围。
 */
static void ring_init(RingBuf_t *r) { r->head = r->tail = 0; }

/**
 * @brief  获取环形队列中有效数据的字节数
 * @param  r: 环形队列指针
 * @return 有效字节数
 *
 * 公式: (head - tail) & (size - 1)
 * 由于 size = 512 = 2^9, size-1 = 0x1FF
 * head - tail 可能是负数 (在 uint16_t 下回绕), & 0x1FF 截断后取模
 *
 * 示例:
 *   head=10, tail=0 → (10-0)&0x1FF = 10 ✓
 *   head=0, tail=500 (回绕) → (0-500)&0x1FF = (65536-500)&0x1FF = 12 ✓
 *     (实际上 ring_put 不会让 head 落后 tail 这么多, 但公式仍然正确)
 */
static uint16_t ring_avail(RingBuf_t *r)
{
    return (uint16_t)((r->head - r->tail) & (BT_RING_BUF_SIZE - 1));
}

/**
 * @brief  向环形队列写入一个字节
 * @param  r: 环形队列指针
 * @param  c: 要写入的字节
 * @retval 0: 成功, -1: 队列满
 *
 * 写入流程:
 *   1. 计算下一个 head 位置: (head + 1) & (size - 1)
 *   2. 如果 next == tail, 说明队列已满 (再写一个字节就会覆盖未读数据)
 *      返回 -1, 丢弃该字节
 *   3. 否则: 将 c 写入 buf[head], head 前进到 next
 *
 * 队列满时的处理策略:
 *   当前策略: 丢弃新字节 (最常用, 保护已有数据)
 *   替代策略: 覆盖旧数据 (适用于实时数据流, 但可能丢失重要指令)
 *   本项目中的蓝牙指令是"即时性"的, 丢旧指令留新指令可能更合理。
 *   但为了兼容 JSON 解析 (需要完整行), 选择不丢弃 —— 宁可丢掉一个字节,
 *   也绝不破坏环形队列状态。
 */
static int ring_put(RingBuf_t *r, uint8_t c)
{
    uint16_t next = (uint16_t)((r->head + 1) & (BT_RING_BUF_SIZE - 1));
    if (next == r->tail) return -1; /* 满 */
    r->buf[r->head] = c;
    r->head = next;
    return 0;
}

/**
 * @brief  从环形队列读取一个字节
 * @param  r: 环形队列指针
 * @param  c: 输出读取到的字节
 * @retval 0: 成功, -1: 队列空
 *
 * 注意:
 *   ISR (ring_put) 和 Task (ring_get) 操作的指针不同:
 *   ISR 只修改 head, Task 只修改 tail。
 *   所以 ISR 中读 tail 是安全的 (Task 在写), 反过来也一样。
 *   这是环形队列的设计精髓 —— 单生产者 + 单消费者无需锁!
 *
 *   但如果有多任务同时调用 ring_get, 则需要加锁。
 *   本项目中 ring_get 只在 BLUETOOTH_GetCmd 中调用,
 *   而 BLUETOOTH_GetCmd 只在蓝牙任务中调用, 所以不存在多消费者。
 */
static int ring_get(RingBuf_t *r, uint8_t *c)
{
    if (r->tail == r->head) return -1;
    *c = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) & (BT_RING_BUF_SIZE - 1));
    return 0;
}

/* ---- 对外接口 ---- */

void BLUETOOTH_Init(UART_HandleTypeDef *huart)
{
    /*
     * 初始化蓝牙模块
     *
     * 步骤如下:
     *
     * 第 1 步: 保存 USART 句柄, 用于后续发送和中断处理
     */
    s_huart = huart;

    /*
     * 第 2 步: 初始化环形队列
     * 环形队列在 BLUETOOTH_IDLE_IRQHandler 和 BLUETOOTH_GetCmd 之间传递数据。
     */
    ring_init(&s_ring);

    /*
     * 第 3 步: 启动 DMA 循环接收
     *
     * HAL_UART_Receive_DMA(s_huart, s_dma_rx_buf, BT_RX_BUF_SIZE);
     *
     * 函数调用后的内部状态:
     *   - DMA 通道使能, Circular 模式
     *   - USART2 CR3 的 DMAR 位置 1 (DMA 使能接收)
     *   - 之后 USART2 每收到一字节, DMA 自动搬运
     *   - 当 NDTR 减到 0, DMA 自动重新加载计数并回绕地址
     *
     * HAL_UART_Receive_DMA 和 HAL_UART_Receive_IT 的区别:
     *   - _DMA: DMA 后台搬运, CPU 不参与, 速度和 CPU 无关
     *   - _IT: 每字节进一次中断, 9600bps 下 960 次/秒
     *   _DMA 适合大数据量/高速率, _IT 适合小数据量/低速率
     *
     * 注意:
     *   对于本函数, 最后一个参数是缓冲区大小, 告诉 DMA 传多少字节算"完成"。
     *   但因为我们使用 Circular 模式 + IDLE 中断, 所以不需要 DMA 完成中断。
     *   HAL_UART_Receive_DMA 默认会注册 DMA 完成中断回调, 但我们不用它。
     */

    /*
     * 第 4 步: 手动使能 IDLE 中断
     *
     * __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
     *
     * 这个宏做了: 设置 USART CR1 寄存器中的 IDLEIE 位 (bit 4)。
     * 使能后, 当 USART 检测到 RX 线路空闲 (持续高电平超过 1 字节时间),
     * 硬件会:
     *   1. 置位 USART SR 寄存器的 IDLE 标志 (bit 4)
     *   2. 触发 USART 全局中断 (IRQ)
     *   3. CPU 跳转到 USART2_IRQHandler()
     *
     * 注意: HAL 库没有 UART_IT_IDLE 这个中断的处理代码!
     * HAL_UART_IRQHandler 中只处理 RXNE, TXE, TC, PE, ERR 等。
     * IDLE 中断必须我们自己处理。所以不能用 HAL 的框架, 要写自定义代码。
     *
     * 使能位置: 在启动 DMA 之后使能 IDLE, 因为如果先使能 IDLE 再启动 DMA,
     * 可能在 DMA 还没启动时就触发一次 IDLE 中断 (初始为空闲状态), 导致空处理。
     */

    /*
     * 第 5 步: 记录初始 NDTR 值
     *
     * s_last_ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
     *
     * 在 IDLE 中断中, 我们需要计算接收的数据量:
     * received = (s_last_ndtr - ndtr_current) & (BT_RX_BUF_SIZE - 1);
     * 所以必须记录初始 NDTR。
     *
     * __HAL_DMA_GET_COUNTER 宏展开:
     *   ((hdma)->Instance->CNDTR)
     * 直接读取 DMA 通道的 CNDTR 寄存器, 这是一个 16 位只读寄存器。
     */
    s_last_ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    /*
     * 调试:
     * 初始化完成后, 可以用 printf 确认 DMA 是否正常启动:
     * printf("BT init OK, NDTR=%u\r\n", s_last_ndtr);
     * 如果 NDTR = 256, 说明 DMA 启动成功。
     * 如果 NDTR = 0, 说明 DMA 未启动或已传输完成 (不应该, 因为是 Circular)。
     */
}

void BLUETOOTH_Send(const char *json_str)
{
    /*
     * 阻塞式发送 JSON 字符串
     *
     * 函数原型:
     *   HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
     *       uint8_t *pData, uint16_t Size, uint32_t Timeout);
     *
     * 参数:
     *   - s_huart: USART 句柄 (已在 Init 中保存)
     *   - pData: 要发送的数据指针
     *   - Size: 要发送的字节数
     *   - Timeout: 超时时间 (ms), 0 = 无限等待
     *
     * 返回值:
     *   HAL_OK: 发送成功
     *   HAL_TIMEOUT: 超时 (常见于蓝牙未连接, TX 缓冲区一直满)
     *   HAL_ERROR: 参数错误或硬件问题
     *
     * 阻塞机制:
     *   HAL_UART_Transmit 使用"忙等待"方式:
     *   1. 将数据写入 USART TX 数据寄存器
     *   2. 等待 TXE (发送数据寄存器空) 标志置位
     *   3. 写下一个字节
     *   4. 重复直到所有字节发送完毕
     *
     *   在 FreeRTOS 环境下, HAL_UART_Transmit 内部使用 DWT 或 SysTick
     *   计时器实现超时, 不是死循环, 所以不会"卡死"系统。
     *   但如果超时, 函数返回 HAL_TIMEOUT, 发送可能不完整。
     *
     * 为什么不用 DMA 发送?
     *   该项目发送频率低 (约每秒 1 次), 阻塞时间约 156ms (9600bps, 150 字节),
     *   对 200ms 周期的 LCD 任务有影响, 但在可接受范围内。
     *   如果未来需要高频发送 (如 10Hz), 建议改用 HAL_UART_Transmit_DMA。
     *
     * 注意: json_str 必须以 '\0' 结尾 (strlen 依赖于此)。
     *   发送前确认字符串长度不超过 USART 发送缓冲区大小 (无硬件缓冲区限制)。
     *
     * TODO: 取消注释下面的代码:
     * HAL_UART_Transmit(s_huart, (uint8_t *)json_str, strlen(json_str), 1000);
     */
    (void)json_str;
}

void BLUETOOTH_SendSensorData(float temp, float humi, float volt, float curr, float pwr)
{
    /*
     * 构造并发送传感器 JSON 数据
     *
     * JSON 格式:
     *   {"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.250,"pwr":3.01}\r\n
     *
     * 实现步骤:
     *
     * 1. 申请局部缓冲区 (在栈上)
     *    char buf[200];
     *    200 字节足够容纳最大 JSON 包, 栈空间足够 (蓝牙任务栈 1024*4 字节)
     *    为什么不在堆上 malloc? 嵌入式系统尽量避免动态内存分配
     *    (碎片化问题, 以及 FreeRTOS heap_4 可能被其他模块占用)
     *
     * 2. 用 snprintf 格式化
     *    snprintf(buf, sizeof(buf),
     *        "{\"temp\":%.1f,\"humi\":%.1f,\"volt\":%.2f,\"curr\":%.3f,\"pwr\":%.2f}\r\n",
     *        temp, humi, volt, curr, pwr);
     *
     *    重点: 格式说明符的位数选择
     *    - %.1f: AHT20 温度精度 ±0.3°C, 显示 0.1 精度已足够
     *    - %.1f: AHT20 湿度精度 ±2%RH, 同 0.1 精度
     *    - %.2f: INA226 电压分辨率 1.25mV, 显示 0.01V (10mV) 精度合理
     *    - %.3f: INA226 电流分辨率取决于分流电阻, 通常显示到 mA
     *    - %.2f: 功率由 V*I 计算, 精度受限于 V 和 I 的精度
     *
     * 3. 发送
     *    BLUETOOTH_Send(buf);
     *
     * 关于 \r\n 结尾:
     *   手机端 App (如 Serial Bluetooth Terminal) 通常以换行为分隔符。
     *   \r\n 是标准换行符 (CR+LF), 兼容各种终端应用。
     *   只有 \n 可能在某些 App 上不换行 (旧版串口助手)。
     *
     * 关于浮点数打印:
     *   snprintf 的 %f 依赖 newlib nano 的浮点数支持。
     *   如果链接时出现 undefined reference to `_printf_float`,
     *   需要在链接器标志中加 -u _printf_float:
     *     target_link_options(main PRIVATE -Wl,-u,_printf_float)
     *   这会增加约 12KB 固件大小。
     *   替代方案: 自己写整数运算, 如:
     *     int temp_int = (int)(temp * 10);
     *     snprintf(buf, size, "%d.%d", temp_int/10, temp_int%10);
     *   这避免了浮点数库, 但代码更复杂。
     *
     * TODO: 取消注释并填入实现。
     *
     * char buf[200];
     * snprintf(buf, sizeof(buf),
     *     "{\"temp\":%.1f,\"humi\":%.1f,\"volt\":%.2f,\"curr\":%.3f,\"pwr\":%.2f}\r\n",
     *     temp, humi, volt, curr, pwr);
     * BLUETOOTH_Send(buf);
     */
    (void)temp; (void)humi; (void)volt; (void)curr; (void)pwr;
}

uint8_t BLUETOOTH_GetCmd(BT_CmdPacket_t *pkt)
{
    /*
     * 从环形队列中读取一行数据并进行 JSON 解析
     *
     * 这是一个非阻塞函数 —— 没有命令时立即返回 0。
     * 调用者应每 10-50ms 调用一次 (在任务循环中), 或使用信号量等待通知。
     *
     * 实现步骤:
     *
     * 1. 初始化输出参数
     *    pkt->cmd = BT_CMD_NONE;
     *    pkt->r = pkt->g = pkt->b = 0;
     *    确保输出结构体在解析失败时也是干净状态
     *
     * 2. 声明局部变量
     *    char line[128];     // 命令行缓冲区, 足够容纳命令 JSON
     *    uint16_t line_len = 0;  // 当前行长度
     *    uint8_t ch;
     *
     * 3. 循环从环形队列取字节
     *    while (ring_get(&s_ring, &ch) == 0) {
     *        // 队列中还有数据, 取出一个字节
     *
     *        // 跳过 \r (回车字符, 与 \n 成对出现)
     *        if (ch == '\r') continue;
     *
     *        if (ch == '\n') {
     *            // 一行结束, 开始解析
     *            line[line_len] = '\0';  // 字符串终止
     *            break;
     *        }
     *
     *        // 普通字符: 追加到 line 缓冲区
     *        if (line_len < sizeof(line) - 1) {
     *            line[line_len++] = ch;
     *        } else {
     *            // 缓冲区满: 丢弃已累积的数据, 重置 (防止恶意超长输入)
     *            // 这是安全措施: 如果有人用超长数据攻击, 不会导致栈溢出
     *            line_len = 0;
     *        }
     *    }
     *
     *    // 如果队列没数据或没收到完整一行
     *    if (line_len == 0) return 0;  // 无完整命令
     *
     * 4. JSON 解析 (简易方案, 不需要 cJSON 库)
     *    // 搜索命令类型
     *    if (strstr(line, "\"get_temp\"")) {
     *        pkt->cmd = BT_CMD_GET_TEMP;
     *    } else if (strstr(line, "\"get_all\"")) {
     *        pkt->cmd = BT_CMD_GET_ALL;
     *    } else if (strstr(line, "\"led\"")) {
     *        pkt->cmd = BT_CMD_SET_LED;
     *
     *        // 提取 LED 颜色参数
     *        // 格式: {"cmd":"led","r":255,"g":0,"b":0}
     *        // 用 strstr 找到 "r": 然后跳过 3 个字符, atoi 读数字
     *        char *p;
     *        if ((p = strstr(line, "\"r\":")) != NULL) {
     *            pkt->r = atoi(p + 4);  // 跳过 "r": 共 4 个字符
     *        }
     *        if ((p = strstr(line, "\"g\":")) != NULL) {
     *            pkt->g = atoi(p + 4);
     *        }
     *        if ((p = strstr(line, "\"b\":")) != NULL) {
     *            pkt->b = atoi(p + 4);
     *        }
     *    } else {
     *        pkt->cmd = BT_CMD_UNKNOWN;
     *    }
     *
     * 5. 返回 1 表示有新命令
     *    return 1;
     *
     * atoi 安全性:
     *   如果 "r":"xxx" 中的 xxx 不是数字, atoi 返回 0。
     *   所以 pkt->r 默认为 0, 安全的"关灯"值。
     *   更严谨的做法: 检查 atoi 之前, p+4 是否真的是数字 (isdigit)。
     *   但对于本项目, atoi 的默认行为就够用了。
     *
     * 为什么不直接用 cJSON?
     *   cJSON 是一个轻量级 (约 3KB 代码) 的 JSON 解析库,
     *   支持嵌套对象、数组、字符串转义等。
     *   但我们的命令只有 3 种, 格式固定, 用 strstr + atoi 完全足够。
     *   引入 cJSON: 固件增大 ~3KB, 使用动态内存 malloc。
     *   不引入: 代码简单, 零动态分配, 零 bug 可能。
     *   这是典型的"够用就行"原则。
     *
     * 调试技巧:
     *   如果命令解析总是不对, 试试打印原始行:
     *     printf("[BT] raw: '%s'\r\n", line);
     *   常见问题:
     *     1. line 中残留 \r (前面没跳过) → strstr 匹配不上
     *     2. 大小写不一致: 手机端发了 "Get_Temp" 而非 "get_temp"
     *     3. 多余空格: "{\"cmd\":  \"get_temp\"}" → strstr 仍可匹配, 但
     *        如果用了更严格的解析逻辑就可能失败
     */
    (void)pkt;
    return 0;
}

void BLUETOOTH_IDLE_IRQHandler(void)
{
    /*
     * USART2 空闲中断处理函数
     *
     * 此函数在 USART2_IRQHandler 中断上下文中调用。
     * 必须极快执行 (几微秒), 不能有:
     *   - printf / HAL_UART_Transmit (可能重入)
     *   - malloc / free (非中断安全)
     *   - 任何信号量/队列操作 (FreeRTOS 中断级 API 调用有特殊限制:
     *     只能调用 FromISR 结尾的函数, 且优先级 < configMAX_SYSCALL_INTERRUPT_PRIORITY)
     *   - 长时间循环 (会阻塞其他中断)
     *
     * 中断服务时间要求:
     *   蓝牙数据率 9600bps, 两个字节间隔约 1.04ms。
     *   理论上中断必须在 1.04ms 内完成, 否则会丢失下一个字节。
     *   实际上我们的 ISR 只做数据搬移 (将 DMA 缓冲区的数据搬到环形队列),
     *   耗时 < 10μs, 完全满足要求。
     *
     * 实现步骤:
     *
     * 1. 检查是否是 IDLE 中断
     *    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET) {
     *        return;  // 不是 IDLE 中断, 不处理
     *    }
     *    UART_FLAG_IDLE = ((USART_SR_IDLE)) = (1 << 4)
     *    RESET = 0
     *    __HAL_UART_GET_FLAG: (USART->SR & FLAG_MASK) 不为 0 则触发
     *
     * 2. 清除 IDLE 标志
     *    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
     *
     *    STM32F4 参考手册 (RM0090) 第 26.3.5 节:
     *    清除 IDLE 位的操作序列:
     *    1) 读 USART_SR 寄存器 (访问 SR)
     *    2) 读 USART_DR 寄存器 (访问 DR)
     *
     *    __HAL_UART_CLEAR_IDLEFLAG 宏展开:
     *      __HAL_UART_CLEAR_FLAG(__HANDLE__, UART_FLAG_IDLE)
     *      实际做了: USART->SR; USART->DR
     *
     *    另一种等效写法 (更直接, 不依赖 HAL 宏):
     *      volatile uint32_t tmp;
     *      tmp = s_huart->Instance->SR;  // 读 SR
     *      tmp = s_huart->Instance->DR;  // 读 DR, 清除 IDLE
     *      (void)tmp;  // 防止编译器告警 "未使用的变量"
     *
     * 3. 获取 DMA 当前 CNDTR
     *    uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
     *    // 直接读取 DMA 通道的 CNDTR 寄存器
     *    // 这是硬件寄存器, 读取无需等待, 单周期操作
     *
     * 4. 计算本次接收的字节数
     *    uint16_t received;
     *    if (ndtr < s_last_ndtr) {
     *        // 没有回绕: DMA 从上一位置继续往前搬运
     *        received = s_last_ndtr - ndtr;
     *    } else if (ndtr > s_last_ndtr) {
     *        // 回绕了: DMA 写到底部, 又回到开头继续写
     *        received = (BT_RX_BUF_SIZE - s_last_ndtr) + ndtr;
     *    } else {
     *        // ndtr == s_last_ndtr, 没有新数据
     *        received = 0;
     *    }
     *    s_last_ndtr = ndtr;
     *
     *    或者用一行公式 (更简洁, 但需要理解位运算):
     *    uint16_t received = (uint16_t)((s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1));
     *    s_last_ndtr = ndtr;
     *
     * 5. 如果没有收到新数据, 直接返回
     *    if (received == 0 || received > BT_RX_BUF_SIZE) return;
     *
     * 6. 计算数据在 s_dma_rx_buf 中的起始读取位置
     *    uint16_t start_pos;
     *    if (ndtr == 0) {
     *        // NDTR 为 0: DMA 刚完成一轮循环
     *        start_pos = 0;
     *    } else {
     *        start_pos = BT_RX_BUF_SIZE - ndtr;
     *    }
     *
     *    简化: start_pos = (BT_RX_BUF_SIZE - ndtr) % BT_RX_BUF_SIZE
     *    但取模运算慢, 当 BT_RX_BUF_SIZE 是 2 的幂时:
     *    start_pos = (BT_RX_BUF_SIZE - ndtr) & (BT_RX_BUF_SIZE - 1)
     *    不过这里的 ndtr 是当前值 (不是差值), 需要小心处理。
     *
     *    另一种更直观的方式: 用一个独立的 "上次读取位置" 变量
     *    从上次读到的位置读取 received 字节:
     *    uint16_t end_pos = (s_dma_read_pos + received) & (BT_RX_BUF_SIZE - 1);
     *    for (uint16_t i = s_dma_read_pos; i != end_pos; i = (i+1) & mask) {
     *        ring_put(&s_ring, s_dma_rx_buf[i]);
     *    }
     *    s_dma_read_pos = end_pos;
     *
     * 7. 将数据从 s_dma_rx_buf 推入环形队列
     *    // 注意: 数据可能跨越缓冲区末尾, 需要分段拷贝
     *    uint16_t mask = BT_RX_BUF_SIZE - 1;
     *    uint16_t start = ...;  // 计算起始位置
     *    for (uint16_t i = 0; i < received; i++) {
     *        uint16_t pos = (start + i) & mask;
     *        if (ring_put(&s_ring, s_dma_rx_buf[pos]) != 0) {
     *            // 环形队列满了 → 丢弃多余数据
     *            // 可以设置一个溢出标志, 供调试使用
     *            break;
     *        }
     *    }
     *
     * 8. ISR 结束
     *
     * 性能估计:
     *   收 80 字节 JSON 包 → for 循环 80 次 → 80 * (1 次 & + 1 次 ring_put)
     *   → 约 80 * 12 个 CPU 周期 = 960 周期 @ 168MHz ≈ 5.7μs
     *   → 完全满足实时性要求
     *
     * 常见错误:
     *   ❌ 在 ISR 中调用 HAL_UART_Transmit (调试打印) → 死锁! 因为 USART 可能正忙
     *   ❌ 在 ISR 中使用信号量/队列的普通 API → 需要用 FromISR 版本
     *   ❌ 在 ISR 中调用 printf → printf 可能使用相同的 USART1 (调试口), 造成重入
     *   ✓ 正确做法: ISR 只搬数据, 处理交给 Task
     *
     * 进阶: 使用 FreeRTOS 信号量通知任务
     *   当有蓝牙数据时, 可以在 ISR 末尾发送一个信号量, 唤醒等待中的蓝牙处理任务。
     *   这样可以避免任务轮询 (每 10ms 调用 BLUETOOTH_GetCmd 检查一次)。
     *
     *     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     *     xSemaphoreGiveFromISR(xSemaphore_BT_Rx, &xHigherPriorityTaskWoken);
     *     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
     *
     *   注意: xSemaphore_BT_Rx 需要在 main.c 中创建。
     *   这种方式比轮询更高效 (任务有数据才运行), 但需要额外创建信号量。
     */
}

/*
 * 总结: 本模块的核心设计思想
 *
 * 1. ISR 最小化原则: IDLE 中断只做数据搬移 (DMA buf → 环形队列)
 *    不做协议解析, 不做数据处理, 不调用操作系统 API (除信号量通知外)
 *
 * 2. DMA 全程运行: 不启动, 不停止, 不回滚
 *    这是 DMA + IDLE 方案的精髓 —— 数据流是"流过"缓冲区的
 *
 * 3. 环形队列解耦: 生产者 (ISR) 和消费者 (Task) 共享的 FIFO
 *    单生产者 + 单消费者无需互斥锁
 *
 * 4. 简易 JSON 解析: 不引入 cJSON, strstr + atoi 满足需求
 *    保持固件精简, 减少动态内存使用
 *
 * 5. 阻塞发送: 低频场景 (1 包/秒) 下简单可靠
 */
