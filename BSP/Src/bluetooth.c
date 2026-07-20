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

    /* 第 2 步: 初始化环形队列 */
    ring_init(&s_ring);

    /* 第 3 步: 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(s_huart, s_dma_rx_buf, BT_RX_BUF_SIZE);

    /* 第 4 步: 手动使能 IDLE 中断 */
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    /* 第 5 步: 记录初始 NDTR 值 */
    s_last_ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
}

void BLUETOOTH_Send(const char *json_str)
{
    if (s_huart != NULL && json_str != NULL) {
        HAL_UART_Transmit(s_huart, (uint8_t *)json_str, strlen(json_str), 1000);
    }
}

void BLUETOOTH_SendSensorData(float temp, float humi, float volt, float curr, float pwr)
{
    char buf[200];
    snprintf(buf, sizeof(buf),
        "{\"temp\":%.1f,\"humi\":%.1f,\"volt\":%.2f,\"curr\":%.3f,\"pwr\":%.2f}\r\n",
        (double)temp, (double)humi, (double)volt, (double)curr, (double)pwr);
    BLUETOOTH_Send(buf);
}

uint8_t BLUETOOTH_GetCmd(BT_CmdPacket_t *pkt)
{
    char line[128];
    uint16_t line_len = 0;
    uint8_t ch;

    pkt->cmd = BT_CMD_NONE;
    pkt->r = pkt->g = pkt->b = 0;

    /* 从环形队列读取直到遇到换行符 */
    while (ring_get(&s_ring, &ch) == 0) {
        if (ch == '\r') continue;          /* 跳过回车 */
        if (ch == '\n') {                  /* 一行结束 */
            line[line_len] = '\0';
            break;
        }
        if (line_len < sizeof(line) - 1) {
            line[line_len++] = (char)ch;
        }
    }

    if (line_len == 0) return 0;           /* 无完整命令 */

    /* 简易 JSON 解析 */
    if (strstr(line, "\"get_temp\"")) {
        pkt->cmd = BT_CMD_GET_TEMP;
    } else if (strstr(line, "\"get_all\"")) {
        pkt->cmd = BT_CMD_GET_ALL;
    } else if (strstr(line, "\"led\"")) {
        pkt->cmd = BT_CMD_SET_LED;
        char *p;
        if ((p = strstr(line, "\"r\":")) != NULL) pkt->r = atoi(p + 4);
        if ((p = strstr(line, "\"g\":")) != NULL) pkt->g = atoi(p + 4);
        if ((p = strstr(line, "\"b\":")) != NULL) pkt->b = atoi(p + 4);
    } else {
        pkt->cmd = BT_CMD_UNKNOWN;
    }
    return 1;
}

void BLUETOOTH_IDLE_IRQHandler(void)
{
    /* 1. 检查是否是 IDLE 中断 */
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET) {
        return;
    }

    /* 2. 清除 IDLE 标志 (读 SR → 读 DR) */
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    /* 3. 获取 DMA 当前 CNDTR */
    uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    /* 4. 计算接收字节数 (用位运算处理回绕) */
    uint16_t received = (uint16_t)((s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1));
    s_last_ndtr = ndtr;

    if (received == 0 || received > BT_RX_BUF_SIZE) return;

    /* 5. 计算起始位置: DMA 当前写位置 */
    uint16_t start = (uint16_t)(BT_RX_BUF_SIZE - ndtr) & (uint16_t)(BT_RX_BUF_SIZE - 1U);
    uint16_t mask = BT_RX_BUF_SIZE - 1U;

    /* 6. 将数据推入环形队列 */
    for (uint16_t i = 0; i < received; i++) {
        uint16_t pos = (uint16_t)(start + i) & mask;
        if (ring_put(&s_ring, s_dma_rx_buf[pos]) != 0) {
            break;  /* 队列满, 丢弃剩余数据 */
        }
    }
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
