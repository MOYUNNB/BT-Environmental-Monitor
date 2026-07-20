# 蓝牙无线环境监测与数据记录仪 — 面试笔记

> 记录项目中遇到的实际问题、解决方案和工程思考。
> 面试官问"你做过什么"时，讲这些比罗列功能强得多。

---

## 目录

1. [I2C 多设备冲突与互斥锁设计](#1-i2c-多设备冲突与互斥锁设计)
2. [SPI 总线共享与 CS 管理](#2-spi-总线共享与-cs-管理)
3. [DMA + IDLE 不定长接收（蓝牙）](#3-dma--idle-不定长接收蓝牙)
4. [WS2812 PWM+DMA 时序问题](#4-ws2812-pwmdma-时序问题)
5. [FreeRTOS 任务优先级与 timing budget](#5-freertos-任务优先级与-timing-budget)
6. [AHT20 80ms 延时问题](#6-aht20-80ms-延时问题)
7. [EC11 编码器查表法解码](#7-ec11-编码器查表法解码)
8. [TF 卡 FATFS 掉电保护](#8-tf-卡-fatfs-掉电保护)
9. [BSP 驱动的接口设计模式](#9-bsp-驱动的接口设计模式)
10. [从 CubeMX 骨架到完整系统](#10-从-cubemx-骨架到完整系统)

---

## 1. I2C 多设备冲突与互斥锁设计

### 背景

6 个 I2C 设备（AHT20、INA226、SD3078、AT24C02、PCA9555PW、ES8388）共用一条 I2C1 总线。FreeRTOS 多任务环境下，如果不加保护，两个任务同时访问 I2C 会造成数据错乱。

### 问题

- 任务 A 读 AHT20 到一半，任务 B 开始写 INA226，总线数据交叉
- 用 `__disable_irq()` 关中断会破坏 RTOS 的实时性

### 方案：互斥信号量

```c
osMutexId_t xSemaphore_I2CHandle;

// I2C 驱动内部的锁封装:
static void i2c_lock(void) {
    if (s_sem != NULL)
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}
static void i2c_unlock(void) {
    if (s_sem != NULL)
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
}
```

### 面试亮点

- **为什么用信号量而不是关中断？** 关中断会影响所有任务，而信号量只阻塞试图访问 I2C 的任务，高优先级任务（10ms 按键扫描）仍然可以运行
- **`void *pSemaphore` 的设计意图？** BSP 层不依赖 CMSIS-RTOS 头文件，传入 NULL 时不加锁，支持裸机调试
- **加锁粒度控制：** INA226 的 `ReadAll()` 一次锁内读 3 个寄存器，保证电压/电流/功率来自同一转换周期

---

## 2. SPI 总线共享与 CS 管理

### 背景

ICM-42688（IMU）和 W25Q128（Flash）共享 SPI2 总线，各有一个独立的 CS 引脚（PE7 和 PE4）。

### 问题

两个任务可能同时在 SPI2 上操作不同的设备，MISO/MOSI 数据冲突。

### 方案

1. 创建 `xSemaphore_SPI2` 互斥信号量（框架中原本没有，是开发过程中发现的遗漏）
2. 每次 SPI 事务在 `spi_lock()/unlock()` 保护中进行
3. CS 在整个事务期间保持低电平，事务结束后拉高

### 面试亮点

- **SPI 共享 vs SPI 独占：** LCD（SPI1）独占一条总线，不需要互斥锁；SPI2 共享需要
- **片选管理：** `cs_select()` 和 `cs_deselect()` 确保每个设备只在属于自己的 CS 有效时响应

---

## 3. DMA + IDLE 不定长接收（蓝牙）

### 背景

蓝牙模块 XW040 通过 USART2（9600bps）发送不定长 JSON 数据包（30~200 字节）。

### 问题

- 每字节中断方式 9600bps 下每秒 960 次中断，虽然 CPU 占用不高，但不够优雅
- 定长 DMA 无法处理不定长的蓝牙数据包（不知道何时结束）

### 方案：DMA Circular + IDLE 中断

```
USART2 RX → DMA Circular (自动搬移到 s_dma_rx_buf)
    → 总线空闲 → IDLE 中断触发
    → 计算 NDTR 差值得到接收长度
    → 数据推入环形队列
    → 任务层从队列取出解析 JSON
```

```c
void BLUETOOTH_IDLE_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET) return;
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    uint16_t received = (s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1);
    s_last_ndtr = ndtr;

    // 从 DMA 缓冲区搬到环形队列
    for (uint16_t i = 0; i < received; i++)
        ring_put(&s_ring, s_dma_rx_buf[(start + i) & mask]);
}
```

### 面试亮点

- **NDTR 差值法的数学原理：** 为什么 `(prev - cur) & (size-1)` 能正确处理 DMA 回绕？（size 是 2 的幂，减法回绕 + 掩码 = 模运算）
- **环形队列的单生产者-单消费者无需锁：** ISR 只写 head，Task 只读 tail，无竞争条件
- **IDLE 中断 vs 帧头帧尾检测：** IDLE 利用物理层空闲检测，比协议层帧检测更简单可靠

---

## 4. WS2812 PWM+DMA 时序问题

### 背景

WS2812 使用单线归零码，通过 TIM5_CH4 的 PWM + DMA 输出精确时序。

### 问题 1：TIM5 Period 错误

CubeMX 生成的 TIM5 Period = 209，得到 400KHz PWM。但 WS2812 需要 800KHz。

- **修正前：** `htim5.Init.Period = 209` → 84MHz / (209+1) = 400KHz（错误）
- **修正后：** `htim5.Init.Period = 104` → 84MHz / (104+1) = 800KHz（正确）

### 问题 2：CubeMX 没有配置 DMA

这是导致系统卡死的根本原因。`HAL_TIM_PWM_Start_DMA()` 返回 `HAL_ERROR`，但返回值被忽略。`s_busy` 被设为 1 后永远无法清除，`WS2812_WaitReady()` 陷入死循环。

**修复：**

```c
if (HAL_TIM_PWM_Start_DMA(...) != HAL_OK) {
    s_busy = 0;  // 防止死锁
}
```

同时手动添加 DMA 配置：

```c
// stm32f4xx_hal_msp.c 中
DMA_HandleTypeDef hdma_tim5_ch4;
hdma_tim5_ch4.Instance = DMA2_Stream5;
hdma_tim5_ch4.Init.Channel = DMA_CHANNEL_7;
hdma_tim5_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
// ...
__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC4], hdma_tim5_ch4);
```

### 面试亮点

- **PWM + DMA 比软件延时好在哪里？** CPU 零占用，时序精确不受中断影响
- **GRB 不是 RGB：** WS2812 的像素格式是 GRB（先绿后红再蓝），和常见的 RGB 顺序不同，写错颜色全乱
- **复位脉冲的作用：** 低电平 > 50µs 锁存颜色，DMA 完成后必须停止 PWM 输出

---

## 5. FreeRTOS 任务优先级与 timing budget

### 背景

7 个任务共享 CPU，优先级分配基于 Rate Monotonic Scheduling（RMS）原则：周期越短，优先级越高。

### 优先级分配

| 任务 | 周期 | 优先级 | 理由 |
|------|------|--------|------|
| 按键扫描 | 10ms | High4 | 最高，消抖需要严格 10ms 周期 |
| 传感器采集 | 100ms | AboveNormal3 | 含 AHT20 的 80ms 延时，必须最高 |
| LCD 刷新 | 200ms | Normal2 | 视觉平滑即可，可被抢占 |
| 蓝牙推送 | 1000ms | Normal2 | 低频，和 LCD 同级 |
| TF 日志 | 1000ms | BelowNormal1 | 最慢，FATFS 写可能阻塞 |
| WS2812 | 1000ms | Low | 颜色变化慢，优先级最低 |
| defaultTask | - | Normal | 系统监控 |

### 面试亮点

- **AHT20 的 80ms 占空比问题：** 80ms 延时 + 20ms 其他工作 ≈ 85% 占空比。`osDelay(80)` 会挂起任务让出 CPU，其他任务在此期间运行，所以高占空比不意味着 CPU 满载
- **为什么 WS2812 优先级最低？** 颜色映射变化可以容忍延迟，即使被抢占 1~2 秒也不影响功能

---

## 6. AHT20 80ms 延时问题

### 背景

AHT20 的读取流程：触发测量（发 0xAC）→ 等待 80ms → 读 6 字节数据。

### 问题

`osDelay(80)` 在传感器读取任务中阻塞了 80ms。虽然 `osDelay` 让出 CPU，但任务被唤醒后立即继续，其他低优先级任务（如 TF 日志）可能无法及时运行。

### 方案

- 当前：简单的 `osDelay(80)` 方案，因为温湿度变化慢，80ms 固定延时对系统整体影响小
- 可选优化：改用状态机，在 80ms 等待期间执行其他传感器的读取

### 面试亮点

- **三种等待方式的权衡：**
  1. `osDelay(80)` — 简单，让出 CPU，但唤醒后只有少量工作
  2. 轮询 bit7 — 可提前返回，但增加 I2C 总线负载
  3. 中断 — AHT20 无此功能
- 选方案 1 的原因是"温湿度变化慢，80ms 对 ms 级系统不敏感"

---

## 7. EC11 编码器查表法解码

### 背景

EC11 旋转编码器产生正交信号（A、B 相），相位差 90°。通过检测相位关系判断旋转方向。

### 方案：查表法

```c
// 2-bit 组合 AB，4 种状态 × 4 种状态 = 16 种输入
static const int8_t ec11_table[16] = {
     0,  1, -1,  0,   /* 00 → 00,01,10,11 */
    -1,  0,  0,  1,   /* 01 → 00,01,10,11 */
     1,  0,  0, -1,   /* 10 → 00,01,10,11 */
     0, -1,  1,  0,   /* 11 → 00,01,10,11 */
};

uint8_t ab = ec11_read_ab();
uint8_t idx = (last_ab << 2) | ab;
s_delta += ec11_table[idx];  // +1 = CW, -1 = CCW, 0 = 无变化/非法
```

### 面试亮点

- **为什么用查表法而不是 if-else？** 常数时间，无分支预测失败，代码简洁
- **非法状态如何处理？** 两个 bit 同时变化（索引 3、6、9、12）意味着采样间隔太长或旋转太快，返回 0 忽略
- **为什么需要读后清零？** `EC11_GetDelta()` 返回累计步数并清零，防止步数丢失

---

## 8. TF 卡 FATFS 掉电保护

### 背景

传感器数据每秒写入 TF 卡 CSV 文件。如果突然掉电，最后几秒的数据可能丢失。

### 方案

```c
// 每秒调用 f_sync 刷缓冲区
TF_LogSensor(...) {
    f_printf(&file, "%s,%.1f,%.1f,...\r\n", ...);
    f_sync(&file);  // 强制刷写到 SD 卡
}
```

### 面试亮点

- **f_sync  vs f_close：** f_sync 刷新缓冲区但不关闭文件，f_close 既刷又关。每秒调 f_sync，掉电最多丢 1 行数据
- **文件按天分割：** 用 RTC 生成文件名 `2026-07-20.csv`，避免单文件过大，方便数据管理
- **CSV 格式的好处：** 秒级写入、Excel 直接打开、手机蓝牙 App 也能解析

---

## 9. BSP 驱动的接口设计模式

### 模板

每个 I2C/SPI 设备驱动遵循统一的接口模式：

```
.h 文件:
  - 地址宏 (#define DEV_ADDR)
  - 寄存器/命令宏
  - 返回值枚举 (DEV_OK, DEV_ERR_I2C, ...)
  - 函数声明: Init(...), Read*(...)

.c 文件:
  - 静态变量: s_handle, s_sem
  - i2c_lock() / i2c_unlock() 封装
  - 内部读写辅助函数
  - Init: 保存句柄 → 复位 → 配置 → 验证
  - Read: 加锁 → 读寄存器 → 换算 → 解锁
```

### 面试亮点

- **`void *pSemaphore` 的设计：** BSP 层不直接引用 CMSIS-RTOS 头文件，依赖倒置原则
- **自定义枚举返回值：** 不暴露 `HAL_StatusTypeDef` 给应用层，解耦
- **读操作用输出参数：** `XXX_ReadData(float *value)`，返回值表示成功/失败
- **所有 Init 可重入：** 多次调用不会产生副作用

---

## 10. 从 CubeMX 骨架到完整系统

### 项目起点

CubeMX 生成了 HAL 配置 + FreeRTOS 任务框架 + FATFS 中间件，但：
- 6 个 I2C 驱动已经移植好（从现有代码复制）
- 6 个核心驱动是空的（LCD、IMU、WS2812、蓝牙、按键、TF 卡）
- 7 个 FreeRTOS 任务体是空的（`osDelay(1)` 死循环）
- 整个 App 层是 TODO 注释

### 遇到的问题汇总

| 问题 | 根因 | 修复 |
|------|------|------|
| 系统启动后卡死 | WS2812 的 DMA 未配置，`HAL_TIM_PWM_Start_DMA` 失败后 `s_busy` 卡死 | 手动添加 DMA2_Stream5 配置 + 失败时清 `s_busy` |
| TIM5 PWM 频率错误 | CubeMX 默认 Period=209（400KHz），WS2812 需要 800KHz | Period 改为 104 |
| I2C 队列大小=1字节 | CubeMX 生成 `sizeof(uint8_t)` 作为队列元素大小 | 改为 `sizeof(SensorData_t)` |
| SPI2 无互斥锁 | ICM42688 和 W25Q128 共享 SPI2 总线 | 添加 `xSemaphore_SPI2` |
| printf 浮点不显示 | newlib nano 默认禁用 `%f` | 链接选项 `-u _printf_float` |

### 面试亮点

- **开发流程：** 先修编译问题 → 逐个补齐 BSP 驱动 → App 层数据流 → 任务体实现
- **验证方法：** printf 调试输出每条驱动初始化的 OK/FAIL，快速定位问题模块
- **文档驱动：** 参考嘉立创示例代码 + 芯片数据手册，不是从零写代码

---

## 面试官可能追问

### Q1：为什么用 FreeRTOS 而不是裸机？

6 个传感器 + LCD + 蓝牙 + TF 卡 + WS2812，裸机主循环难以组织。FreeRTOS 提供：
- 任务隔离（一个任务卡住不会死机）
- 信号量/队列（I2C 互斥、数据发布-订阅）
- 优先级调度（10ms 按键不被 1s 的 TF 写卡阻塞）

### Q2：6 个 I2C 设备如何防止冲突？

`xSemaphore_I2C` 互斥信号量。每个驱动在 Init 时收到信号量指针，每次 I2C 操作前后加锁/解锁。

### Q3：蓝牙接收数据丢失怎么办？

环形队列 512 字节（最大 JSON 包的 6 倍以上），IDLE 中断只搬数据不做解析，防止 ISR 占用过久。如果队列满，丢弃新字节保护已有数据。

### Q4：为什么不用 cJSON？

命令只有 3 种，格式固定，`strstr` + `atoi` 完全够用。不引入 cJSON 避免：
- 动态内存分配（malloc 碎片化）
- 固件增大 ~3KB
- 额外的 bug 可能

### Q5：最难的 bug 是什么？

WS2812 的 DMA 配置缺失问题。输出表现为"上电后 LCD 亮了一下就死机"，排查过程：
1. printf 定位到 WS2812_Init 卡住
2. 分析 WS2812_WaitReady() 死循环
3. 发现 s_busy 没有被清除
4. 回溯到 HAL_TIM_PWM_Start_DMA 返回错误
5. 查看 CubeMX 发现 TIM5 没有配置 DMA
6. 手动添加 DMA2_Stream5 配置 + 中断处理
