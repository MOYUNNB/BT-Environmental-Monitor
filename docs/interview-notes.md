# 蓝牙无线环境监测与数据记录仪 — 面试笔记 (STAR 法则)

> 面试官问"你做过什么"时，用 STAR 讲故事比罗列功能强得多。
> 每个故事按 **S**ituation（背景）→ **T**ask（任务）→ **A**ction（行动）→ **R**esult（结果）组织。

---

## 目录

1. [WS2812 三层嵌套 Bug 排查](#1-ws2812-三层嵌套-bug-排查)
2. [I2C 多设备冲突与互斥锁设计](#2-i2c-多设备冲突与互斥锁设计)
3. [SPI 总线共享与 CS 管理](#3-spi-总线共享与-cs-管理)
4. [DMA + IDLE 不定长接收（蓝牙）](#4-dma--idle-不定长接收蓝牙)
5. [FreeRTOS 任务优先级与 timing budget](#5-freertos-任务优先级与-timing-budget)
6. [AHT20 80ms 延时问题](#6-aht20-80ms-延时问题)
7. [EC11 编码器查表法解码](#7-ec11-编码器查表法解码)
8. [TF 卡 FATFS 掉电保护](#8-tf-卡-fatfs-掉电保护)
9. [BSP 驱动的接口设计模式](#9-bsp-驱动的接口设计模式)
10. [LCD 背光控制与上电时序优化](#10-lcd-背光控制与上电时序优化)
11. [从 CubeMX 骨架到完整系统](#11-从-cubemx-骨架到完整系统)
12. [按键消抖状态机重写 — 页面自己跳的根因](#12-按键消抖状态机重写--页面自己跳的根因)
13. [LCD SPI 行缓冲优化 — IMU 柱状图不实时](#13-lcd-spi-行缓冲优化--imu-柱状图不实时)

---

## 1. WS2812 三层嵌套 Bug 排查

> 整个项目中最出彩的 debugging 故事，涉及 **3 层嵌套 bug**，每修复一层暴露下一层。
> 面试官最可能问的话题，强烈建议反复练习。

### S — Situation

项目使用 3 颗 WS2812 RGB LED 作为温度指示，驱动方案是 **TIM5_CH4 的 PWM + DMA** 输出 800KHz 归零码。CubeMX 生成了工程骨架，我在 `App_Init()` 中调用 `WS2812_Init()`。

### T — Task

让 WS2812 正常工作，上电后所有灯熄灭，后续根据温度显示不同颜色。

### A — Action

#### 第一层：系统卡死，DMA 未配置

**现象：** 下载程序后串口输出 `[WS2812] Init... ` 后卡死，约 10 秒后看门狗复位。

**排查过程：**
1. 在 `WS2812_Init()` 内部逐行加 printf，定位到 `WS2812_WaitReady()` 死循环
2. `s_busy` 始终为 1，说明 `HAL_TIM_PWM_Start_DMA()` 失败了但没报告
3. 查 CubeMX 配置 → **DMA2_Stream5 完全没有生成**（CubeMX 对 TIM5_CH4 的 DMA 支持有 bug）
4. `HAL_TIM_PWM_Start_DMA` 返回 `HAL_ERROR`，但原始代码忽略了返回值

**修复：**
```c
// 1. 手动在 stm32f4xx_hal_msp.c 添加 DMA 配置
DMA_HandleTypeDef hdma_tim5_ch4;
hdma_tim5_ch4.Instance = DMA2_Stream5;
hdma_tim5_ch4.Init.Channel = DMA_CHANNEL_7;
// ... MEM2PERIPH, WORD, NORMAL ...

// 2. 在 WS2812_Update() 中处理 DMA 启动失败
if (HAL_TIM_PWM_Start_DMA(...) != HAL_OK) {
    s_busy = 0;  // 防止死锁
}
```

#### 第二层：DMA 启动了，但传输永不完成

**现象：** 修复第一层后，printf 显示 `HAL_TIM_PWM_Start_DMA` 返回了 `HAL_OK`，但 `s_busy` 仍然一直 = 1，还是在 `WS2812_WaitReady()` 卡死。

**排查过程：**
1. 继续加 printf，发现 DMA 配置成功，但 **TIM5 的计数器从未开始计数**
2. 深入读 HAL 源码发现：`HAL_TIM_PWM_Start_DMA` 内部调用 `__HAL_TIM_ENABLE` 的条件是检测到 TIM 的某个模式寄存器，但 CubeMX 生成的配置导致这个条件不满足
3. TIM5 不计数 → 没有 PWM 更新事件 → DMA 请求永远不会发出 → 传输永远不完成

**修复：**
```c
void WS2812_Init(void)
{
    // 显式启动 TIM 计数器和 PWM 输出
    HAL_TIM_Base_Start(WS2812_TIM);    // ← 关键修复
    HAL_TIM_PWM_Start(WS2812_TIM, WS2812_CHANNEL);
    WS2812_Update();
    WS2812_WaitReady();
}
```

#### 第三层（后续发现）：传感器信号量传参错误

**现象：** 修好 WS2812 后，启动流程继续，传感器初始化串口输出全 OK，但 `StartSensorRead` 任务第一次运行就崩溃。

**根因：** 信号量传参多了一层间接引用：
```c
// ❌ 错误: osMutexId_t 本身就是 void*，再加 & 是传指针的地址
AHT20_Init(&hi2c1, (void *)&xSemaphore_I2CHandle);

// ✅ 正确: 直接传指针值
AHT20_Init(&hi2c1, (void *)xSemaphore_I2CHandle);
```

`i2c_lock()` 内部把 `s_sem` 当 `osSemaphoreId_t` 使用。传了 `&handle` 后，`osSemaphoreAcquire` 拿到的是"指针的地址"而不是"指针的值"，访问了无效内存。

**为什么 App_Init 阶段没崩溃？** 那时 `xSemaphore_I2CHandle` 还是 NULL（互斥量还没创建），取地址得到的是全局变量的地址（非 NULL），`i2c_lock` 尝试获取一个不存在的信号量 → 行为未定义但没立即崩溃。内核启动后信号量创建了，再访问就触发了真正的崩溃。

### R — Result

三层 bug 全部修复后系统正常启动，WS2812 颜色显示正确。这三个 bug 的排查过程用了约 **3 小时**，工具只有：
- 串口 printf 逐步定位
- 看门狗复位时间（~10s）推断卡死位置
- HAL 源码阅读确认 API 行为

**经验教训：**
- HAL 函数的返回值永远不能忽略
- CubeMX 生成的 DMA/TIM 配置不一定完整，要手动验证
- C 语言中句柄类型本身就是指针时，`&` 操作会引入一层意外的间接引用

---

## 2. I2C 多设备冲突与互斥锁设计

### S — Situation

6 个 I2C 设备（AHT20、INA226、SD3078、AT24C02、PCA9555PW、ES8388）共用一条 I2C1 总线，运行在 FreeRTOS 多任务环境下。每个设备可以由不同任务独立访问。

### T — Task

设计一种机制防止多任务并发访问 I2C 总线造成数据错乱，同时不破坏 RTOS 的实时性。

### A — Action

**方案：互斥信号量 + 统一加锁封装**

1. 创建一个 `xSemaphore_I2C` 互斥信号量（CMSIS-RTOS V2 的 `osMutex`）
2. 每个 I2C 驱动内部实现 `i2c_lock()/i2c_unlock()` 静态函数
3. 驱动通过 `void *pSemaphore` 参数接收信号量指针

```c
static void i2c_lock(void) {
    if (s_sem != NULL)
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}
static void i2c_unlock(void) {
    if (s_sem != NULL)
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
}
```

4. 指针设计为 `void *`，BSP 层不依赖 CMSIS-RTOS 头文件
5. 传入 `NULL` 时不加锁，支持裸机调试

**加锁粒度控制：**
- `INA226_ReadAll()` 一次锁内读 3 个寄存器（电压/电流/功率），保证数据来自同一转换周期
- I²C 操作微秒级，锁持有时间极短，不影响 10ms 周期的按键扫描任务

### R — Result

所有 6 个 I2C 设备在多任务环境下稳定运行，没有出现过总线冲突。按键扫描任务（10ms 周期）即使在高 I2C 负载下也能准时执行。

**面试追问准备：**
- **为什么用信号量而不是关中断？** 关中断影响所有任务（包括定时器、调度器），信号量只阻塞等待 I2C 访问的任务
- **为什么信号量参数是 `void *`？** BSP 层不依赖 CMSIS-RTOS，遵循依赖倒置原则。传入 NULL 时跳过加锁，方便裸机调试

---

## 3. SPI 总线共享与 CS 管理

### S — Situation

ICM-42688（IMU）和 W25Q128（Flash 存储）共享 SPI2 总线，各有独立片选（PE7 和 PE4）。两个设备由不同 FreeRTOS 任务操作。

### T — Task

避免两个任务同时操作 SPI2 造成数据冲突，同时确保片选时序正确。

### A — Action

1. **发现缺失：** 项目框架中 SPI2 没有互斥信号量（相比 I2C 有完整的锁机制，SPI2 的保护是遗漏的）
2. **创建 `xSemaphore_SPI2`** 互斥信号量
3. **封装 SPI 事务保护：** 每次读写操作在 `spi_lock()/spi_unlock()` 中执行
4. **片选管理：** `cs_select()` 在整个多字节事务开始前拉低 CS，`cs_deselect()` 在事务完成后拉高 CS

### R — Result

SPI2 总线在多任务环境下安全共享。LCD（SPI1 独占）不做保护以换取最高性能。

**面试追问准备：**
- **为什么 LCD 的 SPI1 不加锁？** SPI1 只有 LCD 一个设备，不存在竞争条件
- **CS 必须在整个事务期间保持低电平，为什么？** 如果 CS 在字节间拉高，W25Q128 会认为当前命令被取消

---

## 4. DMA + IDLE 不定长接收（蓝牙）

### S — Situation

蓝牙模块 XW040 通过 USART2（9600bps）发送不定长 JSON 数据包（30~200 字节），数据包之间间隔不确定。

### T — Task

实现一种接收方案，既能高效接收不定长数据（不丢字节），又不让 CPU 被频繁中断占用。

### A — Action

**方案：DMA Circular Mode + UART IDLE 中断**

架构设计：
```
USART2 RX → DMA Circular (自动搬移到环形缓冲区)
    → 总线空闲 → IDLE 中断触发
    → 计算 NDTR 差值得到接收长度
    → 数据推入环形队列
    → 任务层从队列取出解析 JSON
```

关键实现细节：

1. **DMA Circular 模式**：DMA 自动从 USART2 DR 寄存器搬移到 `s_dma_rx_buf`，CPU 零参与
2. **IDLE 中断检测帧结束**：UART 空闲时硬件自动置位 IDLE 标志
3. **NDTR 差值法计算长度**：
   ```c
   void BLUETOOTH_IDLE_IRQHandler(void)
   {
       if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET) return;
       __HAL_UART_CLEAR_IDLEFLAG(s_huart);

       uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
       uint16_t received = (s_last_ndtr - ndtr) & (BT_RX_BUF_SIZE - 1);
       s_last_ndtr = ndtr;

       for (uint16_t i = 0; i < received; i++)
           ring_put(&s_ring, s_dma_rx_buf[(start + i) & mask]);
   }
   ```
4. **无锁环形队列**：ISR 只写 head，任务只读 tail，单生产者-单消费者天然无竞争
5. **JSON 解析**：`strstr` + `atoi` 手写（仅 3 种命令，不引入 cJSON）

### R — Result

蓝牙接收零丢包，CPU 占用极低（每帧数据只触发一次 IDLE 中断，而不是每字节一次中断）。如果使用传统每字节中断，9600bps 下每秒 960 次中断，DMA+IDLE 方案降至每秒约 1~2 次中断。

**面试追问准备：**
- **NDTR 差值回绕原理：** `(prev - cur) & mask` 利用无符号减法回绕 + 2 的幂掩码实现模运算
- **环形队列为什么不需要锁？** 硬件决定的单生产者-单消费者，head 和 tail 不会被同时修改
- **IDLE vs 帧头帧尾检测：** IDLE 利用物理层空闲检测，更简单可靠

---

## 5. FreeRTOS 任务优先级与 timing budget

### S — Situation

系统有 7 个任务：按键扫描（10ms）、传感器采集（100ms）、LCD 刷新（200ms）、蓝牙推送（1s）、TF 日志（1s）、WS2812 更新（1s）、空闲监控。需要合理分配优先级，保证所有任务在 deadline 前完成。

### T — Task

设计一套优先级方案，确保高频率任务不被低频率任务阻塞，同时所有任务都能在周期内完成。

### A — Action

**基于 Rate Monotonic Scheduling（RMS）的优先级分配：**

| 任务 | 周期 | 优先级 | 理由 |
|------|------|--------|------|
| 按键扫描 | 10ms | osPriorityHigh4 | 最短周期，消抖需要严格 10ms 采样 |
| 传感器采集 | 100ms | osPriorityAboveNormal3 | 含 AHT20 的 80ms 延时 |
| LCD 刷新 | 200ms | osPriorityNormal2 | 视觉平滑即可 |
| 蓝牙推送 | 事件/1000ms | osPriorityNormal2 | 低频数据推送 |
| TF 日志 | 1000ms | osPriorityBelowNormal1 | FATFS 写操作可能阻塞 |
| WS2812 | 1000ms | osPriorityLow | 颜色变化慢，可被抢占 |
| defaultTask | - | osPriorityNormal | 系统监控 |

### R — Result

所有任务在实测中都能在各自的周期内完成。AHT20 的 80ms `osDelay` 虽然占用了传感器任务的大部分时间，但由于 `osDelay` 会挂起任务让出 CPU，不影响其他任务运行。

**面试追问准备：**
- **为什么 WS2812 优先级最低？** 即使被抢占 1~2 秒，用户也看不出颜色变化延迟
- **AHT20 的 85% 占空比是不是问题？** `osDelay(80)` 让出 CPU，所以不意味着 CPU 满载

---

## 6. AHT20 80ms 延时问题

### S — Situation

AHT20 温湿度传感器的数据手册规定：触发测量命令（0xAC）后，需要等待至少 80ms 才能读取结果。这是传感器本身的硬件限制。

### T — Task

在传感器读取任务的 100ms 周期内，处理这个 80ms 等待时间而不浪费 CPU。

### A — Action

**三种方案的评估与选择：**

| 方案 | 优点 | 缺点 | 选择? |
|------|------|------|-------|
| 1. `osDelay(80)` 让出 CPU | 简单，不影响其他任务 | 唤醒后只做少量工作 | ✅ 当前方案 |
| 2. 轮询 bit7 提前结束 | 可提前（如 60ms 读完） | 增加 I2C 总线负载 | ❌ 没必要 |
| 3. 状态机交错读取 | 80ms 内读其他传感器 | 代码复杂，收益小 | ❌ 过度设计 |

最终选择方案 1，因为温湿度变化慢（ms 级不敏感），80ms 固定延时对系统整体影响微乎其微。

### R — Result

AHT20 准确读取温湿度数据，80ms 延时期间 CPU 可用于其他任务。

**面试追问准备：**
- **如果未来有更高速率的传感器怎么办？** 改用状态机方案，在等待期交错读取其他传感器
- **AHT20 忙标志位（bit7）怎么用？** 发完命令后轮询 bit7，为 0 表示转换完成，可提前返回

---

## 7. EC11 编码器查表法解码

### S — Situation

EC11 旋转编码器产生正交信号（A、B 相），通过检测相位差判断旋转方向。需要可靠的解码算法，能处理低速旋转和抖动。

### T — Task

实现一种解码算法，抗抖动、常数时间、无分支预测失败。

### A — Action

**采用查表法（Lookup Table）：**

```c
// 2-bit 组合 AB，4 种状态 × 4 种状态 = 16 种输入
// last_ab = 上次状态(2bit)，ab = 当前状态(2bit)
// 索引 = (last_ab << 2) | ab
static const int8_t ec11_table[16] = {
     0,  1, -1,  0,   /* 00 → 00,01,10,11 */
    -1,  0,  0,  1,   /* 01 → 00,01,10,11 */
     1,  0,  0, -1,   /* 10 → 00,01,10,11 */
     0, -1,  1,  0,   /* 11 → 00,01,10,11 */
};

uint8_t ab = ec11_read_ab();
uint8_t idx = (last_ab << 2) | ab;
s_delta += ec11_table[idx];  // +1 = CW, -1 = CCW, 0 = 无变化/非法
last_ab = ab;
```

**为什么查表法优于 if-else：**
1. 常数时间 — 不随状态数量变化
2. 无分支 — CPU 没有分支预测失败惩罚
3. 自动处理非法状态（两 bit 同时变化返回 0）

### R — Result

EC11 解码可靠，无抖动误触发，$O(1)$ 时间完成解码。

**面试追问准备：**
- **非法状态如何处理？** 索引 3、6、9、12 对应两 bit 同时变化，返回 0 忽略
- **什么情况下会出非法状态？** 旋转太快或采样间隔太长

---

## 8. TF 卡 FATFS 掉电保护

### S — Situation

传感器数据每秒写入 TF 卡 CSV 文件，嵌入式设备可能随时掉电。如果数据只写在文件系统缓冲区而未刷到 SD 卡，掉电会丢失最近几秒甚至几分钟的数据。

### T — Task

实现掉电保护机制，将数据丢失控制在 1 行以内。

### A — Action

```c
FRESULT TF_LogSensor(const SensorData_t *data)
{
    f_printf(&file, "%s,%.1f,%.1f,%.1f,%.1f,...\r\n",
             timestamp, temp, humi, voltage, current, ...);
    f_sync(&file);  // ← 关键：每秒强制刷写到 SD 卡
    return FR_OK;
}
```

**文件管理策略：**
- 按天分割文件：RTC 生成文件名 `2026-07-20.csv`
- 文件存在则追加，不存在则创建并写 CSV 表头
- 日期变更时自动关闭旧文件、创建新文件

### R — Result

掉电最多丢失 1 行数据（最近 1 秒内的那行）。CSV 格式方便 Excel 直接打开分析，手机蓝牙 App 也能解析。

**面试追问准备：**
- **`f_sync` vs `f_close`？** `f_sync` 刷新缓冲区但不关闭文件（可继续写），`f_close` 既刷又关
- **为什么按天分割？** 单文件不会过大，方便数据管理
- **另一个方案：** 如果追求更高可靠性，可以使用双文件轮流写入策略

---

## 12. 按键消抖状态机重写 — 页面自己跳的根因

### S — Situation

项目使用 3 个按键（PA0、PE8、PC13）切换 LCD 页面，KEY1 下一页、KEY3 上一页。上电后 LCD 页面会"自己跳"——没有按键操作时也自动切页。此前尝试过多种方案：
- 增加 EC11 消抖（排除 EC11 干扰）
- 增加 300ms 翻页冷却
- 去掉 EC11 整个模块

但问题依旧。LCD 页面仍然每隔约 1 秒自动循环一遍（PAGE_DATA → PAGE_IMU_CHART → PAGE_STATUS → PAGE_DATA → ...）。

### T — Task

找到"页面自己跳"的根因并修复。涉及：按键消抖算法、页面切换逻辑、清屏机制三个环节，需要逐一排查。

### A — Action

**第一步：检查页面切换调用链**

跟踪 `g_current_page` 的所有修改点，发现只有 KEY1/KEY3 在 `StartKeyScan` 任务中调用 `LCD_Page_Switch()` 才会改页号。没有其他代码路径能触发页面切换。问题定位到按键消抖层。

**第二步：审查消抖算法，发现 3 个 bug**

```c
// 原消抖算法（简化）
for (i = 0; i < 3; i++) {
    level = key_read_raw(i);
    if (level == s_keys[i].stable) {        // 跟稳定状态比
        s_keys[i].count++;
        if (s_keys[i].count >= 3) {
            if (s_keys[i].stable == 0)      // 按下?
                s_pressed = KEY_i;          // ← Bug 2
        }
    } else {
        s_keys[i].count = 0;
        s_keys[i].stable = level;           // ← Bug 1: 立即更新 stable
    }
}
```

**Bug 1 — 稳定状态无消抖：** `else` 分支中 `stable = level` 在第一个不匹配样本就执行了。注释写"连续一致才会确认变化"，代码做的正好相反。等于没有消抖——任何噪声脉冲的第一拍就直接改了 `stable`。

**Bug 2 — 长按持续触发：** `s_pressed = KEY_i` 的条件是 `stable==0 && count>=3`，按键按住期间每 10ms 都为 true。`s_pressed` 每扫描周期都被设置，`KEY_GetPressed()` 每次调用都有返回值。虽然 `StartKeyScan` 有 300ms 冷却，但冷却一过就再次切页。**按住按键（或引脚有噪声）导致每 300ms 自动翻一页。**

**Bug 3 — LCD_Page_Switch 误设 s_last_page：** `LCD_Page_Switch()` 同时设了 `g_current_page` 和 `s_last_page`，导致 `LCD_Page_Refresh()` 中 `page != s_last_page` 永远为假，清屏+全屏重绘分支永不执行。即使新页面的内容不覆盖上一页的所有区域，上一页的残留也不清除。

**第三步：重写消抖算法 — 双变量状态机**

核心思想：从比较 `level == stable` 改为比较 `level == raw`（最新原始采样值）。`stable` 只在连续 N 次一致后才更新，不在 `else` 分支中立即改。

```c
// 新算法（双变量）
for (i = 0; i < 3; i++) {
    level = key_read_raw(i);
    if (level == s_keys[i].raw) {               // 跟原始采样值比，不是 stable
        s_keys[i].count++;
        if (s_keys[i].count >= 3 && s_keys[i].stable != level) {
            s_keys[i].stable = level;           // 连续 3 次一致才更新
            s_keys[i].count = 3;                // 锁定，防止重复触发
            if (level == 0)                     // 只在按下瞬间触发一次
                s_pressed = KEY_i;
        }
    } else {
        s_keys[i].raw = level;                  // 更新最新采样值
        s_keys[i].count = 0;                    // 计数清零
        // stable 不动！只有连续一致后才更新
    }
}
```

关键区别：
- `stable` 不会在 `else` 分支中更新，只在连续 N 次一致后更新 → **真正的消抖**
- `stable != level` 防止按住期间重复触发 → **只触发一次**
- `raw` 追踪最新采样值，后续采样跟 `raw` 比，噪声不会误改 `stable`

**第四步：修复 LCD_Page_Switch**

去掉 `s_last_page = page` 的赋值，`s_last_page` 只在 `LCD_Page_Refresh` 完成全屏绘制后更新。同时在 `page != s_last_page` 分支开头加 `LCD_Clear(BLACK)` 确保切换时清屏。

### R — Result

三个 bug 全部修复后，按键行为变为：

| 场景 | 旧行为 | 新行为 |
|------|--------|--------|
| 按键按下 | 40ms 后触发（等效无消抖） | 40ms 后触发（3 次确认） |
| 按住按键不放 | 每 300ms 自动翻页 | 只触发一次，不再重复 |
| 页面切换 | 上一页内容残留 | 清屏后绘制，画面干净 |
| GPIO 噪声 | 30ms 噪声脉冲 → 误触发 | 30ms 噪声 → 被消抖过滤 |

**根因总结：** "页面自己跳"不是单一 bug 导致，而是三个问题叠加 —— 消抖没生效（噪声可触发）、按住持续写 `s_pressed`（冷却后继续切页）、以及全屏重绘分支被屏蔽（清屏从不执行）。三者共同造成"页面每隔约 1 秒自动循环一遍"的观感。

**面试追问准备：**
- **为什么原设计用 `stable` 做比较？** 设计意图是跟踪"稳定后的状态"，但实现时在 else 分支直接更新 stable 破坏了消抖。本质是"伪状态机"——看起来有消抖，实际没有。
- **新算法为什么用 `raw` 做参考？** `raw` 是最近一次采样值，不稳定但最新。`stable` 是经过消抖确认的值，稳定但不即时。用 `raw` 做参考，噪声只影响 `raw` 和 `count`，不影响 `stable`。
- **为什么 `count = 3` 后就锁定？** 防止 `stable` 更新后 `count` 继续增长，导致 `stable != level` 条件再次满足时误判为新变化。
- **硬件 pull-up 没用吗？** PA0/PC13 都配置了内部 pull-up，但长约 10cm 的飞线在电磁干扰环境中可能耦合噪声。软件消抖是最后一层防线。

---

## 9. BSP 驱动的接口设计模式

### S — Situation

项目涉及 10+ 个外设驱动（I2C、SPI、UART、TIM 等），需要一套统一的接口规范让代码可维护、可测试。

### T — Task

设计一种通用的 BSP 驱动接口模式，兼顾可移植性、可测试性和代码简洁度。

### A — Action

**每个驱动 .h / .c 对遵循统一模板：**

```
.h 文件:
  - 地址宏 (#define DEV_ADDR 0x38)
  - 寄存器/命令宏
  - 返回值枚举 (DEV_OK, DEV_ERR_I2C, DEV_ERR_PARAM)
  - 函数声明: Init(...), Read*(...)

.c 文件:
  - 静态变量: s_handle, s_sem
  - i2c_lock() / i2c_unlock() 封装 (传递 NULL 时跳过)
  - 内部读写辅助函数
  - Init: 保存句柄 → 复位 → 配置 → 验证
  - Read: 加锁 → 读寄存器 → 换算 → 解锁
```

**关键设计决策：**
1. **`void *pSemaphore`** — BSP 层不引用 CMSIS-RTOS 头文件
2. **自定义枚举返回值** — 不暴露 `HAL_StatusTypeDef`，应用层不知道 HAL 的存在
3. **读操作用输出参数** — `XXX_ReadData(float *value)`，返回值表示成功/失败
4. **所有 Init 可重入** — 多次调用不会产生副作用

**举例：**
```c
// AHT20_ReadAHT20 的返回值是 AHT20_Status_t，不是 HAL_StatusTypeDef
AHT20_Status_t AHT20_ReadAHT20(float *temperature, float *humidity);
// 应用层使用:
float t, h;
if (AHT20_ReadAHT20(&t, &h) == AHT20_OK) {
    // 使用 t 和 h
}
```

### R — Result

所有驱动代码风格统一，新人阅读一份驱动就能上手所有驱动。测试时可以传入 NULL 信号量跳过锁，方便裸机单步调试。

**面试追问准备：**
- **为什么 `void *` 而不是 `osMutexId_t`？** 依赖倒置 — BSP 层不应该依赖 RTOS 实现
- **自定义枚举的好处？** 应用层不关心底层是 HAL 还是 LL 还是寄存器操作
- **输出参数 vs 返回值？** C 语言函数只能返回一个值，用输出参数返回数据，返回值表示状态

---

## 10. LCD 背光控制与上电时序优化

> 关于 LCD 初始化流程优化的故事，涉及背光 PWM 驱动编写、上电时序设计、以及"黑屏→亮屏"一瞬间的视觉体验打磨。

### S — Situation

项目使用 ST7789V2 驱动 LCD（SPI1，21MHz），通过 PB0 引脚控制背光。最初的设计是：
1. `App_Init()` 中调用 `LCD_Init()` → `LCD_Clear()` → `LCD_DrawString()` 显示 "Initializing..."
2. 背光由 GPIO 直接驱动（没有独立的背光模块）

实际效果是：上电后 LCD 立即亮起白色背光，用户能看到屏幕上 Sensor 初始化的全过程（闪烁、字符逐步出现），视觉上非常不专业。

### T — Task

实现一个可控的背光方案：
- 上电时背光保持关闭
- LCD 初始化（配置寄存器、清屏）全部完成后才亮起
- 背光支持 PWM 调节（为后续亮度调节留接口）
- 用户看到的是"黑屏→完整画面"的干净切换

### A — Action

**第一步：新建背光驱动模块**

`backlight.c / backlight.h`，封装 TIM5_CH4 的 PWM 输出：

```c
void Backlight_Init(void)
{
    // TIM5_CH4 PWM 初始化
    HAL_TIM_PWM_Start(BACKLIGHT_TIM, BACKLIGHT_CHANNEL);
    __HAL_TIM_SET_COMPARE(BACKLIGHT_TIM, BACKLIGHT_CHANNEL, 0);
}

void Backlight_Set(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(BACKLIGHT_TIM, BACKLIGHT_CHANNEL, duty);
}

void Backlight_On(void)  { Backlight_Set(BACKLIGHT_MAX); }
void Backlight_Off(void) { Backlight_Set(0); }
```

**第二步：重排 LCD 初始化时序**

```
❌ 旧时序：
  LCD_Init() → LCD_Clear() → LCD_ShowString()   ← 背光一直亮着，用户看到初始化过程

✅ 新时序（LCD_Init 内部）：
  Backlight_Init()           → 1. 初始化 PWM，关闭背光
  HAL_GPIO_WritePin(RST, 0)  → 2. 硬件复位（背光关闭，看不到闪烁）
  ST7789 寄存器配置序列       → 3. 配置像素格式、方向、显示开
  LCD_Clear(BLACK)           → 4. 清屏（背光仍关，看不到）
  Backlight_On()             → 5. 最后一步才开背光，用户看到完整画面
```

**第三步：简化 App_Init**

旧代码在 `App_Init()` 中做了 `LCD_Clear()` + `LCD_DrawString("Initializing...")`，但有了新时序后这些动作在背光亮起前就完成了，用户看不见。于是移除冗余代码，让 `LCD_Init()` 内部一站式完成。

**第四步：硬件配置**

在 main.ioc 中添加 PB0 引脚（GPIO_Output, Label=LCD_BL），在 main.h 中生成 `LCD_BL_Pin` / `LCD_BL_GPIO_Port` 宏，更新 `MX_GPIO_Init()` 的 GPIO 初始化包含背光引脚。

### R — Result

上电效果从：
```
❌ 白屏闪烁 → 字符逐行出现 → 正常显示
```
变为：
```
✅ 黑屏（约500ms）→ 完整画面亮起
```

看起来像"一下子就好了"，用户体验提升显著。背光 PWM 接口为后续自动亮度调节（根据环境光传感器）预留了扩展点。

**面试追问准备：**
- **为什么不用 PMW 硬件调光？** 用的就是 TIM5_CH4 的 PWM，`Backlight_Set()` 改占空比即可实现亮度调节
- **初始化的 500ms 黑屏会不会太长？** 主要是 ST7789 的 120ms 复位 + 各寄存器间的延时。如果有要求可以优化，但目前黑屏期用户感觉不到
- **Backlight 为什么要独立成一个模块？** 单一职责 — LCD 管显示内容，背光管亮度控制。后续接环境光传感器改亮度只改 backlight.c，不改 lcd.c

---

## 11. 从 CubeMX 骨架到完整系统

### S — Situation

项目起点是 CubeMX 生成的工程骨架：
- HAL 外设配置已完成
- 6 个 I2C 驱动已移植（从嘉立创示例代码）
- 6 个核心驱动是空的（LCD、IMU、WS2812、蓝牙、按键、TF 卡）
- 7 个 FreeRTOS 任务体全是 `osDelay(1)` 的空循环
- 整个 App 层只有 TODO 注释

### T — Task

在 2 周内补齐所有功能，让系统能稳定运行：采集 4 种传感器数据、显示在 LCD、通过蓝牙推送、记录到 TF 卡、LED 指示温度。

### A — Action

**分阶段实施：**

1. **编译修复阶段：** 先修 CubeMX 生成的明显错误（队列大小用 `sizeof(uint8_t)` → 改为真实类型大小、TIM5 Period 209→104）
2. **逐个补齐 BSP 驱动：** 参考嘉立创示例代码 + 芯片数据手册，从简单的开始（KEY → LCD → IMU → WS2812 → 蓝牙 → TF 卡）
3. **App 层组装：** 传感器数据数据结构 → 页面显示框架 → 应用初始化入口
4. **任务体实现：** 填充 7 个任务循环，连接 BSP 驱动和 App 层
5. **调试与验证：** 串口 printf 监控、逐步排除 9 个 bug

**发现并修复的 9 个问题：**

| 问题 | 根因 | 修复 |
|------|------|------|
| 系统启动后卡死 | WS2812 DMA 未配置，`s_busy` 死锁 | 手动添加 DMA2_Stream5 + 错误处理 |
| DMA 传输不完成 | TIM5 计数器未启动 | `HAL_TIM_Base_Start()` 强制启动 |
| 传感器 INIT 后崩溃 | `&xSemaphore_I2CHandle` 多一层间接引用 | 改为 `xSemaphore_I2CHandle` |
| PWM 频率错误 | Period=209（400KHz），需要 800KHz | Period 改为 104 |
| 队列大小为 1 字节 | CubeMX 默认 `sizeof(uint8_t)` | 改为 `sizeof(SensorData_t)` |
| SPI2 无互斥锁 | 框架遗漏 | 添加 `xSemaphore_SPI2` |
| printf %f 空白 | newlib nano 禁用浮点数 | `-u _printf_float` 链接选项 |
| FATFS 变量名不匹配 | CubeMX 生成 `SDFatFS` 不是 `g_SDFatFS` | 修复 extern 声明 |
| 传感器 Init 参数错误 | SD3078 需要 3 个参数不是 2 个 | 补充 NULL 参数 |

### R — Result

系统完整运行，串口输出：

```
[SENSOR] #0 T=29.5C H=45.2% V=4.75V I=0.000A Accel=-0.01,0.00,1.03g Gyro=0.3,-0.4,2.0dps OK=0x07
```

所有 4 种传感器正常读数，LCD 三页显示、蓝牙推送、TF 日志、WS2812 温度指示全部实现。参考了嘉立创示例代码 + HAL 数据手册，不是从零写代码。

**转项目为面试故事的要点：**
- 强调**发现问题 → 定位根因 → 修复**的完整闭环，而不是"我写了什么代码"
- 重点突出 WS2812 三层嵌套 bug，展示调试能力
- 展示架构思维：接口模式设计、优先级调度、互斥锁粒度控制

---

## 13. LCD SPI 行缓冲优化 — IMU 柱状图不实时

### S — Situation

页面二（IMU 图表页）显示 ICM42688 加速度和陀螺仪的柱状图。用户反馈"柱状图和数据不灵敏，反应时间有点长"——拿着板子晃，柱子跟不上运动。

此前已做过多轮优化：LCD 任务去掉冗余 `osDelay`（200ms 空等 → 数据到达即刷新）、ICM42688 合并为一次 SPI 读。但效果仍不够，数据看起来还是"慢半拍"。

### T — Task

找出让页面二刷新慢的根因，不是"任务调度频率"而是"SPI 传输效率"。

### A — Action

**第一步：审计 SPI 事务量，定位 DrawString 为瓶颈**

用分析工具统计 `page_imu_draw()` 一次全屏重绘的 SPI 事务数：

| 操作 | SPI 事务数 | 占比 |
|------|-----------|------|
| 文字绘制（17 个 DrawString 调用） | 29,400 次 CS 翻转 | 99.6% |
| 大块 FillRect（柱状区/状态栏） | 122 次 | 0.4% |
| **总计** | **29,522 次 CS 翻转** | 100% |

根源在 `LCD_DrawString` 的实现：

```c
// 原实现: 每个字符逐像素调用 LCD_FillRect
for (col = 0; col < 5; col++) {
    for (row = 0; row < 7; row++) {
        LCD_FillRect(cur_x + col*scale, y + row*scale,
                     cur_x + col*scale + scale-1,
                     y + row*scale + scale-1, pixel_color);
    }
}
```

- 每个字符 5 列 × 7 行 = 35 次 `LCD_FillRect`
- 每个 `LCD_FillRect` 执行一次窗口设置（CASET+RASET+RAMWR，5 次 CS 翻转）
- scale=2 时 9 字符字符串 "X: +12.34" = **1575 次 SPI 事务**
- 整个页面 17 个字符串 = **29,400 次 SPI 事务，占全页刷新 99.6%**

每个 `LCD_FillRect` 即使只画 2×2=4 个像素，也要过一遍完整的 CASET(4B) + RASET(4B) + RAMWR + data(8B) 流程，窗口设置的 SPI 开销远大于数据本身。

**第二步：重写 `lcd_set_window` — CS 一次低电平完成所有命令**

原实现 5 次 CS 选通/释放，每次调用 `HAL_SPI_Transmit` 有 ~3μs 固定开销。改为一次 CS 低电平：

```c
cs_sel();
    dc_cmd(); HAL_SPI_Transmit(&hspi1, &cmd_2A, 1, HAL_MAX_DELAY);
    dc_dat(); HAL_SPI_Transmit(&hspi1, seq_x, 4, HAL_MAX_DELAY);
    dc_cmd(); HAL_SPI_Transmit(&hspi1, &cmd_2B, 1, HAL_MAX_DELAY);
    dc_dat(); HAL_SPI_Transmit(&hspi1, seq_y, 4, HAL_MAX_DELAY);
    dc_cmd(); HAL_SPI_Transmit(&hspi1, &cmd_2C, 1, HAL_MAX_DELAY);
cs_des();
```

5 次 CS 翻转 → 1 次。`LCD_FillRect` 和 `LCD_DrawProgressBar` 也受益于此。

**第三步：重写 `LCD_DrawString` — 行缓冲批量 SPI 写入**

核心思路：**设一次窗口覆盖整个字符串，将像素按行缓冲后批量发送**。

```c
lcd_set_window(x, y, x + total_w - 1, y + total_h - 1);  // 一次窗口

for (row = 0; row < 7; row++) {
    while (*cp) {
        glyph = font[c - 0x20];
        for (col = 0; col < 5; col++) {
            pixel = (glyph[col] & (1 << row)) ? fg : bg;
            for (sc = 0; sc < scale; sc++)
                buf[idx++] = pixel_HI, buf[idx++] = pixel_LO;  // 水平缩放
        }
        for (sc = 0; sc < scale; sc++)
            buf[idx++] = bg_HI, buf[idx++] = bg_LO;  // 间距
    }
    for (sr = 0; sr < scale; sr++)
        lcd_write_datas(buf, idx);  // 一次 SPI 批量写
}
```

**第四步：页面一二增量刷新**

`page_data_update` 和 `page_imu_update` 改用只更新变动区域（数值+柱状图+状态栏），静态元素（标题、标签、分隔线）翻页时画一次，同页循环不动。

### R — Result

| 指标 | 优化前 | 优化后 |
|------|--------|--------|
| CS 翻转次数（页面二一次增量更新） | 29,522 次 | ~140 次 |
| 预计 SPI 传输时间 | ~138 ms | ~5 ms |
| 数据落地延迟 | ~400 ms | ~100 ms（传感器周期限制） |

**经验教训：**
1. **性能问题不一定在"调度"上**——表面看是页面响应慢，第一反应是"提高任务优先级、缩短周期"，但实际瓶颈藏在 `LCD_DrawString` 的 SPI 事务量上
2. **`LCD_FillRect` 是昂贵的操作**——设一次窗口做 5 次 SPI 事务，只画 2 字节数据。批量操作效率比逐像素高几个数量级
3. **99% 的瓶颈通常藏在 1% 的代码里**——`LCD_DrawString` 只占代码量的 3%，却占了 99.6% 的 SPI 事务

**面试追问准备：**
- **为什么第一个优化没生效（去 osDelay + ICM42688 合并读）？** 方向错了——去 osDelay 解决"调度延迟"，但 SPI 传输本身耗时 138ms 才是瓶颈
- **为什么之前没发现 DrawString 的问题？** 它能"正常工作"——字能显示，肉眼看不出每帧画了 3 万次 SPI 事务。只有数了 SPI 事务量才发现
- **行缓冲会不会栈溢出？** 1KB 缓冲，LCD 任务栈 8KB，完全安全
- **这个优化影响其他页面吗？** 所有页面受益，因为 `LCD_DrawString` 是共享函数

---

## 面试官可能追问

### Q1：为什么用 FreeRTOS 而不是裸机？

**S：** 6 个传感器 + LCD + 蓝牙 + TF 卡 + WS2812，裸机主循环难以组织各个任务的时序。

**A：**
- 任务隔离 — 一个任务卡住不会死机
- 信号量/队列 — I2C 互斥、数据发布-订阅
- 优先级调度 — 10ms 按键不被 1s 的 TF 写卡阻塞

**R：** 多任务有序运行，每个任务独立维护自己的状态。

### Q2：6 个 I2C 设备如何防止冲突？

**S：** 多任务同时访问 I2C 总线会数据错乱。

**A：** `xSemaphore_I2C` 互斥信号量。每个驱动 Init 时收到信号量指针，每次 I2C 操作前后加解锁。

**关键细节：** `void *` 传递依赖倒置，`NULL` 跳过锁支持裸机调试。

**R：** 零冲突，全稳定。

### Q3：蓝牙接收数据丢失怎么办？

**S：** XW040 不定长发 JSON，9600bps 下可能丢数据。

**A：** DMA Circular + IDLE 中断，环形队列 512 字节（最大 JSON 包的 6 倍+）。IDLE 中断只搬数据不做解析，任务层从队列取数据解析。

**R：** 零丢包，如果队列满，丢弃新字节保护已有数据。

### Q4：为什么不用 cJSON？

**S：** 蓝牙指令 JSON 格式需解析。

**A：** 只有 3 种命令，`strstr` + `atoi` 完全够用。不引入 cJSON 避免：
- 动态内存分配（malloc 碎片化）
- 固件增大 ~3KB
- 额外的 bug 可能

**R：** 代码简洁，固件小巧，功能完全满足。

### Q5：最难的 bug 是什么？

WS2812 三层嵌套 bug（见第 1 节完整 STAR 故事）。

**一句话版：** 第一层 CubeMX 没生成 DMA 配置导致卡死，第二层 TIM 计数器没启动导致 DMA 不触发，第三层信号量传参多一层 `&` 导致任务崩溃。三个 bug 逐一暴露，用串口 printf 逐步定位。

### Q6：Power Loss 怎么处理？

**S：** TF 卡每秒写 CSV，可能随时掉电。

**A：** `f_sync()` 每秒强制刷写数据到 SD 卡物理介质。掉电最多丢 1 行。

**R：** 数据鲁棒性满足嵌入式设备需求。

### Q7：内存够用吗？FreeRTOS heap 怎么分配的？

**S：** STM32F407VET6 有 192KB RAM，HAL + FreeRTOS + 各类任务 + 缓冲区需要精细管理。

**A：** 使用 heap_4（避免碎片化），`configTOTAL_HEAP_SIZE = 93804`。6 个任务栈共约 9KB，全局变量约 2KB，WS2812 缓冲区 ~4KB，剩余 ~80KB 可用。

**R：** 内存充裕，没问题。
