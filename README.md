# 🌡️ 蓝牙无线环境监测与数据记录仪

**基于 STM32F407 的多传感器环境监测终端** — 板载传感器全采集，LCD 多页实时显示，TF 卡自动记录 CSV 日志，蓝牙无线推送到手机 APP，FreeRTOS 多任务调度。

[![STM32](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/)
[![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](https://www.freertos.org/)
[![Toolchain](https://img.shields.io/badge/Toolchain-CubeIDE%20%7C%20CMake%20%7C%20CLion-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)

---

## 📋 项目概述

本项目是一个全面的 STM32 外设练习，集成了 **I²C 多设备管理**、**SPI 总线共享**、**SDIO 文件系统**、**LCD 显示**、**蓝牙串口通信**和 **FreeRTOS 多任务调度**。

> **一句话描述：** 板载传感器全采集，LCD 多页实时显示，TF 卡自动记录 CSV 日志，蓝牙无线推送到手机 APP 实时查看，EC11 旋钮翻历史数据。

### 核心功能

| 功能 | 实现方式 |
|------|---------|
| **温湿度采集** | AHT20  via I²C，400 kHz 快速模式 |
| **电源监控** | INA226（电压/电流/功率） via I²C |
| **高精度 RTC** | SD3078（3.8ppm 温补晶振） via I²C |
| **六轴 IMU** | ICM-42688（加速度+角速度） via SPI，200 Hz |
| **显示屏** | 2.0 寸 ST7789V2 SPI 屏幕，3 页面 UI |
| **数据日志** | TF 卡 via SDIO 4-bit，FATFS，CSV 格式 |
| **无线通信** | 蓝牙 SPP（XW040） via USART2，JSON 协议 |
| **状态灯效** | 3 颗 WS2812 RGB，PWM+DMA 驱动 |
| **用户输入** | 3 个按键 + EC11 旋转编码器 |
| **SPI Flash** | W25Q128 存储校准数据 |
| **实时系统** | FreeRTOS，6 个任务，队列和互斥信号量 |

---

## 🏗 系统架构

### 硬件框图

```
                    ┌─────────────────────────────┐
                    │        STM32F407VET6        │
                    │        168 MHz Cortex-M4     │
                    └──────────┬───┬───┬───┬──────┘
          ┌────────────────────┤   │   │   ├──────────────────────┐
          │         ┌──────────┤   │   │   ├──────────┐           │
          ▼         ▼          │   │   │   │          ▼           ▼
    ┌──────────┐ ┌──────┐     │   │   │   │    ┌──────────┐ ┌──────────┐
    │  AHT20   │ │INA226│     │   │   │   │    │ST7789V2  │ │  XW040   │
    │ 温湿度   │ │电源  │     │   │   │   │    │2.0" LCD  │ │ 蓝牙模块 │
    │ 传感器   │ │监控   │     │   │   │   │    │(SPI1,    │ │(USART2,  │
    └────┬─────┘ └──┬───┘     │   │   │   │    │ 21 MHz)  │ │ 9600bps) │
         │          │         │   │   │   │    └──────────┘ └──────────┘
         │    ┌─────┘         │   │   │   │
         ▼    ▼               │   │   │   │
    ┌─────────────────┐       │   │   │   │    ┌──────────┐ ┌──────────┐
    │   PCA9517       │       │   │   │   │    │ICM-42688│ │ W25Q128  │
    │ I²C 缓冲器      │       │   │   │   │    │六轴 IMU │ │ SPI Flash│
    └──┬──┬──┬──┬──┬──┘       │   │   │   │    │ (SPI2)  │ │ (SPI2)   │
       │  │  │  │  │          │   │   │   │    └──────────┘ └──────────┘
       ▼  ▼  ▼  ▼  ▼          │   │   │   │
    ┌──────────────────┐      │   │   │   │    ┌──────────┐ ┌──────────┐
    │SD3078│AT24C02│   │      │   │   │   │    │ TF 卡    │ │ 3 按键   │
    │RTC   │EEPROM │   │      │   │   │   │    │ (SDIO,   │ │ + EC11   │
    │PCA9555│ES8388│   │      │   │   │   │    │ 12 MHz)  │ │ 编码器   │
    └──────────────────┘      │   │   │   │    └──────────┘ └──────────┘
                              │   │   │   │
                              │   │   │   │    ┌──────────┐
                              │   │   │   │    │ WS2812   │
                              │   │   │   │    │ RGB LED  │
                              │   │   │   │    │(TIM5_CH4 │
                              │   │   │   │    │ PWM+DMA) │
                              │   │   │   │    └──────────┘
                              │   │   │   │
                              ▼   ▼   ▼   ▼
                         ┌────────────────────┐
                         │    I²C1 总线       │
                         │  6 个设备共享      │
                         └────────────────────┘
```

### FreeRTOS 任务架构

| 任务名 | 优先级 | 周期 | 栈大小 | 功能 |
|--------|--------|------|--------|------|
| `Task_Key_Scan` | **5**（最高） | 10 ms | 256 字 | 按键消抖、EC11 编码器读取 |
| `Task_Sensor_Read` | **4** | 100 ms / 10 ms | 1024 字 | 轮询 AHT20、INA226、ICM-42688 |
| `Task_LCD_Update` | **3** | 200 ms | 2048 字 | 多页面显示刷新 |
| `Task_Bluetooth` | **3** | 事件驱动 | 1024 字 | JSON 数据发送 / 指令接收 |
| `Task_TF_Log` | **2** | 1 秒 | 2048 字 | CSV 文件写入 + f_sync |
| `Task_WS2812` | **1**（最低） | 50 ms | 256 字 | RGB LED 颜色更新 |

### 任务间通信

| IPC 对象 | 类型 | 用途 |
|----------|------|------|
| `xQueue_SensorData` | 队列（长度 10） | 传感器数据生产者 → 消费者 |
| `xQueue_BT_Command` | 队列（长度 5） | 蓝牙指令分发 |
| `xSemaphore_I2C` | 互斥信号量 | I²C 总线独占访问（6 个设备） |
| `xSemaphore_SensorData` | 互斥信号量 | 全局传感器数据结构保护 |
| `xSemaphore_SPI2` | 互斥信号量 | SPI2 总线共享（IMU + Flash） |

---

## 🔧 核心技术决策

### 1. I²C 多设备分时复用与互斥保护

六个设备（AHT20、INA226、SD3078、AT24C02、PCA9555PW、ES8388）通过 PCA9517 缓冲器共享 **I²C1** 总线，工作在 **400 kHz 快速模式**。没有使用简单的全局关中断方式（会破坏 RTOS 实时性），而是用 **互斥信号量**（`xSemaphore_I2C`）保证总线独占访问。400 kHz 下轮询 6 个设备仅需约 150 μs，比标准模式（100 kHz）快 4 倍。

### 2. SPI2 总线共享——CS 分时复用

ICM-42688（IMU）和 W25Q128（Flash）通过独立的 CS 引脚（PE7 / PE4）共享 **SPI2** 总线。原因是 F407VET6 仅有 3 个 SPI 外设——SPI1 给了 LCD，SPI3 没有引出可用引脚，资源受限下总线共享是工程常态。每次切换设备前检查 BSY 标志位确保传输完全结束，并用互斥信号量防止并发访问。

### 3. DMA + 空闲中断实现不定长接收

蓝牙数据包为 **JSON** 格式，长度 30~200 字节不等。三种方案对比：

| 方案 | 问题 |
|------|------|
| 中断逐字节接收 | CPU 占用高，高速时容易丢字节 |
| 固定长度 DMA 接收 | 短包需等待缓冲区填满，长包可能溢出 |
| **DMA Circular + 空闲中断** ✅ | **接收期间 CPU 零占用；总线空闲（>1 字节时间）触发 IDLE 中断，NDTR 寄存器给出精确长度** |

### 4. 三通道数据输出——刻意设计的不同速率

传感器数据同时输出到三个通道，刷新率**刻意不相同**：

- **LCD**：200 ms——人眼视觉暂留约 100 ms，再快没有感知差异
- **TF 卡 CSV**：1 秒——FATFS `f_write()` + `f_sync()` 耗时 20~50 ms，200 ms 写入将占用 25% CPU
- **蓝牙 JSON**：1 秒——9600 bps（~960 B/s）下，150 字节 JSON 包占用 15% 带宽

### 5. FreeRTOS 优先级分配——Rate-Monotonic 调度思想

优先级按**截止时间严格程度**排序，而非任务重要性：

- **Level 5**（按键扫描）：10 ms 消抖截止时间——用户感知延迟最敏感
- **Level 4**（传感器采集）：IMU 200 Hz 采样——错过周期丢失数据
- **Level 3**（LCD、蓝牙）：200 ms / 1 秒——容忍轻微抖动
- **Level 2**（TF 日志）：1 秒——几十毫秒延迟无影响
- **Level 1**（WS2812）：50 ms——纯装饰功能

### 6. 传感器数据线程安全设计

传感器数据在**一个写入者**（采集任务）和**三个读取者**（LCD、蓝牙、日志）之间共享。**互斥信号量**保证读写原子性，**队列**解耦生产者-消费者时序——两个不同问题用两种不同原语解决。

### 7. HAL 时基定时器选择

FreeRTOS 使用 **SysTick** 做任务调度时基。将 `HAL_Delay()` 的时基移到 **TIM6** 避免了经典的系统冲突——这是 ST 官方推荐的 FreeRTOS + HAL 共存方案。

---

## 🛠 硬件清单

| 序号 | 组件 | 型号 | 接口 | 用途 |
|------|------|------|------|------|
| 1 | 主控 | STM32F407VET6 | - | Cortex-M4 @ 168 MHz |
| 2 | 温湿度 | AHT20-F | I²C（0x38） | 环境温湿度采集 |
| 3 | 电源监控 | INA226 | I²C（0x40） | 总线电压、电流、功率 |
| 4 | 高精度 RTC | SD3078 | I²C（0x32） | 精确时间戳（±3.8 ppm） |
| 5 | 六轴 IMU | ICM-42688 | SPI2（PE7 CS） | 加速度 + 角速度 |
| 6 | 屏幕 | 2.0 寸 ST7789V2 | SPI1（21 MHz） | 多页数据展示 |
| 7 | TF 卡 | 核心板卡座 | SDIO 4-bit（12 MHz） | CSV 数据记录 |
| 8 | 蓝牙 | XW040 | USART2（9600 bps） | 无线数据推送 |
| 9 | SPI Flash | W25Q128 | SPI2（PE4 CS） | 校准数据存储 |
| 10 | RGB LED | 3 颗 WS2812 | TIM5_CH4 PWM+DMA | 状态指示 |
| 11 | EEPROM | AT24C02 | I²C（0x50） | 配置存储 |
| 12 | IO 扩展 | PCA9555PW | I²C（0x20） | GPIO 扩展 |
| 13 | 用户输入 | 3 按键 + EC11 | GPIO | 页面切换、导航 |
| 14 | I²C 缓冲 | PCA9517 | I²C | 电平转换、信号整形 |

### I²C 设备地址表

| 设备 | 地址 | 备注 |
|------|------|------|
| ES8388 | 0x10 | 音频 Codec（本项目未使用） |
| PCA9555PW | 0x20 | IO 扩展芯片 |
| SD3078 | 0x32 | 高精度 RTC |
| AHT20-F | 0x38 | 温湿度传感器 |
| INA226 | 0x40 | 电源监控 |
| AT24C02 | 0x50 | EEPROM |

---

## 📂 项目结构

```
├── BSP/                   # 板级支持包（传感器驱动 + 外设驱动）
│   ├── Inc/               # 头文件
│   │   ├── aht20.h       # AHT20 温湿度传感器驱动
│   │   ├── ina226.h      # INA226 电源监控驱动
│   │   ├── sd3078.h      # SD3078 RTC 驱动
│   │   ├── icm42688.h    # ICM-42688 IMU 驱动
│   │   ├── lcd.h         # ST7789V2 LCD 驱动 + 字库
│   │   ├── ws2812.h      # WS2812 PWM+DMA 驱动
│   │   ├── key.h         # 按键消抖 + EC11 编码器
│   │   ├── bluetooth.h   # 蓝牙驱动 + JSON 协议
│   │   └── tf_card.h     # TF 卡 FATFS + CSV 记录
│   └── Src/               # 源文件
│       ├── aht20.c
│       ├── ina226.c
│       ├── sd3078.c
│       ├── icm42688.c
│       ├── lcd.c
│       ├── ws2812.c
│       ├── key.c
│       ├── bluetooth.c
│       └── tf_card.c
├── App/                   # 应用层（任务调度 + 业务逻辑）
│   ├── Inc/
│   │   ├── app.h         # 应用初始化入口
│   │   ├── sensor_data.h # 全局传感器数据结构 + IPC
│   │   ├── lcd_page.h    # 多页显示框架
│   │   └── app_conf.h    # 应用配置参数
│   └── Src/
│       ├── app.c         # 应用初始化 + 任务注册
│       ├── sensor_data.c # 数据管理与同步
│       ├── lcd_page.c    # 三个显示页面
│       └── freertos_tasks.c  # 所有 FreeRTOS 任务实现
├── Core/                  # HAL + FreeRTOS 配置
│   ├── Inc/               # 头文件
│   └── Src/               # 源文件（main、freertos 等）
├── Drivers/               # STM32 HAL 驱动库
├── FATFS/                 # FatFs 文件系统
├── Middlewares/            # FreeRTOS 内核
├── CMakeLists.txt          # CMake 构建（CLion / VSCode）
├── main.ioc               # STM32CubeMX 工程文件
└── startup_stm32f407xx.s   # 启动文件
```

---

## 🚀 快速开始

### 前置条件

- **STM32CubeIDE**（推荐）或 **CMake + ARM GCC 工具链**
- 硬件：STM32F407VET6 核心板 + 上述外设
- 手机：Serial Bluetooth Terminal APP（Android/iOS）

### 编译与烧录

**方式一：STM32CubeIDE**
1. `Project → Import Existing Project` 导入此目录
2. 编译（`Ctrl+B`）并烧录（`F8`）

**方式二：CMake + CLion / VSCode**
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
# 通过 OpenOCD 或 ST-Link Utility 烧录
```

### 初始设置

1. 插入 TF 卡（格式化为 FAT32）
3. 上电——LCD 显示传感器数据页
4. 手机蓝牙搜索 "SensorMonitor"，连接后用 Serial Bluetooth Terminal 查看

---

## 📊 数据格式

### CSV 日志（TF 卡）

```
Timestamp,Temp(°C),Humidity(%),Voltage(V),Current(A),Power(W),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ
2026-07-18 12:00:01,25.3,65.2,12.05,0.25,3.01,0.01,0.02,9.81,0.1,0.2,0.0
```

按天自动分割文件：`2026-07-18.csv`、`2026-07-19.csv`……

### 蓝牙 JSON 协议

```json
{"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.25,"pwr":3.01,"accel":{"x":0.01,"y":0.02,"z":9.81},"gyro":{"x":0.1,"y":0.2,"z":0.0}}
```

**手机端指令：**
- `{"cmd":"get_temp"}` → 回复当前温度
- `{"cmd":"get_all"}` → 回复所有传感器数据
- `{"cmd":"led","r":255,"g":0,"b":0}` → 设置 WS2812 颜色

---

## 🎯 面试亮点

本项目在面试中可展示以下技术能力：

| 话题 | 体现的能力 |
|------|-----------|
| **FreeRTOS 调度** | 6 个任务的 Rate-Monotonic 优先级分配 |
| **I²C 多设备** | 互斥信号量保护共享总线，6 个设备无冲突 |
| **SPI 总线共享** | 资源受限下的 CS 分时复用策略 |
| **DMA + 空闲中断** | 不定长 UART 接收的经典方案 |
| **三通道输出管线** | 差异化速率平衡带宽、磨损和用户体验 |
| **线程安全数据** | 互斥信号量 vs 队列——两种原语解决不同问题 |
| **WS2812 时序** | 硬件 PWM+DMA，非软件 bit-bang |
| **FATFS 集成** | 按日 CSV 分割、缓冲写入 + f_sync |
| **系统资源预算** | 192 KB SRAM 分配：堆、栈、DMA 缓冲 |

---

## 🔍 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| AHT20 初始化失败 | I²C 总线冲突 | 检查 PCA9517 是否正常，确认地址 0x38 |
| LCD 花屏 | SPI 时序或排线 | 检查 FPC 排线，降低 SPI 速率 |
| TF 卡挂载失败 | 卡格式或 SDIO 配置 | 格式化为 FAT32，检查时钟分频 |
| 蓝牙搜不到 | 模块未供电或波特率不对 | 检查 VCC=3.3V，默认波特率 9600 |
| WS2812 颜色不对 | GRB 顺序或 PWM 频率 | 调换 RGB 字节顺序，示波器确认 800 kHz |
| 按键不灵敏 | 消抖时间不够 | 增加消抖次数到 5 次（50 ms） |

---

## 📐 内存分配（192 KB SRAM）

| 区域 | 大小 | 说明 |
|------|------|------|
| FreeRTOS 堆 | 96 KB | heap_4 管理器 |
| 任务栈 | ~9 KB | 6 个任务（256~2048 字） |
| DMA 缓冲 | ~4 KB | SPI2 RX + USART2 RX |
| 全局变量 | ~2 KB | 传感器数据、标志位 |
| 剩余可用 | ~80 KB | 空闲可分配 |

---

## 📜 许可证

本项目仅供**学习和简历展示**。欢迎学习、改编和在此基础上构建。

---

## 👨‍💻 作者

**MOYUNNB**

*开发环境：STM32CubeIDE / CMake / CLion — STM32 HAL + FreeRTOS*
