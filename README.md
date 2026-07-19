# 🌡️ Bluetooth Wireless Environmental Monitor & Data Logger

**STM32F407-based multi-sensor environmental monitoring terminal** — real-time data acquisition, LCD display, TF card CSV logging, and Bluetooth wireless push to smartphone, all orchestrated by FreeRTOS.

[![STM32](https://img.shields.io/badge/MCU-STM32F407VET6-03234B?logo=stmicroelectronics)](https://www.st.com/)
[![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](https://www.freertos.org/)
[![Toolchain](https://img.shields.io/badge/Toolchain-CubeIDE%20%7C%20CMake%20%7C%20CLion-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)

---

## 📋 Overview

This project is a comprehensive STM32 peripheral exercise that integrates **I²C multi-device management**, **SPI bus sharing**, **SDIO file system**, **LCD display**, **Bluetooth SPP communication**, and **FreeRTOS multi-task scheduling** into a single embedded system.

> **One-liner:** Onboard sensors fully collected, multi-page LCD real-time display, TF card auto-logging CSV, Bluetooth wireless push to mobile app, EC11 rotary knob for history browsing.

### Key Features

| Feature | Implementation |
|---------|---------------|
| **Temperature & Humidity** | AHT20 via I²C, 400 kHz Fast Mode |
| **Power Monitoring** | INA226 (voltage/current/power) via I²C |
| **Precision RTC** | SD3078 (3.8ppm TCXO) via I²C |
| **6-Axis IMU** | ICM-42688 (accel + gyro) via SPI at 200 Hz |
| **Display** | 2.0" ST7789V2 SPI LCD, 3-page UI |
| **Data Logging** | TF Card via SDIO 4-bit, FATFS, CSV format |
| **Wireless** | Bluetooth SPP (XW040) via USART2, JSON protocol |
| **Status LED** | 3× WS2812 RGB, PWM+DMA driven |
| **User Input** | 3 buttons + EC11 rotary encoder |
| **SPI Flash** | W25Q128 for calibration data |
| **RTOS** | FreeRTOS with 6 tasks, queues, and mutexes |

---

## 🏗 System Architecture

### Hardware Block Diagram

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
    │ Temp/    │ │Power │     │   │   │   │    │2.0" LCD  │ │ Bluetooth│
    │ Humidity │ │Monitor│    │   │   │   │    │(SPI1,    │ │(USART2,  │
    └────┬─────┘ └──┬───┘     │   │   │   │    │ 21 MHz)  │ │ 9600bps) │
         │          │         │   │   │   │    └──────────┘ └──────────┘
         │    ┌─────┘         │   │   │   │
         ▼    ▼               │   │   │   │
    ┌─────────────────┐       │   │   │   │    ┌──────────┐ ┌──────────┐
    │   PCA9517       │       │   │   │   │    │ICM-42688│ │ W25Q128  │
    │ I²C Buffer      │       │   │   │   │    │6-Axis   │ │ SPI Flash│
    └──┬──┬──┬──┬──┬──┘       │   │   │   │    │IMU (SPI2)│ │(SPI2)   │
       │  │  │  │  │          │   │   │   │    └──────────┘ └──────────┘
       ▼  ▼  ▼  ▼  ▼          │   │   │   │
    ┌──────────────────┐      │   │   │   │    ┌──────────┐ ┌──────────┐
    │SD3078│AT24C02│   │      │   │   │   │    │ TF Card  │ │ 3× Button│
    │RTC  │EEPROM │   │      │   │   │   │    │ (SDIO,   │ │ + EC11   │
    │PCA9555│ES8388│   │      │   │   │   │    │ 12 MHz)  │ │ Encoder  │
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
                         │    I²C1 Bus        │
                         │  6 devices shared  │
                         └────────────────────┘
```

### FreeRTOS Task Architecture

| Task | Priority | Period | Stack | Function |
|------|----------|--------|-------|----------|
| `Task_Key_Scan` | **5** (highest) | 10 ms | 256 W | Button debounce, EC11 encoder reading |
| `Task_Sensor_Read` | **4** | 100 ms / 10 ms | 1024 W | Poll AHT20, INA226, ICM-42688 |
| `Task_LCD_Update` | **3** | 200 ms | 2048 W | Multi-page display refresh |
| `Task_Bluetooth` | **3** | Event-driven | 1024 W | JSON data TX / command RX |
| `Task_TF_Log` | **2** | 1 s | 2048 W | CSV file write + f_sync |
| `Task_WS2812` | **1** (lowest) | 50 ms | 256 W | RGB LED color update |

### Inter-Task Communication

| IPC Object | Type | Purpose |
|------------|------|---------|
| `xQueue_SensorData` | Queue (len 10) | Sensor data producer → consumer |
| `xQueue_BT_Command` | Queue (len 5) | Bluetooth commands dispatch |
| `xSemaphore_I2C` | Mutex | I²C bus exclusive access (6 devices) |
| `xSemaphore_SensorData` | Mutex | Global sensor data structure protection |
| `xSemaphore_SPI2` | Mutex | SPI2 bus sharing (IMU + Flash) |

---

## 🔧 Key Engineering Decisions

### 1. I²C Multi-Device Time-Division with Mutex Protection

Six devices (AHT20, INA226, SD3078, AT24C02, PCA9555PW, ES8388) share **I²C1** at **400 kHz Fast Mode** via a PCA9517 buffer. Rather than disabling interrupts (which breaks RTOS real-time guarantees), a **mutex semaphore** (`xSemaphore_I2C`) ensures exclusive bus access. The ~150 μs polling cycle for all 6 devices is a 4× improvement over Standard Mode (100 kHz).

### 2. SPI2 Bus Sharing via CS Multiplexing

ICM-42688 (IMU) and W25Q128 (Flash) share **SPI2** through separate CS pins (PE7 / PE4). This was necessary because F407VET6 has only 3 SPIs — SPI1 drives the LCD, SPI3 has no available pins. A mutex semaphore prevents concurrent access, and the BSY flag is checked before every CS transition to avoid bus contention.

### 3. DMA + IDLE Interrupt for Variable-Length Reception

Bluetooth packets are **JSON** format, ranging from 30 to 200 bytes. Three approaches were evaluated:
| Approach | Problem |
|----------|---------|
| Byte-by-byte interrupt | High CPU load, byte loss at high throughput |
| Fixed-length DMA | Short packets wait for buffer full; long packets overflow |
| **DMA Circular + IDLE IRQ** ✅ | **Zero CPU overhead during reception; IDLE fires when bus idle (>1 byte time), NDTR gives exact length** |

### 4. Triple-Channel Data Output with Staggered Rates

Sensor data flows to three outputs at **deliberately different rates**:
- **LCD**: 200 ms — human visual persistence (~100 ms) makes faster updates imperceptible
- **TF Card CSV**: 1 s — FATFS `f_write()` + `f_sync()` takes 20–50 ms; 200 ms writes would consume 25% CPU
- **Bluetooth JSON**: 1 s — at 9600 bps (~960 B/s), a 150-byte JSON packet uses 15% bandwidth

### 5. FreeRTOS Priority Scheduling (Rate-Monotonic Approach)

Priorities are assigned by **deadline strictness**, not task importance:
- **Level 5** (Key Scan): 10 ms debounce deadline — user perceives delay immediately
- **Level 4** (Sensor Read): IMU 200 Hz sampling — missed periods lose data
- **Level 3** (LCD, Bluetooth): 200 ms / 1 s — tolerant of small jitter
- **Level 2** (TF Log): 1 s — tens-of-ms delay is irrelevant
- **Level 1** (WS2812): 50 ms — purely cosmetic

### 6. Thread-Safe Sensor Data Design

Sensor data is shared between a **writer** (acquisition task) and **three readers** (LCD, Bluetooth, logging). A **mutex** guarantees atomic reads/writes, while a **queue** decouples the producer-consumer timing — two different concerns solved by two different primitives.

### 7. HAL Timer Base Selection

FreeRTOS uses **SysTick** for its scheduling tick. Moving `HAL_Delay()` to **TIM6** avoids the well-known conflict — ST's official recommendation for FreeRTOS + HAL coexistence.

---

## 🛠 Hardware Bill of Materials

| # | Component | Model | Interface | Purpose |
|---|-----------|-------|-----------|---------|
| 1 | MCU | STM32F407VET6 | - | Cortex-M4 @ 168 MHz |
| 2 | Temp/Humidity | AHT20-F | I²C (0x38) | Environment monitoring |
| 3 | Power Monitor | INA226 | I²C (0x40) | Bus voltage, current, power |
| 4 | Precision RTC | SD3078 | I²C (0x32) | Accurate timestamps (±3.8 ppm) |
| 5 | 6-Axis IMU | ICM-42688 | SPI2 (PE7 CS) | Acceleration + gyroscope |
| 6 | LCD | 2.0" ST7789V2 | SPI1 (21 MHz) | Multi-page data display |
| 7 | TF Card | Core board slot | SDIO 4-bit (12 MHz) | CSV data logging |
| 8 | Bluetooth | XW040 | USART2 (9600 bps) | Wireless data push |
| 9 | SPI Flash | W25Q128 | SPI2 (PE4 CS) | Calibration data storage |
| 10 | RGB LED | 3× WS2812 | TIM5_CH4 PWM+DMA | Status indicator |
| 11 | EEPROM | AT24C02 | I²C (0x50) | Configuration storage |
| 12 | I/O Expander | PCA9555PW | I²C (0x20) | GPIO expansion |
| 13 | User Input | 3 buttons + EC11 | GPIO | Page switch, navigation |
| 14 | I²C Buffer | PCA9517 | I²C | Level shifting, signal conditioning |

### I²C Device Address Map

| Device | Address | Note |
|--------|---------|------|
| ES8388 | 0x10 | Audio codec (unused) |
| PCA9555PW | 0x20 | I/O expander |
| SD3078 | 0x32 | Precision RTC |
| AHT20-F | 0x38 | Temp / humidity |
| INA226 | 0x40 | Power monitor |
| AT24C02 | 0x50 | EEPROM |

---

## 📂 Project Structure

```
├── App/                    # Application layer
│   ├── aht20.h/.c         # AHT20 temperature/humidity driver
│   ├── ina226.h/.c        # INA226 power monitor driver
│   ├── sd3078.h/.c        # SD3078 RTC driver
│   ├── icm42688.h/.c      # ICM-42688 IMU driver
│   ├── lcd.h/.c           # ST7789V2 LCD driver + font
│   ├── lcd_page.h/.c      # Multi-page display framework
│   ├── key.h/.c           # Button scan + debounce + EC11
│   ├── bluetooth.h/.c     # Bluetooth + JSON protocol
│   ├── ws2812.h/.c        # WS2812 PWM+DMA driver
│   ├── sensor_data.h/.c   # Global data structure + IPC
│   └── tf_card.h/.c       # TF card FATFS + CSV logger
├── BSP/                   # Board support package
├── Core/                  # HAL + FreeRTOS configuration
│   ├── Inc/               # Headers
│   └── Src/               # Sources (main, freertos, etc.)
├── Drivers/               # STM32 HAL drivers
├── FATFS/                 # FatFs file system
├── Middlewares/            # FreeRTOS kernel
├── CMakeLists.txt          # CMake build (CLion / VSCode)
├── main.ioc               # STM32CubeMX project file
└── startup_stm32f407xx.s   # Startup assembly
```

---

## 🚀 Getting Started

### Prerequisites

- **STM32CubeIDE** (recommended) or **CMake + ARM GCC Toolchain**
- Hardware: STM32F407VET6 development board with peripherals listed above
- Mobile: Serial Bluetooth Terminal app (Android/iOS)

### Build & Flash

**Option 1: STM32CubeIDE**
1. Open `Project → Import Existing Project`
2. Select this directory
3. Build (`Ctrl+B`) and flash (`F8`)

**Option 2: CMake + CLion / VSCode**
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
# Flash via OpenOCD or ST-Link Utility
```

### Initial Setup

1. Set **DIP switch BIT3 = OFF** (SPI2 mode), **BIT7 = OFF** (RGB mode)
2. Insert TF card (FAT32 formatted)
3. Power on — LCD shows sensor data page
4. BT search "SensorMonitor", connect via Serial Bluetooth Terminal

---

## 📊 Data Formats

### CSV Log (TF Card)
```
Timestamp,Temp(°C),Humidity(%),Voltage(V),Current(A),Power(W),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ
2026-07-18 12:00:01,25.3,65.2,12.05,0.25,3.01,0.01,0.02,9.81,0.1,0.2,0.0
```

Files auto-rotate daily: `2026-07-18.csv`, `2026-07-19.csv`, ...

### Bluetooth JSON Protocol
```json
{"temp":25.3,"humi":65.2,"volt":12.05,"curr":0.25,"pwr":3.01,"accel":{"x":0.01,"y":0.02,"z":9.81},"gyro":{"x":0.1,"y":0.2,"z":0.0}}
```

**Commands** (phone → device):
- `{"cmd":"get_temp"}` → Reply with current temperature
- `{"cmd":"get_all"}` → Reply with all sensor data
- `{"cmd":"led","r":255,"g":0,"b":0}` → Set WS2812 color

---

## 🎯 Interview Talking Points

This project is designed to demonstrate in interviews:

| Topic | What It Shows |
|-------|---------------|
| **FreeRTOS scheduling** | Rate-Monotonic priority assignment for 6 tasks |
| **I²C multi-device** | Mutex-protected shared bus with 6 devices |
| **SPI bus sharing** | CS multiplexing under resource constraints |
| **DMA + IDLE IRQ** | Classic variable-length UART reception pattern |
| **Triple-output pipeline** | Staggered rates balancing bandwidth, wear, and UX |
| **Thread-safe data** | Mutex vs queue — two primitives, two concerns |
| **WS2812 timing** | Hardware-timed PWM+DMA, not software bit-bang |
| **FATFS integration** | Daily CSV rotation, buffered writes with f_sync |
| **System resource budget** | 192 KB SRAM allocation: heap, stacks, DMA buffers |

---

## 🔍 Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| AHT20 init failure | I²C bus conflict | Check PCA9517, verify address 0x38 |
| LCD garbage | SPI timing / FPC | Check FPC connection, reduce SPI rate |
| TF mount failure | Format or SDIO config | Format FAT32, check clock divider |
| BT not found | Power or baud rate | Check VCC=3.3V, default baud = 9600 |
| WS2812 wrong color | GRB order / PWM freq | Swap byte order, verify 800 kHz with scope |
| Button lag | Debounce window | Increase debounce count to 5 (50 ms) |

---

## 📐 Resource Allocation (192 KB SRAM)

| Region | Size | Details |
|--------|------|---------|
| FreeRTOS Heap | 96 KB | heap_4 manager |
| Task Stacks | ~9 KB | 6 tasks (256~2048 words) |
| DMA Buffers | ~4 KB | SPI2 RX + USART2 RX |
| Global Variables | ~2 KB | Sensor data, flags |
| Remaining | ~80 KB | Free for allocations |

---

## 📜 License

This project is for **learning and portfolio purposes**. Feel free to study, adapt, and build upon it.

---

## 👨‍💻 Author

**ckj1392612377**

*Built with STM32CubeIDE / CMake / CLion — STM32 HAL + FreeRTOS*
