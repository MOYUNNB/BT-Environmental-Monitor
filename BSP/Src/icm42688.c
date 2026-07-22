/**
 * @file    icm42688.c
 * @brief   ICM-42688 六轴 IMU 驱动实现
 * @note    SPI2, CS=PE7, 需要你来实现核心逻辑！
 *
 * SPI 读写辅助函数 (icm42688_spi_read/write) 已实现完毕,
 * 你需要补充以下 TODO 函数:
 *   - ICM42688_Init:   WHO_AM_I → 软复位 → 配置 PWR_MGMT0 → 设量程/ODR
 *   - ICM42688_ReadAccel: 读 TEMP_DATA1~DATA14, 按灵敏度换算
 *   - ICM42688_ReadGyro:  同上 (与 ReadAccel 共用一次 SPI 读取更高效)
 *   - ICM42688_ReadTemp:  温度值换算 (公式: temp = raw/132.48 + 25)
 *
 * 关键寄存器:
 *   0x75  WHO_AM_I     (应返回 0x47)
 *   0x76  BANK_SEL     (先切到 Bank 0)
 *   0x11  DEVICE_CONFIG (bit0=1 软复位)
 *   0x4E  PWR_MGMT0    (0x0F = 加速度+陀螺仪 低噪声模式)
 *   0x4F  GYRO_CONFIG0 (bit7:5=FSR, bit3:0=ODR)
 *   0x50  ACCEL_CONFIG0
 *   0x1D  TEMP_DATA1   (14 字节: T+Accel+Gyro, 高字节在前)
 *
 * 参考: 嘉立创 fdb-master/0_example/SPI/spi-imu-icm42688/BSP/icm42688.c
 */
#include "icm42688.h"
#include "cmsis_os.h"

/*
 * ============================================================
 *  学习笔记: ICM-42688 SPI 通信协议详解
 * ============================================================
 *
 * ICM-42688 使用标准 SPI (Mode 0 或 Mode 3):
 *   Mode 0: CPOL=0, CPHA=0  — 空闲时 SCLK 低, 第一个时钟沿采样
 *   Mode 3: CPOL=1, CPHA=1  — 空闲时 SCLK 高, 第一个时钟沿采样
 *
 * 通信格式:
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ CS 拉低 → 发送地址字节 → 发送/接收数据字节 → CS 拉高    │
 *   │                                                        │
 *   │ 地址字节: bit7 = 1 读 / 0 写, bit6~0 = 寄存器地址      │
 *   │                                                        │
 *   │ 注意: 读操作时, 发送地址字节后 MISO 上还是上一个字节    │
 *   │ 的回应, 需要再发送一个哑字节 (dummy byte) 产生时钟      │
 *   │ 才能读到真正的数据。                                    │
 *   └─────────────────────────────────────────────────────────┘
 *
 * 为什么先发地址再传数据?
 *   这是 SPI 协议本身的特性 — SPI 是全双工, 收发同时进行。
 *   发送地址字节的同时, MISO 收到的是上一条命令的回应 (无效)。
 *   所以读操作需要额外一个字节的时钟来得到真正的数据。
 *
 * 为什么 CS 在整个事务中保持低电平?
 *   CS (Chip Select) 是片选信号, 低电平选中设备。
 *   如果 CS 在地址和数据之间拉高, ICM-42688 会认为事务结束,
 *   数据字节就被忽略了。
 *
 * 突发读取 (Burst Read):
 *   连续读取多个寄存器时, 只需发送一次地址,
 *   然后持续提供时钟, 数据就会连续从 MISO 输出。
 *   CS 在整个突发读取过程中保持低电平。
 *   示例: 读 14 字节数据需要 1(地址) + 14(数据) = 15 个时钟周期。
 *
 * ============================================================
 *  学习笔记: SPI 互斥锁为什么是 void *?
 * ============================================================
 *
 * 这个项目中的 SPI 总线 (SPI2) 被 W25Q128 和 ICM-42688 共享。
 * 如果两个任务同时访问 SPI2, 数据会互相干扰。
 *
 * 互斥信号量 (osMutex) 解决这个问题:
 *   任务 A 使用前: osSemaphoreAcquire(sem, wait)  — 上锁
 *   任务 A 使用完: osSemaphoreRelease(sem)        — 解锁
 *   任务 B 等待时: 会阻塞在 Acquire 直到锁被释放
 *
 * 为什么参数类型是 void *?
 *   这是设计模式中的"依赖倒置"原则:
 *   BSP 驱动不应该包含 CMSIS-RTOS 的头文件 (否则就绑定了 RTOS)。
 *   用 void * 传递信号量, 由应用层在调用时强制转换。
 *
 *   调用示例:
 *     ICM42688_Init(&hspi2, (void *)&xSemaphore_SPI2);
 *
 *   当 pSemaphore == NULL 时, 跳过加锁:
 *     这个特性用于裸机调试 (没有 RTOS), 或者单任务环境。
 */

/* CS = PE7 (硬编码, 与 main.h 一致) */
#define IMU_CS_PORT             GPIOE
#define IMU_CS_PIN              GPIO_PIN_7

/*
 * ============================================================
 *  学习笔记: 为什么用 #define 而不是变量?
 * ============================================================
 *
 * CS 引脚是硬件固定的 (PCB 上 PE7 接了 IMU 的 CS 引脚),
 * 运行时不会改变, 所以用 #define 编译时常量。
 * 好处:
 *   1. 不占 RAM (编译时直接替换为立即数)
 *   2. 运行速度更快 (不需要内存访问)
 *   3. 编译器可以优化掉不用的分支
 *
 * 如果将来需要软件选择多个 CS 引脚 (如 SPI 多设备),
 * 可以改为函数参数或结构体配置。
 */

/* 模块内部变量 */
static SPI_HandleTypeDef *s_hspi = NULL;
static void              *s_sem  = NULL;
static float              s_accel_lsb = 2048.0f;  /* 当前加速度 LSB 值, 默认 ±16G */
static float              s_gyro_lsb  = 16.4f;    /* 当前陀螺仪 LSB 值, 默认 ±2000dps */
/*
 * 学习笔记: s_accel_lsb / s_gyro_lsb 的作用
 *
 * 传感器输出的是原始数字量 (raw), 需要换算成物理量。
 * LSB (Least Significant Bit) = 每单位物理量对应的数字量。
 *
 * 示例: ±16G 量程
 *   满量程 = 32G
 *   ADC 分辨率 = 16 位 → 65536 个码值
 *   注意 LSB 有两种定义方式:
 *     a) LSB_per_g = 65536 / 32 = 2048 (表示每 g 对应 2048 个码)
 *     b) g_per_LSB = 32 / 65536 = 0.000488 (每个码代表多少 g)
 *
 *   在代码中, s_accel_lsb 使用的是 LSB_per_g 的定义:
 *     accel_g = raw / s_accel_lsb
 *
 *   当初始化中修改了量程设置后, 需要同步更新 s_accel_lsb:
 *     ±16G → 2048 LSB/g
 *     ±8G  → 4096 LSB/g
 *     ±4G  → 8192 LSB/g
 *     ±2G  → 16384 LSB/g
 *
 *   ⚠ 警告: 这些值需要查阅 ICM-42688 数据手册确认!
 *     不同型号的 MEMS 传感器灵敏度不同。
 *     本注释使用的是典型值, 请以数据手册为准。
 */

/* ---- SPI 总线锁 ---- */
static void spi_lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void spi_unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/* ---- CS 控制 ---- */
static void cs_select(void)   { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static void cs_deselect(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET); }

/**
 * @brief  SPI 写寄存器
 * @note   已实现, 无需修改
 *
 * 实现原理:
 *   1. 上锁 (多任务保护)
 *   2. CS 拉低 → 选中 ICM-42688
 *   3. 发送 2 字节: [地址(bit7=0=写)] [数据]
 *      注意: reg & 0x7F 确保 bit7 为 0 (写标志)
 *   4. CS 拉高 → 释放总线
 *   5. 解锁
 *
 * 时序图:
 *   CS    ━━━┛              ┏━━━
 *   SCLK  ━━┳━┳━┳━┳━┳━┳━┳━┳━┳━┳━┳━┳━┳━┳━
 *   MOSI  ══╋═A7═══A0══╋═══D7═══D0══╋═
 *           ↑ 地址字节  ↑  数据字节  ↑
 *   CS 全程低, 地址和数据连续发送
 */
static ICM42688_Status_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};
    /*
     * reg & 0x7F: 清除 bit7, 确保是写操作
     * 0x75 & 0x7F = 0x75 (写 WHO_AM_I — 但 WHO_AM_I 是只读的!)
     * 0x4E & 0x7F = 0x4E (写 PWR_MGMT0 — 可写)
     */
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, tx, 2U, 100U);
    /*
     * HAL_SPI_Transmit 参数:
     *   句柄: s_hspi         — SPI2 句柄
     *   数据: tx              — 要发送的数据
     *   长度: 2               — 地址 + 数据 = 2 字节
     *   超时: 100             — 100ms 超时
     *
     * 注意: 这里用的是阻塞式发送 (没有用 DMA 或中断)。
     * 对于只有 2 字节的 SPI 写操作, 阻塞式非常简单且足够快。
     * 只有在大数据量传输 (如 LCD 刷屏、Flash 读写) 时才需要 DMA。
     */
    cs_deselect();
    return (hal == HAL_OK) ? ICM42688_OK : ICM42688_ERR_SPI;
}

/**
 * @brief  SPI 读多个寄存器
 * @note   已实现, 无需修改
 *
 * 实现原理 (突发读取):
 *   1. 上锁
 *   2. CS 拉低
 *   3. 发送地址字节 (bit7=1=读)
 *      地址字节: 0x1D | 0x80 = 0x9D (读 TEMP_DATA1)
 *   4. 接收 len 字节数据
 *      注意: SPI 是全双工, 接收时 MOSI 仍然需要提供时钟!
 *      HAL_SPI_Receive 会自动发送 0x00 作为时钟源。
 *   5. CS 拉高
 *   6. 解锁
 *
 * 为什么读操作要 reg | 0x80?
 *   如前所述, ICM-42688 的 SPI 协议中,
 *   bit7 = 1 表示读, bit7 = 0 表示写。
 *   这个 | 0x80 操作就是设置读标志。
 *
 * 注意: 地址字节后面没有 CS 拉高!
 *   CS 在整个突发读取过程中保持低电平。
 *   (这与 W25Q128 的行为是一样的,
 *    因为 ICM-42688 和 W25Q128 共享 SPI2 总线)
 */
static ICM42688_Status_t icm42688_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80U);  /* 读标志: bit7=1 */
    cs_select();
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(s_hspi, &addr, 1U, 100U);
    /*
     * 先发地址字节, 告诉 ICM-42688 要读哪个寄存器。
     * 此时 MISO 上的数据是无效的 (上一条指令的余音),
     * 所以只发送 1 字节, 不接收。
     */
    if (hal == HAL_OK)
        hal = HAL_SPI_Receive(s_hspi, data, len, 100U);
    /*
     * 然后接收数据: HAL_SPI_Receive 内部会发送 0x00
     * 来产生时钟, 同时从 MISO 采样数据。
     * 对于突发读取, 连续的时钟脉冲会让 ICM-42688
     * 依次输出 0x1D, 0x1E, 0x1F... 的内容到 MISO。
     *
     * 也就是: 发送地址 → 读第1字节 → 内部地址+1 → 读第2字节 → ...
     */
    cs_deselect();
    return (hal == HAL_OK) ? ICM42688_OK : ICM42688_ERR_SPI;
}

/* ========== TODO: 以下函数需要你自己实现 ========== */

/*
 * ============================================================
 *  学习笔记: ICM42688_Init 实现指南
 * ============================================================
 *
 * 这是 IMU 驱动最关键的函数, 初始化失败会导致所有后续操作无效。
 *
 * 完整的初始化顺序 (来自数据手册):
 *
 *   Step 1: 保存句柄
 *     s_hspi = hspi; s_sem = pSemaphore;
 *     这样后续读取函数不需要每次都传参。
 *
 *   Step 2: CS 初始高电平
 *     cs_deselect(); — 确保 SPI 总线释放状态
 *     HAL_Delay(1);
 *
 *   Step 3: 切到 Bank 0 (确保所有寄存器可访问)
 *     icm42688_write_reg(0x76, 0x00);
 *     ICM-42688 有 4 个寄存器 Bank (0~3),
 *     BANK_SEL(0x76) 选择当前 Bank。
 *     上电后默认就在 Bank 0, 但保险起见还是写一次。
 *
 *   Step 4: 软复位
 *     icm42688_write_reg(0x11, 0x01);
 *     HAL_Delay(10);  — 等待 10ms 复位完成
 *     DEVICE_CONFIG(0x11) 的 bit0=1 触发软复位。
 *     软复位后所有寄存器恢复默认值, 包括回到 Bank 0。
 *
 *   Step 5: 读取 WHO_AM_I 并校验
 *     uint8_t whoami = 0;
 *     icm42688_read_regs(0x75, &whoami, 1);
 *     if (whoami != ICM42688_WHO_AM_I_VALUE)
 *         return ICM42688_ERR_NOT_FOUND;
 *     这是最关键的硬件连接检测! 返回 0x47 表示 SPI 通信正常。
 *     如果返回 0xFF: 可能是 CS 没选中 (读到的全是高电平)
 *     如果返回 0x00: 可能是 SPI 模式不对或 VDD 没供电
 *
 *   Step 6: 配置电源模式
 *     icm42688_write_reg(0x4E, 0x0F);
 *     HAL_Delay(50);  — 等待传感器稳定
 *     PWR_MGMT0 = 0x0F 的含义 (需要查手册确认):
 *       bit[3:2] = 11 → Gyro 低噪声模式 (Low Noise)
 *       bit[1:0] = 11 → Accel 低噪声模式 (Low Noise)
 *     其他模式: 0x0E = 低功耗模式, 0x00 = 休眠模式
 *
 *   Step 7: 配置加速度计量程和输出数据率
 *     示例: ±16G, 1kHz ODR
 *     icm42688_write_reg(0x50, (ICM42688_ACCEL_FSR_16G << 5) | 0x07);
 *     ACCEL_CONFIG0(0x50):
 *       bit[7:5] = FSR  (Full Scale Range, 满量程)
 *       bit[3:0] = ODR  (Output Data Rate, 输出数据率)
 *     ODR 取值: 0x06=100Hz, 0x07=1kHz, 0x08=200Hz, etc.
 *     更新 s_accel_lsb = 2048.0f (根据选择的量程)
 *
 *   Step 8: 配置陀螺仪量程和输出数据率
 *     示例: ±2000dps, 1kHz ODR
 *     icm42688_write_reg(0x4F, (ICM42688_GYRO_FSR_2000DPS << 5) | 0x07);
 *     GYRO_CONFIG0(0x4F): 格式同 ACCEL_CONFIG0
 *     更新 s_gyro_lsb = 16.4f (根据选择的量程)
 *
 *   注意: 所有 SPI 操作都应该在 spi_lock()/unlock() 保护中!
 *         因为 SPI2 被 W25Q128 共享。
 *
 *   注意: 如果 pSemaphore 为 NULL, spi_lock/unlock 什么也不做。
 *         这在单任务调试时很有用, 不需要创建信号量。
 */
ICM42688_Status_t ICM42688_Init(SPI_HandleTypeDef *hspi, void *pSemaphore)
{
    s_hspi = hspi;
    s_sem  = pSemaphore;
    cs_deselect();
    HAL_Delay(1U);

    uint8_t whoami = 0;
    ICM42688_Status_t ret;

    spi_lock();

    /* 1. 切到 Bank 0 */
    ret = icm42688_write_reg(0x76, 0x00);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }

    /* 2. 软复位 */
    ret = icm42688_write_reg(0x11, 0x01);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    HAL_Delay(10);

    /* 3. 读取 WHO_AM_I 校验 */
    ret = icm42688_read_regs(0x75, &whoami, 1);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    if (whoami != ICM42688_WHO_AM_I_VALUE) { spi_unlock(); return ICM42688_ERR_NOT_FOUND; }

    /* 4. 配置电源模式: 加速度+陀螺仪低噪声 */
    ret = icm42688_write_reg(0x4E, 0x0F);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    HAL_Delay(50);

    /* 5. 配置加速度计量程 ±16G, 1kHz ODR */
    ret = icm42688_write_reg(0x50, (ICM42688_ACCEL_FSR_16G << 5) | 0x07);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    s_accel_lsb = 2048.0f;

    /* 6. 配置陀螺仪量程 ±2000dps, 1kHz ODR */
    ret = icm42688_write_reg(0x4F, (ICM42688_GYRO_FSR_2000DPS << 5) | 0x07);
    if (ret != ICM42688_OK) { spi_unlock(); return ret; }
    s_gyro_lsb = 16.4f;

    spi_unlock();
    return ICM42688_OK;
}

/*
 * ============================================================
 *  学习笔记: ICM42688_ReadAccel 实现指南
 * ============================================================
 *
 * 读取加速度数据, 换算为重力加速度 (g)。
 *
 * 实现方案 A: 专用读取 (仅读加速度)
 *   1. 锁定 SPI 总线
 *   2. 从 0x1F 开始读 6 字节 (ACCEL_DATA_X1~Z2)
 *   3. 解析:
 *      int16_t accel_x = (buf[0] << 8) | buf[1];  // Big Endian
 *      int16_t accel_y = (buf[2] << 8) | buf[3];
 *      int16_t accel_z = (buf[4] << 8) | buf[5];
 *   4. 换算:
 *      *x = (float)accel_x / s_accel_lsb;
 *      *y = (float)accel_y / s_accel_lsb;
 *      *z = (float)accel_z / s_accel_lsb;
 *   5. 解锁
 *
 * 实现方案 B: 共用 14 字节读取 (推荐)
 *   1. 从 0x1D 读 14 字节 (温度 + 加速度 + 陀螺仪)
 *   2. 分别解析加速度、陀螺仪和温度
 *   3. 同时更新内部的温度缓存
 *
 * 方案 B 的优势:
 *   - 一次 SPI 事务获取所有数据, 效率更高
 *   - Accel 和 Gyro 数据在同一时刻采集 (同步采样)
 *   - 减少总线占用时间 (SPI2 共享总线)
 *
 * 方案 A 什么时候用?
 *   - 只需要加速度数据, 不需要陀螺仪时
 *   - 分时读取让代码更清晰
 *
 * 数据格式:
 *   偏移 2-3: ACCEL_DATA_X (高字节在前, int16_t)
 *   偏移 4-5: ACCEL_DATA_Y
 *   偏移 6-7: ACCEL_DATA_Z
 *
 * 为什么是 int16_t?
 *   ICM-42688 的 ADC 是 16 位, 输出范围 -32768 ~ +32767。
 *   但实际有效位数 (ENOB) 可能与 16 位不同 (受噪声影响)。
 */
ICM42688_Status_t ICM42688_ReadAccel(float *x, float *y, float *z)
{
    uint8_t buf[6];

    spi_lock();
    ICM42688_Status_t ret = icm42688_read_regs(0x1F, buf, 6);
    spi_unlock();
    if (ret != ICM42688_OK) return ret;

    int16_t raw_x = (int16_t)((uint16_t)(buf[0]) << 8U | buf[1]);
    int16_t raw_y = (int16_t)((uint16_t)(buf[2]) << 8U | buf[3]);
    int16_t raw_z = (int16_t)((uint16_t)(buf[4]) << 8U | buf[5]);

    if (x != NULL) *x = (float)raw_x / s_accel_lsb;
    if (y != NULL) *y = (float)raw_y / s_accel_lsb;
    if (z != NULL) *z = (float)raw_z / s_accel_lsb;

    return ICM42688_OK;
}

/*
 * ============================================================
 *  学习笔记: ICM42688_ReadGyro 实现指南
 * ============================================================
 *
 * 读取陀螺仪数据, 换算为角速度 (°/s)。
 *
 * 数据格式:
 *   偏移  8-9:  GYRO_DATA_X (高字节在前, int16_t)
 *   偏移 10-11: GYRO_DATA_Y
 *   偏移 12-13: GYRO_DATA_Z
 *
 * 换算公式:
 *   gyro_dps = (int16_t)raw / s_gyro_lsb
 *
 * 示例: 以 ±2000dps 量程, s_gyro_lsb = 16.4:
 *   raw = 16400 → 16400 / 16.4 = 1000 °/s (非常快速地旋转)
 *   raw = 164   → 164   / 16.4 = 10   °/s (缓慢转)
 *   raw = 0     → 0     / 16.4 = 0    °/s (静止)
 *   静止时读数应为 0, 但实际有零偏 (bias)。
 *
 * 陀螺仪零偏校准:
 *   陀螺仪在静止时理论上输出应为 0, 但由于 MEMS 工艺误差,
 *   实际会有一个固定的偏移量 (bias)。
 *
 *   校准方法:
 *   1. 在初始化后, 让设备静止 N 秒
 *   2. 采集 N 个样本, 计算平均 bias
 *   3. 后续读数: actual = raw - bias
 *
 *   注意: 零偏会随温度和供电电压变化,
 *         高精度应用需要做温度补偿。
 *
 *   本驱动中为了简化, 没有做自动零偏校准。
 *   如果需要, 可以增加一个 ICM42688_CalibrateGyro() 函数。
 *
 * 为什么 ReadAccel 和 ReadGyro 最好共用?
 *   1. ICM-42688 是同步采样器件: Accel 和 Gyro 在同一时刻采集
 *   2. 如果先读 Accel, 再读 Gyro, 两次读之间有时间差
 *   3. 这个时间差对于静止应用无所谓, 但对于快速运动应用很重要
 *   4. 最佳做法: 一次读 14 字节, 然后分别解析
 *
 * 你可以在这里设计一个合并方案:
 *   方案: ReadAccel 和 ReadGyro 都调用一个内部的
 *   icm42688_read_all() 函数, 缓存 14 字节数据。
 *   如果距离上次读取不到一定时间 (如 1ms), 直接返回缓存值。
 *   这样做的好处: 无论 ReadAccel 和 ReadGyro 哪个先调用,
 *   都不会产生额外的 SPI 事务。
 */
ICM42688_Status_t ICM42688_ReadGyro(float *x, float *y, float *z)
{
    uint8_t buf[6];

    spi_lock();
    ICM42688_Status_t ret = icm42688_read_regs(0x25, buf, 6);
    spi_unlock();
    if (ret != ICM42688_OK) return ret;

    int16_t raw_x = (int16_t)((uint16_t)(buf[0]) << 8U | buf[1]);
    int16_t raw_y = (int16_t)((uint16_t)(buf[2]) << 8U | buf[3]);
    int16_t raw_z = (int16_t)((uint16_t)(buf[4]) << 8U | buf[5]);

    if (x != NULL) *x = (float)raw_x / s_gyro_lsb;
    if (y != NULL) *y = (float)raw_y / s_gyro_lsb;
    if (z != NULL) *z = (float)raw_z / s_gyro_lsb;

    return ICM42688_OK;
}

/*
 * ============================================================
 *  学习笔记: ICM42688_ReadTemp 实现指南
 * ============================================================
 *
 * 读取 ICM-42688 芯片内部温度。
 *
 * 温度传感器特性:
 *   - 测量的是芯片内部结温 (Junction Temperature)
 *   - 不是环境温度 (芯片会自发热, 通常比室温高 5~15°C)
 *   - 范围: -40°C ~ +125°C
 *   - 精度: ±1°C (典型)
 *   - 输出: 16 位有符号值 (int16_t)
 *
 * 换算公式 (来自数据手册):
 *   TEMP_DATA = 14 位温度传感器输出, 扩展为 16 位
 *   Temperature (°C) = (TEMP_DATA / 132.48) + 25
 *
 * 公式来源:
 *   这是 ICM-42688 数据手册给出的特征方程。
 *   每个传感器出厂时都经过校准, 这个公式是校准曲线。
 *
 * 温度数据的用途:
 *   1. 监测芯片工作温度 (是否过热)
 *   2. 作为陀螺仪零偏温度补偿的输入
 *   3. 粗略估计环境温度 (不准确, 仅供参考)
 *
 * 注意: 温度传感器的响应速度较慢,
 *   芯片温度变化需要几秒到几十秒才能稳定。
 *   如果每秒读一次, 不要期望读数快速变化。
 */
ICM42688_Status_t ICM42688_ReadTemp(float *temp_c)
{
    uint8_t buf[2];

    spi_lock();
    ICM42688_Status_t ret = icm42688_read_regs(0x1D, buf, 2);
    spi_unlock();
    if (ret != ICM42688_OK) return ret;

    int16_t raw = (int16_t)((uint16_t)(buf[0]) << 8U | buf[1]);
    if (temp_c != NULL) *temp_c = (float)raw / 132.48f + 25.0f;

    return ICM42688_OK;
}

/*
 * ============================================================
 *  ICM42688_ReadAll — 一次 SPI 突发读 14 字节
 * ============================================================
 *
 * 一次读取 14 字节 (0x1D~0x2A):
 *   偏移 0-1:  TEMP_DATA      (int16_t, Big Endian)
 *   偏移 2-3:  ACCEL_DATA_X
 *   偏移 4-5:  ACCEL_DATA_Y
 *   偏移 6-7:  ACCEL_DATA_Z
 *   偏移 8-9:  GYRO_DATA_X
 *   偏移 10-11: GYRO_DATA_Y
 *   偏移 12-13: GYRO_DATA_Z
 *
 * 比分别调用 ReadAccel + ReadGyro + ReadTemp 节省 2 次 SPI 事务,
 * 且确保三组数据来自同一采样时刻。
 */
ICM42688_Status_t ICM42688_ReadAll(float *accel_x, float *accel_y, float *accel_z,
                                   float *gyro_x,  float *gyro_y,  float *gyro_z,
                                   float *temp_c)
{
    uint8_t buf[14];

    spi_lock();
    ICM42688_Status_t ret = icm42688_read_regs(0x1D, buf, 14);
    spi_unlock();
    if (ret != ICM42688_OK) return ret;

    /* 温度: buf[0..1] */
    if (temp_c != NULL) {
        int16_t raw_t = (int16_t)((uint16_t)(buf[0]) << 8U | buf[1]);
        *temp_c = (float)raw_t / 132.48f + 25.0f;
    }

    /* 加速度: buf[2..7] */
    if (accel_x != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[2]) << 8U | buf[3]);
        *accel_x = (float)raw / s_accel_lsb;
    }
    if (accel_y != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[4]) << 8U | buf[5]);
        *accel_y = (float)raw / s_accel_lsb;
    }
    if (accel_z != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[6]) << 8U | buf[7]);
        *accel_z = (float)raw / s_accel_lsb;
    }

    /* 陀螺仪: buf[8..13] */
    if (gyro_x != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[8])  << 8U | buf[9]);
        *gyro_x = (float)raw / s_gyro_lsb;
    }
    if (gyro_y != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[10]) << 8U | buf[11]);
        *gyro_y = (float)raw / s_gyro_lsb;
    }
    if (gyro_z != NULL) {
        int16_t raw = (int16_t)((uint16_t)(buf[12]) << 8U | buf[13]);
        *gyro_z = (float)raw / s_gyro_lsb;
    }

    return ICM42688_OK;
}

/*
 * ============================================================
 *  总结: IMU 数据流
 * ============================================================
 *
 * ICM-42688
 *    │
 *    │ SPI2 (共享总线, 互斥锁保护)
 *    ▼
 * icm42688_read_regs (突发读取 14 字节)
 *    │
 *    ▼
 * 原始数据 (int16_t, Big Endian)
 *    │
 *    ├──→ TEMP_DATA  →  temp = raw/132.48 + 25    (°C)
 *    ├──→ ACCEL_DATA →  accel = raw / s_accel_lsb  (g)
 *    └──→ GYRO_DATA  →  gyro  = raw / s_gyro_lsb   (°/s)
 *                       │
 *                       ▼
 * 物理量 (float)
 *    │
 *    ▼
 * 应用层: 传感器融合 (互补滤波 / Madgwick / Mahony)
 *    │
 *    ├──→ 姿态角 (Roll/Pitch/Yaw)
 *    ├──→ 步数计
 *    └──→ 运动检测
 *
 * 常见问题排查:
 *   1. 加速度 Z 轴不接近 1g?
 *      → 检查量程配置, 检查 LSB 值是否正确
 *   2. 陀螺仪静止时不为 0?
 *      → 正常! 这是零偏, 需要校准
 *   3. 温度读数明显偏高?
 *      → 芯片自发热, 属于正常现象
 *   4. 读数一直为 0?
 *      → 检查 SPI 通信, 检查电源模式配置
 *   5. 读数跳变异常?
 *      → 检查 SPI 时钟频率 (不要超过 24MHz)
 *      → 检查电源退耦电容
 */
