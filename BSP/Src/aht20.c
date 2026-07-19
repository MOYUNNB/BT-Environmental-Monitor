/**
 * @file    aht20.c
 * @brief   AHT20 温湿度传感器驱动实现
 * @note    从嘉立创 fdb 示例工程提取, 改为 HAL 硬件 I2C, 增加信号量保护
 *
 * ===== 数据转换公式详解 (来源: AHT20 数据手册 V1.1) =====
 *
 *   【湿度换算】
 *      RH = (raw_humi * 100) / 2^20
 *          = (raw_humi * 100) / 1048576
 *
 *      为什么分母是 2^20?
 *      - AHT20 内部 ADC 是 20 位精度
 *      - 20 位 ADC 的最大输出值是 2^20 - 1 = 1048575
 *      - 数据手册将 0 ~ 2^20 映射到 0% ~ 100% RH
 *      - 因此 raw_humi / 2^20 得到相对比例, 乘以 100 得到百分比
 *      - 注意: 这里不是 2^20 - 1, 而是 2^20, 这是数据手册直接规定的公式
 *
 *   【温度换算】
 *      T = (raw_temp * 200) / 2^20 - 50
 *          = (raw_temp * 200) / 1048576 - 50
 *
 *      为什么用 200 和 -50?
 *      - 温度测量范围: -50°C ~ +150°C (总计 200°C 跨度)
 *      - raw_temp = 0 对应 -50°C
 *      - raw_temp = 2^20 对应 +150°C
 *      - 因此: T(°C) = (raw_temp / 2^20) * 200 - 50
 *
 *   【完整信号链】
 *      物理量 → 传感器 → 电容/电压 → ADC(20位) → 原始值 → 公式 → 工程值
 *
 * ===== 为什么 osDelay(80ms) =====
 *   - 数据手册 (Table 10) 列出的典型测量时间: 80ms (精度优先模式)
 *   - 测量时间与分辨率配置有关 (通过 0xAC 的参数 byte 可配置)
 *   - 本文使用默认配置, 需要 80ms 等待
 *   - 如果改为 osDelay(75ms), 有可能读到上一轮的结果
 *   - 如果改为 osDelay(100ms), 对系统影响不大, 但占用任务时间增加
 *   - 选择 80ms 是精度和效率的折中
 *
 * ===== 状态机步骤依赖关系 =====
 *   依赖链: 上电 → 初始化(校准) → 触发测量 → 等待 → 读取 → 解析
 *   不能跳步: 未初始化就触发测量 → 数据无效
 *            未触发测量就读取 → 读到旧数据
 *            触发测量后不等够时间就读 → 数据不完整 (高位已经更新, 低位还在跳变)
 *            读取后不解析 → 无法得到工程值
 *
 * ===== HAL_I2C_Mem_Read vs Master_Transmit/Receive =====
 *   本驱动在两种情况使用不同的 I2C 函数:
 *
 *   【HAL_I2C_Mem_Read (读状态寄存器时使用)】
 *   - I2C 总线上的行为:
 *       START + DEV_ADDR(W) + ACK + REG_ADDR(0x71) + ACK + RESTART + DEV_ADDR(R) + ACK + DATA + NACK + STOP
 *   - 适合: 需要先指定内部寄存器地址, 再读取数据的场景
 *   - HAL 内部自动完成"写子地址 → 重启动 → 读取"全过程
 *   - aht20_read_status() 用此函数, 因为需要先写状态寄存器地址 0x71
 *
 *   【HAL_I2C_Master_Transmit (发送命令时使用)】
 *       START + DEV_ADDR(W) + ACK + DATA0 + ACK + DATA1 + ACK + DATA2 + ACK + STOP
 *   - 适合: 仅发送数据, 不需要读回
 *   - AHT20_Init() 和触发测量时使用
 *
 *   【HAL_I2C_Master_Receive (读取测量数据时使用)】
 *       START + DEV_ADDR(R) + ACK + DATA0 + ACK + ... + DATAN + NACK + STOP
 *   - 适合: 不从指定子地址开始, 直接读取当前数据
 *   - AHT20_ReadData() 中的 6 字节读取使用
 *   - 为什么可以不用 Mem_Read? 因为触发了测量后, 传感器内部数据指针
 *     自动指向测量结果的起始位置, 无需再发寄存器地址
 */
#include "aht20.h"
#include "cmsis_os.h"   /* FreeRTOS 信号量, 若不用 RTOS 可删除此行和锁相关代码 */

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c   = NULL;   /* I2C 句柄 */
static void              *s_sem    = NULL;   /* 互斥信号量 */

/**
 * ===== 模块内部锁机制设计说明 =====
 *
 *   为什么用 static 函数封装锁操作?
 *   1. 避免在 BSP 层直接包含 cmsis_os.h 的复杂类型 (对外接口保持 void*)
 *   2. 统一锁管理, 修改锁策略时只需改这两个函数
 *   3. 支持 NULL 跳过: 裸机调试或单任务时可以不用创建信号量
 *   4. 后期可无缝改为中断锁 (osMutexAcquire/Release) 或调度器锁
 *
 *   为什么锁粒度这么细 (每次 I2C 操作前后加解锁)?
 *   1. I2C 操作是微秒级的, 长时间占用锁会阻塞高优先级任务 (如按键扫描 10ms 周期)
 *   2. 如果锁覆盖 osDelay(80ms), 其他需要 I2C 的任务会被阻塞 80ms, 这是不可接受的
 *   3. FreeRTOS 互斥信号量支持优先级继承, 可防止优先级反转
 */

/* ---- 内部辅助函数 ---- */

static void i2c_lock(void)
{
    if (s_sem != NULL) {
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
    }
}

static void i2c_unlock(void)
{
    if (s_sem != NULL) {
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
    }
}

/**
 * @brief  读取 AHT20 状态寄存器 (0x71)
 *
 * 状态寄存器各 bit 解释:
 *   bit7: Busy (1=测量进行中, 0=空闲)
 *   bit6-4: 保留
 *   bit3: CAL (1=校准系数已加载, 0=未校准)
 *   bit2-0: 保留
 *
 * @note   HAL_I2C_Mem_Read 在这里做了两件事:
 *         1. 先写子地址 0x71 (告诉传感器我要读哪个寄存器)
 *         2. 再读 1 字节 (传感器返回状态值)
 *         这两步在 I2C 总线上是连续完成的, 中间是 RESTART 而不是 STOP
 */
static uint8_t aht20_read_status(void)
{
    uint8_t status = 0xFFU;
    /* 状态寄存器地址 0x71, 读 1 字节 */
    HAL_I2C_Mem_Read(s_hi2c, AHT20_ADDR, 0x71U, I2C_MEMADD_SIZE_8BIT,
                     &status, 1U, 100U);
    return status;
}

/* ---- 对外接口 ---- */

AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    uint8_t cmd[3];
    uint8_t status;

    s_hi2c = hi2c;
    s_sem  = pSemaphore;

    /**
     * 为什么发送 0xBE + 0x08 + 0x00?
     * 0xBE = 初始化命令 (Initialize)
     * 0x08 = 保留字节, 但数据手册要求必须发送 0x08
     * 0x00 = 保留字节, 数据手册要求必须发送 0x00
     * 这 3 个字节缺一不可, 否则传感器不会加载校准系数
     */
    cmd[0] = AHT20_CMD_INIT;
    cmd[1] = 0x08U;
    cmd[2] = 0x00U;

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 为什么等待 40ms 而不是数据手册标称的 10ms?
     * 数据手册 (Table 8) 规定初始化需要 10ms 完成校准加载
     * 但实际中:
     *   1. 不同批次芯片完成时间有差异
     *   2. 某些情况下上电时序会影响启动时间
     *   3. 留 4 倍余量保证在各种工况下都能可靠工作
     *   4. 40ms 对于人机交互系统来说仍然可以忽略
     * 这在嵌入式开发中称为"看门狗参数" (watchdog parameter) 设计
     */
    osDelay(40U);

    /* 检查校准标志 */
    i2c_lock();
    status = aht20_read_status();
    i2c_unlock();

    if (!(status & AHT20_STATUS_CALIBRATED)) {
        /**
         * 如果校准未完成, 可能的原因:
         * 1. 上电后等待时间过短 (传感器上电需要 ≥100ms 稳定)
         * 2. 初始化命令发送失败 (I2C 时序问题)
         * 3. 传感器硬件损坏 (极少见)
         * 4. 传感器处于异常状态 (可尝试先发送软复位)
         */
        return AHT20_ERR_NOT_CALIBRATED;
    }

    return AHT20_OK;
}

AHT20_Status_t AHT20_ReadData(float *temperature, float *humidity)
{
    uint8_t  cmd[3];
    uint8_t  data[AHT20_DATA_LEN];
    uint32_t raw_humi;
    uint32_t raw_temp;

    /* 1. 发送触发测量命令 */
    cmd[0] = AHT20_CMD_MEASURE;         /* 0xAC: 触发测量 */
    cmd[1] = AHT20_CMD_MEASURE_ARG0;    /* 0x33: 开启温度+湿度测量, 默认分辨率 */
    cmd[2] = AHT20_CMD_MEASURE_ARG1;    /* 0x00: 保留 */

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 2. 等待测量完成
     *
     * 为什么 osDelay(80ms) 而不是检查忙标志?
     * 方法 A: osDelay(80ms) - 当前方案
     *   优点: 实现简单, CPU 在此任务挂起期间可以运行其他任务
     *   缺点: 固定延时, 即使传感器提前完成也得等够时间
     *
     * 方法 B: 轮询状态寄存器 bit7 (忙标志)
     *   优点: 用最短时间完成 (传感器可能在 50ms 就完成了)
     *   缺点: 需要频繁 I2C 读操作, 增加总线负载; 代码更复杂
     *         如果传感器卡死在忙状态, 需要超时机制
     *
     * 方法 C: 中断/事件触发 (AHT20 没有这个功能)
     *
     * 当前选择方法 A 的原因: AHT20 不是高速采样 (温湿度本身变化慢),
     * 80ms 等待对系统整体影响小, 代码简洁可靠
     */
    osDelay(80U);

    /**
     * 3. 读取 6 字节数据
     *
     * 使用 Master_Receive 而不是 Mem_Read 的原因:
     * 触发测量后, AHT20 内部的数据指针自动指向测量结果起始位置
     * (状态寄存器 byte0), 此时直接读即可, 不需要先写地址
     *
     * 6 字节分布:
     *   byte[0]: 状态寄存器 (同 aht20_read_status)
     *   byte[1]: 湿度数据位 [19:12] (高 8 位)
     *   byte[2]: 湿度数据位 [11:4]  (中间 8 位)
     *   byte[3]: 湿度数据位 [3:0] + 温度数据位 [19:16] (高半字节)
     *   byte[4]: 温度数据位 [15:8]  (中间 8 位)
     *   byte[5]: 温度数据位 [7:0]   (低 8 位)
     */
    i2c_lock();
    if (HAL_I2C_Master_Receive(s_hi2c, AHT20_ADDR, data, AHT20_DATA_LEN, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 4. 解析数据 (位重组)
     *
     * 湿度是 20 位数据, 分布在 byte[1:3] 中:
     *   byte[1]: HHHHHHHH (高 8 位)
     *   byte[2]: MMMMMMMM (中 8 位)
     *   byte[3]: LLLL???? (低 4 位, 高半字节)
     *
     *   raw_humi = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
     *   - data[1] << 12: 高 8 位移到 bit[19:12]
     *   - data[2] << 4:  中 8 位移到 bit[11:4]
     *   - data[3] >> 4:  低 4 位从高半字节移到 bit[3:0]
     *
     * 温度同理, 共享 byte[3] 的低半字节:
     *   byte[3]: ????TTTT (高 4 位, 低半字节)
     *   byte[4]: TTTTTTTT (中 8 位)
     *   byte[5]: TTTTTTTT (低 8 位)
     *
     *   raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
     *   - 先屏蔽 byte[3] 高半字节 (湿度位), 只取低 4 位
     *   - data[4] << 8: 中 8 位移到 bit[15:8]
     *   - data[5]:      低 8 位在 bit[7:0]
     */
    raw_humi = ((uint32_t)data[1] << 12U)
             | ((uint32_t)data[2] << 4U)
             | ((uint32_t)data[3] >> 4U);

    raw_temp = (((uint32_t)data[3] & 0x0FU) << 16U)
             | ((uint32_t)data[4] << 8U)
             |  (uint32_t)data[5];

    /**
     * 5. 换算为工程值
     *
     * 公式来源: AHT20 数据手册 V1.1 第 10 页
     *
     * 湿度: RH = raw_humi * 100 / 2^20
     *   2^20 = 1048576
     *   当 raw_humi = 500000 时, RH = 500000 * 100 / 1048576 ≈ 47.68%RH
     *
     * 温度: T = raw_temp * 200 / 2^20 - 50
     *   当 raw_temp = 600000 时, T = 600000 * 200 / 1048576 - 50 ≈ 64.42°C
     *
     * 为什么用浮点数?
     *   raw_humi * 100 / 2^20 的结果不是整数, 在 0~100 之间有无线精度的值
     *   如果用整数运算: (raw_humi * 100) / 1048576, 会丢失小数部分
     *   AHT20 的精度是 0.024%RH (1/2^20 * 100), 所以用 float 保留精度
     *
     * 为什么允许传入 NULL?
     *   - 如果调用者只需要温度, 可以传入 humidity=NULL
     *   - 如果只需要湿度, 可以传入 temperature=NULL
     *   - 这样减少了栈上不必要的赋值操作
     */
    if (humidity != NULL) {
        *humidity = (float)raw_humi * 100.0f / 1048576.0f;       /* 2^20 = 1048576 */
    }
    if (temperature != NULL) {
        *temperature = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;
    }

    return AHT20_OK;
}
