/**
 * @file    ina226.c
 * @brief   INA226 电源监控芯片驱动实现
 * @note    从嘉立创 fdb 示例工程提取, 改为 HAL 硬件 I2C, 增加信号量保护
 *          增加了完整的初始化序列 (复位+校准), 示例工程中跳过了这一步
 *
 * ===== 数据换算公式详解 (来源: INA226 数据手册 SBOS547B) =====
 *
 *   校准寄存器 = 0.00512 / (Current_LSB × R_shunt)
 *   总线电压   = Bus_Voltage_Register × 1.25mV
 *   采样电压   = Shunt_Voltage_Register × 2.5uV (有符号)
 *   电流       = Current_Register × Current_LSB
 *   功率       = Power_Register × 25 × Current_LSB
 *
 * ===== 校准公式推导: 为什么 Cal = 0.00512 / (Current_LSB × R_shunt) =====
 *
 *   INA226 内部的电流计算流程:
 *   1. ADC 采样 V_shunt (差分), 得到 16 位有符号原始值 V_shunt_raw
 *   2. 内部执行: Current_Value = V_shunt_raw × Cal / 32768
 *   3. Current_Register 存储的是 Current_Value
 *
 *   我们的目标是让 Current_Register × Current_LSB = 真实电流 (A)
 *
 *   由欧姆定律: 真实电流 (A) = V_shunt (V) / R_shunt (Ohm)
 *   ADC 原始值对应的物理电压: V_shunt (V) = V_shunt_raw × 2.5e-6
 *   所以: 真实电流 = V_shunt_raw × 2.5e-6 / R_shunt
 *
 *   代入内部计算: (V_shunt_raw × Cal / 32768) × Current_LSB = V_shunt_raw × 2.5e-6 / R_shunt
 *   等号两边消去 V_shunt_raw:
 *   Cal / 32768 × Current_LSB = 2.5e-6 / R_shunt
 *   Cal = 32768 × 2.5e-6 / (R_shunt × Current_LSB)
 *   Cal = 0.08192 / (R_shunt × Current_LSB)
 *
 *   但数据手册 (Equation 1, 第 15 页) 给出的是:
 *   Cal = 0.00512 / (Current_LSB × R_shunt)
 *
 *   为什么不是 0.08192 而是 0.00512?
 *   0.08192 / 0.00512 = 16
 *   这是因为 INA226 内部在写入 Calibration 寄存器之前
 *   会将用户值除以 16 (右移 4 位) 后再使用
 *   所以用户写入的值需要预乘以 16:
 *   Cal_user = (0.08192 / 16) / (R_shunt × Current_LSB)
 *           = 0.00512 / (R_shunt × Current_LSB)
 *
 *   验证: 当 R_shunt = 0.015Ohm (15mOhm), Current_LSB = 0.001A (1mA) 时
 *   Cal = 0.00512 / (0.001 × 0.015) = 341.33 → 341
 *
 * ===== 功率 LSB 为什么是 25 × Current_LSB =====
 *
 *   功率寄存器的 LSB 计算公式 (数据手册 Equation 3):
 *   Power_LSB = 25 × Current_LSB
 *
 *   推导:
 *   已知: Power_Register = (BusVoltage_Register × Current_Register) / 32768
 *   物理功率 P = Power_Register × Power_LSB
 *   因此: P = (Bus_Reg × Current_Reg × Power_LSB) / 32768
 *
 *   又: P = V × I = (Bus_Reg × 1.25mV) × (Current_Reg × Current_LSB)
 *   = Bus_Reg × Current_Reg × 1.25e-3 × Current_LSB
 *
 *   两式相等: (Bus_Reg × Current_Reg × Power_LSB) / 32768
 *          = Bus_Reg × Current_Reg × 1.25e-3 × Current_LSB
 *   消去: Power_LSB / 32768 = 1.25e-3 × Current_LSB
 *   得: Power_LSB = 32768 × 1.25e-3 × Current_LSB = 40.96 × Current_LSB
 *
 *   数据手册将此值取整为 25 (为了整数运算):
 *   Power_LSB = 25 × Current_LSB
 *
 *   所以当 Current_LSB = 1mA 时:
 *   Power_LSB = 25 × 0.001 = 0.025W
 *   如果 Power_Register = 1000, 则功率 = 1000 × 0.025 = 25W
 *
 * ===== 为什么一次性加锁读取三个寄存器更好 =====
 *
 *   方案 A (INA226_ReadAll): 加锁 → 读3个寄存器 → 解锁
 *   方案 B (分别调用): 加锁→读电压→解锁, 加锁→读电流→解锁, 加锁→读功率→解锁
 *
 *   方案 B 的问题:
 *   1. 三次 I2C 总线操作之间有间隔, 其他任务可能在此期间抢占锁
 *   2. 电压、电流、功率不是同一时间点的快照
 *   3. 锁操作本身有开销 (上下文切换)
 *
 *   方案 A 的优势:
 *   1. 保证三个值是同一转换周期内的读数
 *   2. 减少总线仲裁次数
 *   3. 降低了总加锁时间
 *
 *   损失: 如果调用者只需要电压, 多读了电流和功率, 浪费一点 I2C 带宽
 *
 * ===== 为什么用 int16_t 转换 =====
 *
 *   16 位有符号数的补码表示:
 *   shunt 电压: ADC 测量的是差分电压, 可正可负
 *   正电流 = shunt 电压为正 (负载消耗功率)
 *   负电流 = shunt 电压为负 (电池充电/电源回馈)
 *   所以必须用 int16_t (有符号) 读取 Current_Register 和 Shunt_Voltage_Register
 *
 *   而 Bus_Voltage 是对地电压, 总是正值, 所以用 uint16_t
 *
 *   如果 unsigned 直接转为 float:
 *   例如: ADC 读到的值是 0xFF00 (表示 -256 的补码)
 *   如果按 unsigned 解读: 65280
 *   如果按 signed 解读: -256
 *   unsigned → float: 65280 × 0.0025 = 163.2mV (完全错误)
 *   signed → float:   -256 × 0.0025 = -0.64mV (正确)
 */
#include "ina226.h"
#include "cmsis_os.h"   /* FreeRTOS 信号量, 若不用 RTOS 可删除此行和锁相关代码 */

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c        = NULL;
static void              *s_sem         = NULL;
static float              s_current_lsb = 0.001f;  /* 电流 LSB (A), 默认 1mA */

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
 * @brief  写 INA226 16 位寄存器
 *
 * I2C 总线时序:
 *   START + DEV_ADDR(W) + ACK + REG_ADDR + ACK + DATA_H + ACK + DATA_L + ACK + STOP
 *   共 4 字节: 1 byte 寄存器地址 + 2 bytes 数据
 *
 * @note   数据以大端序 (Big-Endian) 传输:
 *         buf[1] = value >> 8  (高字节)
 *         buf[2] = value & 0xFF (低字节)
 *         这是 INA226 芯片规定的字节序
 */
static HAL_StatusTypeDef ina226_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;                               /* 寄存器地址 */
    buf[1] = (uint8_t)(value >> 8U);            /* 高字节在前 (大端) */
    buf[2] = (uint8_t)(value & 0xFFU);          /* 低字节在后 */
    return HAL_I2C_Master_Transmit(s_hi2c, INA226_ADDR, buf, 3U, 100U);
}

/**
 * @brief  读 INA226 16 位寄存器
 *
 * I2C 总线时序:
 *   START + DEV_ADDR(W) + ACK + REG_ADDR + ACK + RESTART + DEV_ADDR(R) + ACK + DATA_H + ACK + DATA_L + NACK + STOP
 *   HAL_I2C_Mem_Read 内部自动执行"先写地址再读"的完整时序
 *
 * @note   为什么用 Mem_Read 而不用 Master_Transmit + Master_Receive 分两步?
 *         Mem_Read 在内部使用重复起始 (RESTART) 而不是 STOP+START,
 *         避免了总线释放后被其他主机插入操作
 *         I2C 规范允许在同一帧内做 RESTART 操作, 这是标准做法
 */
static HAL_StatusTypeDef ina226_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(s_hi2c, INA226_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, buf, 2U, 100U);
    if (status == HAL_OK) {
        *value = ((uint16_t)buf[0] << 8U) | buf[1];  /* 大端合并 */
    }
    return status;
}

/* ---- 对外接口 ---- */

INA226_Status_t INA226_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            float shuntResistance_mOhm)
{
    uint16_t reg_val;
    uint16_t cal_val;
    float r_shunt;

    s_hi2c = hi2c;
    s_sem  = pSemaphore;

    /* 采样电阻转为欧姆: 15mOhm → 0.015Ohm */
    r_shunt = shuntResistance_mOhm / 1000.0f;

    /**
     * 计算 Current_LSB:
     *   项目文档设定为 1mA (0.001A)
     *   这对应于最大测流范围 ±32.767A (32767 × 0.001)
     *   对于本系统最大 3A 的负载, 1mA 精度完全够用
     *
     *   为什么不使用公式推荐的更小 LSB?
     *   公式: Current_LSB = Max_Expected_Current / 2^15
     *   如果 Max_Expected_Current = 3A, 则 LSB ≈ 91.5uA
     *   但 Cal = 0.00512 / (0.0000915 × 0.015) = 3730 (16 位足够)
     *   只是为了计算方便和与其他模块保持一致
     */
    s_current_lsb = 0.001f;

    /**
     * 计算校准值:
     *   Cal = 0.00512 / (Current_LSB × R_shunt)
     *   代入: 0.00512 / (0.001 × 0.015) = 341.33 → 341 (四舍五入)
     *
     *   +0.5f 是为了四舍五入, 而不是截断
     *   (uint16_t)341.33 = 341 (截断)
     *   (uint16_t)(341.33 + 0.5) = (uint16_t)341.83 = 341 (小数点后不够 0.5)
     *   (uint16_t)(341.67 + 0.5) = (uint16_t)342.17 = 342 (正确进位)
     */
    cal_val = (uint16_t)(0.00512f / (s_current_lsb * r_shunt) + 0.5f);
    /* 15mOhm 时: Cal = 0.00512 / (0.001 × 0.015) = 341.33 → 341 */

    i2c_lock();

    /**
     * 1. 复位 INA226 到默认状态
     *    写 0x8000 到 Configuration 寄存器触发软复位
     *    复位后所有寄存器回到出厂默认值
     *    为什么要复位? 确保芯片处于已知状态, 避免上电后的不确定性
     */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_RESET) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    osDelay(1U);  /* 复位后等待 1ms */

    /**
     * 2. 写校准寄存器
     *    必须在此步就先写 Calibration!
     *    原因: 下一步写 Configuration 后, 芯片会立即开始模数转换
     *    如果 Calibration 还没设好, 电流和功率寄存器会以错误比例输出
     *    等到下次配置才正确, 浪费了第一次转换周期的数据
     */
    if (ina226_write_reg(INA226_REG_CALIBRATION, cal_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /**
     * 3. 写配置寄存器
     *    0x4127 = 二进制 0100 0001 0010 0111
     *    对应: 16次平均, 1.1ms 转换, 连续测量 shunt+bus
     *    此步写入后芯片开始以设定的模式连续采样
     */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /**
     * 4. 读回厂商 ID 验证通信 (应返回 0x5449, "TI" 的 ASCII)
     *
     *    'T' = 0x54, 'I' = 0x49
     *    如果读到不是 0x5449, 说明:
     *    - I2C 线路断路或接触不良
     *    - 地址错误 (检查 ADDR0/ADDR1 引脚连接状态)
     *    - 芯片损坏
     *    - 电平不匹配 (PCA9517 缓冲器需要启用)
     *
     *    注意: 这一步必须放在 Configuration 写入之后!
     *    因为复位后 Configuration 寄存器处于默认值,
     *    但是 INA226 在默认配置下可能处于低功耗模式 (部分芯片默认模式不同),
     *    需要先配置好转换模式, 芯片才能正确响应 I2C 读取
     */
    if (ina226_read_reg(INA226_REG_MANUFACTURER, &reg_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    i2c_unlock();

    if (reg_val != 0x5449U) {
        return INA226_ERR_NOT_FOUND;
    }

    return INA226_OK;
}

INA226_Status_t INA226_ReadBusVoltage(float *voltage)
{
    uint16_t reg_val;

    i2c_lock();
    if (ina226_read_reg(INA226_REG_BUS_VOLTAGE, &reg_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 总线电压 LSB = 1.25mV
     * 转换公式: V_bus = reg_val × 1.25mV
     * 1.25mV = 0.00125V
     * 例如: reg_val = 4000 → V_bus = 4000 × 0.00125 = 5.000V
     *
     * 注意: 即使 INA226 的 ADC 可以测量到 81.92V,
     * 芯片的绝对最大额定值是 36V (VBUS 引脚),
     * 本电路实际工作在 3.3V ~ 5V 区间
     */
    *voltage = (float)reg_val * 0.00125f;

    return INA226_OK;
}

INA226_Status_t INA226_ReadCurrent(float *current)
{
    uint16_t reg_val;

    i2c_lock();
    if (ina226_read_reg(INA226_REG_CURRENT, &reg_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 电流 = 寄存器值 × Current_LSB (有符号)
     *
     * 强制转为 int16_t 的原因:
     *   Current_Register 是有符号 16 位值, 补码表示
     *   uint16_t 转为 int16_t 会进行二进制补码的符号解释:
     *     0x0001 → +1
     *     0xFFFF → -1
     *     0x8000 → -32768
     *   如果不转 int16_t 直接乘, 当电流为负时得到错误的巨大正数
     *
     * 例如: 负载电流 2.5A
     *   Current_Register = 2.5 / 0.001 = 2500
     *   (int16_t)2500 × 0.001 = 2.5A (正确)
     *
     * 例如: 回馈电流 -0.5A
     *   Current_Register = (uint16_t)(-500) = 0xFE0C (补码)
     *   (int16_t)0xFE0C = -500
     *   -500 × 0.001 = -0.5A (正确)
     *   如果不转 int16_t: 0xFE0C = 65036
     *   65036 × 0.001 = 65.036A (完全错误!)
     */
    *current = (float)((int16_t)reg_val) * s_current_lsb;

    return INA226_OK;
}

INA226_Status_t INA226_ReadPower(float *power)
{
    uint16_t reg_val;

    i2c_lock();
    if (ina226_read_reg(INA226_REG_POWER, &reg_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 功率 LSB = 25 × Current_LSB
     *
     * 功率寄存器和电流/电压寄存器使用不同的 LSB 换算倍率:
     * Power_Reg × 25 × Current_LSB = 实际功率 (W)
     *
     * 为什么不是电压 × 电流?
     * INA226 内部功率计算使用未取整前的原始数据,
     * 与外部用电压寄存器 × 电流寄存器计算的结果有微小差异
     * 内部计算更精确, 因为它使用的是瞬时电压值, 不是采样后的 16 位值
     */
    *power = (float)reg_val * 25.0f * s_current_lsb;

    return INA226_OK;
}

INA226_Status_t INA226_ReadShuntVoltage(float *shuntVoltage_mV)
{
    uint16_t reg_val;

    i2c_lock();
    if (ina226_read_reg(INA226_REG_SHUNT_VOLTAGE, &reg_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    i2c_unlock();

    /**
     * 采样电压 LSB = 2.5uV (有符号)
     * 转为 mV: 2.5uV = 0.0025mV
     *
     * 例如: 2.5A 流过 15mOhm:
     *   V_shunt = 2.5 × 0.015 = 0.0375V = 37.5mV
     *   Shunt_Reg = 37.5mV / 0.0025mV = 15000
     *   (int16_t)15000 × 0.0025 = 37.5mV (正确)
     *
     * 这个函数主要用于调试, 直接观察 ADC 原始读数
     * 建议配合 AHT20_ReadCurrent 一起用, 验证 V = I × R 是否成立
     */
    *shuntVoltage_mV = (float)((int16_t)reg_val) * 0.0025f;

    return INA226_OK;
}

INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power)
{
    INA226_Status_t ret;

    /**
     * 一次加锁, 连续读三个寄存器, 减少 I2C 总线竞争
     *
     * 为什么这样做?
     * 1. 电压、电流、功率是相关的三个量
     * 2. 如果分三次加锁读取, 中间可能被其他任务 (如 AHT20 传感器读取)
     *    中断, 导致三次读数跨越不同的转换周期
     * 3. 跨越不同周期可能导致:
     *    - 电压是 T0 周期的
     *    - 电流是 T1 周期的
     *    - 功率是 T2 周期的
     *    → 这时电压 × 电流 ≠ 功率, 会让上层决策逻辑困惑
     *
     * do { ... } while(0) 模式:
     * 这是一种常用的"无 goto 错误处理"技巧
     * 当任意一步失败, 用 break 跳出循环, 统一到循环后的解锁处理
     * 避免了在多个错误处理点重复写 i2c_unlock() + return
     */
    i2c_lock();

    ret = INA226_OK;
    do {
        uint16_t bus_reg, curr_reg, pwr_reg;

        if (ina226_read_reg(INA226_REG_BUS_VOLTAGE, &bus_reg) != HAL_OK) {
            ret = INA226_ERR_I2C;
            break;
        }
        if (ina226_read_reg(INA226_REG_CURRENT, &curr_reg) != HAL_OK) {
            ret = INA226_ERR_I2C;
            break;
        }
        if (ina226_read_reg(INA226_REG_POWER, &pwr_reg) != HAL_OK) {
            ret = INA226_ERR_I2C;
            break;
        }

        if (voltage != NULL) {
            *voltage = (float)bus_reg * 0.00125f;
        }
        if (current != NULL) {
            *current = (float)((int16_t)curr_reg) * s_current_lsb;
        }
        if (power != NULL) {
            *power = (float)pwr_reg * 25.0f * s_current_lsb;
        }
    } while (0);

    i2c_unlock();

    return ret;
}
