/**
 * @file    ina226.c
 * @brief   INA226 电源监控芯片驱动实现
 * @note    从嘉立创 fdb 示例工程移植, 改为 HAL 硬件 I2C + 信号量保护,
 *          增加完整初始化序列 (示例工程跳过了此步)
 */
#include "ina226.h"
#include "cmsis_os.h"

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c        = NULL;
static void              *s_sem         = NULL;
static float              s_current_lsb = 0.001f;  /* 电流 LSB, 默认 1mA */

static void i2c_lock(void)
{
    if (s_sem != NULL) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}

static void i2c_unlock(void)
{
    if (s_sem != NULL) osSemaphoreRelease((osSemaphoreId_t)s_sem);
}

/*
 * 写 16 位寄存器:
 *   START + DEV_ADDR(W) + REG_ADDR + DATA_H + DATA_L + STOP
 *   INA226 使用大端序 (MSB first)
 */
static HAL_StatusTypeDef ina226_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 8U);            /* 高字节在前 */
    buf[2] = (uint8_t)(value & 0xFFU);          /* 低字节在后 */
    return HAL_I2C_Master_Transmit(s_hi2c, INA226_ADDR, buf, 3U, 100U);
}

/*
 * 读 16 位寄存器:
 *   START + DEV_ADDR(W) + REG_ADDR + RESTART + DEV_ADDR(R) + DATA_H + DATA_L + STOP
 *   用 Mem_Read, HAL 内部自动做 "写寄存器地址 + RESTART + 读"
 *   (比 Master_Transmit+Master_Receive 多一步, 但避免总线释放后被其他主机插入)
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

    r_shunt = shuntResistance_mOhm / 1000.0f;  /* mOhm → Ohm */

    /*
     * Current_LSB = 1mA:
     * 对应最大测流范围 ±32.767A (32767 × 0.001)
     * 本系统最大负载 ~3A, 1mA 精度够用
     * 如果按公式 Max_I/2^15 = 3/32768 ≈ 91.5µA, 但 Cal 值大了不好算
     */
    s_current_lsb = 0.001f;

    /*
     * Cal = 0.00512 / (Current_LSB × R_shunt)
     * 代入: 0.00512 / (0.001 × 0.015) = 341.33 → 341
     * +0.5f 是为了四舍五入 (形参变 uint16_t 时截断)
     */
    cal_val = (uint16_t)(0.00512f / (s_current_lsb * r_shunt) + 0.5f);

    i2c_lock();

    /* 1. 软复位: 写 0x8000 (所有寄存器回默认值) */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_RESET) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    osDelay(1U);

    /*
     * 2. 先写 Calibration (顺序重要!)
     *    下一步写 Configuration 后芯片立即开始转换,
     *    如果 Calibration 还没设好, 电流/功率以错误比例输出
     */
    if (ina226_write_reg(INA226_REG_CALIBRATION, cal_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /* 3. 写 Configuration: 启动连续测量 */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /*
     * 4. 验证厂商 ID (应返回 0x5449 = "TI" ASCII)
     *    这一步必须放在 Configuration 之后, 因为默认模式可能处于低功耗,
     *    配置好转换模式后芯片才能正确响应 I2C 读取
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

    /* LSB=1.25mV: reg=4000 → 5.000V */
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

    /*
     * 强制转 int16_t: current_register 是有符号值 (补码)
     * 如果不转, 负电流 (回馈/充电) 会被当做巨大的正数
     * 例: 0xFE0C = -500, (int16_t) 后 ×0.001 = -0.5A
     * 不转: 65036×0.001 = 65.036A (错误)
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

    /*
     * Power_LSB = 25 × Current_LSB
     * 为什么是 25? 手册公式: P = (Bus_Reg×Current_Reg×25×Current_LSB)/32768
     * 这个 25 是推导取整的结果 (精确计算是 40.96), 为了整数运算
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

    /* LSB=2.5µV, 输出 mV: reg=15000 → 37.5mV (对应 15mOhm@2.5A) */
    *shuntVoltage_mV = (float)((int16_t)reg_val) * 0.0025f;

    return INA226_OK;
}

/*
 * 一次加锁读三寄存器: 保证电压/电流/功率是同一转换周期的快照
 *
 * 如果分三次加锁, 中间可能被其他 I2C 任务 (如 AHT20) 打断,
 * 导致电压来自 T0, 电流来自 T1, 功率来自 T2, 相乘结果不自洽。
 *
 * do{...}while(0): 无 goto 错误处理技巧, break 跳到统一的解锁+return
 */
INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power)
{
    INA226_Status_t ret;

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

        if (voltage != NULL) *voltage = (float)bus_reg * 0.00125f;
        if (current != NULL) *current = (float)((int16_t)curr_reg) * s_current_lsb;
        if (power != NULL)   *power   = (float)pwr_reg * 25.0f * s_current_lsb;
    } while (0);

    i2c_unlock();

    return ret;
}
