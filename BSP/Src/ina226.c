/**
 * @file    ina226.c
 * @brief   INA226 电源监控芯片驱动实现
 * @note    从嘉立创 fdb 示例工程提取, 改为 HAL 硬件 I2C, 增加信号量保护
 *          增加了完整的初始化序列 (复位+校准), 示例工程中跳过了这一步
 *
 * 数据换算公式 (来自 INA226 数据手册):
 *   校准寄存器 = 0.00512 / (Current_LSB × R_shunt)
 *   总线电压   = Bus_Voltage_Register × 1.25mV
 *   采样电压   = Shunt_Voltage_Register × 2.5μV (有符号)
 *   电流       = Current_Register × Current_LSB
 *   功率       = Power_Register × 25 × Current_LSB
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
 */
static HAL_StatusTypeDef ina226_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 8U);   /* 高字节在前 */
    buf[2] = (uint8_t)(value & 0xFFU);
    return HAL_I2C_Master_Transmit(s_hi2c, INA226_ADDR, buf, 3U, 100U);
}

/**
 * @brief  读 INA226 16 位寄存器
 */
static HAL_StatusTypeDef ina226_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(s_hi2c, INA226_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, buf, 2U, 100U);
    if (status == HAL_OK) {
        *value = ((uint16_t)buf[0] << 8U) | buf[1];
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

    /* 采样电阻转为欧姆 */
    r_shunt = shuntResistance_mOhm / 1000.0f;   /* 例如 15mΩ → 0.015Ω */

    /*
     * 计算 Current_LSB:
     *   最大预期电流 6.4A, Current_LSB = Max_I / 2^15 = 6.4 / 32768 ≈ 0.000195A
     *   但为了计算方便, 用 1mA (0.001A), 对应最大电流 = 0.001 × 32767 = 32.767A, 完全够用
     *   项目文档中指定 Current_LSB = 1mA
     */
    s_current_lsb = 0.001f;

    /* 计算校准值: Cal = 0.00512 / (Current_LSB × R_shunt) */
    cal_val = (uint16_t)(0.00512f / (s_current_lsb * r_shunt) + 0.5f);
    /* 15mΩ 时: Cal = 0.00512 / (0.001 × 0.015) = 341.33 → 341 */

    i2c_lock();

    /* 1. 复位 INA226 到默认状态 */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_RESET) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }
    osDelay(1U);  /* 复位后等待 1ms */

    /* 2. 写校准寄存器 */
    if (ina226_write_reg(INA226_REG_CALIBRATION, cal_val) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /* 3. 写配置寄存器: 16次平均, 1.1ms 转换时间, 连续转换 shunt+bus */
    if (ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT) != HAL_OK) {
        i2c_unlock();
        return INA226_ERR_I2C;
    }

    /* 4. 读回厂商 ID 验证通信 (应返回 0x5449, "TI" 的 ASCII) */
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

    /* 总线电压 LSB = 1.25mV → 转为 V */
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

    /* 电流 = 寄存器值 × Current_LSB (有符号) */
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

    /* 功率 LSB = 25 × Current_LSB */
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

    /* 采样电压 LSB = 2.5μV (有符号) → 转 mV */
    *shuntVoltage_mV = (float)((int16_t)reg_val) * 0.0025f;

    return INA226_OK;
}

INA226_Status_t INA226_ReadAll(float *voltage, float *current, float *power)
{
    INA226_Status_t ret;

    /* 一次加锁, 连续读三个寄存器, 减少 I2C 总线竞争 */
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