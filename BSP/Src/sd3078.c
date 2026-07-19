/**
 * @file    sd3078.c
 * @brief   SD3078 高精度 RTC 驱动实现
 * @note    从嘉立创 fdb 示例工程移植, soft_i2c → HAL 硬件 I2C + 信号量保护
 */
#include "sd3078.h"
#include "cmsis_os.h"

static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

/*
 * 寄存器地址映射 (0x00~0x06 = 时间, 0x0F~0x10 = 控制)
 * 地址 | 名称      | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0
 * ------+-----------+------+------+------+------+------+------+------+------
 * 0x00 | 秒        | OSF  | 秒十位(BCD)           | 秒个位(BCD)
 * 0x01 | 分        | -    | 分十位(BCD)           | 分个位(BCD)
 * 0x02 | 时        |24/12 | 0   | 时十位(BCD)     | 时个位(BCD)
 * 0x03 | 周        | 0-6 = 周日~周六
 * 0x04 | 日        | 0    | 0   | 日十位(BCD)     | 日个位(BCD)
 * 0x05 | 月        | 0    | 0   | 0   | 月十位    | 月个位(BCD)
 * 0x06 | 年        | 年十位(BCD)           | 年个位(BCD)
 * ...
 * 0x0F | CONTROL1  | WRTC3 | OSF | -   | -   | -   | WRTC2 | -   | -
 * 0x10 | CONTROL2  | WRTC1 | ...
 * 0x16 | 温度      | 有符号 8 位, °C
 * 0x18 | 充电      | CHGEN | ...
 * 0x1B | 电池      | 低 8 位, 高位在 CTRL1.bit3
 */
#define SD3078_REG_SECOND       0x00U
#define SD3078_REG_CONTROL1     0x0FU
#define SD3078_REG_CONTROL2     0x10U
#define SD3078_REG_CHARGE       0x18U
#define SD3078_REG_TEMP         0x16U
#define SD3078_REG_BATTERY      0x1BU

/* 控制位 */
#define SD3078_CTR1_WRTC3      (1U << 7)   /* 写保护锁 3 */
#define SD3078_CTR1_WRTC2      (1U << 2)   /* 写保护锁 2 */
#define SD3078_CTR1_OSF        (1U << 6)   /* 晶振停振标志 (上电=1) */

#define SD3078_CTR2_WRTC1      (1U << 7)   /* 写保护锁 1 (在 CTRL2) */

#define SD3078_HOUR_24H_MODE   (1U << 7)   /* 时寄存器 bit7=1 表示 24H */

/* ---- 内部辅助 ---- */

static void i2c_lock(void)   { if (s_sem) osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever); }
static void i2c_unlock(void) { if (s_sem) osSemaphoreRelease((osSemaphoreId_t)s_sem); }

/*
 * BCD (Binary-Coded Decimal): 高 4 位=十位, 低 4 位=个位
 * 0x59 → 5×10+9 = 59   59 → (5<<4)|9 = 0x59
 * 如果按纯二进制解读 0x59 = 89 (错误!)
 */
static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}

static uint8_t dec_to_bcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10U) << 4) | (dec % 10U));
}

/*
 * 闰年: 400 年一闰, 100 年不闰, 4 年再闰
 * 年份 2000~2099 范围内只有 2000 是整除 400 的特殊情况
 */
static bool is_leap_year(uint16_t year)
{
    if ((year % 400U) == 0U) return true;
    if ((year % 100U) == 0U) return false;
    return ((year % 4U) == 0U);
}

/* "一三五七八十腊, 三十一天永不差" */
static uint8_t days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1U || month > 12U) return 0U;
    uint8_t d = days[month - 1U];
    if (month == 2U && is_leap_year(year)) d = 29U;
    return d;
}

/*
 * 寄存器读写: 用 Mem_Read/Mem_Write
 * 因为 RTC 是"存储器映射"外设, 需要先指定内部地址再传数据
 */
static SD3078_Status_t sd3078_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(SD3078_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, 100U) != HAL_OK)
        return SD3078_ERR_I2C;
    return SD3078_OK;
}

static SD3078_Status_t sd3078_write_regs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(SD3078_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100U) != HAL_OK)
        return SD3078_ERR_I2C;
    return SD3078_OK;
}

/*
 * 三级写保护: "三把钥匙开一把锁"
 *   CTRL2.WRTC1 | CTRL1.WRTC2 | CTRL1.WRTC3 同时=1 才可写时间/充电寄存器
 * 这样设计:
 *   1. 防程序跑飞 — 三个位分在两个寄存器中, 误置概率极低
 *   2. 防 EMI — 同时翻转 3 个分散位的概率极低
 */
static SD3078_Status_t sd3078_enable_write(void)
{
    uint8_t ctrl;
    if (sd3078_read_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;
    ctrl |= SD3078_CTR2_WRTC1;
    if (sd3078_write_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;

    if (sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;
    ctrl |= (SD3078_CTR1_WRTC3 | SD3078_CTR1_WRTC2);
    if (sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;

    return SD3078_OK;
}

static SD3078_Status_t sd3078_disable_write(void)
{
    uint8_t ctrl;
    SD3078_Status_t ret = SD3078_OK;

    if (sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        ret = SD3078_ERR_I2C;
    else {
        ctrl &= (uint8_t)~(SD3078_CTR1_WRTC3 | SD3078_CTR1_WRTC2);
        if (sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
            ret = SD3078_ERR_I2C;
    }

    if (sd3078_read_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
        ret = SD3078_ERR_I2C;
    else {
        ctrl &= (uint8_t)~SD3078_CTR2_WRTC1;
        if (sd3078_write_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
            ret = SD3078_ERR_I2C;
    }

    return ret;
}

/* ---- 对外接口 ---- */

bool SD3078_TimeIsValid(const SD3078_Time_t *time)
{
    if (time == NULL) return false;
    if (time->year < 2000U || time->year > 2099U) return false;
    if (time->month < 1U || time->month > 12U) return false;
    if (time->day < 1U || time->day > days_in_month(time->year, time->month)) return false;
    if (time->weekday > 6U) return false;
    if (time->hour > 23U || time->minute > 59U || time->second > 59U) return false;
    return true;
}

SD3078_Status_t SD3078_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore,
                            const SD3078_Config_t *config)
{
    s_hi2c = hi2c;
    s_sem  = pSemaphore;

    /* 清除首次上电 OSF 标志 */
    uint8_t ctrl;
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U);
    i2c_unlock();
    if (ret != SD3078_OK)
        return ret;

    if ((ctrl & SD3078_CTR1_OSF) != 0U) {
        /* OSF=1 表示晶振曾停振 (首次上电或电池耗尽), 清除它 */
        i2c_lock();
        ret = sd3078_enable_write();
        if (ret == SD3078_OK) {
            ctrl &= (uint8_t)~SD3078_CTR1_OSF;
            ret = sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U);
        }
        (void)sd3078_disable_write();
        i2c_unlock();
    }

    /* 充电配置 (可选) */
    if (config != NULL && config->enable_charge != 0U) {
        uint8_t charge = (config->charge_value != 0U) ? config->charge_value : 0x82U;
        i2c_lock();
        ret = sd3078_enable_write();
        if (ret == SD3078_OK) {
            ret = sd3078_write_regs(SD3078_REG_CHARGE, &charge, 1U);
        }
        (void)sd3078_disable_write();
        i2c_unlock();
    }

    return SD3078_OK;
}

/*
 * 一次突发读 7 个寄存器 (0x00~0x06), 实现原子性读取
 * 避免在两次独立读之间时间发生进位
 */
SD3078_Status_t SD3078_GetTime(SD3078_Time_t *time)
{
    if (time == NULL) return SD3078_ERR_INVALID_PARAM;

    uint8_t buf[7];

    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_SECOND, buf, 7U);
    i2c_unlock();
    if (ret != SD3078_OK) return ret;

    /*
     * BCD 掩码: 每个寄存器的高位可能有非时间位 (OSF/24H/保留)
     * 秒: &0x7F 屏蔽 OSF | 时: &0x3F 屏蔽 24H+AM/PM
     * 日: &0x3F | 月: &0x1F | 周: &0x07
     */
    time->second  = bcd_to_dec(buf[0] & 0x7FU);
    time->minute  = bcd_to_dec(buf[1] & 0x7FU);
    time->hour    = bcd_to_dec(buf[2] & 0x3FU);
    time->weekday = buf[3] & 0x07U;
    time->day     = bcd_to_dec(buf[4] & 0x3FU);
    time->month   = bcd_to_dec(buf[5] & 0x1FU);
    time->year    = (uint16_t)(2000U + bcd_to_dec(buf[6]));

    if (!SD3078_TimeIsValid(time))
        return SD3078_ERR_INVALID_PARAM;

    return SD3078_OK;
}

SD3078_Status_t SD3078_SetTime(const SD3078_Time_t *time)
{
    if (!SD3078_TimeIsValid(time))
        return SD3078_ERR_INVALID_PARAM;

    uint8_t buf[7];
    buf[0] = dec_to_bcd(time->second);
    buf[1] = dec_to_bcd(time->minute);
    buf[2] = (uint8_t)(SD3078_HOUR_24H_MODE | dec_to_bcd(time->hour)); /* bit7=1 表示 24H */
    buf[3] = time->weekday;
    buf[4] = dec_to_bcd(time->day);
    buf[5] = dec_to_bcd(time->month);
    buf[6] = dec_to_bcd((uint8_t)(time->year % 100U));

    i2c_lock();
    SD3078_Status_t ret = sd3078_enable_write();
    if (ret == SD3078_OK)
        ret = sd3078_write_regs(SD3078_REG_SECOND, buf, 7U);
    (void)sd3078_disable_write();
    i2c_unlock();

    return ret;
}

/* 温度寄存器 (0x16): 芯片内部结温, 用于 TCXO 自动补偿, 也可监测环境 */
SD3078_Status_t SD3078_ReadTemperature(int8_t *temperature)
{
    if (temperature == NULL) return SD3078_ERR_INVALID_PARAM;

    uint8_t raw;
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_TEMP, &raw, 1U);
    i2c_unlock();
    if (ret != SD3078_OK) return ret;

    /* 有符号 8 位, 单位 °C */
    *temperature = (int8_t)raw;
    return SD3078_OK;
}

/*
 * 电池电压: 9 位 ADC, LSB=10mV
 *   CONTROL1.bit3 = 高 1 位 (值 0x100)
 *   0x1B 寄存器  = 低 8 位
 *   val = raw_high : raw_low, battery = val × 10 (mV)
 * 例: 3.0V → 300 → status bit3=1, raw=0x2C → val=0x12C → 300×10=3000mV
 */
SD3078_Status_t SD3078_ReadBattery(uint16_t *battery_mv)
{
    if (battery_mv == NULL) return SD3078_ERR_INVALID_PARAM;

    uint8_t raw, status;
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_CONTROL1, &status, 1U);
    if (ret == SD3078_OK)
        ret = sd3078_read_regs(SD3078_REG_BATTERY, &raw, 1U);
    i2c_unlock();
    if (ret != SD3078_OK) return ret;

    uint16_t val = ((status & 0x08U) != 0U) ? (uint16_t)(0x100U | raw) : raw;
    *battery_mv = val * 10U;
    return SD3078_OK;
}
