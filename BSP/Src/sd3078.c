/**
 * @file    sd3078.c
 * @brief   SD3078 高精度 RTC 驱动实现
 * @note    从嘉立创 fdb 示例工程移植, 将底层从 soft_i2c 改为 HAL 硬件 I2C,
 *          增加信号量保护以适配 FreeRTOS 多任务环境。
 *
 * SD3078 写保护机制:
 *   写操作前需依次设置 CTRL2.WRTC1=1 + CTRL1.WRTC2=WRTC3=1,
 *   写完后清空这三个位, 防止意外篡改。
 *
 * 时间寄存器地址: 0x00~0x06 (秒分时周日日月年), BCD 编码
 *
 * ============================================================
 * SD3078 寄存器地址映射
 * ============================================================
 * 地址 | 名称      | bit7     | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0
 * ------+-----------+----------+------+------+------+------+------+------+------
 * 0x00 | 秒       | OSF      | 秒十位(BCD)           | 秒个位(BCD)
 * 0x01 | 分       | -        | 分十位(BCD)           | 分个位(BCD)
 * 0x02 | 时       | 24/12    | 0   | 时十位(BCD)     | 时个位(BCD)
 * 0x03 | 周       | 0-6 表示周日~周六
 * 0x04 | 日       | 0        | 0   | 日十位(BCD)     | 日个位(BCD)
 * 0x05 | 月       | 0        | 0   | 0   | 月十位    | 月个位(BCD)
 * 0x06 | 年       | 年十位(BCD)           | 年个位(BCD)
 * ...
 * 0x0F | CONTROL1 | WRTC3    | OSF | -   | -   | -   | WRTC2 | -   | -
 * 0x10 | CONTROL2 | WRTC1    | ... | ... | ... | ... | ... | ... | ...
 * 0x16 | 温度     | 温度值 (有符号 8 位, 单位 °C)
 * 0x18 | 充电     | CHGEN    | ... | ... | ... | 充电电阻配置
 * 0x1B | 电池     | 电池电压 (低 8 位), 高位在 CONTROL1.bit3
 */
#include "sd3078.h"
#include "cmsis_os.h"

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

/* ---- 寄存器地址 ---- */
#define SD3078_REG_SECOND       0x00U
#define SD3078_REG_CONTROL1     0x0FU
#define SD3078_REG_CONTROL2     0x10U
#define SD3078_REG_CHARGE       0x18U
#define SD3078_REG_TEMP         0x16U
#define SD3078_REG_BATTERY      0x1BU

/* ---- 控制寄存器位定义 ---- */
#define SD3078_CTR1_WRTC3      (1U << 7)
#define SD3078_CTR1_WRTC2      (1U << 2)
#define SD3078_CTR1_OSF        (1U << 6)  /* 晶振停振标志 */

#define SD3078_CTR2_WRTC1      (1U << 7)

#define SD3078_HOUR_24H_MODE   (1U << 7)

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
 * @brief  BCD 码转十进制
 * @param  bcd: BCD 编码的值 (如 0x59 表示 59)
 * @return 十进制值 (0~99)
 *
 * BCD (Binary-Coded Decimal) 使用 4 位二进制表示 1 位十进制:
 *   高 4 位 = 十位数, 低 4 位 = 个位数
 *
 * 位运算详解:
 *   (bcd >> 4): 将高 4 位右移到低 4 位位置
 *     例如 0x59 = 0101 1001
 *     >> 4 得到 0000 0101 = 0x05 = 5 (十位)
 *
 *   (bcd & 0x0F): 用掩码保留低 4 位, 清零高 4 位
 *     0x59 & 0x0F = 0101 1001 & 0000 1111 = 0000 1001 = 0x09 = 9 (个位)
 *
 *   ((bcd >> 4) * 10) + (bcd & 0x0F)
 *   = 5 * 10 + 9 = 59 (十进制)
 *
 * 为什么不用纯二进制?
 * 因为 RTC 硬件存储的就是 BCD, 如果按纯二进制解析:
 *   0x59 (二进制 0101 1001) = 89 (十进制) -- 错误的 89 秒!
 * 所以必须 BCD 转 十进制 得到正确的 59。
 */
static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}

/**
 * @brief  十进制转 BCD 码
 * @param  dec: 十进制值 (0~99)
 * @return BCD 编码的值
 *
 * 位运算详解:
 *   (dec / 10): 十进制除法得到十位数
 *     59 / 10 = 5 (整数除法)
 *
 *   << 4: 将十位数左移到高 4 位
 *     5 << 4 = 0101 << 4 = 0101 0000 = 0x50
 *
 *   (dec % 10): 十进制取模得到个位数
 *     59 % 10 = 9
 *
 *   ((dec / 10) << 4) | (dec % 10)
 *   = 0x50 | 0x09 = 0x59 (BCD 编码的 59)
 */
static uint8_t dec_to_bcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10U) << 4) | (dec % 10U));
}

/**
 * @brief  判断闰年
 * @param  year: 公历年份 (如 2024)
 * @return true: 闰年, false: 平年
 *
 * 闰年判断规则 (格里高利历):
 *   1. 能被 400 整除 -> 闰年 (如 2000)
 *   2. 能被 100 整除但不能被 400 -> 平年 (如 1900, 2100)
 *   3. 能被 4 整除但不能被 100 -> 闰年 (如 2024)
 *   4. 其他 -> 平年
 *
 * 为什么 400 年一闰, 100 年不闰, 4 年一闰?
 *   地球公转周期 ≈ 365.2422 天
 *   如果每 4 年加 1 天, 平均年长 = 365.25 天, 多了 0.0078 天
 *   每 100 年去掉 1 天, 平均年长 = 365.24 天, 少了 0.0022 天
 *   每 400 年再加 1 天, 平均年长 = 365.2425 天, 误差仅 0.0003 天 ≈ 26 秒
 *
 * 代码逻辑:
 *   优先级: 能被 400 整除 > 能被 100 整除 > 能被 4 整除
 *   因为能被 400 整除的也一定能被 100 整除和 4 整除,
 *   所以先从最特殊的情况 (400) 开始检查。
 */
static bool is_leap_year(uint16_t year)
{
    if ((year % 400U) == 0U) return true;
    if ((year % 100U) == 0U) return false;
    return ((year % 4U) == 0U);
}

/**
 * @brief  获取指定月份的天数
 * @param  year: 年份 (用于闰年判断)
 * @param  month: 月份 (1~12)
 * @return 该月的天数, 非法月份返回 0
 *
 * 为什么用查表法而不是计算?
 *   各月天数没有统一公式 (除了二月受闰年影响),
 *   用查表法是最直接、最不易出错的方式:
 *     - 性能: O(1) 查表, 计算法需要各种条件判断
 *     - 可读性: 直接列出 12 个月天数, 一目了然
 *     - 维护性: 无需额外解释 "7 月前单月 31 天, 8 月后双月 31 天" 的规律
 *
 * "一三五七八十腊, 三十一天永不差" -- 谚语对应数组下标
 */
static uint8_t days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1U || month > 12U) return 0U;
    uint8_t d = days[month - 1U];
    if (month == 2U && is_leap_year(year)) d = 29U;
    return d;
}

/**
 * @brief  读取多个连续的 SD3078 寄存器 (通过 I2C Mem_Read)
 *
 * HAL_I2C_Mem_Read 底层时序:
 *   START + 设备地址(写) + 寄存器地址 + RESTART + 设备地址(读) + 数据...
 *
 * 这与 HAL_I2C_Master_Recv 的区别:
 *   Master_Recv: 直接 START + 设备地址(读) + 数据
 *     适用于 "智能" 外设 (如某些传感器, 收到读命令后自动发送)
 *   Mem_Read:  先写寄存器地址, 再重新起始读
 *     适用于 "存储器映射" 外设 (如 RTC, EEPROM), 需指定内部地址
 *
 * SD3078_ADDR << 1: 将 7 位 I2C 地址左移 1 位, 空出 R/W 位
 *   HAL 函数内部在最低位填入 0 (写) 或 1 (读)
 */
static SD3078_Status_t sd3078_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(SD3078_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, 100U) != HAL_OK)
        return SD3078_ERR_I2C;
    return SD3078_OK;
}

/**
 * @brief  写入多个连续的 SD3078 寄存器
 *
 * HAL_I2C_Mem_Write 底层时序:
 *   START + 设备地址(写) + 寄存器地址 + 数据...
 *
 * 注意: 寄存器地址是 8 位, 所以用 I2C_MEMADD_SIZE_8BIT
 *   (有些 EEPROM 如 AT24C02 需要 16 位地址, 用 I2C_MEMADD_SIZE_16BIT)
 */
static SD3078_Status_t sd3078_write_regs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(SD3078_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100U) != HAL_OK)
        return SD3078_ERR_I2C;
    return SD3078_OK;
}

/**
 * @brief  使能写保护 (写时间/配置寄存器前必须先调用)
 *
 * ============================================================
 * SD3078 写保护原理
 * ============================================================
 * SD3078 用 **三个分散的位** 联合控制写使能:
 *   CTRL2 (0x10) bit 7 = WRTC1
 *   CTRL1 (0x0F) bit 2 = WRTC2
 *   CTRL1 (0x0F) bit 7 = WRTC3
 * 仅当 WRTC1 && WRTC2 && WRTC3 同时为 1 时, 时间/闹钟/充电寄存器才可写。
 *
 * 为什么这么设计?
 *   1. **防程序跑飞**: 如果仅需一个位, 一个指针错误可能恰好写入 WRTC=1
 *      导致后续误写时间寄存器。三个位 + 两个分开的寄存器大幅降低概率。
 *   2. **防 EMI**: 工业环境电磁干扰可能使某寄存器位意外翻转,
 *      但同时翻转 3 个分散位的概率极低。
 *   3. **行业惯例**: DS1307 仅需一个 WP 引脚, BQ32000 需要写 KEY 字节,
 *      SD3078 这种方式在 STM32 的 RTC 备份寄存器中也很常见。
 *
 * 操作顺序:
 *   先读回再写而非直接写 -- 因为需要保留其他位不变 (读-改-写模式)
 *   如果直接写 0x84 到 CTRL1, 会清掉 OSF 等其他位的状态。
 */
static SD3078_Status_t sd3078_enable_write(void)
{
    uint8_t ctrl;
    /* CTRL2.WRTC1 = 1 */
    if (sd3078_read_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;
    ctrl |= SD3078_CTR2_WRTC1;
    if (sd3078_write_regs(SD3078_REG_CONTROL2, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;

    /* CTRL1.WRTC3 | WRTC2 = 1 */
    if (sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;
    ctrl |= (SD3078_CTR1_WRTC3 | SD3078_CTR1_WRTC2);
    if (sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        return SD3078_ERR_I2C;

    return SD3078_OK;
}

/**
 * @brief  禁止写保护 (写完时间后调用)
 */
static SD3078_Status_t sd3078_disable_write(void)
{
    uint8_t ctrl;
    SD3078_Status_t ret = SD3078_OK;

    /* 清除 CTRL1.WRTC3 | WRTC2 */
    if (sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
        ret = SD3078_ERR_I2C;
    else {
        ctrl &= (uint8_t)~(SD3078_CTR1_WRTC3 | SD3078_CTR1_WRTC2);
        if (sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U) != SD3078_OK)
            ret = SD3078_ERR_I2C;
    }

    /* 清除 CTRL2.WRTC1 */
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

    /* 清除首次上电标志 (如果存在) */
    uint8_t ctrl;
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_CONTROL1, &ctrl, 1U);
    i2c_unlock();
    if (ret != SD3078_OK)
        return ret;

    /**
     * OSF (Oscillator Stop Flag, CTRL1 bit6):
     *   上电时为 1, 表示晶振曾停振 (例如第一次上电或电池耗尽)
     *   时间寄存器可能有无效数据, 需清除此标志让 RTC 正常工作
     *
     * 为什么清除 OSF 需要进写保护?
     *   因为 OSF 位于 CONTROL1 寄存器, 而 CONTROL1 同时也是写保护的一部分
     *   (WRTC2, WRTC3 也在其中)。修改 OSF 必须经过写保护流程。
     *   这并非所有 RTC 都如此, 但 SD3078 数据手册明确要求。
     */
    if ((ctrl & SD3078_CTR1_OSF) != 0U) {
        /* 晶振曾经停振, 清除标志 */
        i2c_lock();
        ret = sd3078_enable_write();
        if (ret == SD3078_OK) {
            ctrl &= (uint8_t)~SD3078_CTR1_OSF;
            ret = sd3078_write_regs(SD3078_REG_CONTROL1, &ctrl, 1U);
        }
        (void)sd3078_disable_write();
        i2c_unlock();
    }

    /* 配置充电 (可选) */
    if (config != NULL && config->enable_charge != 0U) {
        /**
         * 充电寄存器 (0x18):
         *   bit7 = CHGEN (充电使能)
         *   bit1:0 = 充电电阻选择
         *     0x82 = CHGEN=1, 2KΩ (推荐值)
         *   SD3078 支持对备份电池 (ML1220 或超级电容) 进行恒压/恒流充电,
         *   通过选择合适的电阻限制充电电流, 防止过充。
         */
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

SD3078_Status_t SD3078_GetTime(SD3078_Time_t *time)
{
    if (time == NULL) return SD3078_ERR_INVALID_PARAM;

    uint8_t buf[7];

    /**
     * 一次读取 7 个连续寄存器 (地址 0x00~0x06):
     *   实现原子性读取 -- 防止在两次独立读之间时间发生进位
     *   (例如先读了秒, 下一秒才读分, 但中间恰好进位了)
     *
     * 虽然我们这里没有硬件锁定, 但一次 I2C 突发读取比 7 次单独读
     * 将时间跨度的概率降低了 7 倍, 这是 RTC 驱动的通用最佳实践。
     */
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_SECOND, buf, 7U);
    i2c_unlock();
    if (ret != SD3078_OK) return ret;

    /**
     * BCD 掩码解析 -- 为什么需要对每个字段做 & 操作?
     *
     * RTC 寄存器中并非所有位都用于存储时间值:
     *
     * buf[0] = 秒寄存器
     *   bit7 = OSF (晶振停振标志), bit6:4 = 秒十位(BCD), bit3:0 = 秒个位(BCD)
     *   & 0x7F = 0b0111 1111 -- 屏蔽 bit7 (OSF), 只保留时间部分
     *
     * buf[1] = 分寄存器 (bit7 保留, 一般无意义, 同样屏蔽)
     *   & 0x7F
     *
     * buf[2] = 时寄存器
     *   bit7 = 12/24 小时制标志 (1=24H), bit6 = AM/PM (12H 制时有效)
     *   & 0x3F = 0b0011 1111 -- 屏蔽 bit7 和 bit6
     *
     * buf[3] = 星期寄存器, 值范围 0~6, 只需低 3 位:
     *   & 0x07 = 0b0000 0111
     *
     * buf[4] = 日寄存器:
     *   bit5:4 = 日十位(BCD), bit3:0 = 日个位(BCD)
     *   & 0x3F = 0b0011 1111 -- 日最大 31, BCD 范围 0x01~0x31, 高位无用
     *
     * buf[5] = 月寄存器:
     *   bit4 = 月十位(BCD), bit3:0 = 月个位(BCD)
     *   & 0x1F = 0b0001 1111 -- 月最大 12, BCD 范围 0x01~0x12
     *
     * buf[6] = 年寄存器:
     *   纯 BCD, 所有位都是时间, 无需掩码, 但取值 0x00~0x99 代表 2000~2099
     */
    time->second  = bcd_to_dec(buf[0] & 0x7FU);
    time->minute  = bcd_to_dec(buf[1] & 0x7FU);
    time->hour    = bcd_to_dec(buf[2] & 0x3FU);  /* 24 小时制 */
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
    /**
     * SD3078_HOUR_24H_MODE = 0x80 (bit7 = 1):
     *   时寄存器的 bit7 写入 1 表示使用 24 小时制,
     *   写入 0 则启用 12 小时制 (此时 bit6 表示 AM/PM)
     *   本项目统一使用 24 小时制, 故始终设置此位。
     */
    buf[2] = (uint8_t)(SD3078_HOUR_24H_MODE | dec_to_bcd(time->hour));
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

SD3078_Status_t SD3078_ReadTemperature(int8_t *temperature)
{
    if (temperature == NULL) return SD3078_ERR_INVALID_PARAM;

    /**
     * 温度寄存器 (0x16) 存的是温度补偿晶振内部传感器测量的温度值,
     * 以有符号 8 位整数格式 (补码), 单位 °C, 步长 1°C。
     * 范围 -128~127°C, 实际有效范围约 -40~85°C。
     *
     * 此温度值也用于晶振的自动频率补偿:
     *   温度变化 -> 频率变化 -> 查补偿表 -> 调整负载电容 -> 保持频率稳定
     * 用户读取它主要用于监测环境温度, 并非补偿所必需。
     */
    uint8_t raw;
    i2c_lock();
    SD3078_Status_t ret = sd3078_read_regs(SD3078_REG_TEMP, &raw, 1U);
    i2c_unlock();
    if (ret != SD3078_OK) return ret;

    *temperature = (int8_t)raw;
    return SD3078_OK;
}

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

    /**
     * 电池电压寄存器 (0x1B) - 10mV LSB:
     *
     * SD3078 的电池电压测量是一个 9 位 ADC 结果:
     *   CONTROL1.bit3 = 高 1 位 (相当于 bit8, 值 0x100)
     *   0x1B 寄存器  = 低 8 位
     *
     * 为什么是 9 位?
     *   因为要测量的电池电压范围 (典型 2.0~3.6V) 用 8 位 (0~255) 步长约 14mV,
     *   精度不够。用 9 位 (0~511) 步长 10mV, 覆盖 0~5.11V, 正好够用。
     *
     * 为什么 LSB = 10mV?
     *   这是芯片内部 ADC 参考电压和分压比决定的:
     *     ADC 参考电压 Vref / 2^9 量程 = 5.12V / 512 ≈ 10mV
     *   所以 raw_value * 10 = 电压值 (mV)
     *
     * 代码逻辑:
     *   ((status & 0x08) != 0U) 检查 CONTROL1.bit3 (0x08 的二进制 0000 1000)
     *   如果为 1, 则高 1 位 = 0x100, 否则为 0
     *
     * 典型值:
     *   3.0V 电池 -> 3.0 * 1000 / 10 = 300
     *   raw = 0x12C (二进制 1 0010 1100)
     *     status bit3 = 1, raw = 0x2C = 44
     *     val = 0x100 | 0x2C = 0x12C = 300
     *     battery_mv = 300 * 10 = 3000 mV
     */
    uint16_t val = ((status & 0x08U) != 0U) ? (uint16_t)(0x100U | raw) : raw;
    *battery_mv = val * 10U;  /* LSB = 10mV */
    return SD3078_OK;
}
