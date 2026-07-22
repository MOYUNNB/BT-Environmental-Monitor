/**
 * @file    aht20.c
 * @brief   AHT20 温湿度传感器驱动实现
 * @note    从嘉立创 fdb 示例工程移植, 改为 HAL 硬件 I2C + 信号量保护
 */
#include "aht20.h"
#include "cmsis_os.h"   /* FreeRTOS 信号量 */

/* 模块内部变量 */
static I2C_HandleTypeDef *s_hi2c   = NULL;
static void              *s_sem    = NULL;

/* ---- 锁封装: 用 void* 避免 BSP 层对 RTOS 的依赖 ---- */
/* 传入 NULL 时跳过加锁, 支持裸机调试 */
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
 * @brief  读状态寄存器 (0x71)
 * @param  status: 输出状态值
 * @retval HAL_OK 成功, 否则 I2C 错误
 * @note   用 Mem_Read: 先写子地址 0x71, 再读 1 字节, 中间 RESTART 不释放总线
 *         Master_Transmit/Receive 则需 STOP+START, 不如 RESTART 高效
 */
static HAL_StatusTypeDef aht20_read_status(uint8_t *status)
{
    *status = 0xFFU;
    return HAL_I2C_Mem_Read(s_hi2c, AHT20_ADDR, 0x71U, I2C_MEMADD_SIZE_8BIT,
                            status, 1U, 100U);
}

/* ---- 对外接口 ---- */

AHT20_Status_t AHT20_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    uint8_t cmd[3];
    uint8_t status;

    s_hi2c = hi2c;
    s_sem  = pSemaphore;

    /* 发送初始化命令 0xBE + 0x08 + 0x00 (三个字节缺一不可) */
    cmd[0] = AHT20_CMD_INIT;
    cmd[1] = 0x08U;     /* 保留, 手册要求发 0x08 */
    cmd[2] = 0x00U;     /* 保留, 手册要求发 0x00 */

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /* 等 40ms (手册标称 10ms, 留 4 倍余量, 不同批次芯片完成时间有差异) */
    osDelay(40U);

    /* 检查校准标志 bit3 */
    i2c_lock();
    if (aht20_read_status(&status) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    if (!(status & AHT20_STATUS_CALIBRATED)) {
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

    /*
     * === 依赖链: 触发→等待→读取→解析 (不可跳步, 不可乱序) ===
     * 未触发就读 → 读到旧数据; 不等够时间就读 → 数据不完整
     */

    /* 1. 发 0xAC 触发测量 */
    cmd[0] = AHT20_CMD_MEASURE;
    cmd[1] = AHT20_CMD_MEASURE_ARG0;
    cmd[2] = AHT20_CMD_MEASURE_ARG1;

    i2c_lock();
    if (HAL_I2C_Master_Transmit(s_hi2c, AHT20_ADDR, cmd, 3U, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /*
     * 2. 等 80ms — 测量等待方式的选择:
     *    方案 A (本驱动): osDelay(80ms) — 简单可靠, 但固定延时
     *    方案 B: 轮询 bit7 — 可提前返回但增加 I2C 总线负载
     *    方案 C: 中断 — AHT20 无此功能
     *    温湿度变化慢, 80ms 固定延时对系统整体影响小, 选择方案 A
     */
    osDelay(80U);

    /*
     * 3. 读 6 字节 (直接用 Master_Receive, 因为触发测量后传感器内部指针
     *    自动指向数据起始位置, 无需再发寄存器地址)
     *
     *    字节分布:
     *    [0]: 状态 | [1]: 湿度[19:12] | [2]: 湿度[11:4]
     *    [3]: 湿度[3:0]+温度[19:16] | [4]: 温度[15:8] | [5]: 温度[7:0]
     */
    i2c_lock();
    if (HAL_I2C_Master_Receive(s_hi2c, AHT20_ADDR, data, AHT20_DATA_LEN, 100U) != HAL_OK) {
        i2c_unlock();
        return AHT20_ERR_I2C;
    }
    i2c_unlock();

    /*
     * 4. 位重组 — 20 位数据分布在 3 个字节中
     *
     * 湿度: data[1]<<12 | data[2]<<4 | data[3]>>4
     * 温度: (data[3]&0x0F)<<16 | data[4]<<8 | data[5]
     *
     * 为什么湿度是 data[3]>>4 而不是 &0xF0?
     *   data[3] 高半字节=湿度低4位, 低半字节=温度高4位
     *   >>4 把高半字节移到低半字节, 低半字节被丢弃 (右移高位补0)
     *   如果 & 0xF0 再 >>4: 结果一样, 但多一次位与操作
     */
    raw_humi = ((uint32_t)data[1] << 12U)
             | ((uint32_t)data[2] << 4U)
             | ((uint32_t)data[3] >> 4U);

    raw_temp = (((uint32_t)data[3] & 0x0FU) << 16U)
             | ((uint32_t)data[4] << 8U)
             |  (uint32_t)data[5];

    /*
     * 5. 换算为工程值 (公式来源: AHT20 数据手册 V1.1)
     *
     * 湿度: RH = raw_humi × 100 / 2^20
     *   为什么分母是 2^20 而不是 2^20-1?
     *   AHT20 内部 ADC 是 20 位, 手册直接规定将 0~2^20 映射到 0%~100%RH
     *
     * 温度: T(°C) = raw_temp × 200 / 2^20 - 50
     *   raw=0 → -50°C, raw=2^20 → +150°C, 跨度 200°C
     *
     * 用 float: raw_humi×100/2^20 不是整数, AHT20 精度 0.024%RH
     */
    if (humidity != NULL) {
        *humidity = (float)raw_humi * 100.0f / 1048576.0f;  /* 2^20 = 1048576 */
    }
    if (temperature != NULL) {
        *temperature = (float)raw_temp * 200.0f / 1048576.0f - 50.0f;
    }

    return AHT20_OK;
}
