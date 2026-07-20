/**
 * @file    tf_card.c
 * @brief   TF 卡 FATFS + CSV 日志实现 (SDIO 4-bit, 按天分割, f_sync 每秒)
 * @note    CubeMX 已生成 diskio 层, 本模块提供应用层 CSV 接口
 *          使用 SD3078 RTC 获取日期生成按天分割的文件名
 */
#include "tf_card.h"
#include "sd3078.h"     /* RTC 日期读取 (不影响文件名时 RTC 可异常) */
#include <string.h>
#include <stdio.h>

/*
 * FATFS 对象 (CubeMX 生成, 在 fatfs 源文件中定义):
 *   SDFatFS — 文件系统状态; SDFile — 打开的文件; SDPath — 驱动路径
 */
extern FATFS SDFatFS;
extern FIL   SDFile;
extern char  SDPath[4];

extern I2C_HandleTypeDef hi2c1;

static char   s_filename[32];
static uint8_t s_mounted  = 0;
static uint8_t s_file_open = 0;

/*
 * 从 SD3078 RTC 读取日期, 生成文件名 YYYY-MM-DD.csv
 * 如果 RTC 读取失败, 使用 "00-00-00.csv" 作为降级
 */
static void get_filename(char *buf, size_t size)
{
    SD3078_Time_t rtc_time;
    SD3078_Init(&hi2c1, NULL, NULL);  /* 裸机模式, 确保 RTC 已初始化 */

    if (SD3078_GetTime(&rtc_time) == SD3078_OK) {
        snprintf(buf, size, "%04u-%02u-%02u.csv",
                 (unsigned)rtc_time.year, (unsigned)rtc_time.month, (unsigned)rtc_time.day);
    } else {
        snprintf(buf, size, "00-00-00.csv");
    }
}

/*
 * 初始化:
 *   f_mount (opt=1) → 创建当日 CSV → f_lseek 到末尾 → 新文件写表头
 */
bool TF_Init(void)
{
    FRESULT res;

    get_filename(s_filename, sizeof(s_filename));

    /* 挂载 FATFS */
    res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK) return false;
    s_mounted = 1;

    /* 尝试打开已有文件 (追加模式) */
    res = f_open(&SDFile, s_filename, FA_WRITE | FA_OPEN_EXISTING);
    if (res == FR_OK) {
        /* 文件已存在, 追加 */
        f_lseek(&SDFile, f_size(&SDFile));
    } else {
        /* 文件不存在, 新建并写表头 */
        res = f_open(&SDFile, s_filename, FA_WRITE | FA_CREATE_NEW);
        if (res != FR_OK) return false;
        f_puts(TF_CSV_HEADER, &SDFile);
    }
    s_file_open = 1;
    return true;
}

/*
 * 写入一行 CSV:
 *   2026-07-18 12:00:01,25.3,65.2,12.05,0.250,3.01,...
 *   f_write 仅写入 FATFS 缓冲区 (微秒级), f_sync 才刷到 SD 卡 (10~50ms)
 */
bool TF_LogSensor(const char *timestamp,
                  float temp, float humi,
                  float volt, float curr, float pwr,
                  const float accel_g[3], const float gyro_dps[3])
{
    if (!s_file_open) return false;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "%s,%.1f,%.1f,%.2f,%.3f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f\r\n",
        timestamp ? timestamp : "00:00:00",
        (double)temp, (double)humi,
        (double)volt, (double)curr, (double)pwr,
        (double)accel_g[0], (double)accel_g[1], (double)accel_g[2],
        (double)gyro_dps[0], (double)gyro_dps[1], (double)gyro_dps[2]);

    FRESULT res = f_puts(buf, &SDFile);
    if (res == FR_OK) {
        f_sync(&SDFile);  /* 每秒同步, 最多丢 1 秒数据 */
    }
    return (res == FR_OK);
}

/* f_sync: 强制刷出 FATFS 缓冲区到物理介质 */
void TF_Flush(void)
{
    if (s_file_open) {
        f_sync(&SDFile);
    }
}

/* 卸载: f_close → f_mount(NULL) */
void TF_Deinit(void)
{
    if (s_file_open) {
        f_close(&SDFile);
        s_file_open = 0;
    }
    if (s_mounted) {
        f_mount(NULL, SDPath, 0);
        s_mounted = 0;
    }
}
