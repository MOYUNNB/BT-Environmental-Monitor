/**
 * @file    tf_card.c
 * @brief   TF 卡 FATFS + CSV 日志实现 (SDIO 4-bit, 按天分割, f_sync 每秒)
 * @note    CubeMX 已生成 diskio 层, 本模块提供应用层 CSV 接口
 */
#include "tf_card.h"
#include <string.h>
#include <stdio.h>

/*
 * FATFS 对象 (CubeMX 生成, 在 fatfs 源文件中定义):
 *   g_SDFatFS — 文件系统状态; g_SDFile — 打开的文件; g_SDPath — 驱动路径
 */
extern FATFS g_SDFatFS;
extern FIL   g_SDFile;
extern char  g_SDPath[4];

static char   s_filename[32];
static uint8_t s_mounted  = 0;
static uint8_t s_file_open = 0;

/*
 * 文件名: YYYY-MM-DD.csv
 * 从 RTC (SD3078 或 STM32 内部 RTC) 读取日期
 */
static void get_filename(char *buf, size_t size)
{
    /* TODO: snprintf(buf, size, "%04u-%02u-%02u.csv", year, month, day); */
    (void)buf; (void)size;
}

/*
 * 初始化:
 *   f_mount (opt=1) → 创建当日 CSV → f_lseek 到末尾 → 新文件写表头
 *
 * 挂载策略 (CubeMX 惯用):
 *   先 MX_FATFS_Init 中 f_mount(opt=0) 注册但不挂载
 *   这里再 f_mount(opt=1) 实际挂载 (读 BPB+FAT 表)
 *   两步法避免上电初期 SD 卡未就绪导致超时
 */
bool TF_Init(void)
{
    FRESULT res;
    (void)res;
    /* TODO: 实现 */
    return false;
}

/*
 * 写入一行 CSV:
 *   2026-07-18 12:00:01,25.3,65.2,12.05,0.250,3.01,...
 *   f_write 仅写入 FATFS 缓冲区 (微秒级), f_sync 才刷到 SD 卡 (10~50ms)
 *
 * 调用前通过 xSemaphore_SensorData 读取全局传感器值,
 * 确保读取时不被其他任务修改
 */
bool TF_LogSensor(const char *timestamp,
                  float temp, float humi,
                  float volt, float curr, float pwr,
                  const float accel_g[3], const float gyro_dps[3])
{
    (void)timestamp; (void)temp; (void)humi;
    (void)volt; (void)curr; (void)pwr;
    (void)accel_g; (void)gyro_dps;
    return false; /* TODO */
}

/*
 * f_sync: 强制刷出 FATFS 缓冲区到物理介质
 * 掉电保护: 每秒调一次 → 最多丢 1 秒数据; 不调则丢 ~512 字节
 * 本项目仅 Task_TF_Log 调用, 无需互斥锁
 */
void TF_Flush(void)
{
    /* TODO: if (s_file_open) f_sync(&g_SDFile); */
}

/*
 * 卸载: f_close (刷缓冲+更新目录) → f_mount(NULL, ...) (分离驱动)
 */
void TF_Deinit(void)
{
    if (s_file_open) {
        /* TODO: f_close(&g_SDFile); */
        s_file_open = 0;
    }
    if (s_mounted) {
        /* TODO: f_mount(NULL, g_SDPath, 0); */
        s_mounted = 0;
    }
}
