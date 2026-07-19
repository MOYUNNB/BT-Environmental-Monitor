/**
 * @file    tf_card.h
 * @brief   TF 卡 FATFS 文件系统 + CSV 日志记录
 * @note    SDIO 4-bit 模式, FATFS, CSV 按天分割文件
 *          底层 FATFS 由 CubeMX 初始化 (MX_FATFS_Init),
 *          本模块提供应用层 CSV 日志接口。
 *
 *          === 需要你自己实现 ===
 *          你需要实现:
 *          1. TF_Init()       — 挂载 FATFS
 *          2. TF_LogSensor()  — 写入一行 CSV 传感器数据
 *          3. TF_LogFlush()   — f_sync 刷新缓冲区 (由 Task_TF_Log 周期调用)
 *
 *          CubeMX 已生成 FATFS 的 diskio, 你只需要调用 f_open/f_write/f_close/f_sync.
 *
 * ============================================================
 * ==                  FATFS 文件系统工作原理                   ==
 * ============================================================
 *
 * 【FATFS 分层架构】
 *   FATFS (FatFs) 是一个通用的 FAT/exFAT 文件系统模块, 在 STM32 中通常分为两层:
 *
 *   应用层 (Application Layer):
 *     f_open / f_read / f_write / f_close / f_sync / f_mount ...
 *     ─────────────────────── 这是你直接调用的 API ───────────────────────
 *
 *   中间层 (FATFS Module):
 *     文件分配表管理, 目录项读写, 簇分配, 长文件名支持 ...
 *     这是 FatFs 库自身的代码, 你不需要修改。
 *
 *   物理层 (Disk I/O Layer / diskio.c):
 *     disk_initialize() — 初始化 SD 卡 (发送 CMD0, CMD8, ACMD41 等)
 *     disk_read()       — 读扇区 (用 SDIO 发送 CMD17)
 *     disk_write()      — 写扇区 (用 SDIO 发送 CMD24)
 *     disk_status()     — 检查卡状态
 *     disk_ioctl()      — 控制命令 (GET_SECTOR_COUNT, CTRL_SYNC 等)
 *     ──────── diskio.c 由 CubeMX 生成, 你通常不需要改 ────────
 *
 *   硬件驱动层 (SDIO 外设):
 *     SDIO 控制器 (SDIO 外设寄存器操作 + 中断/DMA)
 *
 *   f_mount 做的不是"挂载 TF 卡"—— 那是 disk_initialize 做的!
 *   f_mount 做的是: 在 FATFS 模块内部注册一个"逻辑驱动器" (如 "0:" 或 ""),
 *   将其与一个 FATFS 对象关联, 然后读取该驱动器的文件分配表等元数据。
 *   简单说: disk_initialize 让硬件能读写 SD 卡, f_mount 让软件能读写文件。
 *
 * 【SDIO 4-bit 模式】
 *   相比 SPI 模式 (1 bit 单向), SDIO 4-bit 模式使用 4 条数据线并行传输:
 *
 *   SDIO 模式 (本项目):
 *     - CLK (时钟) + CMD (命令) + D0-D3 (4 条数据线)
 *     - 1 个时钟周期可传输 4 bit
 *     - 本项目 SDIO 时钟 12 MHz → 最大理论速度 12M * 4 = 48 Mbps
 *     - 需要 SD 卡支持 (大部分 TF 卡都支持)
 *
 *   SPI 模式 (对比):
 *     - CS + MOSI + MISO + CLK
 *     - 1 个时钟周期只能传输 1 bit
 *     - 速度只有 SDIO 的 1/4
 *     - 但引脚少, 兼容性好 (几乎所有卡都支持 SPI 模式)
 *
 *   4-bit 模式初始化流程:
 *     CMD0  GO_IDLE_STATE     → 复位卡
 *     CMD8  SEND_IF_COND      → 检查是否为 SDHC/SDXC 卡 (2.0 协议)
 *     ACMD41 SD_SEND_OP_COND  → 初始化并协商电压
 *     CMD2  ALL_SEND_CID      → 获取卡片识别号
 *     CMD3  SEND_RELATIVE_ADDR → 获取相对地址 (RCA)
 *     CMD7  SELECT_CARD       → 选中卡, 切换到传输模式
 *     ACMD6 SET_BUS_WIDTH     → 将总线宽度从 1-bit 切换到 4-bit
 *     以上流程由 diskio.c 和 stm32f4xx_hal_sd.c 自动完成。
 *
 * 【CSV 格式说明】
 *   CSV (Comma-Separated Values, 逗号分隔值):
 *   - 纯文本表格格式
 *   - 每行一条记录, 各字段用逗号分隔
 *   - 首行通常为表头 (字段名)
 *   - 可以用 Excel / WPS / 记事本直接打开
 *
 *   示例:
 *     Timestamp,Temp(C),Humidity(%),Voltage(V),Current(A),Power(W),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ
 *     2026-07-18 12:00:01,25.3,65.2,12.05,0.250,3.01,0.01,0.02,1.00,0.1,0.2,0.3
 *     2026-07-18 12:01:01,25.4,65.1,12.04,0.248,2.99,0.01,0.02,1.00,0.1,0.2,0.3
 *
 *   优点: 人类可读, 跨平台, 传输效率高 (无冗余标记, JSON 的 1/3 大小)
 *   缺点: 语义不明确 (得看表头才知道哪列是什么), 不支持嵌套结构
 *   适用范围: 数据记录 (日志/导出/分析) —— CSV; 实时通信 —— JSON
 *
 * 【按天分割文件设计】
 *   ├── 2026-07-18.csv   ← 每天一个文件
 *   ├── 2026-07-19.csv
 *   └── 2026-07-20.csv
 *
 *   为什么按天分割?
 *   1. 避免单个文件过大: 1 条记录 ~130 字节, 1 分钟记录 60 次
 *      → 一天约 130*60*60*24 ≈ 11.2 MB
 *      → FAT32 单个文件最大 4 GB, 但读取几万条记录的 CSV 非常慢
 *   2. 便于数据管理: 拷贝某天的数据直接复制对应文件
 *   3. 便于上传: 手机 App 每次读取一天的数据, 传输量适中
 *   4. 自动归档: 按文件名可清晰追溯历史数据
 *
 * 【f_open + f_lseek + f_write + f_sync 写入策略】
 *
 *   策略选型: 保持文件打开 vs 每次打开/关闭?
 *
 *   方案 A (本项目采用): 保持文件打开
 *     初始化时 f_open, 之后直接 f_write, 循环周期 f_sync
 *     优点: 避免频繁 f_open/f_close 的开销 (各约 5-10ms)
 *     缺点: f_close 之前掉电可能丢最后一点数据 (靠 f_sync 缓解)
 *
 *   方案 B: 每次写入时打开 → 写入 → 关闭
 *     优点: 写入即落盘, 数据最安全
 *     缺点: f_open + f_close = 10-20ms, 影响其他任务
 *
 *   选型理由: 我们的数据是传感器日志, 丢几秒数据不影响使用。
 *   重要的是不能丢太多——所以每 1 秒 f_sync 一次。
 *
 *   各函数作用:
 *     f_open(&fil, path, FA_OPEN_ALWAYS | FA_WRITE):
 *       - 文件存在则打开 (不清空), 不存在则创建
 *       - FA_OPEN_ALWAYS = 打开已有或创建新文件
 *       - FA_WRITE = 允许写入
 *       - 打开后文件指针在文件开头
 *
 *     f_lseek(&fil, f_size(&fil)):
 *       - 将文件指针移动到文件末尾
 *       - f_size(&fil) 返回文件总字节数
 *       - f_lseek 到该位置 = 追加写入模式
 *
 *     f_write(&fil, buf, btw, &bw):
 *       - 将 buf 写入文件, btw = 要写入的字节数
 *       - bw = 实际写入的字节数 (输出参数)
 *       - 正确性检查: if (bw != btw) 写入失败 (卡满了?)
 *       - 写入是在 FATFS 缓冲区中, 并未真的写到 SD 卡!
 *
 *     f_sync(&fil):
 *       - 将 FATFS 内部缓冲区刷新到物理介质
 *       - 调用后, 数据才真正写到 SD 卡
 *       - 相当于"强制落盘", 防止掉电丢数据
 *       - 建议: 周期性调用 (本项目 1 秒一次)
 *
 *   为什么不每次 f_write 后都 f_sync?
 *     因为 f_sync 涉及物理写入, 耗时约 10-50ms。
 *     如果每秒写 1 次, 1 秒 fsync 1 次, 性能损耗 ≈ 1-5%。
 *     但如果每次写都 sync, 每秒 sync 60 次 (传感器每 100ms 采集),
 *     性能损耗可能到 50% 以上。
 *     所以: 批量写入 + 定时 fsync = 性能和数据安全的最佳平衡。
 *
 * 【为什么 Task_TF_Log 优先级最低?】
 *   TF_Log 任务优先级 = osPriorityBelowNormal1 (CMSIS-RTOS 优先级 1)
 *   在 6 个任务中优先级最低, 原因:
 *
 *   1. FATFS 文件写入慢:
 *      f_write / f_sync 涉及 SDIO 寄存器操作, 每次写入耗时 10-50ms
 *      这 10-50ms 内如果更高优先级的任务需要 CPU, 会被阻塞
 *
 *   2. 日志数据可容忍延迟:
 *      传感器数据多存几秒无影响 (不会因为日志没写就丢数据)
 *      其他任务 (按键扫描 10ms, 传感器采集 100ms) 的实时性要求高得多
 *
 *   3. 优先级反转预防:
 *      如果 TF_Log 优先级高 → 频繁进入 SDIO 中断 → 低优先级任务得不到 CPU
 *      最低优先级意味着: 其他任务都空闲了, TF_Log 才执行
 *
 * 【f_mount 参数详解】
 *   f_mount(&fatfs_obj, path, opt):
 *   - &fatfs_obj: 指向 FATFS 对象, 存储文件系统状态
 *   - path: 逻辑驱动器路径, "" 表示默认驱动器, "0:" 表示第一个驱动器
 *   - opt: 0 = 只注册不挂载 (不读取文件表), 1 = 立即挂载 (读取文件表)
 *   第一次调用: f_mount(&g_SDFatFS, "", 1);
 *   卸载调用: f_mount(NULL, "", 0);  // 传入 NULL 表示取消注册
 *
 * 【f_getfree 获取剩余空间】
 *   f_getfree(path, &fre_clust, &pfs):
 *   - fre_clust: 输出空闲簇数量
 *   - pfs: 输出 FATFS 对象指针 (可获得簇大小)
 *   - 空闲字节 = fre_clust * fatfs.csize * 512
 *   - 建议: 初始化时检查剩余空间, 小于 1MB 时报警或停止写入
 *
 * 【SD 卡初始化失败排查】
 *   如果 TF_Init 返回 false:
 *   1. 检查卡是否插好 (接触不良是最常见原因)
 *   2. 检查 SDIO 引脚: D0-D3 + CLK + CMD 是否虚焊
 *   3. 检查卡格式: ExFAT 格式卡 FATFS 需要配置支持
 *   4. disk_initialize 返回 STA_NOINIT 说明卡初始化失败
 *   5. 用逻辑分析仪看 SDIO CLK 和 CMD 线上是否有正常时序
 *   6. 试另一张卡 —— 杂牌卡兼容性差, 宜选 SanDisk / Kingston
 */
#ifndef __TF_CARD_H
#define __TF_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* FATFS 相关 (由 CubeMX 生成, 在 fatfs.h 中) */
#include "fatfs.h"

/*
 * CSV 表头定义
 * 第一行写入 CSV 文件, 标识各列含义。
 * 共 12 个字段: 时间戳 + 5 个传感器 + 6 个 IMU 数据。
 * 这里的逗号分隔位置必须和 TF_LogSensor 中的 snprintf 格式严格一致!
 * 如果表头和数据的列数不对应, Excel 打开会错位。
 *
 * 注意最后有 \r\n, 因为 CSV 标准要求行结束符, 表头也是一行。
 */
#define TF_CSV_HEADER \
    "Timestamp,Temp(C),Humidity(%),Voltage(V),Current(A),Power(W)," \
    "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ\r\n"

/**
 * @brief  初始化 TF 卡文件系统
 * @retval true: 挂载成功, false: 失败
 * @note   TODO: f_mount → 检查空闲空间 → 创建当日 CSV 文件
 *
 * 实现指引:
 *   1. 调用 f_mount(&g_SDFatFS, g_SDPath, 1) 挂载 FATFS:
 *      - g_SDFatFS: CubeMX 生成的 FATFS 对象
 *      - g_SDPath:  路径字符串, 如 "0:" 或 ""
 *      - 第三个参数 1: 立即挂载 (读取 BPB 和 FAT 表)
 *      - 返回值: FR_OK 表示成功, FR_NO_FILESYSTEM 表示未格式化
 *        (未格式化: 可用 PC 格式化为 FAT32)
 *   2. 挂载成功后设置 s_mounted = 1
 *   3. 可选: 获取空闲空间告警:
 *      DWORD fre_clust;
 *      FATFS *pfs;
 *      f_getfree(g_SDPath, &fre_clust, &pfs);
 *      uint32_t free_bytes = fre_clust * pfs->csize * 512;
 *      if (free_bytes < 1024*1024) {
 *          printf("WARN: SD card space < 1MB!\r\n");
 *          // 空间不足, 但仍可继续写入, 无法写入时 f_write 会返回错误
 *      }
 *   4. 生成当日文件名并打开文件:
 *      char fname[32];
 *      get_filename(fname, sizeof(fname));  // 生成 "2026-07-18.csv"
 *      FRESULT res = f_open(&g_SDFile, fname, FA_OPEN_ALWAYS | FA_WRITE);
 *      if (res != FR_OK) return false;
 *      s_file_open = 1;
 *      strncpy(s_filename, fname, sizeof(s_filename));
 *   5. 移动文件指针到末尾 (追加模式):
 *      f_lseek(&g_SDFile, f_size(&g_SDFile));
 *      // f_size 返回当前文件总字节数, lseek 到这个位置即为末尾
 *   6. 如果是新文件 (文件大小为 0), 写入 CSV 表头:
 *      if (f_size(&g_SDFile) == 0) {
 *          UINT bw;
 *          f_write(&g_SDFile, TF_CSV_HEADER, strlen(TF_CSV_HEADER), &bw);
 *      }
 *   7. 返回 true
 *
 * 关于 f_open 模式:
 *   FA_OPEN_ALWAYS = 0x08: 文件存在则打开, 不存在则创建
 *   等效于 C 标准库的 "a" 模式 (追加)
 *   注意: 文件打开后指针在开头, 需要 f_lseek 移到末尾。
 *
 * 可能失败的原因:
 *   - TF 卡未插入或接触不良
 *   - 卡未格式化 (需要 FAT32)
 *   - SDIO 引脚虚焊
 *   - 卡容量 > 32GB 且格式化为 exFAT (FATFS 默认不支持 exFAT)
 */
bool TF_Init(void);

/**
 * @brief  写入一条传感器 CSV 记录
 * @param  timestamp: 时间戳字符串 (如 "2026-07-18 12:00:01")
 * @param  temp, humi, volt, curr, pwr: 传感器数据
 * @param  accel_g, gyro_dps: IMU 六轴数据
 * @retval true: 成功
 * @note   TODO: f_open 已有文件 → f_lseek 到末尾 → f_write
 *
 * 实现指引:
 *   1. 构造一行 CSV 文本:
 *      char line[256];
 *      int len = snprintf(line, sizeof(line),
 *          "%s,%.1f,%.1f,%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\r\n",
 *          timestamp, temp, humi, volt, curr, pwr,
 *          accel_g[0], accel_g[1], accel_g[2],
 *          gyro_dps[0], gyro_dps[1], gyro_dps[2]);
 *   2. 写入文件:
 *      UINT bw;
 *      FRESULT res = f_write(&g_SDFile, line, len, &bw);
 *   3. 检查返回值:
 *      if (res != FR_OK || bw != len) return false;
 *      return true;
 *
 * 关于 snprintf 格式化:
 *   %.1f → 温度、湿度: 1 位小数 (AHT20 精度有限)
 *   %.2f → 电压、功率: 2 位小数 (INA226 分辨率 1.25mV)
 *   %.3f → 电流、加速度、角速度: 3 位小数 (高精度传感器)
 *
 * 性能说明:
 *   f_write 只是将数据写入 FATFS 缓冲区 (通常 512 字节, 即一个扇区),
 *   不涉及 SD 卡物理写入。
 *   真正的物理写入在以下时刻发生:
 *   - f_sync 被调用
 *   - 缓冲区满 (512 字节)
 *   - f_close 被调用
 *   - f_mount(NULL, ...) 卸载
 *   所以 f_write 本身很快 (微秒级), 瓶颈在 f_sync (毫秒级)。
 *
 * 注意:
 *   - 所有数字用纯文本, 不写单位 —— 单位在 CSV 表头中说明
 *   - 数据之间不要有空格 (CSV 标准是逗号后无空格)
 *   - 浮点数用英文小数点 (.), 不用逗号 (CSV 的逗号是分隔符!)
 *   - 字符串不含逗号或双引号 (我们不在 CSV 中使用引号转义)
 *   - 如果某个传感器读数无效, 填入 NaN 或 0.0
 */
bool TF_LogSensor(const char *timestamp,
                  float temp, float humi,
                  float volt, float curr, float pwr,
                  const float accel_g[3], const float gyro_dps[3]);

/**
 * @brief  强制刷新文件缓冲区 (防掉电丢数据)
 * @note   TODO: f_sync 当前文件, 由 Task_TF_Log 每秒调用
 *
 * 实现指引:
 *   if (s_file_open) {
 *       f_sync(&g_SDFile);
 *   }
 *
 * f_sync 的作用:
 *   FATFS 为了提高性能, f_write 时不会立即写入 SD 卡,
 *   而是将数据暂存在内部缓冲区中。
 *   如果在缓冲区刷出之前掉电, 这些数据就丢了。
 *   f_sync 强制将缓冲区中的脏数据写入 SD 卡。
 *
 * 为什么每秒调用一次?
 *   1 秒的间隔是"可接受的最大数据丢失量":
 *   - 传感器每秒采 10 次 (100ms 周期), 但 Task_TF_Log 每秒只写 1 次
 *   - 所以最多丢 1 秒的 10 条数据 (~1.3KB)
 *   - 对监测记录来说, 丢 1 秒的温湿度数据可以接受
 *   - 如果要求不丢数据 (医疗/工业), 每 100ms f_sync 一次, 但性能下降
 *
 * 注意:
 *   如果文件没有打开, f_sync 什么都不做或返回错误。
 *   所以调用了 f_sync 也不一定真的 sync 了 —— 先检查 s_file_open。
 */
void TF_Flush(void);

/**
 * @brief  卸载文件系统 (系统关机时调用)
 * @note   依次: f_close → f_mount(NULL)
 *
 * 实现指引:
 *   1. 如果文件打开: f_close(&g_SDFile), 关闭文件并刷出缓冲区
 *   2. 如果文件系统已挂载: f_mount(NULL, g_SDPath, 0),
 *      传入 NULL 表示取消注册该驱动器
 *   3. 清除所有状态标志
 *
 * 为什么要先 f_close 再 f_mount(NULL)?
 *   f_close 内部会调用 f_sync, 确保所有数据落盘后再关闭文件句柄。
 *   f_mount(NULL) 分离文件系统, 但不涉及硬件操作。
 *   在掉电前调用这个函数, 可以最大限度地避免数据丢失。
 *
 * 调用时机:
 *   系统掉电检测中断 → 通知任务 → TF_Deinit() → 系统停机
 *   如果没有掉电检测, 在系统进入 STOP/STANDBY 模式前调用。
 */
void TF_Deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __TF_CARD_H */
