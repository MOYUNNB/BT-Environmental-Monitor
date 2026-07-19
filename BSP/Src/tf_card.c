/**
 * @file    tf_card.c
 * @brief   TF 卡 FATFS + CSV 日志实现
 * @note    按天分割文件: 2026-07-18.csv, 2026-07-19.csv ...
 *          由 Task_TF_Log (优先级 2, 1 秒周期) 驱动。
 *
 * 写入策略:
 *   f_open + f_lseek 到末尾 + f_write, 保持文件打开不关闭,
 *   每秒 f_sync 一次 (平衡性能和数据安全)。
 *
 * CubeMX 已生成:
 *   - FATFS 的 diskio 层 (SDIO 驱动)
 *   - MX_FATFS_Init() 和 FATFS 对象
 *   你只需要调用 f_open/f_write/f_sync/f_close.
 *
 * ============================================================
 * ==              FATFS 文件系统深入理解                      ==
 * ============================================================
 *
 * 【FatFs 模块文件结构】
 *   在 STM32 项目中, FATFS 由以下文件组成:
 *
 *   fatfs/
 *   ├── fatfs.h              ← CubeMX 生成的配置头文件
 *   ├── diskio.h             ← disk I/O 接口定义
 *   ├── diskio.c             ← 底层驱动: 读写 SD 卡扇区
 *   ├── ff.h / ff.c          ← FatFs 核心模块 (f_open 等 API 实现)
 *   ├── ff_gen_drv.h / .c    ← 通用驱动接口封装
 *   └── sd_diskio.c          ← STM32 SDIO 驱动的 diskio 实现
 *
 *   注意: 这些文件由 CubeMX 生成, 不要在 USER CODE 保护区外修改它们。
 *   如果需要修改配置 (如文件名编码, 是否支持长文件名), 在 fatfs.h 的
 *   USER CODE 保护区内修改。建议: 初次生成后不要重新生成 FATFS。
 *
 * 【SD 卡的存储结构 (逻辑视图)】
 *
 *   一张 FAT32 格式的 SD 卡的存储结构:
 *
 *   ┌─────────────────────────┐
 *   │ MBR (主引导记录, 扇区 0)  │ ← 分区表, 指向第一个分区
 *   ├─────────────────────────┤
 *   │ DBR (DOS 引导记录)       │ ← BPB (BIOS Parameter Block)
 *   │                         │    记录: 每扇区字节数, 每簇扇区数,
 *   │                         │    FAT 表个数, 根目录最大项数等
 *   ├─────────────────────────┤
 *   │ FAT1 (文件分配表 1)       │ ← 记录每个簇的分配状态 (空闲/已用/坏)
 *   ├─────────────────────────┤
 *   │ FAT2 (文件分配表 2, 备份)  │ ← FAT1 的镜像, 冗余保护
 *   ├─────────────────────────┤
 *   │ 根目录区                  │ ← 文件和目录的条目 (名字, 属性,
 *   │                         │    首簇号, 文件大小, 时间戳)
 *   ├─────────────────────────┤
 *   │ 数据区 (簇 2 开始)        │ ← 文件内容真正存储的地方
 *   └─────────────────────────┘
 *
 *   文件分配表 (FAT) 的工作原理:
 *   - 文件系统将数据区划分为"簇"(Cluster), 簇是文件分配的最小单位
 *   - 每个簇在 FAT 表中有一个表项, 存储"下一个簇的编号"
 *   - 文件的首簇号记录在目录条目中
 *   - 读文件: 首簇号 → 查 FAT 获得下一簇 → 继续查 → 直到遇到 EOF
 *   - 写文件: 找到空闲簇, 更新 FAT 链
 *
 *   这就像一个链表: 目录条目是头指针, FAT 表是 next 指针, 数据区是节点。
 *
 *   扇区 (Sector) = 512 字节 (SD 卡的最小读写单位)
 *   簇 (Cluster)  = n 个扇区 (通常 8/16/32 扇区, 取决于卡容量)
 *   SD 卡读写必须以扇区为单位, 不能只读一个字节。
 *   所以 FatFs 内部有缓冲区: 写 1 字节也要先读整扇区, 改 1 字节, 再写回。
 *   这就是为什么 f_sync 耗时 10-50ms —— 它要完整写回一个扇区。
 *
 * 【SDIO 4-bit 模式的工作原理】
 *
 *   SDIO (Secure Digital Input/Output) 是 SD 卡的原生通信协议:
 *
 *   物理信号:
 *   - SDIO_CLK:  时钟信号, 本配置 12 MHz
 *   - SDIO_CMD:  双向命令通道 (发送命令, 接收响应)
 *   - SDIO_D0~D3: 4 条数据线 (双向)
 *
 *   传输协议:
 *   - 命令 (Command): 主机 → 卡, 48 bit (6 字节)
 *     bits[47] = 起始位 (0)
 *     bits[46] = 传输位 (1 = 主机到卡)
 *     bits[45:40] = 命令索引 (如 CMD17 = 读单块)
 *     bits[39:8] = 参数 (如读地址)
 *     bits[7:1] = CRC7
 *     bits[0] = 结束位 (1)
 *   - 响应: 卡 → 主机, 48 bit 或 136 bit (R1/R2/R3 等)
 *   - 数据: 块传输, 每块 512 字节, 后跟 CRC16
 *
 *   读取一个扇区 (f_read) 的 SDIO 操作序列:
 *   CMD17 (读单块, 参数 = 扇区号) → 卡响应 R1 → 卡开始发送 512 字节数据
 *   → 卡发送 CRC16 → 主机验证 → 结束
 *   整个过程由 SDIO 外设和 DMA 自动完成, CPU 不参与逐字节搬运。
 *
 *   写入一个扇区 (f_write):
 *   CMD24 (写单块, 参数 = 扇区号) → 卡响应 R1 → 主机发送 512 字节数据
 *   → 主机发送 CRC16 → 卡接收后响应 CRC 状态 → 卡执行写入 (忙)
 *   → 卡置 DAT0 为高 (不忙了)
 *   写入时间: 典型的 Class 10 卡写入一个扇区约 2-10ms
 *
 * 【为什么 f_mount 需要 2 次调用? (常见坑)】
 *   第一次调用 f_mount:
 *     f_mount(&g_SDFatFS, g_SDPath, 1);
 *     如果卡还未初始化 (刚上电), disk_initialize 会初始化 SDIO 和 SD 卡。
 *     但如果卡初始化需要时间 (有些卡需要几百 ms), 第一次可能失败。
 *
 *   正确的做法 (CubeMX 中已经做了):
 *     1. MX_FATFS_Init() 中:
 *        f_mount(&g_SDFatFS, g_SDPath, 0);
 *        // 先注册文件系统对象但不挂载 (opt=0)
 *     2. 使用时 (我们的 TF_Init):
 *        f_mount(&g_SDFatFS, g_SDPath, 1);
 *        // 再次挂载 (opt=1), 这次做真正的初始化
 *    如果一开始就用 opt=1, 可能在 disk_initialize 返回前就超时了。
 *    CubeMX 这种"先注册后挂载"的做法是一种典型的容错设计。
 *
 * 【FATFS 错误代码速查】
 *   FR_OK          (0):  成功
 *   FR_DISK_ERR    (1):  底层硬件错误 (SD 卡通信失败)
 *   FR_INT_ERR     (2):  断言失败 (FATFS 内部 bug, 很少见)
 *   FR_NOT_READY   (3):  物理驱动器未就绪 (卡没插好)
 *   FR_NO_FILE     (4):  文件不存在
 *   FR_NO_PATH     (5):  路径不存在
 *   FR_INVALID_NAME(6):  文件名非法
 *   FR_DENIED      (7):  访问被拒绝 (文件只读 / 写保护)
 *   FR_EXIST       (8):  文件已存在 (创建时但不是 FA_OPEN_ALWAYS)
 *   FR_INVALID_OBJECT (9): 文件对象无效 (未初始化)
 *   FR_WRITE_PROTECTED (10): SD 卡写保护
 *   FR_INVALID_DRIVE  (11): 无效驱动器号
 *   FR_NOT_ENABLED    (12): 文件系统未挂载
 *   FR_NO_FILESYSTEM  (13): SD 卡未格式化 (不是 FAT32)
 *   FR_MKFS_ABORTED   (14): 格式化失败
 *   FR_TIMEOUT        (15): 超时
 *   FR_LOCKED         (16): 文件被锁定
 *   FR_NOT_ENOUGH_CORE(17): 内存不足
 *   FR_TOO_MANY_OPEN_FILES (18): 打开文件数过多
 *   FR_INVALID_PARAMETER (19): 非法参数
 *
 *   常见于本项目的错误: FR_DISK_ERR (SIO 通信失败, 接触不良)
 *   FR_NO_FILESYSTEM (新卡未格式化, 用 PC 格式化为 FAT32)
 *   FR_NOT_READY (卡没插好或初始化失败)
 *
 * 【CSV 按天分割文件详解】
 *
 *   文件名格式: YYYY-MM-DD.csv
 *   示例: 2026-07-18.csv
 *
 *   文件名生成: 从 RTC (SD3078 或 STM32 内部 RTC) 读取年月日
 *   char fname[32];
 *   snprintf(fname, sizeof(fname), "%04u-%02u-%02u.csv", year, month, day);
 *
 *   跨日处理: 每秒检查当前日期, 如果日期变了:
 *   1. f_close 旧文件
 *   2. get_filename 生成新文件名
 *   3. f_open 新文件 (写入表头 → 或 f_lseek 到末尾)
 *
 *   文件大小估算:
 *     每条 CSV 记录 ≈ 110-130 字节 (含时间戳和 11 个数值)
 *     每秒记录 1 次 (由 Task_TF_Log 驱动) → 一天 ~11 MB
 *     FAT32 最大文件 4 GB → 理论上可存 360+ 天
 *     但超过数万行的 CSV 用 Excel 打开会很慢, 所以仍按天分割。
 *
 * 【嵌入式 FATFS 的性能优化】
 *
 *   1. f_write 不立即写卡 (缓冲机制):
 *      批量写入, 定时 fsync, 而不是每次写入都 fsync。
 *      每次 fsync 耗时 ~10-50ms, 每秒 1 次 fsync 约 1-5% CPU 开销。
 *
 *   2. 保持文件打开:
 *      f_open + f_close 各约 5-10ms。如果每秒开一次 -> 10-20ms CPU 开销。
 *     保持打开, 每次 f_lseek 到末尾 (微秒级) 即可。
 *
 *   3. 扇区对齐写入:
 *      如果发现写入性能低下, 检查是否每次写入都跨越了扇区边界。
 *      FATFS 的缓冲区是 512 字节对齐的, 写入 1 字节不会触发物理写。
 *      (缓冲区积攒到 512 字节才触发物理写, 或 fsync 强制写)
 *
 *   4. SDIO 时钟:
 *      12 MHz 是驱动能力较强的配置。如果 SD 卡支持, 可以提高到 24 MHz。
 *      提高时钟提升读写速度, 但可能影响稳定性 (信号质量问题)。
 *      建议: 通过 CubeMX 配置 SDIO 时钟分频系数。
 *
 * 【写入失败恢复策略】
 *   如果 f_write 或 f_sync 失败:
 *   1. 检查 FRESULT 错误码:
 *      - FR_DISK_ERR: SD 卡通信断了 → 尝试重新初始化
 *      - FR_NOT_READY: 卡被拔出 → 等待卡重新插入
 *      - FR_WRITE_PROTECTED: 卡写保护 → 通知用户
 *   2. 保存损坏的文件:
 *      - 尝试 f_close 当前文件
 *      - 等待 1-2 秒后重新 f_mount
 *      - 如果依然失败, 报警指示 (LED 闪烁 / LCD 显示 "SD ERR")
 *
 * 【常见面试问题】
 *
 *   Q: 为什么不用 SPI 模式?
 *   A: SPI 模式兼容性更好 (所有卡都支持), 但速度只有 SDIO 的 1/4。
 *      用 SDIO 4-bit 可以充分利用 STM32F407 的硬件资源。
 *      如果引脚不够用 (SDIO 需要 6 个引脚: CLK+CMD+D0-D3), 才降级到 SPI。
 *
 *   Q: 为什么文件满了不及时处理?
 *   A: TF 卡 32GB, 日志每天约 11MB, 可以用约 2900 天 (8 年)
 *      所以文件满在设备生命周期内不会发生。
 *      如果真的要处理, 可以在 TF_Init 中用 f_getfree 检查,
 *      空间 < 100MB 时停止写入并报警。
 *
 *   Q: f_mount 和 f_open 之间需要做什么?
 *   A: f_mount (opt=1) 后, 文件系统已经就绪, 可以直接 f_open。
 *      f_mount 内部调用了 disk_initialize (硬件初始化) 和
 *      读取 BPB + FAT 表 (文件元数据), 完成后即可正常读写。
 *
 *   Q: 如果系统在 f_write 时掉电会怎样?
 *   A: FATFS 缓冲区中的数据会丢失 (没有写入物理介质)。
 *      FAT 表可能损坏 (簇分配状态不一致) → 重启后 f_mount 检测到
 *      文件系统不干净, 会执行 FAT 扫描修复 (如果配置了自动修复)。
 *      如果 FAT 表损坏严重, 卡需要 PC 上 chkdsk 修复。
 *      所以: 定期 f_sync 极其重要! 本项目 1 秒 sync 1 次, 最多丢 1 秒数据。
 *
 *   Q: SDIO 的 4-bit 模式怎么和 SPI 模式切换?
 *   A: 初始化时先工作在 1-bit SD 模式 (CMD0 → CMD8 → ACMD41),
 *      ACMD41 响应中包含了卡是否支持 4-bit 的信息。
 *      如果支持, 再发 ACMD6 设置总线宽度为 4-bit。
 *      这些过程在 diskio.c 中自动完成, 用户无需关心。
 */
#include "tf_card.h"
#include <string.h>
#include <stdio.h>

/*
 * FATFS 对象: 由 CubeMX 声明并分配内存
 *
 * g_SDFatFS: FATFS 文件系统对象, 存储文件系统的状态信息:
 *   - fs_type: FAT12/16/32?
 *   - winsect: 当前工作扇区 (用于缓冲)
 *   - win[]: 扇区缓冲 (512 字节)
 *   - csize: 每簇扇区数
 *   - n_fatents: FAT 表项数
 *   - 等等
 *
 * g_SDFile: 文件对象, 代表一个打开的文件:
 *   - id: 文件对象 ID (用于一致性检查)
 *   - fptr: 当前文件指针位置
 *   - size: 文件大小
 *   - sclust: 文件首簇号 (从目录条目中读取)
 *   - cltbl: 簇链表指针 (可选, 用于快速定位大文件)
 *   - flag: 文件打开模式 (FA_READ, FA_WRITE 等)
 *
 * g_SDPath: 逻辑驱动器路径字符串
 *   CubeMX 通常设为一个空字符串 "" 或 "0:"
 *   这是 FATFS 的逻辑驱动器标识, 不是物理路径。
 *   f_mount 的第一个参数 (驱动器路径) 必须匹配此值。
 *
 * 重点: 这些变量在 fatfs.c 中定义 (CubeMX 生成), 我们在此文件中通过
 * extern 声明引用它们。不要在多个 .c 文件中定义同一变量!
 */
extern FATFS g_SDFatFS;     /* 文件系统对象 */
extern FIL   g_SDFile;      /* 文件对象 */
extern char  g_SDPath[4];   /* 路径 */

/* ---- 模块内部静态变量 ---- */

/*
 * s_filename: 当前打开的日志文件名
 * 用于: 1) 跨日检查时比较日期; 2) 错误报告中输出文件名
 * 最长文件名: "2026-07-18.csv\0" = 15 字节, 所以 32 足够
 */
static char   s_filename[32];

/*
 * s_mounted: 文件系统是否已成功挂载
 * s_file_open: 日志文件是否已打开
 * 这两个标志用于检查模块状态:
 * - TF_LogSensor 前检查 s_file_open, 未打开则返回 false
 * - TF_Flush 前检查 s_file_open, 未打开则跳过
 * - TF_Deinit 根据这两个标志决定是否需要 f_close/f_mount(NULL)
 */
static uint8_t s_mounted = 0;
static uint8_t s_file_open = 0;

/**
 * @brief  生成当日日志文件名
 * @param  buf:  输出缓冲区 (应 >= 32 字节)
 * @param  size: 缓冲区大小
 * @note   TODO: 从 RTC 获取当前日期, 生成 "YYYY-MM-DD.csv" 格式文件名
 *
 * 实现指引:
 *   从 RTC 读取年/月/日:
 *
 *   方案 A: 使用 STM32 内部 RTC (如果已初始化):
 *     RTC_DateTypeDef date;
 *     HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
 *     snprintf(buf, size, "%04u-%02u-%02u.csv",
 *         (uint16_t)2000 + date.Year, date.Month, date.Date);
 *
 *   方案 B: 使用外部 RTC 芯片 SD3078 (如果已实现驱动):
 *     // 假设有 SD3078_GetDateTime 函数
 *     // 本项目中用 SD3078 作为主力 RTC (带电池备份)
 *     uint16_t year; uint8_t month, day;
 *     SD3078_GetDate(&year, &month, &day);
 *     snprintf(buf, size, "%04u-%02u-%02u.csv", year, month, day);
 *
 *   方案 C: 使用编译时间 (如果不依赖 RTC):
 *     // 只在编译日期当天有效, 跨天后文件名就错了
 *     // 不推荐, 但可以作为 RTC 初始化失败的后备
 *     snprintf(buf, size, "%s.csv", __DATE__);  // 如 "Jul 18 2026.csv"
 *
 *   建议: 先用方案 A 或 B, RTC 初始化失败的 fallback 到方案 C。
 *
 *   文件名只能包含: 字母、数字、下划线、减号、点
 *   不能有: 空格、中文、特殊符号 (FAT32 支持长文件名, 但为兼容旧系统尽量简化)
 *
 *   注意: 如果 buf 不够大, snprintf 会自动截断, 不会造成溢出。
 */
static void get_filename(char *buf, size_t size)
{
    /* TODO: 从 RTC 获取日期, 生成文件名如 "2026-07-18.csv" */
    /* snprintf(buf, size, "%04u-%02u-%02u.csv", year, month, day); */
    (void)buf; (void)size;
}

bool TF_Init(void)
{
    /*
     * 初始化 TF 卡文件系统
     *
     * 步骤详解:
     *
     * 第 1 步: 再次挂载文件系统
     *   FRESULT res = f_mount(&g_SDFatFS, g_SDPath, 1);
     *   参数 3 = 1: 立即挂载 (读取 BPB 和 FAT 表)
     *   如果 CubeMX 的 MX_FATFS_Init 中已经用 opt=0 注册了文件系统,
     *   这里再次用 opt=1 做实际挂载 —— 这就是"先注册后挂载"模式。
     *
     *   如果是第一次挂载, f_mount 内部:
     *     1) 调用 disk_initialize() 初始化 SDIO + SD 卡
     *     2) 读取 MBR → 找到 FAT 分区的起始扇区
     *     3) 读取 DBR (BPB) → 获取文件系统参数 (扇区大小, 簇大小等)
     *     4) 读取 FAT 表 → 建立簇分配地图
     *     5) 验证文件系统合法性
     *
     *   如果返回 FR_OK → 挂载成功
     *   如果返回 FR_NO_FILESYSTEM → SD 卡未格式化或格式不支持
     *   如果返回 FR_NOT_READY → SD 卡未插入或硬件初始化失败
     *
     *   FRESULT 的详细含义见文件顶部的错误码速查表。
     */
    FRESULT res;

    /*
     * 第 2 步 (可选): 检查剩余空间
     *   如果挂载成功, 可以顺便获取剩余空间:
     *
     *   DWORD fre_clust;
     *   FATFS *pfs;
     *   res = f_getfree(g_SDPath, &fre_clust, &pfs);
     *
     *   空闲字节 = fre_clust * pfs->csize * 512
     *   (簇数 * 每簇扇区数 * 每扇区字节数)
     *
     *   if (res == FR_OK) {
     *       uint32_t free_bytes = fre_clust * pfs->csize * 512;
     *       printf("SD free: %lu bytes\r\n", free_bytes);
     *       // 如果小于 1MB 可以报警, 但继续尝试写入
     *   }
     *
     *   注意: f_getfree 需要遍历 FAT 表计算空闲簇数,
     *   对于大容量卡 (32GB), 这可能需要几百 ms。
     *   如果不想增加启动时间, 可以省略此检查, 让 f_write 失败时再处理。
     *
     * 第 3 步: 生成当日文件名
     *   char fname[32];
     *   get_filename(fname, sizeof(fname));
     *
     * 第 4 步: 打开日志文件
     *   res = f_open(&g_SDFile, fname, FA_OPEN_ALWAYS | FA_WRITE);
     *
     *   FA_OPEN_ALWAYS = 0x08: 文件存在则打开, 不存在则创建
     *   FA_WRITE       = 0x02: 允许写入
     *   组合: 文件存在 → 打开可写; 不存在 → 创建新文件并打开可写
     *
     *   f_open 打开后, 文件指针在开头 (位置 0)。
     *   如果文件已有内容, 需要移到末尾才能追加写入。
     *
     *   如果 f_open 失败 (返回 FR_NOT_READY 等):
     *   - 检查 g_SDPath 是否正确 (需要和 MX_FATFS_Init 中一致)
     *   - 检查文件名是否合法 (FAT32 不支持 / ? < > \ : * | 等字符)
     *   - 检查 SD 卡是否存在
     *
     * 第 5 步: 文件指针移到末尾 (追加模式)
     *   f_lseek(&g_SDFile, f_size(&g_SDFile));
     *
     *   f_size: 获取文件字节数
     *   f_lseek: 移动文件指针
     *   定位到文件末尾后, 后续的 f_write 都会追加到文件末尾。
     *
     *   注意: f_lseek 也可以用于创建"空洞文件"
     *   (指针移到超过文件末尾的位置, 中间填零), 但我们不需要。
     *
     * 第 6 步: 如果是新文件, 写入 CSV 表头
     *   if (f_size(&g_SDFile) == 0) {
     *       UINT bw;
     *       res = f_write(&g_SDFile, TF_CSV_HEADER, strlen(TF_CSV_HEADER), &bw);
     *       if (res != FR_OK || bw != strlen(TF_CSV_HEADER)) {
     *           printf("TF: write header failed\r\n");
     *           // 写入表头失败不是致命错误, 但不写表头的话 CSV 没列名
     *           // 可以选择继续执行, 也可以返回 false
     *       }
     *   }
     *
     *   如何判断新文件?
     *   f_size(&g_SDFile) == 0 说明文件刚创建, 没有任何内容
     *   如果文件已有数据 (如上电重启时), f_size > 0, 跳过写入表头
     *
     * 第 7 步: 设置标志
     *   s_mounted = 1;
     *   s_file_open = 1;
     *   strncpy(s_filename, fname, sizeof(s_filename) - 1);
     *   s_filename[sizeof(s_filename) - 1] = '\0';
     *
     * 第 8 步: 返回成功
     *   return true;
     *
     * 初始化失败后的处理:
     *   不要在这里死等或进入错误循环 —— 这会导致整个系统卡住。
     *   正确的做法: 返回 false 并让调用者 (Task_TF_Log) 决定重试策略。
     *   例如: 每 5 秒尝试重新初始化一次, 并在 LCD 上显示 "SD ERR"。
     */
    (void)res;
    return false; /* TODO: 替换为 true */
}

bool TF_LogSensor(const char *timestamp,
                  float temp, float humi,
                  float volt, float curr, float pwr,
                  const float accel_g[3], const float gyro_dps[3])
{
    /*
     * 写入一条 CSV 格式的传感器数据到日志文件
     *
     * CSV 格式 (与表头 TF_CSV_HEADER 一一对应):
     *   2026-07-18 12:00:01,25.3,65.2,12.05,0.250,3.01,0.01,0.02,1.00,0.1,0.2,0.3\r\n
     *
     * 实现步骤:
     *
     * 第 1 步: 检查文件是否已打开
     *   if (!s_file_open) return false;
     *   如果文件未打开, 直接返回失败 —— 不要试图在这里 f_open,
     *   因为文件打开的逻辑由 TF_Init 和跨日检查处理, 职责分离。
     *
     * 第 2 步: 构造 CSV 行
     *   char line[256];   // 足够容纳最大行
     *   int len = snprintf(line, sizeof(line),
     *       "%s,%.1f,%.1f,%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\r\n",
     *       timestamp, temp, humi, volt, curr, pwr,
     *       accel_g[0], accel_g[1], accel_g[2],
     *       gyro_dps[0], gyro_dps[1], gyro_dps[2]);
     *
     *   注意: snprintf 的返回值是"应该写入的字符数" (不包括 \0),
     *   如果返回值 >= sizeof(line), 说明输出被截断了。
     *   但 256 字节对于一行 CSV 绝对够用 (正常约 130 字节)。
     *
     *   为什么用 snprintf 而不是 sprintf?
     *   - snprintf 限制写入长度, 防止缓冲区溢出
     *   - sprintf 不检查长度 —— 如果将来增加字段时忘记调整缓冲区,
     *     sprintf 会写越界, 导致栈破坏 (系统崩溃)
     *   - snprintf 是 C99 标准, 几乎所有编译器都支持
     *
     * 第 3 步: 写入文件
     *   UINT bw;
     *   FRESULT res = f_write(&g_SDFile, line, (UINT)len, &bw);
     *   注意 len 是 int, 需要转型为 UINT。
     *   一般情况下 len < 255, 不会溢出 UINT。
     *
     *   可能失败的原因:
     *   - 卡满了 (FR_DISK_ERR): 文件系统无剩余空间
     *   - 卡被拔出 (FR_NOT_READY): 物理连接断开
     *   - 参数错误 (FR_INVALID_OBJECT): g_SDFile 未正确初始化
     *
     * 第 4 步: 检查写入结果
     *   if (res != FR_OK || bw != (UINT)len) {
     *       // 写入失败, 记录错误
     *       printf("TF: write err %d\r\n", res);
     *       return false;
     *   }
     *
     *   bw != len 意味着写入的字节数不对 —— 可能因为卡满了
     *   或文件系统内部错误。这时候应该触发一个恢复流程。
     *
     * 第 5 步: 返回成功
     *   return true;
     *
     * 写入的时机:
     *   Task_TF_Log 每秒调用本函数一次。
     *   因为: 传感器每 100ms 采集一次, 但不需要每笔都记入日志。
     *   每秒记录 1 个平均值 (或最新值), 既减少了日志量,
     *   又保持了足够的时间分辨率 (分析温湿度变化趋势, 1 秒间隔够了)。
     *
     * 数据的准确性问题:
     *   在写入前, 数据应通过互斥信号量从全局传感器结构体读出:
     *   - xSemaphoreTake(xSemaphore_SensorData, portMAX_DELAY)
     *   - 读取 temp, humi, volt, curr, pwr 等
     *   - xSemaphoreGive(xSemaphore_SensorData)
     *   这确保了读取时传感器数据没有被其他任务正在修改。
     *   但 Task_TF_Log 的优先级最低, 所以它拿到的数据可能不是"最新的",
     *   但对日志来说, 1-2 秒的延迟完全可接受。
     *
     * 关于浮点数:
     *   STM32F407 有硬件 FPU (单精度), 所以 float 运算很快。
     *   但 printf 族的 %f 格式化需要链接浮点库 (见 bluetooth.c 中的说明)。
     */
    (void)timestamp; (void)temp; (void)humi;
    (void)volt; (void)curr; (void)pwr;
    (void)accel_g; (void)gyro_dps;
    return false; /* TODO: 替换为 true */
}

void TF_Flush(void)
{
    /*
     * 强制刷新文件缓冲区 (f_sync)
     *
     * f_sync 的作用:
     *   FatFs 为了提高性能, f_write 写入的数据先存在内部缓冲区 (512 字节扇区),
     *   缓冲区满了才写入 SD 卡。f_sync 强制将缓冲区中的脏数据立即刷出。
     *
     * 调用时机:
     *   由 Task_TF_Log 每秒调用一次。Task_TF_Log 的执行流程是:
     *   1. 从传感器结构体读取最新数据
     *   2. 调用 TF_LogSensor 写入一行 CSV
     *   3. 调用 TF_Flush 刷新缓冲区
     *   整个过程约 20-50ms, 每秒执行一次。
     *
     * 掉电保护:
     *   如果不调 f_sync, 系统掉电时最多丢失 512 字节 = 约 4-5 条日志记录。
     *   每秒调一次 f_sync → 最多丢失 1 秒的数据。
     *   这是"可接受的最大数据丢失"设计: 监控记录允许丢 1 秒, 不影响使用。
     *
     * 实现:
     *   if (s_file_open) {
     *       f_sync(&g_SDFile);
     *   }
     *
     *   注意: 每次 f_sync 相当于一次完整的 512 字节 SD 卡扇区写入,
     *   耗时约 10-50ms (取决于 SD 卡速度)。在此期间, 如果高优先级任务
     *   就绪, FreeRTOS 会抢占, 所以不会阻塞系统。
     *   但如果所有任务都等待 SD 卡, 系统会卡住。—— 不要让多个任务同时
     *   访问 FATFS! FATFS 默认不是线程安全的, 需要用互斥量保护。
     *
     * 多任务保护:
     *   如果多个任务同时调用 f_write/f_sync, 需要互斥锁保护文件对象。
     *   本项目只有 Task_TF_Log 一个人使用 FATFS, 不需要锁。
     *   如果上层代码在其他任务中调用了本模块的函数, 需要:
     *     extern osMutexId_t xMutex_FATFS;
     *     osMutexAcquire(xMutex_FATFS, osWaitForever);
     *     f_write(...);
     *     osMutexRelease(xMutex_FATFS);
     */
    /* TODO: f_sync(&g_SDFile); */
}

void TF_Deinit(void)
{
    /*
     * 卸载文件系统 (系统关机/掉电前调用)
     *
     * 确保关机前所有数据已写入 SD 卡, 避免数据丢失。
     *
     * 步骤:
     *
     * 1. 关闭当前日志文件
     *    if (s_file_open) {
     *        f_close(&g_SDFile);
     *        s_file_open = 0;
     *    }
     *    f_close 内部做了:
     *      - 调用 f_sync 刷出缓冲区数据到 SD 卡
     *      - 更新目录条目 (文件大小, 修改时间等)
     *      - 释放文件对象 (FIL) 占用的资源
     *    f_close 后, g_SDFile 不再代表一个打开的文件。
     *
     * 2. 卸载文件系统
     *    if (s_mounted) {
     *        f_mount(NULL, g_SDPath, 0);
     *        s_mounted = 0;
     *    }
     *    f_mount(NULL, path, 0) 传入 NULL 作为文件系统对象:
     *      - 取消注册该逻辑驱动器
     *      - 刷出内部所有脏缓冲区
     *      - 但不涉及硬件操作 (SDIO 不关闭)
     *    如果之后需要重新使用, 可以再次 f_mount(&g_SDFatFS, g_SDPath, 1)。
     *
     * 3. 清空文件名缓冲区 (可选, 安全起见)
     *    s_filename[0] = '\0';
     *
     * 调用场景:
     *   - 系统检测到掉电 (通过 PVD / EXTI 引脚中断)
     *   - 用户按下"关机"按钮
     *   - 系统进入 STOP/STANDBY 低功耗模式前
     *   如果直接断电 (没有调用 Deinit), 最多丢失 1 秒的数据 (f_sync 周期内)。
     *   但如果正好在写入 FAT 表时断电, 文件系统可能损坏。
     *   FAT32 的 chkdsk 可以修复大多数 FAT 表损坏, 但最好避免。
     */
    if (s_file_open) {
        /* TODO: f_close(&g_SDFile); */
        s_file_open = 0;
    }
    if (s_mounted) {
        /* TODO: f_mount(NULL, g_SDPath, 0); */
        s_mounted = 0;
    }
}

/*
 * 总结: 本模块的设计思路
 *
 * 分层依赖:
 *   TF_LogSensor() → f_write → FATFS 核心 → disk_read/write → SDIO 驱动
 *   我们只关心最上层的 API 调用, 底层由 CubeMX 保证。
 *
 * 数据流:
 *   Sensor Task (100ms) → 全局传感器结构体 (共享) → Task_TF_Log (1s)
 *   → TF_LogSensor(snprintf + f_write) → TF_Flush(f_sync) → SD 卡
 *
 * 容错设计:
 *   - TF_Init 失败: 不阻塞系统, 由任务重试
 *   - f_write 失败: 返回 false, 任务可决定重试或报警
 *   - f_sync 失败: 同返回 false, 但继续尝试
 *   - 文件写入过程中卡拔出: 下次操作返回错误, 等待重新插入
 *
 * 线程安全:
 *   单任务访问 (Task_TF_Log), 无需互斥量。
 *   如果将来多任务需要写文件, 必须加互斥量保护所有 FATFS 操作。
 */
