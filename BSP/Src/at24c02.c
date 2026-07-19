/**
 * @file    at24c02.c
 * @brief   AT24C02 EEPROM 驱动实现
 * @note    HAL 硬件 I2C, 页写入自动跨页分割。
 *          从嘉立创 fdb 示例工程参考, 将 soft_i2c 改为 HAL 硬件 I2C。
 *
 * AT24C02 写时序:
 *   1. 发送设备地址 + 字地址 + 数据
 *   2. 等待 5ms 内部编程 (由外部调用者确保)
 *   页写入: 起始地址的低 3 位 + 连续数据, 不能跨 8 字节页边界
 *
 * ===== 为什么页写入不能跨"页边界" =====
 *
 *   【内部结构】
 *   AT24C02 内部存储阵列按"页" (Page) 组织:
 *   - 每页 8 字节
 *   - 共 32 页 (256 / 8 = 32)
 *   - 每页对应一个页缓冲区 (Page Buffer)
 *
 *   【页写入过程】
 *   1. 主机通过 I2C 发送: 设备地址 + 字地址 + 数据字节 (1~8 个)
 *   2. 芯片收到 I2C STOP 信号后:
 *      a. 将页缓冲区内容一次性写入对应的页存储单元
 *      b. 开始内部编程 (tWR = 5ms)
 *      c. 编程期间 I2C 接口不响应
 *
 *   【地址计数器机制 - 关键!】
 *   页缓冲区的地址计数器是一个 3 位计数器 (只对应页内偏移)
 *   当地址到达页边界时, 计数器会回绕 (roll over) 到页内基地址
 *
 *   举例: 从地址 7 (页 0 的最后一个字节) 写 3 个字节
 *   - 第 1 字节: 写入地址 7 (正常)
 *   - 第 2 字节: 地址计数器 +1 = 8, 但 3 位计数器溢出!
 *                实际对应页内偏移 (8 % 8) = 0  → 写入页 0 的地址 0
 *   - 第 3 字节: 写入地址 1
 *   - 结果: 地址 0 和 1 的内容被意外覆盖!
 *   这就是"页回绕" (Page Rollover) 导致的数据损坏
 *
 *   【解决方案】
 *   在 WriteBuffer 中保证: 一次 I2C 传输的字节数
 *   不超过"起始地址到当前页末尾的距离"
 *
 * ===== addr % PAGE_SIZE 计算原理 =====
 *
 *   addr % PAGE_SIZE = addr & (PAGE_SIZE - 1)   (当 PAGE_SIZE 是 2 的幂时)
 *
 *   addr = 0:   0 % 8 = 0   → 页内偏移 0, 还剩 8 字节到页结束
 *   addr = 5:   5 % 8 = 5   → 页内偏移 5, 还剩 3 字节 (8-5=3)
 *   addr = 7:   7 % 8 = 7   → 页内偏移 7, 还剩 1 字节 (8-7=1)
 *   addr = 8:   8 % 8 = 0   → 新页开始, 还剩 8 字节
 *
 *   page_remain = PAGE_SIZE - (addr % PAGE_SIZE)
 *   这是当前页的剩余容量, 本次最多写入这么多字节
 *
 *   例如: addr = 5, len = 10
 *   第 1 轮: page_remain = 8 - 5 = 3 → chunk = min(10, 3) = 3 (写 5, 6, 7)
 *           等待 5ms, 更新 len = 7
 *   第 2 轮: addr = 8, page_remain = 8 - 0 = 8 → chunk = min(7, 8) = 7 (写 8~14)
 *           等待 5ms, 更新 len = 0 → 完成
 *
 * ===== HAL_Delay(5) 是 EEPROM 内部编程时间 =====
 *
 *   【内部编程时间 tWR】
 *   AT24C02 数据手册规定:
 *   - 写周期时间 (Write Cycle Time): tWR = max 5ms
 *   - 这是从 I2C STOP 信号到数据真正写入非易失性存储单元的时间
 *   - 在此期间 EEPROM 内部在进行"擦除 → 写入 → 校验"操作
 *   - 高压泵 (Charge Pump) 在内部产生编程所需的高电压 (~15V)
 *   - 如果此时访问 EEPROM, 芯片不会响应 (NACK), 因为内部总线被编程器占用
 *
 *   为什么有的代码用 HAL_Delay(10) 而不是 5?
 *   - 给更多余量, 但会降低写入速度
 *   - 保守设计: 5ms 是最大值, 实际大多数时候 ~2ms 就完成了
 *   - 推荐还是 5ms, 因为长时间等待可能影响系统实时性
 *   - 如果 HAL_Delay(5) 后 I2C 仍然 NACK, 可以尝试重试
 *
 *   高级技巧 - 轮询等待 (ACK Polling):
 *   不固定延时, 而是持续发送 I2C 设备地址, 直到收到 ACK:
 *   do {
 *       HAL_Delay(1);
 *       status = HAL_I2C_IsDeviceReady(hi2c, addr, 1, 100);
 *   } while (status != HAL_OK);
 *   这样最快只需 ~2ms 就继续, 而不是每次都等 5ms
 *
 * ===== 为什么 ReadBuffer 可以连续读而 WriteBuffer 需要分块 =====
 *
 *   【读取 - 没有页缓冲区问题】
 *   1. 读操作是组合逻辑: 地址译码器直接选中存储单元,
 *      数据通过 I2C 接口输出, 不需要内部编程
 *   2. 每次读取不改变存储内容, 不消耗编程寿命
 *   3. I2C 协议本身支持连续读 (Sequential Read):
 *      主机持续提供 SCL 时钟和 ACK, EEPROM 内部地址自动递增
 *      直到主机发送 NACK + STOP 停止
 *   4. 读取没有"页边界"限制, 可以从 0 连续读到 255
 *   5. 读操作可以在同一个 I2C 帧内完成, 不需要中间等待
 *
 *   【写入 - 需要分块的原因】
 *   1. 写操作需要内部编程 (高压写入存储单元)
 *   2. 内部编程只能在收到 STOP 信号后触发一次
 *   3. 页缓冲区只有 8 字节, 不能跨页
 *   4. 每完成一次编程, 必须等待 5ms 才能进行下一次
 *   5. 所以 WriteBuffer 必须:
 *      a. 分页发送 (不超过页边界)
 *      b. 每写完一页, 等待 5ms
 *      c. 处理下一页
 *
 *   【总结: 读写流程对比】
 *   读取:   START + 地址 + 数据[0..N] + NACK + STOP  (一气呵成)
 *   写入:   START + 地址 + 数据[0..page_remain-1] + STOP
 *           → 等待 5ms
 *           → START + 地址 + 数据[page_remain..] + STOP
 *           → 等待 5ms
 *           → ... (重复直到写完)
 */
#include "at24c02.h"
#include "cmsis_os.h"

static I2C_HandleTypeDef *s_hi2c = NULL;
static void              *s_sem  = NULL;

static void lock(void)
{
    if (s_sem != NULL)
        osSemaphoreAcquire((osSemaphoreId_t)s_sem, osWaitForever);
}

static void unlock(void)
{
    if (s_sem != NULL)
        osSemaphoreRelease((osSemaphoreId_t)s_sem);
}

void AT24C02_Init(I2C_HandleTypeDef *hi2c, void *pSemaphore)
{
    s_hi2c = hi2c;
    s_sem  = pSemaphore;
}

AT24C02_Status_t AT24C02_ReadByte(uint8_t addr, uint8_t *data)
{
    if (addr >= AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;

    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                              (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                              data, 1U, 100U);
    unlock();
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

AT24C02_Status_t AT24C02_WriteByte(uint8_t addr, uint8_t data)
{
    if (addr >= AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;

    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                               (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                               &data, 1U, 100U);
    unlock();

    /**
     * 注意: WriteByte 没有在此函数内等待 5ms!
     * 因为 AT24C02 的 HAL_I2C_Mem_Write 在发送完 STOP 后立即返回
     * 此时 EEPROM 开始内部编程, 但 CPU 继续执行
     * 需要调用者在连续写操作之间自行等待 HAL_Delay(5)
     * 或者: 单字节写后, 下次调用时第一次 I2C 会 NACK
     * HAL 会超时重试, 实现"隐式等待"
     *
     * 但推荐的做法是在写操作后立即等待, 避免后续操作失败:
     * HAL_Delay(5);
     */
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

AT24C02_Status_t AT24C02_ReadBuffer(uint8_t addr, uint8_t *data, uint16_t len)
{
    if (addr + len > AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;

    /**
     * 连续读: 一次 I2C 帧完成
     *
     * HAL_I2C_Mem_Read 的时序:
     * START + 0xA0(W) + ACK + addr + ACK
     *   → RESTART
     *   → 0xA1(R) + ACK + data[0] + ACK + ... + data[len-1] + NACK + STOP
     *
     * 为什么不需要分页?
     * 1. 读操作不触发内部编程, 不需要等待
     * 2. 内部地址计数器支持自动递增 (auto-increment)
     *    从 addr 开始, 每读一个字节自动 +1
     * 3. 到达 0xFF 后自动回绕到 0x00 (Freescale 的 EEPROM 规格)
     * 4. I2C 主机通过发送 NACK 信号通知从机结束读取
     *
     * 注意: HAL_I2C_Mem_Read 内部已经处理了 STOP 和 RESTART 信号
     * 无需手动管理
     */
    lock();
    HAL_StatusTypeDef hal = HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                              (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                              data, len, 100U);
    unlock();
    return (hal == HAL_OK) ? AT24C02_OK : AT24C02_ERR_I2C;
}

AT24C02_Status_t AT24C02_WriteBuffer(uint8_t addr, const uint8_t *data, uint16_t len)
{
    if (addr + len > AT24C02_SIZE) return AT24C02_ERR_INVALID_ADDR;

    /**
     * 分页写入循环
     *
     * 这个循环处理三种情况:
     * 情况 1: len ≤ page_remain → 一次写入 (单页内)
     * 情况 2: len > page_remain 且 addr 在页中间 → 先写第一部分填满当前页
     * 情况 3: len > page_remain 且 addr 在页开头 → 每次写一页
     *
     * 每次循环:
     * 1. 计算当前页剩余空间 (chunk)
     * 2. 发送 chunk 字节给 EEPROM
     * 3. 等待 5ms 内部编程
     * 4. 更新指针到下一页
     *
     * 第 1 步为什么用 uint16_t 计算而不是 uint8_t?
     * 因为 page_remain 最大值 = 8, len 最大值 = 256
     * uint16_t 保证不会溢出, 虽然此处 uint8_t 也够用
     * (MSVC 的 /W4 会警告隐式转换, 所以显式使用 uint16_t)
     */
    while (len > 0U) {
        /* 计算当前页剩余字节 (AT24C02 页大小 8 字节) */
        uint16_t page_remain = AT24C02_PAGE_SIZE - (addr % AT24C02_PAGE_SIZE);
        uint16_t chunk = (len < page_remain) ? len : page_remain;

        /**
         * HAL_I2C_Mem_Write 发送的 I2C 时序:
         * START + 0xA0(W) + ACK + addr + ACK
         *   + data[0] + ACK + data[1] + ACK + ... + data[chunk-1] + ACK + STOP
         *
         * 注意:
         * - 这里的 addr 是 EEPROM 内部存储地址 (8 位)
         * - HAL 层会自行将 AT24C02_ADDR 左移 1 位 + 写入
         * - Mem_Write 中的 mem_address 参数是 EEPROM 内部地址
         *   不是 I2C 从机地址! 这是 Mem_Xxx 和 Master_Xxx 的区别
         */
        lock();
        HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(AT24C02_ADDR << 1),
                                                   (uint16_t)addr, I2C_MEMADD_SIZE_8BIT,
                                                   (uint8_t *)data, chunk, 100U);
        unlock();

        if (hal != HAL_OK) return AT24C02_ERR_I2C;

        addr += (uint8_t)chunk;
        data += chunk;
        len  -= chunk;

        /**
         * 等待内部编程完成 (数据手册: 5ms)
         *
         * 为什么这个延时不能省?
         * 如果不等 5ms 就发送下一次 I2C 请求:
         * - EEPROM 正在内部编程, I2C 接口不会 ACK 设备地址
         * - HAL_I2C_Mem_Write 超时等待 (默认 100ms)
         * - 上层函数收到 HAL_TIMEOUT, 返回 I2C 错误
         *
         * 改进方案 (ACK Polling):
         * 使用 HAL_I2C_IsDeviceReady() 轮询等待设备 ACK:
         *   这样可以自适应, 最快 ~2ms 就完成, 而不是死等 5ms
         * 本代码使用 HAL_Delay(5) 保持简洁
         */
        HAL_Delay(5U);
    }

    return AT24C02_OK;
}
