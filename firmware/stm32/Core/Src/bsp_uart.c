/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   UART 环形缓冲 + 中断/DMA 接收实现。
  *
  *  ⚠️ 与 CubeMX 生成区的关系：
  *     stm32f1xx_it.c 里的 USART1/2/3_IRQHandler 只调 HAL_UART_IRQHandler()，
  *     我们不修改它（重新生成会覆盖）。缓冲搬运放在 HAL 的回调里：
  *       HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback
  *     这些回调由 HAL_UART_IRQHandler 调用，属于 HAL 的弱符号，
  *     在我们的文件里实现即可，CubeMX 重生成不受影响。
  *
  *     唯一例外是 USART2 的 IDLE 检测：HAL 的 IDLE 中断不触发任何回调，
  *     必须自己读寄存器。为了不碰 stm32f1xx_it.c，用【主循环轮询 IDLE 标志】
  *     实现（bsp_uart_poll_dma），效果相同：IDLE 是"总线静默"标志，
  *     只要轮询间隔远小于帧间隔（下个字节 10 µs 后就来），就不会漏。
  *     ⚠️ P3 实测若发现漏帧，再改为在 it.c 的 USER CODE 段里挂 IDLE 处理。
  ******************************************************************************
  */

#include <string.h>

#include "bsp_uart.h"

/* UART 句柄由 CubeMX 在 main.c 中定义（这里只引用，不重新定义） */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/*==============================================================================
 * 静态缓冲区（⚠️ 全部静态分配，不用 malloc）
 *============================================================================*/

static uint8_t s_buf_pi[RLCA_UART1_RX_BUF];
static uint8_t s_buf_servo[RLCA_UART2_RX_BUF];
static uint8_t s_buf_esp[RLCA_UART3_RX_BUF];
static uint8_t s_dma_servo[RLCA_SERVO_DMA_RX_BUF];

bsp_uart_t g_uart_pi    = { 0 };
bsp_uart_t g_uart_servo = { 0 };
bsp_uart_t g_uart_esp   = { 0 };

/* 编译期自检：环形缓冲必须是 2 的幂（用位与取模） */
#if (RLCA_UART1_RX_BUF & (RLCA_UART1_RX_BUF - 1)) != 0
#error "RLCA_UART1_RX_BUF must be a power of two"
#endif
#if (RLCA_UART2_RX_BUF & (RLCA_UART2_RX_BUF - 1)) != 0
#error "RLCA_UART2_RX_BUF must be a power of two"
#endif
#if (RLCA_UART3_RX_BUF & (RLCA_UART3_RX_BUF - 1)) != 0
#error "RLCA_UART3_RX_BUF must be a power of two"
#endif
#if (RLCA_SERVO_DMA_RX_BUF & (RLCA_SERVO_DMA_RX_BUF - 1)) != 0
#error "RLCA_SERVO_DMA_RX_BUF must be a power of two"
#endif

/*==============================================================================
 * 内部：环形缓冲写入（ISR 与主循环共用）
 *============================================================================*/

static inline void rb_push(bsp_uart_t *p, uint8_t b)
{
    const uint16_t next = (uint16_t)((p->head + 1u) & (uint16_t)(p->size - 1u));
    if (next == p->tail) {
        /* 缓冲满：丢弃新字节并计数。
         * ⚠️ 不覆盖最旧数据 —— 协议帧是"从头解析"的，丢掉中间字节反而
         *    能让状态机更快重新同步；覆盖头部会造成难以复现的半帧。 */
        p->rx_overflow++;
        return;
    }
    p->buf[p->head] = b;
    p->head = next;
    p->rx_total++;
}

/** 批量写入（DMA 搬运用），返回成功写入的字节数 */
static uint16_t rb_push_block(bsp_uart_t *p, const uint8_t *src, uint16_t n)
{
    uint16_t written = 0u;
    for (uint16_t i = 0u; i < n; ++i) {
        const uint16_t next = (uint16_t)((p->head + 1u) & (uint16_t)(p->size - 1u));
        if (next == p->tail) {
            p->rx_overflow += (uint32_t)(n - i);
            break;
        }
        p->buf[p->head] = src[i];
        p->head = next;
        written++;
    }
    p->rx_total += written;
    return written;
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

uint16_t bsp_uart_available(const bsp_uart_t *p)
{
    return (uint16_t)((p->head - p->tail) & (uint16_t)(p->size - 1u));
}

int16_t bsp_uart_getc(bsp_uart_t *p)
{
    if (p->tail == p->head) {
        return -1;
    }
    const uint8_t b = p->buf[p->tail];
    p->tail = (uint16_t)((p->tail + 1u) & (uint16_t)(p->size - 1u));
    return (int16_t)b;
}

uint16_t bsp_uart_read(bsp_uart_t *p, uint8_t *dst, uint16_t n)
{
    if ((dst == NULL) || (n == 0u)) {
        return 0u;
    }
    uint16_t got = 0u;
    while (got < n) {
        const int16_t b = bsp_uart_getc(p);
        if (b < 0) {
            break;
        }
        dst[got] = (uint8_t)b;
        got++;
    }
    return got;
}

bool bsp_uart_write_to(bsp_uart_t *p, const uint8_t *d, uint16_t n, uint32_t timeout_ms)
{
    if ((p == NULL) || (p->huart == NULL) || (d == NULL) || (n == 0u)) {
        return true;
    }
    if (HAL_UART_Transmit(p->huart, (uint8_t *)d, n, timeout_ms) == HAL_OK) {
        return true;
    }
    /* ⚠️ 失败必须计数：否则 ESP32 侧不读数据时，这里是【静默丢帧】，
     *    上机排查时完全看不出问题（发送侧此前没有任何计数器）。 */
    p->tx_errors++;
    return false;
}

bool bsp_uart_write(bsp_uart_t *p, const uint8_t *d, uint16_t n)
{
    if ((d == NULL) || (n == 0u)) {
        return true;
    }
    /* ⚠️ 用阻塞发送：单帧最长 103 B，调用点（100 Hz 周期的固定相位）
     *    能接受这个阻塞时长。若将来发现抖动超标，再改成 DMA 发送。
     *
     *  ⚠️⚠️ 超时公式必须随波特率放宽（2026-10-18 修）：
     *     原公式 `2 + n/64` 是按 **1 Mbps** 估的（103 B → 3 ms，
     *     而实际传输只要 1.03 ms，余量 2 ms）。
     *     波特率降到 500 kbps 后，103 B 需要 **2.06 ms**，
     *     3 ms 只剩 0.94 ms 余量 —— HAL 内部逐字节等 TXE，
     *     稍有中断延迟就会 HAL_TIMEOUT ⇒ 静默丢帧 ⇒ 看起来像"掉链路"。
     *
     *  ⇒ 新公式 `3 + n/16`：103 B → 9 ms（约 4.4 倍传输时间的余量），
     *     对 1 Mbps 也安全（失败时最多多等 6 ms，正常路径不受影响）。
     *  ⚠️ 为什么不用"按波特率精确算"：这里拿不到 p->baud 的可靠值，
     *     而且超时只在**异常**路径上生效，宽松一点不会影响正常时序。 */
    const uint32_t timeout_ms = 3u + (uint32_t)(n / 16u);
    return bsp_uart_write_to(p, d, n, timeout_ms);
}

bool bsp_uart_putc(bsp_uart_t *p, uint8_t b)
{
    return bsp_uart_write(p, &b, 1u);
}

void bsp_uart_flush(bsp_uart_t *p)
{
    if (p == NULL) {
        return;
    }

    /* ⚠️⚠️ DMA 端口（USART2）的 flush 必须【重启 DMA】，不能只操作指针。
     *
     *  我在这上面连错三次，记录全过程以免再犯：
     *
     *  ❌ 错法 A：只做 tail = head
     *       DMA 缓冲里已到但未搬的字节，会在下次 drain_dma() 时
     *       被灌进刚清空的环形缓冲 → 解析器读到别的事务的应答。
     *
     *  ❌ 错法 B：把 dma_last 追平到当前 DMA 写位置
     *       那些"已进 DMA、还没搬进环形缓冲"的字节被【永久丢弃】。
     *
     *  ❌ 错法 C（我第三次的"先搬再丢"）：先把未搬的字节 push 进环形缓冲，
     *       再 tail = head 把它们丢掉。看起来干净，实际上——
     *       每次 flush 都会把"恰好在这一刻到达"的字节吃掉。
     *       而 servo_bus_pause() 每条单机指令都调 flush，
     *       `A` 命令 12 个 ID 就是 12 次丢数据。
     *       实测证据：硬件 2 秒收到 16896 字节，而环形缓冲只进了 274 字节
     *       （且 uart_overflow = 0，说明不是满了丢，是被 flush 主动丢的）。
     *
     *  ✅ 正确做法：重启 DMA 接收。
     *       位置清零、指针归零、缓冲清空，全部一次性回到确定状态，
     *       没有任何"边界上恰好到达"的歧义。
     *       bsp_uart_restart_rx() 已经实现了这套（它本来就做对了）。
     *
     *  ⚠️ 代价：重启会丢掉"半帧"—— 可接受，因为 flush 的语义就是
     *     "从现在起，之前的总线数据我全不要了"。
     *  ⚠️ 中断接收的端口（USART1/3）没有 dma_buf，走 tail = head 即可。 */
    if (p->dma_buf != NULL) {
        bsp_uart_restart_rx(p);     /* 内部会 DMAStop + dma_last=0 + Receive_DMA */
    }
    p->tail = p->head;
}

/*==============================================================================
 * 接收启动
 *============================================================================*/

static void start_rx_it(bsp_uart_t *p)
{
    /* 逐字节中断接收：每次 RxCplt 回调里再挂一次 */
    (void)HAL_UART_Receive_IT(p->huart, &p->rx_byte, 1u);
}

void bsp_uart_init_all(void)
{
    /* ---- USART1：树莓派 ---- */
    g_uart_pi.huart = &huart1;
    g_uart_pi.buf   = s_buf_pi;
    g_uart_pi.size  = (uint16_t)sizeof(s_buf_pi);

    /* ---- USART2：舵机总线（DMA 循环 + IDLE 轮询） ---- */
    g_uart_servo.huart    = &huart2;
    g_uart_servo.buf      = s_buf_servo;
    g_uart_servo.size     = (uint16_t)sizeof(s_buf_servo);
    g_uart_servo.dma_buf  = s_dma_servo;
    g_uart_servo.dma_size = (uint16_t)sizeof(s_dma_servo);

    /* ---- USART3：ESP32 ---- */
    g_uart_esp.huart = &huart3;
    g_uart_esp.buf   = s_buf_esp;
    g_uart_esp.size  = (uint16_t)sizeof(s_buf_esp);

    /* USART2 用 DMA 循环接收，永远在收 */
    (void)HAL_UART_Receive_DMA(g_uart_servo.huart,
                               g_uart_servo.dma_buf,
                               g_uart_servo.dma_size);
    g_uart_servo.dma_last = 0u;

    start_rx_it(&g_uart_pi);
    start_rx_it(&g_uart_esp);
}

void bsp_uart_restart_rx(bsp_uart_t *p)
{
    if (p->dma_buf != NULL) {
        /* DMA 模式：复位 DMA 位置并重启 */
        (void)HAL_UART_DMAStop(p->huart);
        p->dma_last = 0u;
        (void)HAL_UART_Receive_DMA(p->huart, p->dma_buf, p->dma_size);
    } else {
        (void)HAL_UART_AbortReceive(p->huart);
        start_rx_it(p);
    }
}

/*==============================================================================
 * DMA + IDLE 轮询（USART2）
 *============================================================================*/

/** ⭐⭐ 纯硬件层面的接收字节计数（不经过任何软件缓冲/状态）。
 *
 *  ⚠️⚠️ 为什么必须有这个（方法论上的修正，2026-10-11）：
 *     排查舵机 RX 问题时，我一直在看 `rx_frames` / `bytes` /
 *     `dma_restarts` —— 但这些【全部经过我自己写的代码路径】
 *     （drain_dma → 环形缓冲 → 解析器 → 状态机）。
 *     所以"收不到"既可能是【总线上真的没字节】，也可能是
 *     【我的搬运逻辑有 bug】，两者在那些指标上无法区分，
 *     导致我连续 4 次改错方向。
 *
 *     `dma_hw_bytes` 每次轮询直接读 DMA 的 CNDTR 计数器，
 *     累计它前进的字节数 —— 【不写环形缓冲、不碰 dma_last、
 *     不依赖 dma_buf 内容】。它只回答一个问题：
 *         "USART2 的接收线上到底有没有字节进来？"
 *
 *     判读：
 *       dma_hw_bytes 在涨  -> 硬件在收字节；收不到数据是我的软件问题
 *       dma_hw_bytes 不动  -> 线上真的没字节；是我的代码之外的问题
 *                             （接线 / 适配器方向 / 舵机没回 / DMA 没武装） */
static uint32_t s_dma_hw_bytes = 0u;
static uint16_t s_dma_hw_last  = 0u;

uint32_t bsp_uart_dma_hw_bytes(void)
{
    return s_dma_hw_bytes;
}

/** 只更新硬件计数，不做任何其它事。 */
static void dma_track_hw(bsp_uart_t *p)
{
    if ((p->huart == NULL) || (p->huart->hdmarx == NULL) || (p->dma_size == 0u)) {
        return;
    }
    const uint16_t pos = (uint16_t)(p->dma_size -
                         (uint16_t)__HAL_DMA_GET_COUNTER(p->huart->hdmarx));
    if (pos != s_dma_hw_last) {
        s_dma_hw_bytes += (uint16_t)((pos >= s_dma_hw_last)
                                     ? (pos - s_dma_hw_last)
                                     : (p->dma_size - s_dma_hw_last + pos));
        s_dma_hw_last = pos;
    }
}

void bsp_uart_dma_hw_reset(void)
{
    s_dma_hw_bytes = 0u;
    s_dma_hw_last  = 0u;
}

/** 把 DMA 缓冲里新到达的字节搬进环形缓冲 */
static void drain_dma(bsp_uart_t *p, bool force)
{
    /* ⭐ 先做纯硬件计数（在任何可能丢弃字节的逻辑之前） */
    dma_track_hw(p);

    /* 已写入的内存位置 = size − 剩余传输数 */
    const uint16_t pos = (uint16_t)(p->dma_size -
                          (uint16_t)__HAL_DMA_GET_COUNTER(p->huart->hdmarx));

    if (pos == p->dma_last) {
        return;                        /* 没有新字节 */
    }

    if (pos > p->dma_last) {
        (void)rb_push_block(p, &p->dma_buf[p->dma_last], (uint16_t)(pos - p->dma_last));
    } else {
        /* 跨过缓冲末尾：分两段搬 */
        (void)rb_push_block(p, &p->dma_buf[p->dma_last], (uint16_t)(p->dma_size - p->dma_last));
        if (pos > 0u) {
            (void)rb_push_block(p, &p->dma_buf[0], pos);
        }
    }
    p->dma_last = pos;

    (void)force;   /* 目前恒为搬完即止；保留参数以便将来做"半帧"处理 */
}

void bsp_uart_poll_dma(void)
{
    bsp_uart_t *p = &g_uart_servo;

    /* IDLE 置位 = 总线已静默（一帧结束）。此时把 DMA 里剩下的字节搬完。
     * ⚠️ 清标志用"读 SR 再读 DR"的序列，这与 HAL 内部一致。
     *    这里直接操作寄存器，不改 it.c，也不与 CubeMX 生成代码冲突。 */
    if (__HAL_UART_GET_FLAG(p->huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(p->huart);
        drain_dma(p, true);
    } else {
        /* 即使 IDLE 没到，也定期搬运，避免缓冲里积压太多导致 IDLE 判空 */
        drain_dma(p, false);
    }

    /* ⚠️⚠️ 接收恢复：两道检查，缺一不可。
     *
     *  ① rx_need_restart —— 由 HAL_UART_ErrorCallback 置位。
     *     ⚠️ 必须用标志：HAL 出错时会先 UART_EndRxTransfer() 关掉 DMA，
     *        再调回调，而回调里把 ErrorCode 清成了 0 ——
     *        所以【只看 ErrorCode 是永远看不到的】。
     *
     *  ② DMA 通道 EN 位被硬件关掉 —— 这是最后的物理事实检查。
     *     ⚠️ 为什么不能只信 ① 和 HAL 状态：
     *        实测抓到 dma_ccr 的 EN=0、cndtr 不递减，
     *        而 hal_ErrorCode=0、hal_dma_state=READY(1) ——
     *        **HAL 自认为一切正常，硬件却已停摆**。
     *        只看软件状态会漏掉这种情况。
     *
     *  ⚠️ 重启会丢弃缓冲里的残留字节 —— 可接受：半帧本来没用，
     *     而"永久死掉"比"丢半帧"严重得多。 */
    bool need_restart = (p->rx_need_restart != 0u);

    if (!need_restart && (p->huart->hdmarx != NULL)) {
        const DMA_Channel_TypeDef *ch =
            (const DMA_Channel_TypeDef *)p->huart->hdmarx->Instance;
        if ((ch->CCR & 1u) == 0u) {        /* EN 位 = 0 → DMA 没在跑 */
            need_restart = true;
        }
    }

    if (need_restart) {
        p->rx_need_restart = 0u;
        p->dma_restarts++;
        bsp_uart_restart_rx(p);
    }

    /* 兜底：HAL 报错但标志没置上（例如非 USART2 路径） */
    if (p->huart->ErrorCode != HAL_UART_ERROR_NONE) {
        p->rx_errors++;
        p->huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
}

/*==============================================================================
 * HAL 回调（ISR 上下文！只做搬字节，绝不做耗时操作）
 *============================================================================*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        rb_push(&g_uart_pi, g_uart_pi.rx_byte);
        start_rx_it(&g_uart_pi);
    } else if (huart->Instance == USART3) {
        rb_push(&g_uart_esp, g_uart_esp.rx_byte);
        start_rx_it(&g_uart_esp);
    }
    /* USART2 走 DMA，不会进这里 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        g_uart_pi.rx_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        start_rx_it(&g_uart_pi);
    } else if (huart->Instance == USART3) {
        g_uart_esp.rx_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        start_rx_it(&g_uart_esp);
    /* ⚠️⚠️ 接收错误（ORE/FE/NE/PE）会【关掉 DMA 接收通道】，必须真正重启。
     *
     *  这是实测抓到的真 bug（2026-10-11），而且是上面那段"自动恢复"
     *  【为什么没生效】的原因：
     *
     *      HAL 的处理顺序是：
     *        ① 出错 → HAL_UART_IRQHandler 调 UART_EndRxTransfer()
     *                 ⇒ 关掉 DMAR 位 + 停掉 DMA 通道（dma_ccr 的 EN 变 0）
     *        ② 接着调 HAL_UART_ErrorCallback()
     *        ③ 而本函数的 USART2 分支只 rx_errors++ / 清 ErrorCode
     *           ⇒ 没有重启
     *        ④ 主循环随后跑 bsp_uart_poll_dma()，
     *           但 ErrorCode 已在③被清成 0 → 检查条件为假 → 什么也不做
     *
     *      ⇒ **一次 ORE 就永久杀掉接收**，主循环里那段"自动恢复"
     *        因为标志被提前清掉而【从未执行】。
     *
     *      硬件证据（用户上机读 D 命令）：
     *        dma_ccr = 160，bit0(EN) = 0   ← DMA 通道被关
     *        dma_cndtr 不递减              ← 没在搬运
     *        dma_wr_pos == dma_last 三次不变
     *        ring_rx_total = 2952 是历史累积（开机早期的 96 帧）
     *        hal_ErrorCode = 0（已被清）、hal_dma_state = 1（HAL 以为 READY，脱节）
     *
     *  ⚠️ 为什么用【标志】而不是在这里直接重启：
     *      本函数运行在 ISR 上下文，而 bsp_uart_restart_rx() 内部要调
     *      HAL_UART_DMAStop / HAL_UART_Receive_DMA —— 在中断里调这些
     *      HAL 函数有状态机重入风险。置标志、由主循环重启更安全。 */
    } else if (huart->Instance == USART2) {
        g_uart_servo.rx_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        g_uart_servo.rx_need_restart = 1u;   /* ⭐ 交给主循环重启 */
    }
}
