/**
  ******************************************************************************
  * @file    servo_bus.c
  * @brief   舵机总线事务状态机实现。
  *
  *  一轮事务（servo_bus_start_cycle → 完成）：
  *
  *    t=0       SVB_WRITE     阻塞发 SyncWrite（44 B ≈ 440 µs）
  *    t=440µs   SVB_TURNAROUND 等 RLCA_SERVO_TURNAROUND_US
  *    t=460µs   SVB_READ_REQ  阻塞发 SyncRead 请求（20 B ≈ 200 µs）
  *    t=660µs   SVB_READ_RSP  异步：12 帧回复陆续到达（≈1.8 ms）
  *    t≈2.5ms   SVB_PARSE     全部收齐 → 逐帧解析
  *    t≈2.5ms   SVB_DONE      → 下一轮可发起
  *
  *  ⚠️ 关键约束：SVB_READ_RSP 由【端口可用字节数】驱动，不阻塞。
  *     若 3 ms 内收不齐 → SVB_TIMEOUT，本轮作废但不清空已有进度，
  *     下一轮照常发起（不因为一次丢帧就停总线）。
  *
  *  ⚠️ 零动态分配：所有缓冲静态。
  ******************************************************************************
  */

#include <string.h>

#include "servo_bus.h"
#include "rlca_frame.h"   /* rlca_get_i16 / 小端读取 */
/* ⚠️ 刻意【不】包含 bsp_tick.h：那会把 stm32f1xx_hal.h 拉进来，
 *    破坏本模块"纯逻辑、可在 PC 上单测"的定位（已实测踩到：
 *    host 单测因找不到 hal.h 直接构建失败）。
 *    微秒延时改用端口抽象里的 now_us()，不需要任何 HAL。 */

/** 判"回复结束"的静默阈值（微秒）。
 *  ⚠️ 依据：12 帧反馈是【背靠背】发的，帧间没有总线静默；
 *     1 Mbps 下一帧 15 B ≈ 150 µs，但那是"传输时间"不是"静默时间"。
 *     200 µs 的静默已经足够确认"不会再来了"，同时远小于 3 ms 硬超时。 */
#define SVB_IDLE_GAP_DEFAULT_US   200u

/*==============================================================================
 * ⭐⭐ 可在串口上【在线调整】的总线时序参数
 *
 * ⚠️⚠️ 为什么把它们做成运行期变量（2026-10-11，实测踩坑后）：
 *     上机现象是"12 个舵机里只有 1 个偶尔能读出数据"（≈1% 成功率），
 *     而 PING 是 0%。这一看就是**半双工总线的收发切换余量不够**：
 *        RLCA_SERVO_TURNAROUND_US 原为 20 µs
 *        而 1 Mbps 下 1 个字节 = 10 µs  →  20 µs 只有【2 个字节时间】
 *     对 Waveshare 适配器（它自己要做 RS-485 方向控制）来说太紧了。
 *
 *     问题在于：**这个值该多大，只能靠实测**（取决于适配器、线长、舵机）。
 *     如果做成编译期常量，每试一个值就要"改代码 → 重编译 → 重烧"，
 *     一轮 5 分钟，扫描 10 个值就是一小时 —— 不可接受。
 *
 *     所以做成运行期变量 + 串口命令，可以边调边看 B 的成功率。
 *     出厂默认仍是编译期常量，不影响正常使用。 */
static uint16_t s_turnaround_us = RLCA_SERVO_TURNAROUND_US;
static uint16_t s_frame_timeout_us = RLCA_SERVO_FRAME_TIMEOUT_US;
static uint16_t s_idle_gap_us   = SVB_IDLE_GAP_DEFAULT_US;

/** ⭐ 诊断计数：舵机应答帧"按下发次数归一化"的成功率。
 *  ⚠️ 单独统计是因为 n_ok_reads 要求【12 帧全到】，
 *     只接少数舵机时它永远是 0，看不出"到底通了没有"。 */
static uint32_t s_diag_rounds   = 0u;   /* 发起过多少轮读 */
static uint32_t s_diag_frames   = 0u;   /* 累计解析出多少应答帧 */
static uint32_t s_diag_bytes    = 0u;   /* 累计从总线收到多少字节 */

void servo_bus_set_timing(uint16_t turnaround_us, uint16_t frame_timeout_us,
                          uint16_t idle_gap_us)
{
    if (turnaround_us   > 0u) { s_turnaround_us   = turnaround_us; }
    if (frame_timeout_us > 0u) { s_frame_timeout_us = frame_timeout_us; }
    if (idle_gap_us     > 0u) { s_idle_gap_us     = idle_gap_us; }
}

void servo_bus_get_timing(uint16_t *turnaround_us, uint16_t *frame_timeout_us,
                          uint16_t *idle_gap_us)
{
    if (turnaround_us   != NULL) { *turnaround_us   = s_turnaround_us; }
    if (frame_timeout_us != NULL) { *frame_timeout_us = s_frame_timeout_us; }
    if (idle_gap_us     != NULL) { *idle_gap_us     = s_idle_gap_us; }
}

void servo_bus_get_diag(uint32_t *rounds, uint32_t *frames, uint32_t *bytes)
{
    if (rounds != NULL) { *rounds = s_diag_rounds; }
    if (frames != NULL) { *frames = s_diag_frames; }
    if (bytes  != NULL) { *bytes  = s_diag_bytes; }
}

void servo_bus_reset_diag(void)
{
    s_diag_rounds = 0u;
    s_diag_frames = 0u;
    s_diag_bytes  = 0u;
}

/*==============================================================================
 * 内部状态
 *============================================================================*/

#define SVB_N  RLCA_N_JOINTS

/** 帧缓冲：SyncWrite 12 舵机 = 44 B；留余量到 64 */
static uint8_t s_tx[64];
/** 接收缓冲：12 帧 × 15 B = 180 B；留余量到 256 */
static uint8_t s_rx[256];

static const rlca_servo_port_t *s_port = NULL;
static rlca_calib_t             s_calib;

static servo_bus_state_t s_state = SVB_IDLE;
static uint32_t s_t0_us = 0u;          /* 本轮发起时刻（用于耗时统计） */
static uint32_t s_phase_t0_us = 0u;    /* 当前阶段的起始时刻（用于超时） */

static uint8_t  s_rx_len = 0u;         /* s_rx 已收字节数 */
static uint32_t s_last_rx_us = 0u;     /* 最后一次收到字节的时刻（判回复结束） */

/** ⭐⭐ 总线互斥标志：为 true 时 servo_bus_poll() 不发起任何新事务。
 *
 *  ⚠️⚠️ 为什么需要它（实测抓到的竞态，2026-10-11）：
 *     `bsp_shell_poll()` 与 `servo_bus_poll()` 跑在【同一个主循环】里
 *     （见 app_poll）。所以下面这种交错是可能的：
 *
 *        ① servo_bus_poll()  → SVB_READ_REQ：发出 SyncRead(1..12)
 *        ② bsp_shell_poll()  → 执行 `A`：servo_bus_ping() 直接发 PING
 *        ③ 结果：总线上同时有【同步读请求】和【PING 请求】两家在等回复，
 *           RX 缓冲里混着两种应答，谁也对不上 → 双方都判失败
 *
 *     ⚠️ 表现就是那个很反直觉的现象：
 *        `Y 5` 能收到应答（它自己反复 poll、等 50 ms，运气好读到对的），
 *        而 `A` 全 no response（它用状态机式的 3 ms 窗口，必然混入垃圾）。
 *
 *  ⚠️ 用"暂停"而不是"abort"就够的原因：
 *     abort 只清当前状态，但下一个 app_poll() 立刻又会发起新事务；
 *     必须同时阻止"发起"，才能真正把总线让给 shell 命令。
 *
 *  ⚠️ 只在【阻塞式单机指令】前后短暂置位（shell / 标定），
 *     10 Hz 控制环不受影响。 */
static bool s_paused = false;

/*==============================================================================
 * ⭐⭐ 诊断痕迹（排查"某个舵机的应答没被认出来"）
 *
 * ⚠️⚠️ 为什么需要（实测卡住很久的地方）：
 *     修好 DMA 搬运后，`A`（PING）报告 id 5 与 id 9 都 ACK，
 *     但 `B`（SyncRead）里只有 id 5 有数据，id 9 全是 0。
 *     也就是说同步读的应答里，有一个明明到了却【没被解析出来】。
 *
 *     而原代码在丢弃字节时只做 `n_short++` / `n_crc_err++` ——
 *     看不到被拒的【到底是什么字节】，也无法知道陌生 ID 是几号。
 *     所以只能猜。这一段把证据留下来。
 *
 * ⚠️ 和 dma_hw_bytes 同一思路：让【原始数据】说话，而不是靠推理。
 *============================================================================*/

#define SVB_TRACE_LEN  24u
static uint8_t  s_trace[SVB_TRACE_LEN];   /* 最近一次被丢弃处的原始字节 */
static uint16_t s_trace_len = 0u;         /* 记录了多少 */
static uint16_t s_trace_off = 0u;         /* 被丢弃时在 s_rx 里的偏移 */
static uint16_t s_trace_total = 0u;       /* 被丢弃时的 s_rx_len */
static uint8_t  s_diag_unknown_id = 0u;   /* 见过的陌生 ID（0 = 没有） */
static uint32_t s_diag_parse_fail = 0u;   /* 校验/LEN 拒收次数 */

/* 丢弃原因编码（进 trace，便于一眼看出为什么拒了） */
#define SVB_DROP_NO_HEAD   1u   /* 不在帧头 */
#define SVB_DROP_UNKNOWN   2u   /* ID 不在期望列表里 */
#define SVB_DROP_BADFRAME  3u   /* 整帧到齐但校验/LEN 错 */

static uint8_t  s_trace_reason = 0u;

/** 记录 s_rx[off..] 的若干原始字节。
 *
 *  ⚠️⚠️ 必须【只在真正丢弃时】调用，绝不能放在循环开头。
 *     我第一版把它放在 `while` 循环体的第一行，结果它记录的是
 *     "循环最后一次经过的位置" —— 而那往往是一个被【成功解析】的帧，
 *     于是 trace 显示的字节完全误导人（实测踩到，白看了一轮）。
 *     只有"被丢弃那一刻"的字节才有诊断价值。 */
static void record_drop(uint16_t off, uint8_t reason)
{
    if (off >= (uint16_t)s_rx_len) {
        return;
    }
    uint16_t n = (uint16_t)(s_rx_len - off);
    if (n > SVB_TRACE_LEN) { n = SVB_TRACE_LEN; }

    for (uint16_t i = 0u; i < n; ++i) {
        s_trace[i] = s_rx[off + i];
    }
    s_trace_len    = n;
    s_trace_off    = off;
    s_trace_total  = (uint16_t)s_rx_len;
    s_trace_reason = reason;
}

uint8_t servo_bus_trace_reason(void)
{
    return s_trace_reason;
}

void servo_bus_get_trace(uint8_t *dst, uint16_t dst_cap, uint16_t *out_len,
                         uint16_t *out_off, uint16_t *out_total)
{
    if ((dst != NULL) && (dst_cap > 0u)) {
        uint16_t n = (s_trace_len < dst_cap) ? s_trace_len : dst_cap;
        for (uint16_t i = 0u; i < n; ++i) { dst[i] = s_trace[i]; }
        if (out_len != NULL) { *out_len = n; }
    } else if (out_len != NULL) {
        *out_len = 0u;
    }
    if (out_off   != NULL) { *out_off   = s_trace_off; }
    if (out_total != NULL) { *out_total = s_trace_total; }
}

uint8_t servo_bus_unknown_id(void)
{
    return s_diag_unknown_id;
}

uint32_t servo_bus_parse_fail_count(void)
{
    return s_diag_parse_fail;
}

/** 解析应答帧；返回成功解析的帧数。
 *  ⚠️ 前置声明：servo_bus_abort() 和阻塞式指令都要用它。 */
static uint8_t parse_feedback_frames(void);

/** 中止当前事务并清空接收（总线互斥用）。
 *  ⚠️ 只清"事务状态"，不动 s_paused —— 调用方负责暂停/恢复。 */
static void bus_abort_now(void)
{
    if ((s_port != NULL) && (s_port->flush != NULL)) {
        s_port->flush();
    }
    s_rx_len = 0u;
    s_state  = SVB_IDLE;
}

void servo_bus_pause(void)
{
    s_paused = true;
    bus_abort_now();
}

void servo_bus_resume(void)
{
    s_paused = false;
}

static int16_t  s_q_target_step[SVB_N];  /* 本轮下发的目标（步），限幅后 */
static uint8_t  s_ids[SVB_N];

static servo_fb_t          s_fb[SVB_N];
static rlca_joint_state_t  s_js[SVB_N];
static bool                s_new_feedback = false;
static servo_bus_stats_t   s_stats;
static uint8_t             s_err_flags = 0u;

/** ⭐⭐ 最近一次"错误位被置起"的时刻（us，0 = 从未）。
 *  用于 RLCA_ERR_AUTOCLEAR_MS 的自动解除，见 servo_bus_poll()。 */
static uint32_t            s_err_last_us = 0u;
static bool                s_err_ever    = false;

/** ⭐⭐ 连续失败的轮数（2026-10-17 加）。
 *
 *  ⚠️ 为什么需要：实测总线约 **1%/轮** 会整轮收不到回复（偶发）。
 *     旧实现"一次失败就 raise_err(BUS_TIMEOUT)"，于是：
 *        err_flags 在 0 与 0x40 之间跳
 *        → safety_reasons 在 0 与 0x20(SERVO_ERR) 之间跳
 *        → amp_scale 在 100% 与 75% 之间跳
 *     表现为面板上"错误"和"幅度"不停闪，而总线其实基本是好的。
 *
 *  ⭐ 新语义：
 *     · `s_stats.n_timeout`        —— **每次失败都计数**（诊断用，一个都不漏）
 *     · `RLCA_ERRF_BUS_TIMEOUT` 位 —— 只有**连续 N 轮**失败才置起
 *       ⇒ 单次抖动不再污染安全层；真正断线/接错 5 轮（50 ms）就报。
 *
 *  ⚠️ 成功一轮就清零，所以"断续失败但总体健康"不会被误报。 */
static uint16_t            s_bus_fail_streak = 0u;

/* ⚠️ 前向声明：raise_err 定义在下面（小工具区），而这两个函数要在它之前用 */
static void raise_err(uint8_t bit);

static void bus_round_failed(void)
{
    if (s_bus_fail_streak < 0xFFFFu) {
        s_bus_fail_streak++;
    }
    if (s_bus_fail_streak >= (uint16_t)RLCA_BUS_FAIL_STREAK_ERR) {
        raise_err(RLCA_ERRF_BUS_TIMEOUT);
    }
}

static void bus_round_ok(void)
{
    s_bus_fail_streak = 0u;
}

uint16_t servo_bus_fail_streak(void)
{
    return s_bus_fail_streak;
}

/*==============================================================================
 * 小工具
 *============================================================================*/

static inline uint32_t now_us(void)
{
    return (s_port != NULL) ? s_port->micros() : 0u;
}

/** 自某时刻起经过的微秒数（回绕安全）。
 *  ⚠️ 不能用 bsp_us_since()：那条路径读的是 DWT，本模块要能被 PC 测试
 *     驱动，时间源必须来自注入的端口。 */
static inline uint32_t since_us(uint32_t t0)
{
    return now_us() - t0;
}

static void fill_ids(void)
{
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        s_ids[i] = RLCA_SERVO_ID(i);
    }
}

static void set_state(servo_bus_state_t st)
{
    s_state = st;
    s_phase_t0_us = now_us();
}

/** ⭐⭐ 置起一个错误位，并记录时刻（供自动解除用）。
 *
 *  ⚠️ 不要直接写 `s_err_flags |= X`：那样 s_err_last_us 不会刷新，
 *     自动解除就会把【正在持续发生】的错误也清掉。 */
static void raise_err(uint8_t bit)
{
    s_err_flags |= bit;
    s_err_last_us = now_us();
    s_err_ever    = true;
}

/** ⭐⭐ 错误位自动解除：连续 RLCA_ERR_AUTOCLEAR_MS 没有新错误就全部清零。
 *  语义从"曾经出过错（latch）"改成"最近还在出错吗"。
 *  ⚠️ 真正持续的故障会不断 raise_err，所以不会被误清。 */
static void err_flags_tick(void)
{
    if (s_err_flags == 0u) {
        return;
    }
    if (!s_err_ever) {
        return;
    }
    if (since_us(s_err_last_us) >= (RLCA_ERR_AUTOCLEAR_MS * 1000u)) {
        s_err_flags = 0u;
    }
}

/*==============================================================================
 * 初始化
 *============================================================================*/

void servo_bus_init(const rlca_servo_port_t *port, const rlca_calib_t *calib)
{
    s_port = port;

    if (calib != NULL) {
        s_calib = *calib;
    } else {
        rlca_calib_load_defaults(&s_calib);
    }

    fill_ids();
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_fb, 0, sizeof(s_fb));
    memset(s_js, 0, sizeof(s_js));
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        s_q_target_step[i] = 0;
    }

    s_rx_len = 0u;
    s_err_flags = 0u;
    s_err_last_us = 0u;
    s_err_ever    = false;
    s_new_feedback = false;

    if (s_port->flush != NULL) {
        s_port->flush();
    }
    set_state(SVB_IDLE);
}

/*==============================================================================
 * 标定表热更新
 *============================================================================*/

/** ⭐⭐⭐ 只更新标定表，不碰端口/统计/反馈（2026-10-17 修的关键 bug）
 *
 *  ⚠️⚠️ 为什么必须有这个函数：
 *
 *     `servo_bus_init()` 里是 `s_calib = *calib;` —— **复制一份**。
 *     而标定完成时（`app_calib_take_result`）、手动零点（`app_calib_manual_zero`）、
 *     恢复默认（`RESTORE_DEFAULT`）都只更新 `app_main` 自己的 `s_calib`，
 *     **从来没有同步给 servo_bus**。
 *
 *     ⇒ 标定完之后：
 *          · `R` 里打印的 offset 是【新的、正确的】
 *          · 而 `q_rad` 是用 servo_bus 那份【上电时的旧表】算的
 *     ⇒ 现象：在标定姿态（pos≈2048）下，关节角报 ±π（因为旧表 offset=0），
 *        而 `STAT_CALIBRATED` 却是亮的 —— 自相矛盾，极难排查。
 *     ⇒ 更麻烦的是：**必须重启 32 才生效**，因为上电时
 *        `bsp_flash_load_calib()` 会把新表读进来再传给 `servo_bus_init()`。
 *
 *  ⭐ 修法：把"标定表变化"做成一个显式的刷新点，标定流程结束时调用。
 *     ⚠️ 故意【不】复用 `servo_bus_init()`：那会把统计、反馈、
 *        错误位全部清零，标定途中调用会丢掉正在用的反馈。
 *     ⚠️ 只更新 `s_calib` 并让下一轮反馈重新换算即可 —— 因为
 *        `rlca_joint_state_update()` 每帧都用 `s_calib` 现算 `q_rad`，
 *        所以下一次解析就会用上新表。 */
void servo_bus_set_calib(const rlca_calib_t *calib)
{
    if (calib == NULL) {
        return;
    }
    s_calib = *calib;
}

/*==============================================================================
 * 发起一轮
 *============================================================================*/

/** ⭐⭐ 按【原始步数】发起一轮同步写（上电安全保持专用）。
 *
 *  ⚠️⚠️ 为什么需要这个入口（2026-10-16 定位到的根因）：
 *
 *     上电闸门打开后，"播种"走的是角度域：
 *         s_q_limited[i] = js[i].q_rad            （播种）
 *         link_limits_apply(...)                  （角度域限幅）
 *         servo_bus_start_cycle(...)
 *             q = rlca_q_clamp(&s_calib, i, q);   （⚠️ 又夹一次）
 *             s_q_target_step[i] = rlca_q_to_step(&s_calib, i, q);
 *
 *     而【角度域的含义完全依赖标定表】。未标定时 s_calib 是默认表：
 *         offset = 0，dir = +1，limit_min/max 由 rlca_calib_fill_limits() 按
 *         "offset=0" 推出。于是：
 *             当前角度 → 被夹到软限位【边界】→ 换算回步数 → 一个固定的错值
 *         ⇒ 12 个舵机全部奔向那个错值，表现为【同向、同速、持续】运动。
 *
 *     现场判据（用户观察，完全吻合）：
 *         "没接齐 12 个舵机就不动" —— 因为播种要求 n_valid == 12。
 *         12 个一齐，播种 + 限幅同时生效，于是 12 个一起动。
 *
 *  ⇒ 本入口用【当前实测的原始步数】当目标，一步都不经过角度域：
 *         目标步数 = 当前位置步数  ⇒  舵机无处可去
 *     对"标定表是否正确"完全免疫。 */
bool servo_bus_start_cycle_raw(const int16_t *pos_step)
{
    if ((s_port == NULL) || (pos_step == NULL)) {
        return false;
    }
    if (!servo_bus_is_idle()) {
        return false;                 /* 上一轮还没完 */
    }

    /* ⚠️ 只夹到【编码器物理范围】，不碰标定表、不做角度换算 */
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        int32_t p = (int32_t)pos_step[i];
        if (p < 0)      { p = 0; }
        if (p > 4095)   { p = 4095; }
        s_q_target_step[i] = (int16_t)p;
    }

    uint16_t n = 0u;
    const sts_tx_result_t r = rlca_sts_build_sync_write_pos(s_ids, SVB_N,
                                                            s_q_target_step,
                                                            s_tx, (uint16_t)sizeof(s_tx), &n);
    if (r != STS_TX_OK) {
        return false;
    }

    if (s_port->flush != NULL) {
        s_port->flush();
    }
    s_rx_len = 0u;
    s_t0_us = now_us();
    set_state(SVB_WRITE);
    return true;
}

bool servo_bus_start_cycle(const float *q_target)
{
    if (s_port == NULL) {
        return false;
    }
    if (!servo_bus_is_idle()) {
        return false;                 /* 上一轮还没完 */
    }

    /* ---- ① 目标角：NaN 防护 → 限幅 → 换算成步 ---- */
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        float q = (q_target != NULL) ? q_target[i] : RLCA_Q_DEFAULT_RAD;
        q = rlca_q_clamp(&s_calib, i, q);          /* 内部已处理 NaN/Inf */
        s_q_target_step[i] = rlca_q_to_step(&s_calib, i, q);
    }

    /* ---- ② 组 SyncWrite 帧 ---- */
    uint16_t n = 0u;
    const sts_tx_result_t r = rlca_sts_build_sync_write_pos(s_ids, SVB_N,
                                                            s_q_target_step,
                                                            s_tx, (uint16_t)sizeof(s_tx), &n);
    if (r != STS_TX_OK) {
        return false;
    }

    /* ---- ③ 清残留，然后进 WRITE 状态 ---- */
    if (s_port->flush != NULL) {
        s_port->flush();
    }
    s_rx_len = 0u;
    s_t0_us = now_us();
    set_state(SVB_WRITE);
    return true;
}

bool servo_bus_start_read_only(void)
{
    if (s_port == NULL) {
        return false;
    }
    if (!servo_bus_is_idle()) {
        return false;
    }

    if (s_port->flush != NULL) {
        s_port->flush();
    }
    s_rx_len = 0u;
    s_t0_us = now_us();
    set_state(SVB_TURNAROUND);       /* 跳过写，直接切换后发读请求 */
    return true;
}

/*==============================================================================
 * 解析收到的 12 帧反馈
 *============================================================================*/

/** 解析 s_rx 里的连续反馈帧。
 *  @return 成功解析的帧数
 *
 *  ⚠️ 反馈帧是【背靠背】来的：12 帧连续 180 字节，没有间隔。
 *     所以必须按帧边界逐个切，不能"等 IDLE"。 */
static uint8_t parse_feedback_frames(void)
{
    uint8_t got = 0u;
    uint16_t off = 0u;

    /* ⚠️ 用"确定性上界"而不是递增的帧计数：
     *    每个分支要么消费 1 个字节、要么消费一整帧，所以 off 单调不减，
     *    上界 s_rx_len 必然退出。
     *    绝不能写成 `for (k=0; k<12; ++k)` 再在错误分支里 `k--` ——
     *    uint8_t 下溢会变成 255 从而死循环（P1 的 STS 测试真实踩到过）。 */
    while ((off + (uint16_t)STS_FB_FRAME_LEN) <= (uint16_t)s_rx_len) {

        /* 找帧头 FF FF：允许帧间夹噪声字节 */
        if (!((s_rx[off] == STS_HEAD) && (s_rx[off + 1u] == STS_HEAD))) {
            record_drop(off, SVB_DROP_NO_HEAD);   /* ⭐ 只在真正丢弃时记录 */
            off++;
            s_stats.n_short++;         /* 丢弃的杂散字节 */
            continue;
        }

        const uint8_t id = s_rx[off + 2u];

        /* 反查这份反馈属于哪个关节（按 ID 匹配，不假设回复顺序） */
        int8_t idx = -1;
        for (uint8_t i = 0u; i < SVB_N; ++i) {
            if (s_ids[i] == id) {
                idx = (int8_t)i;
                break;
            }
        }
        if (idx < 0) {
            s_diag_unknown_id = id;    /* ⭐ 诊断：记下这个陌生 ID */
            record_drop(off, SVB_DROP_UNKNOWN);
            off++;                     /* 未知 ID：丢一个字节继续找头 */
            s_stats.n_short++;
            continue;
        }

        sts_feedback_t f;
        const sts_fb_result_t fr = rlca_sts_parse_feedback(&s_rx[off], STS_FB_FRAME_LEN,
                                                          id, STS_FB_DATA_LEN, &f);
        if ((fr != STS_FB_OK) && (fr != STS_FB_ERR_SERVO)) {
            record_drop(off, SVB_DROP_BADFRAME);
            off++;                     /* 校验/长度错：丢一个字节重新找头 */
            s_stats.n_crc_err++;
            s_diag_parse_fail++;       /* ⭐ 诊断：整帧到齐但被拒收的次数 */
            continue;
        }

        /* 填充反馈（原始值） */
        servo_fb_t *fb = &s_fb[(uint8_t)idx];
        fb->pos_step   = rlca_get_i16(&f.data[0]);
        fb->speed_step = rlca_get_i16(&f.data[2]);
        fb->load_raw   = rlca_get_i16(&f.data[4]);
        fb->voltage_dv = f.data[6];
        fb->temp_c     = f.data[7];
        fb->moving     = f.data[8];
        fb->err        = f.err;
        fb->valid      = 1u;
        s_diag_frames++;       /* ⭐ 诊断：成功解析一帧应答 */

        /* ⚠️⚠️⚠️ 关键修正（2026-10-16）：必须先把【新位置】写进 s_js，
         *        再调 rlca_joint_state_update()。
         *
         *  原代码写成：
         *      rlca_joint_state_update(&s_calib, idx,
         *                              s_js[idx].pos_step,   // ⚠️ 旧的
         *                              &s_js[idx]);
         *  而 fb->pos_step（刚解析出的新值）赋了之后【再没被用过】。
         *
         *  rlca_joint_state_update(prev_step, st) 的契约是：
         *      st->pos_step  【已经是新值】
         *      prev_step      = 上一轮的旧值（用来算 0↔4095 回绕）
         *  所以旧写法缺了"把新值存进 st->pos_step"这一步 ⇒
         *  s_js[].pos_step 永远停在 servo_bus_init 的 memset 结果 0，
         *  q_rad 也永远是 (0 − offset)×dir×2π/4096。
         *
         *  后果（现场现象）：闸门期读到的"当前位置"全是 0，
         *  于是 1 s 闸门打开后播种出 q=0，首条同步写把 12 个舵机
         *  全部命令到 step 0 —— 表现为"上电 1 秒后开始、每次同方向"。
         *
         *  ⚠️ 顺序不能反：velocity / load 也同样要先落进 st，
         *     因为这些字段是被 rlca_joint_state_update 读取的。 */
        {
            rlca_joint_state_t *st = &s_js[(uint8_t)idx];
            const int16_t prev_step = st->pos_step;   /* 旧值：供回绕判断 */

            st->pos_step   = fb->pos_step;            /* ⭐ 新值先落状态 */
            st->speed_step = fb->speed_step;
            st->load_raw   = fb->load_raw;
            st->voltage_dv = fb->voltage_dv;
            st->temp_c     = fb->temp_c;
            st->moving     = fb->moving;
            st->err        = fb->err;

            rlca_joint_state_update(&s_calib, (uint8_t)idx, prev_step, st);
            st->valid = 1u;
        }

        /* 舵机错误码汇总 + 分类上报 */
        s_stats.err_accum |= (uint32_t)f.err;
        if ((f.err & STS_ERR_VOLTAGE) != 0u)     { raise_err(RLCA_ERRF_VOLTAGE); }
        if ((f.err & STS_ERR_ANGLE) != 0u)       { raise_err(RLCA_ERRF_ANGLE); }
        if ((f.err & STS_ERR_OVERHEAT) != 0u)    { raise_err(RLCA_ERRF_OVERHEAT); }
        if ((f.err & STS_ERR_OVERCURRENT) != 0u) { raise_err(RLCA_ERRF_OVERCURRENT); }
        if ((f.err & STS_ERR_ANGLE_LIMIT) != 0u) { raise_err(RLCA_ERRF_ANGLE_LIMIT); }
        if ((f.err & STS_ERR_OVERLOAD) != 0u)    { raise_err(RLCA_ERRF_OVERLOAD); }

        s_stats.rx_frames++;
        got++;
        off = (uint16_t)(off + STS_FB_FRAME_LEN);
    }

    /* 保留未消费的尾部字节，供下一轮拼接（正常情况下为 0） */
    if (off > 0u) {
        const uint16_t left = (uint16_t)(s_rx_len - off);
        if (left > 0u) {
            memmove(s_rx, &s_rx[off], left);
        }
        s_rx_len = (uint8_t)left;
    }
    return got;
}

/*==============================================================================
 * 状态机推进
 *============================================================================*/

void servo_bus_poll(void)
{
    if (s_port == NULL) {
        return;
    }

    /* ⭐⭐ 错误位自动解除（见 err_flags_tick 的说明）
     *  ⚠️ 必须放在 s_paused 判断【之前】：shell 阻塞期间也要能解除，
     *     否则调试期间错误位会一直挂着。 */
    err_flags_tick();

    /* ⭐ 互斥：被暂停时不发起任何新事务。
     *   ⚠️ 只挡住"发起"，不清状态 —— 状态已由 servo_bus_pause() 清过。
     *   ⚠️ 这个检查必须放在最前面：否则 shell 正阻塞读总线时，
     *      本函数会在同一个主循环里插进来发新帧。 */
    if (s_paused) {
        return;
    }

    switch (s_state) {

    case SVB_IDLE:
    case SVB_DONE:
    case SVB_TIMEOUT:
        return;                       /* 等待上层发起下一轮 */

    case SVB_WRITE:
    {
        /* SyncWrite 帧已在 start_cycle 里组好，这里只发。
         * ⚠️ 唯一允许阻塞的阶段：44 B @1 Mbps ≈ 440 µs。 */
        uint16_t n = 0u;
        const sts_tx_result_t r = rlca_sts_build_sync_write_pos(s_ids, SVB_N,
                                                                s_q_target_step,
                                                                s_tx, (uint16_t)sizeof(s_tx), &n);
        if (r != STS_TX_OK) {
            s_state = SVB_TIMEOUT;
            return;
        }
        s_port->write(s_tx, n);
        s_stats.tx_frames++;
        set_state(SVB_TURNAROUND);
        return;
    }

    case SVB_TURNAROUND:
        /* 半双工总线的收发方向切换：控制板需要一点时间。
         * 太短 → 丢首字节；太长 → 浪费周期。经验值 20 µs
         * （伺服总线模块设计 §5.4：官方库的 SCDelay 约 10 µs 起）。 */
        if (since_us(s_phase_t0_us) >= s_turnaround_us) {
            set_state(SVB_READ_REQ);
        }
        return;

    case SVB_READ_REQ:
    {
        uint16_t n = 0u;
        const sts_tx_result_t r = rlca_sts_build_sync_read(STS_FB_START_ADDR, STS_FB_DATA_LEN,
                                                           s_ids, SVB_N,
                                                           s_tx, (uint16_t)sizeof(s_tx), &n);
        if (r != STS_TX_OK) {
            s_state = SVB_TIMEOUT;
            return;
        }
        s_port->write(s_tx, n);
        s_stats.tx_frames++;
        s_diag_rounds++;       /* ⭐ 诊断：发起一轮同步读 */
        s_rx_len = 0u;
        set_state(SVB_READ_RSP);
        return;
    }

    case SVB_READ_RSP:
    {
        /* 把端口里已有的字节搬进 s_rx（不阻塞） */
        const uint16_t av = s_port->avail();
        if (av > 0u) {
            const uint16_t room = (uint16_t)(sizeof(s_rx) - s_rx_len);
            const uint16_t want = (av < room) ? av : room;
            if (want > 0u) {
                const uint16_t got = s_port->read(&s_rx[s_rx_len], want);
                if (got > 0u) {
                    s_rx_len = (uint8_t)(s_rx_len + got);
                    s_diag_bytes += (uint32_t)got;   /* ⭐ 诊断：累计收到字节 */
                    s_last_rx_us = now_us();      /* 刷新"最后收到字节"时刻 */
                }
            }
        }

        /* 收全 12 帧 → 立即解析（最快路径） */
        if (s_rx_len >= (uint8_t)(SVB_N * STS_FB_FRAME_LEN)) {
            set_state(SVB_PARSE);
            return;
        }

        /* ⚠️ 关键：不能只等"收满 12 帧"。
         *    12 帧是背靠背来的（帧间没有静默），但一旦有舵机掉线，
         *    就永远收不到 180 字节，会白白干等满 3 ms。
         *    所以用"短暂静默"判回复结束：帧间 gap ≈ 0，
         *    而 200 µs 的静默已经足够确认"不会再来了"。 */
        if (s_rx_len > 0u) {
            if (since_us(s_last_rx_us) >= s_idle_gap_us) {
                s_stats.n_idle_timeout++;
                set_state(SVB_PARSE);         /* 把手上的帧解析掉 */
                return;
            }
        }

        /* 整帧超时 3 ms：⚠️ 官方库缺失的那一层（缺陷 1） */
        if (since_us(s_phase_t0_us) > s_frame_timeout_us) {
            s_stats.n_timeout++;
            s_stats.n_failed_reads++;         /* 整轮失败：一帧都没等到 */
            /* ⚠️ 计数照旧每次都加，但【错误位】只在连续 N 轮失败时才置 */
            bus_round_failed();
            s_state = SVB_TIMEOUT;
        }
        return;
    }

    case SVB_PARSE:
    {
        const uint8_t got = parse_feedback_frames();

        if (got >= SVB_N) {
            s_stats.n_ok_reads++;
            s_new_feedback = true;
            bus_round_ok();               /* ⭐ 成功一轮 → 连续失败计数清零 */
        } else {
            s_stats.n_failed_reads++;
            if (got == 0u) {
                s_stats.n_timeout++;
                /* ⚠️ 同整帧超时分支：计数照加，错误位只在连续 N 轮失败时置 */
                bus_round_failed();
            }
        }

        /* ⚠️ 耗时统计要设上限，否则一个异常轮次会永久污染 max_txn_us。
         *
         *  实测踩到（2026-10-11）：`max_txn_us = 4235316701`（≈4235 秒）
         *  —— 这是不可能的值，因为 DWT 计数器在 72 MHz 下约 60 秒就回绕，
         *  正常的无符号差值【不可能超过约 60 秒】。
         *
         *  成因：s_t0_us 是在【发起事务】时设的（见 start_cycle），
         *  而如果某一轮因为异常（DMA 被关、状态机长时间停摆）从未走到
         *  本分支，等它终于走到时 dt 已经是个巨大的陈旧值。
         *
         *  ⚠️ 这个值只是显示用，不影响任何控制逻辑 ——
         *     但会让人误判"总线抖动 4000 秒"，所以必须夹住。
         *     判据用帧超时（3 ms）的 100 倍：正常轮次远小于它，
         *     超过就一定不是真实耗时。 */
        const uint32_t dt = since_us(s_t0_us);
        s_stats.last_txn_us = dt;
        if ((dt <= ((uint32_t)s_frame_timeout_us * 100u)) &&
            (dt > s_stats.max_txn_us)) {
            s_stats.max_txn_us = dt;
        }

        s_state = (got >= SVB_N) ? SVB_DONE : SVB_TIMEOUT;
        return;
    }

    default:
        s_state = SVB_IDLE;
        return;
    }
}

/*==============================================================================
 * 单机指令（阻塞，用于调试与标定）
 *============================================================================*/

/** ⚠️ 单机指令要等回复，所以必须在本模块内部做一个小等待循环。
 *     这类调用【不允许】出现在 100 Hz 周期里（会破坏节拍），
 *     只用于 shell / 标定 / 上电自检。 */
static bool txn_expect_ack(uint16_t timeout_us)
{
    /* ⚠️⚠️ 关键：发送后必须等【收发方向切换】完成再开始找应答。
     *     半双工总线上适配器要做 RS-485 方向控制，发送刚结束时
     *     它还在驱动总线，此时收到的很可能是自己发出去的字节
     *     （或干脆漏掉应答的第一字节）。
     *
     *     这是实测抓到的 bug（2026-10-11）：
     *       同步读那一路有 SVB_TURNAROUND 等待（20 µs）→ 能收到数据；
     *       而 PING 这一路【完全没有等待】→ 永远收不到。
     *       表现：`B` 里 id=5 有 pos/V/T，而 `A` 和 `Y` 全 no response。
     *     这解释了"同一个舵机对同步读应答、对 PING 不应答"的怪现象。 */
    /* ⚠️ 用端口抽象的 now_us() 做忙等，不用 bsp_delay_us ——
     *    后者要 bsp_tick.h → stm32f1xx_hal.h，会让本模块无法在 PC 上单测。 */
    if (s_turnaround_us > 0u) {
        const uint32_t tw = now_us();
        while (since_us(tw) < (uint32_t)s_turnaround_us) {
            /* 忙等：只有几十微秒，不阻塞别的 */
        }
    }

    /* ⚠️⚠️ 必须【累积 + 重同步】，不能"读一次就判"。
     *
     *  这是实测抓到的真 bug（2026-10-11）：
     *     `Y` 命令能收到完整的 6 字节应答（FF FF 05 02 00 F8），
     *     而本函数在同样的总线上、同样的 3 ms 窗口内却总是判失败。
     *
     *  原因是【读取会碎片化】：DMA 搬运与 IDLE 判帧是分开的，
     *  应答可能被切成"前半帧 + 后半帧"两次进入环形缓冲。
     *  原来的实现只要 avail() 够 4 字节就 read() 一次，
     *  读到半帧 → 解析失败 → 直接 return，不再等后半帧。
     *
     *  ⚠️ `Y` 之所以成功：它自己用 50 ms 窗口反复 poll 并累积字节，
     *     等于绕过了这个缺陷 —— 所以 `Y` 的"成功"掩盖了本函数的 bug。
     *
     *  正确做法：把到的字节累积起来，每次尝试在缓冲里找一个完整帧；
     *  找到就判定、找不到就继续等，直到超时。 */
    uint8_t  acc[STS_FB_FRAME_LEN + 8u];
    uint16_t acc_len = 0u;

    /* ⚠️⚠️ 自旋上限是【必须的】防御，理由和 link_pi_wait_cmd 里那个一样：
     *     如果微秒时间源不前进（DWT 未使能、时钟停摆、或时间源被换掉），
     *     `since_us(t0) < timeout_us` 会永远为真，把【整个主循环】锁死。
     *
     *     真实踩到：PING 的单测里假时间由测试驱动，
     *     那些"期望失败"的用例（对不发应答的 ID 发 PING）不推进时间，
     *     于是本函数永久自旋 —— 单测直接挂死。
     *
     *     上限取 timeout 的若干倍余量：正常一次 poll 只需几微秒，
     *     真正等到 3 ms 也只迭代几百次，绝不会触顶。 */
    uint32_t spin_cap = (uint32_t)timeout_us * 100u;

    const uint32_t t0 = now_us();
    while ((since_us(t0) < timeout_us) && (spin_cap > 0u)) {

        spin_cap--;

        /* ⚠️ 只在【真的读到新字节】时才继续尝试解析。
         *    否则若缓冲里已有一些字节但凑不成帧，本循环会
         *    反复读同一批数据、空转到超时（实测导致单测挂死）。
         *    没有新字节时就让循环转一圈、把时间让给 DMA 搬运。 */
        bool got_new = false;

        const uint16_t av = s_port->avail();
        if (av > 0u) {
            const uint16_t room = (uint16_t)(sizeof(acc) - acc_len);
            const uint16_t want = (av < room) ? av : room;
            if (want > 0u) {
                const uint16_t n = s_port->read(&acc[acc_len], want);
                if (n > 0u) {
                    acc_len = (uint16_t)(acc_len + n);
                    got_new = true;
                }
            }
        }

        if (!got_new) {
            continue;                  /* 没新数据：再看一次时间与 avail */
        }

        if (acc_len < (uint16_t)(STS_PING_FB_LEN + 2u)) {
            continue;                  /* 还不够一帧，继续等 */
        }

        /* ---- 重同步：把游标移到帧头 ---- */
        uint16_t off = 0u;
        while ((off + 2u) <= acc_len) {
            if ((acc[off] == STS_HEAD) && (acc[off + 1u] == STS_HEAD)) {
                break;
            }
            off++;
        }
        if (off > 0u) {
            const uint16_t left = (uint16_t)(acc_len - off);
            memmove(acc, &acc[off], left);
            acc_len = left;
        }

        if (acc_len < 4u) {
            continue;              /* 连 LEN 字段都没到，继续等 */
        }

        /* ---- ⭐ 先读 LEN，确认【整帧到齐】再解析 ----
         *
         * ⚠️⚠️ 这一步是修一个实测抓到的 bug（由单测 test_ping_reply_fragmented
         *     的 "4+2 切法" 抓到）：
         *     原实现只要有了 STS_PING_FB_LEN 个字节就去解析，
         *     而 4 字节时 LEN 已可读、但校验和字节还没到 ——
         *     解析必然失败，然后代码 memmove(acc, &acc[2], ...)
         *     把【半个有效帧的帧头丢掉】，
         *     第二批字节到达时已经无法恢复 → 永远判失败。
         *
         *  正确做法：帧长 = LEN + 4（头2 + ID + LEN + LEN 字节含 CHK）。
         *  没到齐就【继续等，绝不动缓冲里的字节】。
         *  只有 LEN 非法（不可能的值）时，才认为帧头是噪声、跳过 2 字节。 */
        const uint8_t flen = acc[3];

        /* LEN 合法性：PING 应答实测 LEN=2；给足余量到 STS_MAX_PAYLOAD+2 */
        if ((flen < 2u) || (flen > (STS_MAX_PAYLOAD + 2u))) {
            /* 非法 LEN：帧头不可信，跳过 2 字节继续找 */
            memmove(acc, &acc[2], (uint16_t)(acc_len - 2u));
            acc_len = (uint16_t)(acc_len - 2u);
            continue;
        }

        const uint16_t frame_total = (uint16_t)flen + 4u;
        if (acc_len < frame_total) {
            continue;              /* ⭐ 整帧没到齐：继续等，不丢字节 */
        }

        sts_feedback_t f;
        /* ⚠️ exp_len 必须是 STS_PING_FB_DATA_LEN。
         *     ⚠️ 这个值我改错过一次：曾拿一帧来源可疑的抓包
         *        （USB-TTL 抢总线时的产物）把 exp_len 改成 2，
         *        导致 `A` 命令 12 个 ID 全部误报 no response。
         *        用 Y 命令实测的真实应答是 FF FF 05 02 00 F8
         *        （LEN=2 → data_len=0），所以 exp_len 就是 0。
         *        详见 rlca_sts.h 里 STS_PING_FB_LEN 的注释。 */
        const sts_fb_result_t r = rlca_sts_parse_feedback(acc, frame_total,
                                                          STS_BROADCAST_ID,
                                                          STS_PING_FB_DATA_LEN,
                                                          &f);
        if ((r == STS_FB_OK) || (r == STS_FB_ERR_SERVO)) {
            return true;
        }

        /* 整帧到齐但解析失败（校验和错 / ID 不对 / LEN 不符）：
         * 跳过这 2 个头字节继续找下一帧 */
        memmove(acc, &acc[2], (uint16_t)(acc_len - 2u));
        acc_len = (uint16_t)(acc_len - 2u);
    }
    return false;
}

bool servo_bus_ping(uint8_t servo_id)
{
    if (s_port == NULL) {
        return false;
    }
    uint16_t n = 0u;
    if (rlca_sts_build_ping(servo_id, s_tx, (uint16_t)sizeof(s_tx), &n) != STS_TX_OK) {
        return false;
    }

    /* ⭐⭐ 总线互斥：本函数会阻塞等应答，期间必须让后台状态机停手。
     *   ⚠️ 否则同一个主循环里的 servo_bus_poll() 可能插进来发 SyncRead，
     *      总线上两家同时等回复，RX 里混着两种应答 → 双方都判失败。
     *      这正是 `A` 全 no response 而 `Y` 偶然能成功的原因。
     *   ⚠️ pause() 内部已经 flush（清环形缓冲 + 追平 dma_last），
     *      所以这里不用再 flush 一次。 */
    servo_bus_pause();
    s_port->write(s_tx, n);
    s_stats.tx_frames++;
    const bool ok = txn_expect_ack(s_frame_timeout_us);
    servo_bus_resume();
    return ok;
}

bool servo_bus_set_torque(uint8_t joint_idx, bool enable)
{
    if ((s_port == NULL) || (joint_idx >= SVB_N)) {
        return false;
    }
    uint16_t n = 0u;
    if (rlca_sts_build_torque(s_ids[joint_idx], enable,
                              s_tx, (uint16_t)sizeof(s_tx), &n) != STS_TX_OK) {
        return false;
    }
    /* ⭐ 总线互斥：写指令不回等应答，但若后台正好在等 SyncRead 的回复，
     *   本帧的字节会被混进那份应答里 → 后台那轮解析失败。
     *   ⚠️ 这里不用 pause（本函数不阻塞等回复，几十微秒就返回），
     *      只需在【发送前】确认没有事务在飞行中。
     *   ⚠️ bus_abort_now() 内部已 flush（清环形缓冲 + 追平 dma_last）。 */
    bus_abort_now();
    s_port->write(s_tx, n);
    s_stats.tx_frames++;
    return true;
}

void servo_bus_set_all_torque(bool enable)
{
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        (void)servo_bus_set_torque(i, enable);
    }
}

bool servo_bus_write_one(uint8_t joint_idx, int16_t pos_step)
{
    if ((s_port == NULL) || (joint_idx >= SVB_N)) {
        return false;
    }
    /* 用 SyncWrite 发单个舵机：复用同一条路径，避免再写一套帧构造 */
    const uint8_t id = s_ids[joint_idx];
    uint16_t n = 0u;
    if (rlca_sts_build_sync_write_pos(&id, 1u, &pos_step,
                                      s_tx, (uint16_t)sizeof(s_tx), &n) != STS_TX_OK) {
        return false;
    }
    /* ⭐ 总线互斥：同 servo_bus_set_torque()，发送前清掉在飞行中的事务。
     *   ⚠️ 这是 `P` 命令能驱动舵机的路径，也是调试时最常用的写操作。 */
    bus_abort_now();
    s_port->write(s_tx, n);
    s_stats.tx_frames++;
    return true;
}

bool servo_bus_write_one_raw(uint8_t joint_idx, int16_t pos_step)
{
    if ((s_port == NULL) || (joint_idx >= SVB_N)) {
        return false;
    }

    /* ⚠️ 只夹到【编码器物理范围】，不碰标定表、不做角度换算。
     *    语义是"让这个舵机停在物理步数 pos_step"，
     *    而不是"它当前标定表下的步数目标等于 pos_step"
     *    （后者会随标定表变化而漂移）。 */
    int32_t p = (int32_t)pos_step;
    if (p < 0)    { p = 0; }
    if (p > 4095) { p = 4095; }
    const int16_t step = (int16_t)p;

    const uint8_t id = s_ids[joint_idx];
    uint16_t n = 0u;
    if (rlca_sts_build_sync_write_pos(&id, 1u, &step,
                                      s_tx, (uint16_t)sizeof(s_tx), &n) != STS_TX_OK) {
        return false;
    }
    bus_abort_now();
    s_port->write(s_tx, n);
    s_stats.tx_frames++;
    return true;
}

/*==============================================================================
 * 查询与恢复
 *============================================================================*/

bool servo_bus_is_idle(void)
{
    return (s_state == SVB_IDLE) || (s_state == SVB_DONE) || (s_state == SVB_TIMEOUT);
}

servo_bus_state_t servo_bus_state(void)
{
    return s_state;
}

const servo_fb_t *servo_bus_feedback(void)
{
    return s_fb;
}

const rlca_joint_state_t *servo_bus_joint_states(void)
{
    return s_js;
}

bool servo_bus_take_new_feedback(void)
{
    const bool v = s_new_feedback;
    s_new_feedback = false;
    return v;
}

const servo_bus_stats_t *servo_bus_stats(void)
{
    return &s_stats;
}

uint8_t servo_bus_max_temp(void)
{
    uint8_t mx = 0u;
    for (uint8_t i = 0u; i < SVB_N; ++i) {
        if (s_fb[i].valid != 0u) {
            if (s_fb[i].temp_c > mx) {
                mx = s_fb[i].temp_c;
            }
        }
    }
    return mx;
}

bool servo_bus_feedback_fresh(void)
{
    return (s_stats.n_ok_reads > 0u);
}

uint8_t servo_bus_err_flags(void)
{
    return s_err_flags;
}

void servo_bus_abort(void)
{
    bus_abort_now();
}
