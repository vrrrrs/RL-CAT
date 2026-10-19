/**
  ******************************************************************************
  * @file    link_pi.c
  * @brief   树莓派链路状态层实现。
  *
  *  ⚠️ 所有 I/O 与时间都走注入的 rlca_link_port_t，所以本文件不直接依赖 HAL，
  *     状态机可以在 PC 上被确定性地单测（见 tests/host/test_link_pi.c）：
  *       · 模型 A 的 4 ms 等待窗口
  *       · 序列号丢帧统计
  *       · 失联阈值（30 ms 降级 / 100 ms 急停）
  *       · 半帧超时复位、CRC 错、未识别 TYPE
  ******************************************************************************
  */

#include <string.h>

#include "link_pi.h"
#include "link_port.h"
#include "rlca_frame.h"

/*==============================================================================
 * 内部状态
 *============================================================================*/

#define LNK_TX_BUF   (RLCA_FRAME_OVERHEAD + RLCA_FRAME_MAX_PAYLOAD)
#define LNK_RX_CHUNK 64u

/** 端口（依赖注入）。默认绑到真实硬件，测试可注入替身。 */
static const rlca_link_port_t *s_port = NULL;

static rlca_rx_decoder_t s_rx;

static uint32_t s_last_rx_ms = 0u;
static bool     s_ever_rx    = false;
static uint8_t  s_last_seq   = 0u;
static bool     s_seq_valid  = false;

/** ⭐⭐ 历史最大链路空档（ms）—— 诊断用（2026-10-17 加）。
 *
 *  ⚠️ 为什么需要：现场只看到"ESTOP 每秒跳一次"，但 `link_lost_ms` 是**瞬时值**，
 *     每次敲命令看到的都是 20~40 ms，看不出到底有没有真的超过 300 ms 阈值。
 *     这个计数器记住【上电以来最大的那一次空档】，一敲就看得到：
 *        · max_gap 远小于 300  ⇒ ESTOP 不是链路触发的（要查别的）
 *        · max_gap 明显 > 300  ⇒ 确实有长时间断流，再查是谁卡的
 *     ⚠️ 它只在 `link_gap_ms()` 被调用时更新（即每个控制周期一次），
 *        所以两次调用之间的极短空档可能漏采 —— 但对"是否越过阈值"这个
 *        目的足够了（阈值 300 ms 远大于 10 ms 的采样间隔）。
 *
 *  ⚠️ 重启清零（不落盘）。 */
static uint32_t s_max_gap_ms  = 0u;
static uint32_t s_over_estop_cnt = 0u;   /* 空档首次越过 ESTOP 阈值的次数 */
static bool     s_over_active = false;

/** ⭐⭐ 链路空档（ms）—— 对"时间戳在未来"严格防下溢（定义在文件末尾，先声明）。
 *  ⚠️ 必须用它统一计算，不要在各处直接写 `now - s_last_rx_ms`：
 *     那个式子会因无符号下溢把 gap 变成 ~2³²，导致 shell 打印完就判失联急停。
 *     详见文件末尾的定义与注释。 */
static int32_t link_gap_ms(uint32_t now);

static pi_cmd_t   s_cmd_joint;
static bool       s_new_cmd_joint = false;
static pi_mode_t  s_cmd_mode;
static bool       s_new_cmd_mode = false;
static pi_calib_t s_cmd_calib;
static bool       s_new_cmd_calib = false;

static uint8_t s_mode = (uint8_t)RLCA_MODE_IDLE;

/** Pi 最新目标角缓存（见 link_pi_set_pi_q_target 的说明）。
 *  初值 = q_default（全 0），保证"还没收到指令"时也不会下发垃圾角度。 */
static float s_pi_q[RLCA_N_JOINTS];
static bool  s_pi_q_valid = false;

static link_pi_stats_t s_stats;

static uint8_t s_tx_buf[LNK_TX_BUF];
static uint8_t s_tx_payload[RLCA_FRAME_MAX_PAYLOAD];
static uint8_t s_tx_seq = 0u;

/* 收帧缓冲放静态区：rlca_rx_frame_t 含 128 B payload，
 * 本工程栈只有 1 KB，不该在热路径上占它。单线程调用，无重入问题。 */
static rlca_rx_frame_t s_rx_frame;

/*==============================================================================
 * 时间与端口
 *============================================================================*/

static inline uint32_t now_ms(void)
{
    return (s_port != NULL) ? s_port->millis() : 0u;
}

static inline uint32_t now_us(void)
{
    return (s_port != NULL) ? s_port->micros() : 0u;
}

static inline uint32_t us_since(uint32_t t0)
{
    return now_us() - t0;          /* 无符号减法，回绕安全 */
}

void link_pi_set_port(const rlca_link_port_t *port)
{
    s_port = (port != NULL) ? port : link_port_hardware();
}

/*==============================================================================
 * 内部：分发
 *============================================================================*/

static void dispatch(const rlca_rx_frame_t *f)
{
    switch (f->type) {

    case RLCA_TYPE_CMD_JOINT:
    {
        pi_cmd_t c;
        if (link_decode_cmd_joint(f->payload, f->len, &c) == LNK_DEC_OK) {
            /* ⚠️ 序列号丢帧统计只在 CMD_JOINT 上做 ——
             *    它是唯一按 100 Hz 连续发的下行消息。 */
            if (s_seq_valid) {
                const uint8_t expected = (uint8_t)(s_last_seq + 1u);
                if (f->seq != expected) {
                    s_stats.seq_lost += (uint32_t)(uint8_t)(f->seq - expected);
                    /* ⚠️ 丢 1~2 帧是正常的，不做任何降级（协议 §8.2） */
                }
            }
            s_last_seq  = f->seq;
            s_seq_valid = true;

            s_cmd_joint     = c;
            s_new_cmd_joint = true;
            s_stats.cmd_joint_count++;
        }
        break;
    }

    case RLCA_TYPE_CMD_MODE:
    {
        pi_mode_t m;
        if (link_decode_cmd_mode(f->payload, f->len, &m) == LNK_DEC_OK) {
            s_cmd_mode     = m;
            s_new_cmd_mode = true;
        }
        break;
    }

    case RLCA_TYPE_CMD_CALIB:
    {
        pi_calib_t c;
        if (link_decode_cmd_calib(f->payload, f->len, &c) == LNK_DEC_OK) {
            s_cmd_calib     = c;
            s_new_cmd_calib = true;
        }
        break;
    }

    case RLCA_TYPE_HEARTBEAT:
        /* 收到心跳只算"链路活着"（s_last_rx_ms 已在调用方更新），不改状态 */
        break;

    default:
        /* ⚠️ 未识别 TYPE 必须【静默丢弃】并计数，
         *    不要复位状态机、不要刷屏（协议 §3） */
        s_stats.unknown_type++;
        break;
    }
}

/*==============================================================================
 * 生命周期
 *============================================================================*/

void link_pi_init(const rlca_calib_t *calib)
{
    (void)calib;      /* 链路层不做限幅；限幅在上层的 link_limits_apply */

    if (s_port == NULL) {
        s_port = link_port_hardware();
    }

    rlca_rx_init(&s_rx);
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_cmd_joint, 0, sizeof(s_cmd_joint));
    memset(&s_cmd_mode, 0, sizeof(s_cmd_mode));
    memset(&s_cmd_calib, 0, sizeof(s_cmd_calib));

    s_new_cmd_joint = false;
    s_new_cmd_mode  = false;
    s_new_cmd_calib = false;
    s_seq_valid     = false;
    s_ever_rx       = false;
    s_last_seq      = 0u;
    s_mode          = (uint8_t)RLCA_MODE_IDLE;
    s_last_rx_ms    = now_ms();

    /* Pi 目标角缓存初值 = q_default（全 0）：
     * 收到第一条 CMD_JOINT 之前，绝不能让未初始化的浮点值下去。 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_pi_q[i] = RLCA_Q_DEFAULT_RAD;
    }
    s_pi_q_valid = false;
}

void link_pi_poll(void)
{
    if (s_port == NULL) {
        return;
    }

    /* 一次读一批而不是逐字节：逐字节要过一次函数指针 + 一次环缓判断，
     * 1 ms 内上百字节时开销明显。 */
    uint8_t chunk[LNK_RX_CHUNK];
    for (;;) {
        const uint16_t av = s_port->avail();
        if (av == 0u) {
            break;
        }
        const uint16_t want = (av < (uint16_t)sizeof(chunk)) ? av : (uint16_t)sizeof(chunk);
        const uint16_t got = s_port->read(chunk, want);
        if (got == 0u) {
            break;
        }

        const uint32_t t = now_ms();
        for (uint16_t i = 0u; i < got; ++i) {
            const rlca_feed_result_t r = rlca_rx_feed(&s_rx, chunk[i], t, &s_rx_frame);
            switch (r) {
            case RLCA_FEED_FRAME:
                s_stats.rx_frames++;
                s_last_rx_ms = t;
                s_ever_rx    = true;
                dispatch(&s_rx_frame);
                break;
            case RLCA_FEED_CRC_ERR:
                s_stats.crc_errors++;
                break;
            case RLCA_FEED_LEN_ERR:
                s_stats.len_errors++;
                break;
            default:
                break;
            }
        }
    }

    /* 整帧超时：收半个帧后 RLCA_FRAME_RX_TIMEOUT_MS 内没补齐 → 复位状态机。
     * ⚠️ 上位机崩溃/断开时状态机必须能自己重置，否则永远停在半帧状态。 */
    if (rlca_rx_check_timeout(&s_rx, now_ms())) {
        s_stats.rx_timeouts++;
    }
}

uint8_t link_pi_check_link(uint32_t now)
{
    if (!s_ever_rx) {
        /* 从未收到过：仍按最长阈值判，但用上电时刻起算，
         * 避免上电瞬间就报急停。 */
        if ((now - s_last_rx_ms) > RLCA_LINK_ESTOP_MS) {
            return 2u;
        }
        return 0u;
    }

    /* ⚠️ 必须用 link_gap_ms()：见它的说明（shell 阻塞会让时间戳跑到"未来"） */
    const int32_t gap = link_gap_ms(now);
    if (gap > (int32_t)RLCA_LINK_ESTOP_MS) {
        return 2u;
    }
    if (gap > (int32_t)RLCA_LINK_LOST_MS) {
        return 1u;
    }
    return 0u;
}

/*==============================================================================
 * 发送
 *============================================================================*/

static bool send_frame(uint8_t type, const uint8_t *payload, uint8_t len,
                       bool mirror_to_esp)
{
    if (s_port == NULL) {
        return false;
    }
    const uint16_t n = rlca_frame_pack(s_tx_buf, (uint16_t)sizeof(s_tx_buf),
                                       type, s_tx_seq, payload, len);
    if (n == 0u) {
        return false;
    }
    s_tx_seq++;

    /* ⚠️ 先发树莓派（控制链路，优先级最高），再镜像给 ESP32。
     *    顺序不能反：ESP32 侧可能因 TCP 卡住而阻塞到超时，
     *    若排在前面就会推迟 Pi 的遥测，进而挤掉 4 ms 指令等待窗口。 */
    const bool ok_pi = s_port->write(s_tx_buf, n);

    if (mirror_to_esp && (s_port->write_mirror != NULL)) {
        /* ⭐ 真正的"原样镜像"：发的是【同一份已组好的字节】，
         *    而不是重新组一遍（重组会让 SEQ 与内容不一致）。
         *    ⚠️ 走【端口】而不是直接调 esp_fwd —— 后者会让本文件
         *       依赖硬件层，PC 上就链不上、也就无法单测了。 */
        (void)s_port->write_mirror(s_tx_buf, n);
    }

    return ok_pi;
}

bool link_pi_send_tel_fast(const tel_fast_src_t *src)
{
    if (src == NULL) {
        return false;
    }
    const uint8_t n = link_pack_tel_fast(src, s_tx_payload);
    if (n == 0u) {
        return false;
    }
    return send_frame(RLCA_TYPE_TEL_FAST, s_tx_payload, n, true);
}

bool link_pi_send_tel_status(const tel_status_src_t *src)
{
    if (src == NULL) {
        return false;
    }
    const uint8_t n = link_pack_tel_status(src, s_tx_payload);
    if (n == 0u) {
        return false;
    }
    /* ESP32 也要这一帧（方案 A）：它提供 status 仲裁位与关节温度。
     * ⚠️ 本项目里 TEL_STATUS 本就是 10 Hz，比 ESP32 需要的还快，
     *    所以 esp_fwd.c 对【不】对它降频。 */
    return send_frame(RLCA_TYPE_TEL_STATUS, s_tx_payload, n, true);
}

bool link_pi_send_ack(uint8_t action, uint8_t result)
{
    s_tx_payload[0] = action;
    s_tx_payload[1] = result;
    return send_frame(RLCA_TYPE_TEL_ACK, s_tx_payload, RLCA_LEN_TEL_ACK, false);
}

bool link_pi_send_heartbeat(void)
{
    /* 心跳也镜像给 ESP32：它靠心跳判断"STM32 还活着"（32-esp32-bridge.md §4.3）。 */
    return send_frame(RLCA_TYPE_HEARTBEAT, NULL, 0u, true);
}

/*==============================================================================
 * 接收结果读取
 *============================================================================*/

bool link_pi_take_cmd_joint(pi_cmd_t *out)
{
    if (!s_new_cmd_joint) {
        return false;
    }
    if (out != NULL) {
        *out = s_cmd_joint;
    }
    s_new_cmd_joint = false;
    return true;
}

bool link_pi_take_cmd_mode(pi_mode_t *out)
{
    if (!s_new_cmd_mode) {
        return false;
    }
    if (out != NULL) {
        *out = s_cmd_mode;
    }
    s_new_cmd_mode = false;
    return true;
}

void link_pi_set_pi_q_target(const float *q_target)
{
    if (q_target == NULL) {
        return;
    }
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_pi_q[i] = q_target[i];
    }
    s_pi_q_valid = true;
}

const float *link_pi_pi_q_target(void)
{
    return s_pi_q;
}

bool link_pi_take_cmd_calib(pi_calib_t *out)
{
    if (!s_new_cmd_calib) {
        return false;
    }
    if (out != NULL) {
        *out = s_cmd_calib;
    }
    s_new_cmd_calib = false;
    return true;
}

uint8_t link_pi_mode(void)
{
    return s_mode;
}

void link_pi_set_mode(uint8_t mode)
{
    s_mode = link_mode_sanitize(mode);
}

/*==============================================================================
 * 模型 A 的等待窗口
 *============================================================================*/

bool link_pi_wait_cmd(uint32_t timeout_us)
{
    if (s_port == NULL) {
        return false;
    }

    const uint32_t t0 = now_us();

    /* 先看一眼：可能这一帧已经到了，省掉整个窗口 */
    link_pi_poll();
    if (s_new_cmd_joint) {
        return true;
    }

    /* ⚠️ 自旋上限是【必须的】防御：
     *    如果微秒时间源不前进（DWT 未使能、时钟停摆、或时间源被换掉），
     *    `us_since(t0) < timeout_us` 会永远为真，把 100 Hz 主循环彻底锁死。
     *    真实踩到过：link_pi 的单测里假时间由测试驱动，
     *    如果测试不推进时间，这个函数就永久自旋。
     *    上限取 timeout 的 100 倍余量：正常 poll 一次仅几微秒，
     *    真正需要 4 ms 时也只迭代几百次，绝不会触顶。 */
    const uint32_t spin_cap = timeout_us * 100u;

    for (uint32_t spin = 0u; spin < spin_cap; ++spin) {
        link_pi_poll();
        if (s_new_cmd_joint) {
            return true;
        }
        if (us_since(t0) >= timeout_us) {
            break;
        }
    }

    s_stats.wait_timeouts++;
    return false;
}

/*==============================================================================
 * 查询
 *============================================================================*/

const link_pi_stats_t *link_pi_stats(void)
{
    return &s_stats;
}

/** ⭐⭐ 链路空档（ms）—— 对"时间戳在未来"严格防下溢。
 *
 *  ⚠️⚠️ 为什么必须防（2026-10-17 实测定位：这是 ESTOP 反复跳的根因）
 *
 *     `s_last_rx_ms` 是在 `link_pi_poll()` 里用 `now_ms()` 打戳的；
 *     而 `app_poll()` 在【本轮开头】就捕获了一个 `now`，之后才调 `link_pi_poll()`。
 *     正常情况下两者相差几十微秒，无影响。
 *
 *     但 **shell 命令是阻塞的**（`bsp_uart_write` 同步发完才返回）：
 *        'R' 在 115200 baud（ESP32 无线口）下输出约 1400 字符 ≈ 122 ms
 *        'K' 约 3150 字符 ≈ 274 ms
 *     ⇒ shell 跑完才调 `link_pi_poll()`，于是 `s_last_rx_ms` 比 `now` 大 100~270 ms
 *     ⇒ `now - s_last_rx_ms` 按无符号算会下溢成 2³²−x 的巨值
 *     ⇒ 判定"失联 > 100 ms" ⇒ **每按一次 R/K 就进 ESTOP**
 *     ⇒ 而 ESTOP 会开力矩、把关节推向趴下姿态
 *     ⇒ **观察工具把机器人弄动了**（现场现象：R 连按几次，mode_changes 涨 100+）
 *
 *  ⭐ 修法：用带符号差值（补码减法本身对回绕安全），
 *     负数 = "对方时间戳在未来" ⇒ 一律当 0。
 *     ⚠️ 这同时让 tick 回绕（49.7 天）也安全 —— 原写法在回绕瞬间也会误判。 */
static int32_t link_gap_ms(uint32_t now)
{
    const int32_t d = (int32_t)(now - s_last_rx_ms);
    const int32_t gap = (d < 0) ? 0 : d;

    /* ⭐ 记历史最大空档 + 统计"越过 ESTOP 阈值的次数"（诊断用，见 s_max_gap_ms） */
    if ((uint32_t)gap > s_max_gap_ms) {
        s_max_gap_ms = (uint32_t)gap;
    }
    if ((uint32_t)gap > (uint32_t)RLCA_LINK_ESTOP_MS) {
        if (!s_over_active) {
            s_over_active = true;
            s_over_estop_cnt++;      /* 只数"进入"次数，不数持续周期 */
        }
    } else {
        s_over_active = false;
    }
    return gap;
}

/* ⭐ 诊断查询（供 shell 的 R 命令） */
uint32_t link_pi_max_gap_ms(void)
{
    return s_max_gap_ms;
}

uint32_t link_pi_over_estop_count(void)
{
    return s_over_estop_cnt;
}


uint32_t link_pi_lost_ms(uint32_t now)
{
    if (!s_ever_rx) {
        return 0xFFFFFFFFu;
    }
    return (uint32_t)link_gap_ms(now);
}

bool link_pi_is_alive(uint32_t now)
{
    if (!s_ever_rx) {
        return false;
    }
    return link_gap_ms(now) <= (int32_t)RLCA_LINK_LOST_MS;
}
