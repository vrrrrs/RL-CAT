/**
  ******************************************************************************
  * @file    test_link_pi.c
  * @brief   树莓派链路【状态层】单元测试（注入假端口，不需要硬件）。
  *
  *  覆盖 link_proto 单测覆盖不到的部分：
  *    · 帧接收与分发（CMD_JOINT / CMD_MODE / CMD_CALIB / HEARTBEAT）
  *    · 序列号丢帧统计（协议 §8.2）
  *    · 失联阈值：30 ms 降级 / 100 ms 急停（协议 §8.1）
  *    · 半帧超时复位（协议 §6 容错③）
  *    · ⭐ 模型 A 的 4 ms 等待窗口：等得到 / 等不到两种情况
  *    · 未识别 TYPE 静默丢弃并计数（协议 §3）
  *    · 发送侧的帧长（TEL_FAST 103 / TEL_STATUS 47 / HEARTBEAT 7 / TEL_ACK 9）
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_frame.h"
#include "rlca_math.h"
#include "link_proto.h"
#include "link_pi.h"

/*==============================================================================
 * 测试框架
 *============================================================================*/

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                                  \
    do {                                                                  \
        if (cond) { g_pass++; }                                           \
        else {                                                            \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__);                 \
            printf(__VA_ARGS__);                                          \
            printf("\n");                                                 \
        }                                                                 \
    } while (0)

#define CHECK_EQ_U(a, b)                                                  \
    CHECK((a) == (b), "(got %u, want %u)", (unsigned)(a), (unsigned)(b))

/* ⚠️ CHECK() 要求带格式串（-Wformat 在 -Werror 下必须成立）。
 *   只判条件、不需要说明时用 CHECK_M。 */
#define CHECK_M(cond)                                                     \
    do {                                                                  \
        if (cond) { g_pass++; }                                           \
        else {                                                            \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  (condition false)\n", __FILE__, __LINE__); \
        }                                                                 \
    } while (0)

static void section(const char *name)
{
    printf("\n[%s]\n", name);
    fflush(stdout);
}

/*==============================================================================
 * 假链路端口
 *============================================================================*/

#define FB_RXQ 4096

typedef struct {
    uint32_t now_ms;
    uint32_t now_us;
    /** 每次读微秒时自动前进的量（模拟真实 DWT 一直走）。
     *  0 = 冻结（测试自己控制时间）。 */
    uint32_t us_auto_advance;

    uint8_t  rxq[FB_RXQ];
    uint16_t rx_head;
    uint16_t rx_tail;

    /* 发送记录 */
    uint8_t  tx[1024];
    uint16_t tx_len;
    uint32_t tx_count;
    uint16_t last_tx_len;

    /* ---- 镜像（发给 ESP32）记录 ----
     * ⚠️ 用来断言"哪些帧该被镜像、哪些不该"：
     *    TEL_FAST / HEARTBEAT → 镜像
     *    TEL_STATUS / TEL_ACK → 不镜像
     *    这在真实硬件上很难验（要靠抓 UART3 波形），
     *    但在 PC 上是一行断言的事。 */
    uint8_t  mirror[1024];
    uint16_t mirror_len;
    uint32_t mirror_count;
    uint8_t  mirror_type;     /* 镜像帧的 TYPE 字节（偏移 4） */
    uint8_t  mirror_seq;      /* 镜像帧的 SEQ 字节（偏移 3） */

} fake_link_t;

static fake_link_t fl;

static void fl_reset(void)
{
    memset(&fl, 0, sizeof(fl));
}

static void fl_push(const uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0u; i < n; ++i) {
        fl.rxq[fl.rx_head] = d[i];
        fl.rx_head = (uint16_t)((fl.rx_head + 1u) % FB_RXQ);
    }
}

static uint16_t fl_avail(void)
{
    return (uint16_t)((fl.rx_head - fl.rx_tail) % FB_RXQ);
}

static uint16_t fl_read(uint8_t *dst, uint16_t n)
{
    uint16_t got = 0u;
    while ((got < n) && (fl.rx_tail != fl.rx_head)) {
        dst[got++] = fl.rxq[fl.rx_tail];
        fl.rx_tail = (uint16_t)((fl.rx_tail + 1u) % FB_RXQ);
    }
    return got;
}

static void fl_flush(void)
{
    fl.rx_tail = fl.rx_head;
}

static bool fl_write(const uint8_t *d, uint16_t n)
{
    if (n > (uint16_t)sizeof(fl.tx)) {
        return false;
    }
    memcpy(fl.tx, d, n);
    fl.tx_len = n;
    fl.last_tx_len = n;
    fl.tx_count++;
    return true;
}


/** 带自动前进的微秒读取（link_pi 通过端口指针调用它）
 *  ⚠️ 真实 DWT 是"读的时候自然在走"，所以这里每次读都前进一点，
 *     才能测出"等待窗口靠时间耗尽而退出"。 */
static uint32_t fl_micros_advancing(void)
{
    const uint32_t v = fl.now_us;
    fl.now_us += fl.us_auto_advance;
    return v;
}

static uint32_t fl_millis(void)
{
    return fl.now_ms;
}

/** 镜像回调：记录帧内容与 TYPE/SEQ，供断言使用。 */
static bool fl_write_mirror(const uint8_t *frame, uint16_t n)
{
    if (n > (uint16_t)sizeof(fl.mirror)) {
        return false;
    }
    memcpy(fl.mirror, frame, n);
    fl.mirror_len = n;
    fl.mirror_count++;
    /* 帧布局：0=AA 1=55 2=LEN 3=SEQ 4=TYPE */
    if (n >= 5u) {
        fl.mirror_seq  = frame[3];
        fl.mirror_type = frame[4];
    }
    return true;
}

static const rlca_link_port_t fl_port = {
    fl_write, fl_write_mirror, fl_avail, fl_read, fl_flush, fl_micros_advancing, fl_millis
};

/* link_port_hardware() 的桩在 host_link_port_stub.c 里（共享给所有套件）。
 * 测试结尾会断言它一次都没被调用过 —— 即"所有用例都显式注入了端口"。 */
extern int host_link_port_uses(void);

/*==============================================================================
 * 构造下行帧
 *============================================================================*/


static void push_cmd_joint(uint8_t seq, uint8_t mode, const float *q)
{
    uint8_t payload[RLCA_LEN_CMD_JOINT];
    memset(payload, 0, sizeof(payload));
    payload[0] = mode;
    payload[1] = RLCA_FLAG_ENABLE;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const float v = (q != NULL) ? q[i] : 0.0f;
        rlca_put_i16(&payload[2u + (2u * i)], link_enc_i16(v, RLCA_Q_UNIT_RAD));
    }
    payload[29] = 1u;

    uint8_t frame[64];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                       RLCA_TYPE_CMD_JOINT, seq, payload,
                                       RLCA_LEN_CMD_JOINT);
    fl_push(frame, n);
}

static void push_cmd_mode(uint8_t seq, uint8_t mode)
{
    uint8_t payload[RLCA_LEN_CMD_MODE] = { mode, 0u };
    uint8_t frame[16];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                       RLCA_TYPE_CMD_MODE, seq, payload,
                                       RLCA_LEN_CMD_MODE);
    fl_push(frame, n);
}

static void push_cmd_calib(uint8_t seq, uint8_t action)
{
    uint8_t payload[RLCA_LEN_CMD_CALIB] = { action };
    uint8_t frame[16];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                       RLCA_TYPE_CMD_CALIB, seq, payload,
                                       RLCA_LEN_CMD_CALIB);
    fl_push(frame, n);
}

static void push_heartbeat(uint8_t seq)
{
    uint8_t frame[16];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                       RLCA_TYPE_HEARTBEAT, seq, NULL, 0u);
    fl_push(frame, n);
}

/** 构造一个"未知 TYPE"的合法帧（CRC 正确） */
static void push_unknown_type(uint8_t seq)
{
    uint8_t payload[4] = { 1u, 2u, 3u, 4u };
    uint8_t frame[16];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                       0x5Au, seq, payload, 4u);
    fl_push(frame, n);
}

/*==============================================================================
 * 用例
 *============================================================================*/

static void test_init_and_idle(void)
{
    section("link_pi: init / idle behaviour");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    CHECK_EQ_U(link_pi_mode(), RLCA_MODE_IDLE);
    CHECK(!link_pi_is_alive(0u), "must not claim alive before any frame");
    CHECK_EQ_U(link_pi_lost_ms(0u), 0xFFFFFFFFu);

    /* 没有任何字节时 poll 不应做任何事 */
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 0u);

    /* 上电初期不应报急停（用上电时刻起算） */
    CHECK_EQ_U(link_pi_check_link(10u), 0u);
    CHECK_EQ_U(link_pi_check_link(RLCA_LINK_ESTOP_MS), 0u);
    CHECK_EQ_U(link_pi_check_link(RLCA_LINK_ESTOP_MS + 1u), 2u);
}

static void test_receive_and_dispatch(void)
{
    section("link_pi: frame receive and dispatch");

    fl_reset();
    fl.now_ms = 1000u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* ---- CMD_JOINT ---- */
    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.01f * (float)i; }
    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, q);
    link_pi_poll();

    CHECK_EQ_U(link_pi_stats()->rx_frames, 1u);
    CHECK_EQ_U(link_pi_stats()->cmd_joint_count, 1u);
    CHECK(link_pi_is_alive(1000u), "must be alive right after a frame");
    CHECK_EQ_U(link_pi_lost_ms(1000u), 0u);

    pi_cmd_t c;
    CHECK(link_pi_take_cmd_joint(&c), "must have a new CMD_JOINT");
    CHECK_EQ_U(c.mode, RLCA_MODE_ACTIVE);
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK(fabsf(c.q_target[i] - q[i]) < 1e-6f,
              "q_target[%u] mismatch (got %.6f want %.6f)",
              (unsigned)i, (double)c.q_target[i], (double)q[i]);
    }
    /* 取走后标志必须清掉 */
    CHECK(!link_pi_take_cmd_joint(&c), "take must clear the new-flag");

    /* ---- CMD_MODE ---- */
    push_cmd_mode(1u, (uint8_t)RLCA_MODE_DAMP);
    link_pi_poll();
    pi_mode_t m;
    CHECK(link_pi_take_cmd_mode(&m), "must have a new CMD_MODE");
    CHECK_EQ_U(m.mode, RLCA_MODE_DAMP);

    /* ---- CMD_CALIB ---- */
    push_cmd_calib(2u, RLCA_CALIB_START);
    link_pi_poll();
    pi_calib_t cb;
    CHECK(link_pi_take_cmd_calib(&cb), "must have a new CMD_CALIB");
    CHECK_EQ_U(cb.action, RLCA_CALIB_START);

    /* ---- HEARTBEAT：不改状态、只刷新存活时间 ---- */
    fl.now_ms = 2000u;
    push_heartbeat(3u);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 4u);
    CHECK_EQ_U(link_pi_lost_ms(2000u), 0u);
    CHECK_EQ_U(link_pi_stats()->unknown_type, 0u);

    /* ---- 未识别 TYPE：静默丢弃 + 计数 ---- */
    push_unknown_type(4u);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 5u);
    CHECK_EQ_U(link_pi_stats()->unknown_type, 1u);
    CHECK(!link_pi_take_cmd_joint(&c), "unknown type must not produce a command");
}

static void test_crc_and_len_errors(void)
{
    section("link_pi: CRC error and LEN error accounting");

    fl_reset();
    fl.now_ms = 100u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* ---- CRC 错：帧体照发，破坏 CRC ---- */
    uint8_t payload[RLCA_LEN_CMD_JOINT];
    memset(payload, 0, sizeof(payload));
    uint8_t frame[64];
    uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                 RLCA_TYPE_CMD_JOINT, 0u, payload, RLCA_LEN_CMD_JOINT);
    frame[n - 1u] ^= 0xFFu;
    fl_push(frame, n);
    link_pi_poll();

    CHECK_EQ_U(link_pi_stats()->crc_errors, 1u);
    CHECK_EQ_U(link_pi_stats()->rx_frames, 0u);
    /* ⚠️ CRC 错不能把链路判死：收到过字节就应该刷新存活时间？——
     *    这里明确【不刷新】：只有合法帧才算"Pi 活着"，
     *    否则噪声会让固件误以为链路正常。 */
    CHECK(!link_pi_is_alive(100u), "corrupted frame must NOT count as alive");

    /* ---- LEN 超限 ---- */
    const uint8_t over[5] = { 0xAAu, 0x55u, 0xFFu, 0u, 0u };
    fl_push(over, sizeof(over));
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->len_errors, 1u);

    /* ---- 合法帧仍能被正确接收（说明前面没把状态机卡死） ---- */
    n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                        RLCA_TYPE_CMD_JOINT, 0u, payload, RLCA_LEN_CMD_JOINT);
    fl_push(frame, n);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 1u);
    CHECK(link_pi_is_alive(100u), "legal frame must count as alive");
}

static void test_seq_loss(void)
{
    section("link_pi: sequence-loss accounting (protocol 8.2)");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* 正常连续：0,1,2,3 —— 不该记丢帧 */
    for (uint8_t s = 0u; s < 4u; ++s) {
        push_cmd_joint(s, (uint8_t)RLCA_MODE_ACTIVE, NULL);
        link_pi_poll();
    }
    CHECK_EQ_U(link_pi_stats()->seq_lost, 0u);

    /* 跳 3 个：3 -> 7（丢了 4,5,6 共 3 帧） */
    push_cmd_joint(7u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->seq_lost, 3u);

    /* ⚠️ 丢帧【不能】触发任何降级：模式与存活状态都不变 */
    CHECK_EQ_U(link_pi_check_link(0u), 0u);

    /* 往回绕（seq 满 255 回 0）不应算丢帧 */
    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    link_pi_init(&cal);
    push_cmd_joint(254u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    push_cmd_joint(255u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->seq_lost, 0u);
}

static void test_link_loss_thresholds(void)
{
    section("link_pi: 30 ms degrade / 100 ms emergency stop (protocol 8.1)");

    fl_reset();
    fl.now_ms = 5000u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();

    /* 边界必须精确：> 30 ms 才降级、> 100 ms 才急停 */
    CHECK_EQ_U(link_pi_check_link(5000u), 0u);
    CHECK_EQ_U(link_pi_check_link(5000u + RLCA_LINK_LOST_MS), 0u);
    CHECK_EQ_U(link_pi_check_link(5000u + RLCA_LINK_LOST_MS + 1u), 1u);
    CHECK_EQ_U(link_pi_check_link(5000u + RLCA_LINK_ESTOP_MS), 1u);
    CHECK_EQ_U(link_pi_check_link(5000u + RLCA_LINK_ESTOP_MS + 1u), 2u);

    /* 收到新帧后立刻恢复 */
    fl.now_ms = 5000u + RLCA_LINK_ESTOP_MS + 50u;
    push_cmd_joint(1u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    CHECK_EQ_U(link_pi_check_link(fl.now_ms), 0u);
    CHECK(link_pi_is_alive(fl.now_ms), "must be alive after a fresh frame");
}

/*==============================================================================
 * ⭐⭐ 回归：`now` 早于 `s_last_rx_ms` 时绝不能判失联（2026-10-17 修的无符号下溢）
 *
 *  现场症状：每按一次 shell 的 `R`/`K`，固件就进一次 ESTOP，
 *            mode_changes 连按几次就涨 100+，且 ESTOP 会开力矩把关节推向趴下姿态。
 *
 *  根因：`app_poll()` 在【本轮开头】捕获 now，之后才调 `link_pi_poll()`；
 *        而 shell 命令是阻塞的（ESP32 无线口 115200 baud 下：
 *        'R' ≈ 122 ms、'K' ≈ 274 ms），所以 `s_last_rx_ms` 会比 now 大。
 *        原实现 `now - s_last_rx_ms` 按 uint32 算 ⇒ 下溢成 ~2³²
 *        ⇒ 判成"失联 > 100 ms" ⇒ 急停。
 *
 *  ⇒ 本用例把"时间戳在未来"这种情况钉死：必须一律当 gap = 0（健康）。
 *============================================================================*/

static void test_now_earlier_than_last_rx(void)
{
    section("link_pi: now EARLIER than last_rx must NOT look lost (underflow regression)");

    fl_reset();
    fl.now_ms = 0u;
    fl.now_us = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* 在 t = 5000 ms 收到一帧 ⇒ s_last_rx_ms = 5000 */
    fl.now_ms = 5000u;
    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();

    CHECK(link_pi_is_alive(5000u), "baseline: alive at the same instant");

    /* ⭐ 核心：用【更小的】now 去查 —— 模拟 shell 阻塞后 now 还是旧值。
     *  修之前：5000-5000 没问题，但 now=4000 时 4000-5000 下溢成巨值 ⇒ 判失联。
     *  ⚠️ 这里 4800 = 阻塞前捕获的 now，实际收帧发生在 5000。 */
    CHECK(link_pi_is_alive(4800u),
          "now earlier than last_rx must be treated as gap=0, not as huge gap");
    CHECK_EQ_U(link_pi_lost_ms(4800u), 0u);
    CHECK_EQ_U(link_pi_check_link(4800u), 0u);

    /* 极端：now 比 last_rx 小很多（长 shell 命令 + 时钟抖动） */
    CHECK_EQ_U(link_pi_check_link(1000u), 0u);
    CHECK_EQ_U(link_pi_lost_ms(1000u), 0u);

    /* 正常方向仍然要能判出来（不能把真失联也一起放过） */
    CHECK_EQ_U(link_pi_check_link(5000u + RLCA_LINK_ESTOP_MS + 1u), 2u);
    CHECK(!link_pi_is_alive(5000u + RLCA_LINK_LOST_MS + 1u),
          "a real gap must still be reported");
}

static void test_wait_cmd_window(void)
{
    section("link_pi: model-A 4 ms wait window (the critical path)");

    fl_reset();
    fl.now_ms = 0u;
    fl.now_us = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* ---- ① 指令【已经】到了：应立即返回，不消耗窗口 ---- */
    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    fl.now_us = 0u;
    {
        const bool got = link_pi_wait_cmd(RLCA_LINK_CMD_WAIT_US);
        CHECK(got, "must return true when the frame is already queued");
        CHECK_EQ_U(fl.now_us, 0u);   /* 一次 poll 就拿到，时间没走 */
    }

    /* 取走，准备下一轮 */
    pi_cmd_t c;
    (void)link_pi_take_cmd_joint(&c);

    /* ---- ② 指令【永远不来】：必须在 4 ms 后返回 false（不能死等） ----
     * ⚠️ 手法：让假微秒源随每次读取自动前进 50 µs，模拟真实 DWT。
     *    这样等待窗口是靠"时间耗尽"退出，而不是靠自旋上限 ——
     *    后者只是兜底（见 link_pi_wait_cmd 里的 spin_cap 注释）。 */
    fl_flush();
    fl.now_us = 1000000u;
    fl.us_auto_advance = 50u;
    {
        const uint32_t t0 = fl.now_us;
        const bool got = link_pi_wait_cmd(RLCA_LINK_CMD_WAIT_US);
        const uint32_t dt = fl.now_us - t0;
        CHECK(!got, "must return false when nothing arrives");
        CHECK(dt >= RLCA_LINK_CMD_WAIT_US,
              "must wait out the whole window (took %u us)", (unsigned)dt);
        CHECK(dt <= (RLCA_LINK_CMD_WAIT_US + 500u),
              "must not block much longer than the window (took %u us)", (unsigned)dt);
    }
    fl.us_auto_advance = 0u;
    CHECK_EQ_U(link_pi_stats()->wait_timeouts, 1u);

    /* ---- ③ 指令在窗口中途到达：必须在窗口内被取到 ----
     * ⚠️ 用"分片推进"而不是真线程：假时间只在 poll 内推进，
     *    另开线程等时间会死锁。这里按 200 µs 一片推进，
     *    在第 1.4 ms 时投递帧 —— 与真实"Pi 在 1~2 ms 后回帧"等价，
     *    且完全确定性、无竞态。 */
    fl_reset();
    fl.now_ms = 0u;
    fl.now_us = 0u;
    link_pi_set_port(&fl_port);
    link_pi_init(&cal);
    (void)link_pi_take_cmd_joint(&c);

    {
        const uint32_t t0 = fl.now_us;
        bool got = false;
        bool injected = false;
        uint32_t arrived_at = 0u;

        while ((fl.now_us - t0) < RLCA_LINK_CMD_WAIT_US) {
            if (!injected && (fl.now_us - t0) >= 1400u) {
                push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
                injected = true;
            }
            link_pi_poll();
            if (link_pi_take_cmd_joint(&c)) {
                got = true;
                arrived_at = fl.now_us - t0;
                break;
            }
            fl.now_us += 200u;
        }

        CHECK(injected, "sanity: the frame was injected");
        CHECK(got, "must catch the frame that arrives inside the window");
        CHECK(arrived_at < RLCA_LINK_CMD_WAIT_US,
              "must be caught inside the 4 ms window (at %u us)", (unsigned)arrived_at);
        printf("    frame injected at 1400 us, caught at %u us\n", (unsigned)arrived_at);
    }
}

static void test_send_frame_lengths(void)
{
    section("link_pi: outgoing frame lengths");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* TEL_FAST：整帧 103 字节（LEN=96 + 7） */
    tel_fast_src_t ts;
    memset(&ts, 0, sizeof(ts));
    ts.quat_wxyz[0] = 1.0f;
    CHECK(link_pi_send_tel_fast(&ts), "send TEL_FAST");
    CHECK_EQ_U(fl.last_tx_len, 103u);
    CHECK_EQ_U(fl.tx[0], 0xAAu);
    CHECK_EQ_U(fl.tx[1], 0x55u);
    CHECK_EQ_U(fl.tx[2], RLCA_LEN_TEL_FAST);
    CHECK_EQ_U(fl.tx[4], RLCA_TYPE_TEL_FAST);

    /* TEL_STATUS：47 字节 */
    static uint8_t temps[RLCA_N_JOINTS];
    tel_status_src_t ss;
    memset(&ss, 0, sizeof(ss));
    ss.temp_deci_c = temps;
    CHECK(link_pi_send_tel_status(&ss), "send TEL_STATUS");
    CHECK_EQ_U(fl.last_tx_len, 47u);
    CHECK_EQ_U(fl.tx[4], RLCA_TYPE_TEL_STATUS);

    /* HEARTBEAT：7 字节（LEN=0） */
    CHECK(link_pi_send_heartbeat(), "send HEARTBEAT");
    CHECK_EQ_U(fl.last_tx_len, 7u);
    CHECK_EQ_U(fl.tx[2], 0u);
    CHECK_EQ_U(fl.tx[4], RLCA_TYPE_HEARTBEAT);

    /* TEL_ACK：9 字节 */
    CHECK(link_pi_send_ack(RLCA_CALIB_SAVE, 0u), "send TEL_ACK");
    CHECK_EQ_U(fl.last_tx_len, 9u);
    CHECK_EQ_U(fl.tx[4], RLCA_TYPE_TEL_ACK);
    CHECK_EQ_U(fl.tx[5], RLCA_CALIB_SAVE);
    CHECK_EQ_U(fl.tx[6], 0u);

    /* ⚠️ 所有上行帧都必须能被同一个解帧器解回来（自洽性） */
    fl.rx_tail = fl.rx_head;      /* 清空接收 */
    fl_push(fl.tx, fl.tx_len);
    rlca_rx_decoder_t d;
    rlca_rx_init(&d);
    rlca_rx_frame_t f;
    rlca_feed_result_t r = RLCA_FEED_NONE;
    for (uint16_t i = 0u; i < fl.tx_len; ++i) {
        r = rlca_rx_feed(&d, fl.tx[i], 0u, &f);
    }
    CHECK(r == RLCA_FEED_FRAME, "our own TEL_ACK must decode");
    CHECK_EQ_U(f.type, RLCA_TYPE_TEL_ACK);
    CHECK_EQ_U(f.len, RLCA_LEN_TEL_ACK);
}

/** ⭐ ESP32 镜像：哪些帧该转发到 UART3，哪些不该。
 *
 *  ⚠️ 这个行为在真机上很难验（要抓 UART3 波形，还得同时看 USART1），
 *     但在 PC 上就是几行断言。策略依据 docs/32-esp32-bridge.md §4.3
 *     与 docs/36-esp32-plan.md §2.2。
 *
 *  ⚠️ 注意本测试只验【link_pi 的镜像决策】（哪些帧送进 write_mirror）；
 *     降频（100 Hz → 10 Hz）发生在 esp_fwd.c 里，属于硬件层，
 *     不在本套件覆盖范围内（见 firmware/stm32/Core/Src/esp_fwd.c）。 */
static void test_esp_mirror_policy(void)
{
    section("link_pi: ESP32 mirror policy (which frames are handed to UART3)");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* ---- TEL_FAST：必须镜像，且是【同一份字节】---- */
    tel_fast_src_t ts;
    memset(&ts, 0, sizeof(ts));
    ts.quat_wxyz[0] = 1.0f;
    CHECK_M(link_pi_send_tel_fast(&ts));
    CHECK_EQ_U(fl.mirror_count, 1u);
    CHECK_EQ_U(fl.mirror_len, 103u);
    CHECK_EQ_U(fl.mirror_type, RLCA_TYPE_TEL_FAST);
    /* ⭐ 核心断言：镜像的就是发给 Pi 的那一份，逐字节相同。
     *    若这里改成"重新组帧"，SEQ 就会不一致，两条链路看到的
     *    就不是同一份数据了。 */
    CHECK_EQ_U(fl.mirror_len, fl.last_tx_len);
    CHECK(memcmp(fl.mirror, fl.tx, fl.last_tx_len) == 0,
          "mirror must be byte-identical to the Pi frame");
    CHECK_EQ_U(fl.mirror_seq, fl.tx[3]);

    /* ---- TEL_STATUS：也必须镜像（方案 A）----
     * ESP32 侧靠它拿 status 仲裁位与关节温度。 */
    static uint8_t temps[RLCA_N_JOINTS];
    tel_status_src_t ss;
    memset(&ss, 0, sizeof(ss));
    ss.temp_deci_c = temps;
    const uint32_t before_ss = fl.mirror_count;
    CHECK_M(link_pi_send_tel_status(&ss));
    CHECK_EQ_U(fl.mirror_count, before_ss + 1u);
    CHECK_EQ_U(fl.mirror_len, 47u);
    CHECK_EQ_U(fl.mirror_type, RLCA_TYPE_TEL_STATUS);
    CHECK(memcmp(fl.mirror, fl.tx, fl.last_tx_len) == 0,
          "TEL_STATUS mirror must match too");

    /* ---- HEARTBEAT：镜像（ESP32 靠它判断 STM32 是否存活）---- */
    const uint32_t before_hb = fl.mirror_count;
    CHECK_M(link_pi_send_heartbeat());
    CHECK_EQ_U(fl.mirror_count, before_hb + 1u);
    CHECK_EQ_U(fl.mirror_len, 7u);
    CHECK_EQ_U(fl.mirror_type, RLCA_TYPE_HEARTBEAT);

    /* ---- TEL_ACK：不镜像 ----
     * 它是【控制应答】（标定命令的回执），ESP32 不需要，
     * 转发过去只是白占带宽。 */
    const uint32_t before_ack = fl.mirror_count;
    CHECK_M(link_pi_send_ack(RLCA_CALIB_SAVE, 0u));
    CHECK_EQ_U(fl.mirror_count, before_ack);
    CHECK_EQ_U(fl.last_tx_len, 9u);      /* 但确实发给 Pi 了 */

    printf("    mirror_count=%u (TEL_FAST+TEL_STATUS+HEARTBEAT, not TEL_ACK)\n",
           (unsigned)fl.mirror_count);
}

static void test_partial_frame_timeout(void)
{
    section("link_pi: half-frame timeout resets the state machine");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* 只发半个帧头 + LEN，然后时间前进到超过 5 ms */
    const uint8_t half[3] = { 0xAAu, 0x55u, 0x1Fu };
    fl_push(half, sizeof(half));
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_timeouts, 0u);

    fl.now_ms = RLCA_FRAME_RX_TIMEOUT_MS;
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_timeouts, 0u);      /* 边界上不算超时 */

    fl.now_ms = RLCA_FRAME_RX_TIMEOUT_MS + 1u;
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_timeouts, 1u);

    /* ⚠️ 复位后必须能正常收帧（这正是超时机制存在的意义） */
    push_cmd_joint(0u, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 1u);
}

static void test_many_frames_in_one_poll(void)
{
    section("link_pi: many frames in a single poll (no data loss)");

    fl_reset();
    fl.now_ms = 0u;
    link_pi_set_port(&fl_port);
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    link_pi_init(&cal);

    /* 一次塞 50 帧，一次 poll 必须全部处理掉 ——
     * ⚠️ 如果实现里"每 poll 只读一个字节"，这里会明显失败。 */
    for (uint8_t i = 0u; i < 50u; ++i) {
        push_cmd_joint(i, (uint8_t)RLCA_MODE_ACTIVE, NULL);
    }
    CHECK(fl_avail() > 200u, "sanity: 50 frames must be queued (%u bytes)", (unsigned)fl_avail());
    link_pi_poll();
    CHECK_EQ_U(link_pi_stats()->rx_frames, 50u);
    CHECK_EQ_U(fl_avail(), 0u);
    CHECK_EQ_U(link_pi_stats()->crc_errors, 0u);
    CHECK_EQ_U(link_pi_stats()->seq_lost, 0u);
}

/*==============================================================================
 * main
 *============================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT link_pi unit tests (fake port)\n");
    printf("========================================\n");
    fflush(stdout);

    test_init_and_idle();
    test_receive_and_dispatch();
    test_crc_and_len_errors();
    test_seq_loss();
    test_link_loss_thresholds();
    test_now_earlier_than_last_rx();
    test_send_frame_lengths();
    test_esp_mirror_policy();
    test_partial_frame_timeout();
    test_many_frames_in_one_poll();
    test_wait_cmd_window();

    /* ⚠️ 关键自检：所有用例都必须【显式注入】端口。
     *    若这一项不为 0，说明有测试漏了注入，
     *    那么它的结论就是无效的（等于什么都没测）。 */
    CHECK(host_link_port_uses() == 0,
          "every test must inject a port (hardware port was consulted %d times)",
          host_link_port_uses());

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
