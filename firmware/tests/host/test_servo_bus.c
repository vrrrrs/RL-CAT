/**
  ******************************************************************************
  * @file    test_servo_bus.c
  * @brief   舵机总线事务状态机的离线单元测试（带假舵机模拟器）。
  *
  *  核心手法：servo_bus 通过 rlca_servo_port_t 做依赖注入，
  *           所以这里注入一个"假总线"：
  *             · 记录发出去的每一帧（可断言 SyncWrite/SyncRead 的字节布局）
  *             · 按真实 STS 布局自动回 12 帧反馈
  *             · 可注入故障：不回帧 / 回错校验和 / 只回一部分 / 打乱顺序
  *
  *  ⚠️ 时间由测试完全控制（假 micros），所以"3 ms 超时"这类判据
  *     可以在 PC 上确定性地验证，不需要真等 3 ms。
  *
  *  覆盖伺服总线模块设计 §8 验收清单中【不需要真舵机】的部分：
  *     5  12 舵机 SyncRead 反馈 valid 全为 1
  *     6  连续多轮零错误（timeouts/crc_errors/溢出 均为 0）
  *     7  丢帧恢复（热拔一个舵机，其余仍正常）
  *     8  全丢帧恢复（3 ms 内超时返回，"周期"不被无限拉长）
  *     9  错误码读取
  *    12  事务耗时 < 3 ms
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_sts.h"
#include "rlca_frame.h"   /* rlca_get_i16 */
#include "rlca_math.h"
#include "servo_bus.h"

/*==============================================================================
 * 极简测试框架（与本目录其它测试一致）
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
#define CHECK_EQ_D(a, b)                                                  \
    CHECK((a) == (b), "(got %d, want %d)", (int)(a), (int)(b))

static void section(const char *name)
{
    printf("\n[%s]\n", name);
    fflush(stdout);
}

/*==============================================================================
 * 假总线（fake bus）
 *============================================================================*/

#define FB_N            RLCA_N_JOINTS
#define FB_TXLEN        256
#define FB_RXLEN        512

typedef struct {
    /* 时间 */
    uint32_t now_us;

    /* 收：模拟器写进 rxq，servo_bus 从这里读 */
    uint8_t  rxq[FB_RXLEN];
    uint16_t rx_head, rx_tail;
    uint16_t rx_overflow;

    /* 发：记录 servo_bus 写出的帧 */
    uint8_t  tx[FB_TXLEN];
    uint16_t tx_len;

    /* 上一次 SyncWrite 每个舵机的目标位置（按 ID 反查） */
    int16_t  goal_step[FB_N];
    bool     goal_valid[FB_N];

    /* 模拟器：每个舵机的"真实"状态 */
    int16_t  sim_pos[FB_N];
    int16_t  sim_speed[FB_N];
    int16_t  sim_load[FB_N];
    uint8_t  sim_volt_dv;
    uint8_t  sim_temp_c;
    uint8_t  sim_err[FB_N];

    /* 故障注入 */
    uint8_t  dead_id;          /* 该 ID 不回帧；0 = 不注入        */
    uint8_t  badcrc_id;        /* 该 ID 回帧但校验和错；0 = 不注入 */
    uint8_t  partial_frac;     /* 只回前 N 帧；0 = 全回             */
    bool     shuffle;          /* 打乱回复顺序                     */
    bool     no_reply;         /* 完全不回                         */
    bool     noise_prefix;     /* 在回复前插几个噪声字节            */
    uint16_t latency_us;       /* 回复延迟（模拟真实总线时间）       */

    /* 延迟建模：SimRead 时数据先写进 rxq，但在 rx_ready_us 之前 avail() 返回 0
     * —— 这就是"12 帧要 1.8 ms 才到齐"的模拟。 */
    uint32_t rx_ready_us;

    /* ⭐ PING 支持 + 碎片化投递（用于钉住 txn_expect_ack 的累积逻辑）
     *
     * ⚠️ 为什么要模拟"碎片化"：
     *     实测中 `Y` 能收到完整的 6 字节应答，而 servo_bus_ping()
     *     在同样的总线上却总判失败 —— 因为 DMA 搬运与 IDLE 判帧是
     *     分开的，应答可能被切成两段先后进入环形缓冲。
     *     原实现"读一次就判"，读到半帧就 return。
     *
     * ping_reply_id 非 0 时回应 PING；ping_frag1 指定"第一次只放几个字节"，
     * 0 = 不碎片化（一次性给完）。 */
    uint8_t  ping_reply_id;
    uint16_t ping_frag1;       /* 第一段的字节数；0 = 一次给完 */
    uint16_t ping_pending;     /* 还剩多少字节没放出去           */
    uint32_t ping_first_us;    /* 第一段的到达时刻（算第二段释放时间）*/
    uint32_t ping_release_us;  /* 第二段的释放时刻               */
    bool     auto_tick;        /* ⭐ 读时间时自动 +1µs（见 fb_port_micros）*/
    uint8_t  only_ids[FB_N];   /* ⭐ 只让这些 ID 应答 SyncRead；n_only=0 时全应答 */
    uint8_t  n_only;

    /* 统计 */
    uint32_t n_syncwrite;
    uint32_t n_syncread;
    uint32_t n_other;
} fake_bus_t;

static fake_bus_t fb;

static void fb_reset(void)
{
    memset(&fb, 0, sizeof(fb));
    fb.sim_volt_dv = 120u;      /* 12.0 V */
    fb.sim_temp_c  = 35u;
    for (uint8_t i = 0u; i < FB_N; ++i) {
        fb.sim_pos[i]   = (int16_t)(2048 + (int16_t)i * 10);
        fb.sim_speed[i] = 0;
        fb.sim_load[i]  = 0;
        fb.sim_err[i]   = 0u;
    }
    fb.latency_us = 1500u;      /* 12 帧 ≈ 1.8 ms，取 1.5 ms */
}

/* ---- 端口实现 ---- */

static void fb_port_write(const uint8_t *data, uint16_t len)
{
    if (len > FB_TXLEN) { len = FB_TXLEN; }
    memcpy(fb.tx, data, len);
    fb.tx_len = len;

    /* 解析帧类型 */
    if ((len >= 6u) && (data[0] == STS_HEAD) && (data[1] == STS_HEAD)) {
        const uint8_t instr = data[4];

        if (instr == STS_INST_SYNC_WRITE) {
            fb.n_syncwrite++;
            /* 参数区：addr item_len {id L H}*  —— 从偏移 5 开始 */
            const uint8_t addr = data[5];
            const uint8_t item_len = data[6];
            if ((addr == STS_REG_GOAL_POSITION_L) && (item_len == 2u)) {
                const uint8_t n = (uint8_t)((len - 5u - 2u - 1u) / 3u);
                for (uint8_t k = 0u; k < n; ++k) {
                    const uint16_t o = (uint16_t)(7u + (uint16_t)k * 3u);
                    const uint8_t id = data[o];
                    const int16_t pos = rlca_get_i16(&data[o + 1u]);
                    for (uint8_t i = 0u; i < FB_N; ++i) {
                        if (RLCA_SERVO_ID(i) == id) {
                            fb.goal_step[i] = pos;
                            fb.goal_valid[i] = true;
                            fb.sim_pos[i] = pos;      /* 假舵机立刻到位 */
                            break;
                        }
                    }
                }
            }
        } else if (instr == STS_INST_SYNC_READ) {
            fb.n_syncread++;

            if (fb.no_reply) {
                return;
            }

            /* 构造回复顺序 */
            uint8_t order[FB_N];
            uint8_t cnt = 0u;
            for (uint8_t i = 0u; i < FB_N; ++i) {
                if (fb.dead_id != 0u && RLCA_SERVO_ID(i) == fb.dead_id) { continue; }
                /* ⭐ n_only > 0 时：只让 only_ids[] 里的舵机应答
                 * （模拟"只接了 2 个舵机"的真实场景） */
                if (fb.n_only > 0u) {
                    bool in_list = false;
                    for (uint8_t k = 0u; k < fb.n_only; ++k) {
                        if (fb.only_ids[k] == RLCA_SERVO_ID(i)) { in_list = true; break; }
                    }
                    if (!in_list) { continue; }
                }
                order[cnt++] = i;
            }
            if (fb.partial_frac != 0u && fb.partial_frac < cnt) {
                cnt = fb.partial_frac;
            }
            if (fb.shuffle) {
                /* 简单确定性交换：首尾对调 */
                for (uint8_t i = 0u; i + 1u < cnt; i += 2u) {
                    const uint8_t t = order[i]; order[i] = order[i + 1u]; order[i + 1u] = t;
                }
            }
            if (fb.noise_prefix) {
                for (uint8_t i = 0u; i < 3u; ++i) {
                    fb.rxq[fb.rx_head] = 0x5Au;
                    fb.rx_head = (uint16_t)((fb.rx_head + 1u) % FB_RXLEN);
                }
            }
            for (uint8_t k = 0u; k < cnt; ++k) {
                const uint8_t i = order[k];
                uint8_t data9[STS_FB_DATA_LEN];
                rlca_sts_pack_fb_data(data9, fb.sim_pos[i], fb.sim_speed[i],
                                      fb.sim_load[i], fb.sim_volt_dv,
                                      fb.sim_temp_c, 0u);
                uint8_t frame[STS_FB_FRAME_LEN + 1u];
                (void)rlca_sts_build_feedback(frame, RLCA_SERVO_ID(i),
                                              fb.sim_err[i], data9);
                if ((fb.badcrc_id != 0u) && (RLCA_SERVO_ID(i) == fb.badcrc_id)) {
                    frame[5u + STS_FB_DATA_LEN] ^= 0xFFu;   /* 破坏校验和 */
                }
                for (uint8_t b = 0u; b < (uint8_t)(STS_FB_DATA_LEN + 6u); ++b) {
                    fb.rxq[fb.rx_head] = frame[b];
                    fb.rx_head = (uint16_t)((fb.rx_head + 1u) % FB_RXLEN);
                }
            }
            /* 回复不是立刻可读的：数据已写进 rxq，但 avail() 在
             * rx_ready_us 之前返回 0 —— 模拟"12 帧共 1.8 ms 陆续到达"。 */
            fb.rx_ready_us  = fb.now_us + fb.latency_us;
        } else if (instr == STS_INST_PING) {
            /* ⭐ PING 应答：FF FF ID 02 ERR CHK（6 字节，LEN=2 → data_len=0）
             * ⚠️ 支持"碎片化投递"，用来验证 txn_expect_ack 的累积逻辑。 */
            fb.n_other++;
            const uint8_t pid = data[2];
            if (fb.no_reply || (fb.ping_reply_id != pid)) {
                return;                     /* 不响应这个 ID */
            }
            uint8_t pf[6];
            pf[0] = STS_HEAD; pf[1] = STS_HEAD; pf[2] = pid;
            pf[3] = 2u;   pf[4] = 0u;           /* LEN=2, ERR=0 */
            pf[5] = (uint8_t)(~(pid + 2u + 0u) & 0xFFu);

            uint16_t n_put = 6u;
            if (fb.ping_frag1 > 0u && fb.ping_frag1 < 6u) {
                n_put = fb.ping_frag1;      /* 只放第一段 */
                fb.ping_pending = (uint16_t)(6u - n_put);
            } else {
                fb.ping_pending = 0u;
            }
            for (uint16_t b = 0u; b < n_put; ++b) {
                fb.rxq[fb.rx_head] = pf[b];
                fb.rx_head = (uint16_t)((fb.rx_head + 1u) % FB_RXLEN);
            }
            fb.ping_first_us = fb.now_us;
            fb.rx_ready_us   = fb.now_us;   /* PING 应答很快，不额外延迟 */
        } else {
            fb.n_other++;
        }
    }
}

static uint16_t fb_port_avail(void)
{
    /* ⭐ 若 PING 应答被碎片化：过一小段时间后放出第二段。
     * ⚠️ 这正是真实总线上的行为 —— DMA 搬运与 IDLE 判帧分离，
     *    应答可能被切成两次进入环形缓冲。 */
    if (fb.ping_pending > 0u) {
        /* 用 30 µs 模拟"第二段稍后到达" */
        fb.ping_release_us = fb.ping_first_us + 30u;
        if (fb.now_us >= fb.ping_release_us) {
            const uint8_t pid = fb.ping_reply_id;
            uint8_t pf[6];
            pf[0] = STS_HEAD; pf[1] = STS_HEAD; pf[2] = pid;
            pf[3] = 2u;   pf[4] = 0u;
            pf[5] = (uint8_t)(~(pid + 2u + 0u) & 0xFFu);
            const uint16_t start = (uint16_t)(6u - fb.ping_pending);
            for (uint16_t b = start; b < 6u; ++b) {
                fb.rxq[fb.rx_head] = pf[b];
                fb.rx_head = (uint16_t)((fb.rx_head + 1u) % FB_RXLEN);
            }
            fb.ping_pending = 0u;
        }
    }

    /* 延迟未到 → 数据还"在路上" */
    const uint16_t total = (uint16_t)(fb.rx_head - fb.rx_tail);
    if (fb.now_us < fb.rx_ready_us) {
        return 0u;
    }
    (void)total;
    return total;
}

static uint16_t fb_port_read(uint8_t *dst, uint16_t n)
{
    uint16_t got = 0u;
    while ((got < n) && (fb.rx_tail != fb.rx_head)) {
        dst[got++] = fb.rxq[fb.rx_tail];
        fb.rx_tail = (uint16_t)((fb.rx_tail + 1u) % FB_RXLEN);
    }
    return got;
}

static void fb_port_flush(void)
{
    fb.rx_tail = fb.rx_head;
}

static uint32_t fb_port_micros(void)
{
    /* ⭐ 可选：每次读时间时自动推进 1 µs。
     *
     * ⚠️ 为什么需要它（实测踩到单测挂死）：
     *     本夹具的时间原先是静态的（由测试手动设置 now_us）。
     *     而 servo_bus 里有两处"等时间前进"的忙等：
     *        · txn_expect_ack() 开头的收发切换等待
     *        · txn_expect_ack() 的应答超时窗口
     *     时间不前进时它们会【永久自旋】—— 单测直接挂死，
     *     而那些"期望失败"的用例（对不应答的 ID 发 PING）必然触发。
     *
     *  ⚠️ 用开关控制：需要手动精确控制时间的用例（超时边界测试）
     *     保持关闭，行为与以前完全一致。 */
    if (fb.auto_tick) {
        fb.now_us += 1u;
    }
    return fb.now_us;
}

static const rlca_servo_port_t fb_port = {
    fb_port_write,
    fb_port_avail,
    fb_port_read,
    fb_port_flush,
    fb_port_micros
};

/* 需要给 fake_bus_t 补上这里用到的字段（放在结构体之后会编译不过，
 * 所以在结构体里已经声明；这里只是注释提醒） */
#define FB_EXTRA_FIELDS_DONE 1

/** 清空上一轮的回复（保留配置），用于"恢复后重试"的用例 */
static void fb_reset_reply_only(void)
{
    fb.rx_tail = fb.rx_head;
    fb.rx_ready_us = 0u;
}

/* ---- 驱动辅助 ---- */

/** 推进状态机直到空闲或超过 max_us。每次 poll 前把时间前进 step_us。
 *  @return 实际推进到的时间 */
static uint32_t fb_run(uint32_t max_us, uint32_t step_us)
{
    const uint32_t start = fb.now_us;
    uint32_t guard = 0u;

    while (!servo_bus_is_idle() && ((fb.now_us - start) < max_us)) {
        servo_bus_poll();
        fb.now_us += step_us;
        if (++guard > 100000u) {
            printf("  !! fb_run guard tripped (state=%d)\n", (int)servo_bus_state());
            break;
        }
    }
    return fb.now_us - start;
}

/*==============================================================================
 * 测试用例
 *============================================================================*/

static void test_syncwrite_frame(void)
{
    section("servo_bus: SyncWrite frame goes out correctly");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle must be accepted");
    fb_run(20000u, 50u);

    CHECK_EQ_U(fb.n_syncwrite, 1u);
    CHECK_EQ_U(fb.n_syncread, 1u);
    CHECK_EQ_U(fb.tx_len == 0u ? 0u : 1u, 1u);   /* 最后一次是读请求 */

    /* q=0 → 全部应在 offset=0 处，即 0 步 */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK(fb.goal_valid[i], "servo id %u was not written", (unsigned)RLCA_SERVO_ID(i));
        CHECK_EQ_D(fb.goal_step[i], 0);
    }

    /* 状态机应回到空闲 */
    CHECK(servo_bus_is_idle(), "state machine must return to idle");
}

static void test_happy_path_12_frames(void)
{
    section("servo_bus: 12 feedback frames parsed (acceptance #5)");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    const uint32_t elapsed = fb_run(20000u, 50u);

    const servo_bus_stats_t *st = servo_bus_stats();
    CHECK_EQ_U(st->n_ok_reads, 1u);
    CHECK_EQ_U(st->n_failed_reads, 0u);
    CHECK_EQ_U(st->n_timeout, 0u);
    CHECK_EQ_U(st->n_crc_err, 0u);
    CHECK_EQ_U(st->rx_frames, 12u);
    CHECK(servo_bus_take_new_feedback(), "new feedback flag must be set");

    const servo_fb_t *f = servo_bus_feedback();
    bool all_valid = true;
    for (uint8_t i = 0u; i < 12u; ++i) {
        if (!f[i].valid) { all_valid = false; }
    }
    CHECK(all_valid, "all 12 servos must report valid data");

    /* 逐项核对物理量 */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK_EQ_D(f[i].pos_step, fb.sim_pos[i]);
        CHECK_EQ_U(f[i].voltage_dv, 120u);
        CHECK_EQ_U(f[i].temp_c, 35u);
    }

    /* 事务耗时应 < 3 ms（验收 #12） */
    printf("    txn elapsed (simulated) = %u us\n", (unsigned)elapsed);
    CHECK(elapsed < 3000u, "transaction must complete well under 3 ms (got %u)", (unsigned)elapsed);
    printf("    stats: last_txn=%u us max_txn=%u us\n",
           (unsigned)st->last_txn_us, (unsigned)st->max_txn_us);
}

/*==============================================================================
 * ⭐⭐ 回归：关节状态的 q_rad 必须真的跟着【新解析的位置】走
 *
 *  背景（2026-10-16 现场 bug）：
 *     parse_feedback_frames() 里把新位置解析进 fb->pos_step 之后，
 *     调用 rlca_joint_state_update() 时却传了 s_js[idx].pos_step（旧值），
 *     而且从没把新值写进 s_js[].pos_step。
 *     ⇒ s_js[].pos_step 永远停在 servo_bus_init 的 memset 结果 0，
 *        q_rad 永远等于 (0 − offset)×dir×2π/4096。
 *
 *     后果：上电闸门期读到的"当前位置"全是 0 → 播种出 q=0 →
 *           1 s 后首条同步写把 12 个舵机命令到 step 0。
 *
 *  ⚠️ 为什么原有测试没抓到：test_servo_bus 只核对了 f[i].pos_step
 *     （那是 fb 里的值，本来就是对的），从没核对 q_rad。
 *     本用例补上这一环：从【位置】一路验到【关节角】。
 *============================================================================*/

static void test_joint_state_follows_parsed_position(void)
{
    section("servo_bus: parsed position must land in the joint state");

    /* ⚠️ 本用例锁住 2026-10-16 的现场 bug：
     *    parse_feedback_frames() 把新位置解析进 fb->pos_step 之后，
     *    调用 rlca_joint_state_update() 时传的是 s_js[idx].pos_step（旧值），
     *    而且【从没把新值写进 s_js[].pos_step】。
     *    ⇒ s_js[].pos_step 永远停在 servo_bus_init 的 memset 结果 0。
     *    ⇒ 闸门期读到的"当前位置"全是 0 → 播种出 q=0 →
     *      1 s 后首条同步写把 12 个舵机命令到 step 0
     *      （现场现象：上电 1 秒后开始动，每次同方向）。
     *
     *  ⚠️ 走【只读】事务，不要用 start_cycle ——
     *     后者会先写目标位置，而假总线忠实模拟"舵机奔向目标"，
     *     位置立刻变成被命令的值，就测不出解析有没有落进状态了。 */

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    CHECK(servo_bus_start_read_only(), "read_only must start");
    fb_run(20000u, 50u);

    const servo_fb_t *f = servo_bus_feedback();
    const rlca_joint_state_t *js = servo_bus_joint_states();
    CHECK(js != NULL, "joint states must be available");

    /* fb_reset 给的位置是 2048 + 10i：非 0 且互不相同 */
    uint8_t ok = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* ① ⭐ 核心断言：关节状态里的位置必须等于刚解析出的反馈位置。
         *    旧 bug 下 js[].pos_step 恒为 0，而反馈是 2048+10i ⇒ 必被抓住。 */
        if (js[i].pos_step == f[i].pos_step) {
            ok++;
        } else {
            printf("    joint %u: js.pos_step=%d but parsed feedback=%d\n",
                   (unsigned)i, (int)js[i].pos_step, (int)f[i].pos_step);
        }

        /* ② 位置不能是 0 —— 旧 bug 的典型特征 */
        CHECK(js[i].pos_step != 0,
              "joint %u pos_step == 0 -- parsed position was dropped!",
              (unsigned)i);

        /* ③ valid 必须置位 */
        CHECK(js[i].valid != 0u,
              "joint %u must be marked valid", (unsigned)i);
    }
    CHECK_EQ_U(ok, RLCA_N_JOINTS);

    /* ④ 12 个反馈位置互不相同 ⇒ 12 个 js.pos_step 也必须互不相同。
     *    若全部相同，说明它们没跟着反馈走。 */
    uint8_t distinct = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        bool unique = true;
        for (uint8_t k = 0u; k < RLCA_N_JOINTS; ++k) {
            if ((k != i) && (js[i].pos_step == js[k].pos_step)) {
                unique = false;
            }
        }
        if (unique) { distinct++; }
    }
    CHECK_EQ_U(distinct, RLCA_N_JOINTS);
}

/*==============================================================================
 * ⭐⭐ 回归：标定表热更新必须立刻影响角度换算（2026-10-17 修的真 bug）
 *
 *  现场症状（极难排查，务必钉住）：
 *     标定完成后敲 R：
 *        · `-- calibration --` 里的 offset 是【新的、正确的】
 *        · 但关节角报 ±3.14 rad（≈π）
 *        · 而 STAT_CALIBRATED 亮着 ✅
 *     ⇒ 自相矛盾：offset 对、角度错、状态位还说已标定。
 *     ⇒ 而且**必须重启 32 才生效**。
 *
 *  根因：`servo_bus_init()` 里是 `s_calib = *calib;`（复制一份）。
 *        标定完成 / 手动零点 / 恢复默认 都只更新了 app_main 自己的表，
 *        从没同步给 servo_bus ⇒ 角度换算一直用【上电时的旧表】。
 *        上电时旧表来自 flash（若当时没标定过就是全零默认表）。
 *
 *  ⇒ 修法：`servo_bus_set_calib()`，标定流程结束时调用。
 *     本用例锁住"调它之后角度必须立刻变"。
 *============================================================================*/

static void test_set_calib_takes_effect_immediately(void)
{
    section("servo_bus: set_calib() must change q_rad immediately (no reboot)");

    fb_reset();
    rlca_calib_t def;
    rlca_calib_load_defaults(&def);          /* offset[] 全 0 */
    servo_bus_init(&fb_port, &def);

    /* 走只读事务：位置由 fb_reset 造（2048 + 10i），不会被目标位置改写 */
    CHECK(servo_bus_start_read_only(), "read_only must start");
    fb_run(20000u, 50u);

    const rlca_joint_state_t *js = servo_bus_joint_states();
    const servo_fb_t *f = servo_bus_feedback();

    /* ① offset = 0 时：q = pos × dir × 2π/4096 ⇒ |q| 很大（≈π 量级） */
    CHECK(fabsf(js[0].q_rad) > 2.0f,
          "with a zero-offset table q must be huge, got %.4f",
          (double)js[0].q_rad);

    /* ② 构造一张"已标定"的表：offset 取反馈位置 ⇒ 该关节的 q 应该变成 ~0 */
    rlca_calib_t real = def;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        real.offset[i] = f[i].pos_step;
    }
    real.crc16 = rlca_calib_crc(&real);

    /* ⭐ 热更新（这正是标定完成时该做的事） */
    servo_bus_set_calib(&real);

    /* ③ 再跑一轮只读 ⇒ 角度换算必须用上新表 */
    CHECK(servo_bus_start_read_only(), "second read_only must start");
    fb_run(20000u, 50u);

    const rlca_joint_state_t *js2 = servo_bus_joint_states();
    uint8_t near_zero = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* offset 恰好等于当前位置 ⇒ q 必须是 0（容差 2 步 ≈ 0.003 rad） */
        if (fabsf(js2[i].q_rad) <= (2.0f * RLCA_SERVO_RAD_PER_STEP)) {
            near_zero++;
        } else {
            printf("    joint %u: q=%.4f, expected ~0 after set_calib()\n",
                   (unsigned)i, (double)js2[i].q_rad);
        }
    }
    CHECK_EQ_U(near_zero, RLCA_N_JOINTS);

    /* ④ set_calib 必须【不】破坏统计与反馈（它只换表） */
    CHECK(servo_bus_stats()->n_ok_reads >= 1u,
          "set_calib() must not reset the statistics");
    CHECK(servo_bus_feedback()[0].valid != 0u,
          "set_calib() must not invalidate the feedback");
}

static void test_1000_rounds_zero_error(void)
{    section("servo_bus: 1000 rounds with zero errors (acceptance #6)");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    for (uint32_t r = 0u; r < 1000u; ++r) {
        if (!servo_bus_start_cycle(q)) {
            /* 上一轮没完 —— 这是设计上不允许的，记失败并跳出 */
            CHECK(false, "round %u: start_cycle rejected", (unsigned)r);
            break;
        }
        /* 模拟 100 Hz 周期：每轮之间推进 10 ms */
        fb_run(20000u, 50u);
        (void)servo_bus_take_new_feedback();
        fb.now_us += 10000u;      /* 到下一个控制周期 */
    }

    const servo_bus_stats_t *st = servo_bus_stats();
    printf("    syncwrite=%u syncread=%u rx_frames=%u ok_reads=%u\n",
           (unsigned)fb.n_syncwrite, (unsigned)fb.n_syncread,
           (unsigned)st->rx_frames, (unsigned)st->n_ok_reads);
    printf("    timeouts=%u crc_err=%u short=%u failed=%u\n",
           (unsigned)st->n_timeout, (unsigned)st->n_crc_err,
           (unsigned)st->n_short, (unsigned)st->n_failed_reads);

    CHECK_EQ_U(st->n_ok_reads, 1000u);
    CHECK_EQ_U(st->n_failed_reads, 0u);
    CHECK_EQ_U(st->n_timeout, 0u);
    CHECK_EQ_U(st->n_crc_err, 0u);
    CHECK_EQ_U(st->n_short, 0u);
    CHECK_EQ_U(fb.rx_head == fb.rx_tail ? 0u : 1u, 0u);   /* 接收队列已清空 */
    CHECK_EQ_U(servo_bus_err_flags(), 0u);
}

static void test_one_dead_servo(void)
{
    section("servo_bus: one dead servo -> rest still OK (acceptance #7)");

    fb_reset();
    fb.dead_id = 3u;              /* 逻辑索引 2（rf_hip）不回帧 */
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(20000u, 50u);

    const servo_bus_stats_t *st = servo_bus_stats();
    CHECK_EQ_U(st->rx_frames, 11u);          /* 只回了 11 帧 */
    CHECK_EQ_U(st->n_failed_reads, 1u);      /* 本轮不算成功 */
    CHECK(servo_bus_is_idle(), "must still return to idle");

    const servo_fb_t *f = servo_bus_feedback();
    CHECK(!f[2].valid, "dead servo must be marked invalid");
    CHECK(f[0].valid && f[11].valid, "other servos must remain valid");

    /* ⚠️ 关键：下一轮必须能照常发起（不能因为一个舵机挂了就停总线） */
    fb.dead_id = 0u;
    CHECK(servo_bus_start_cycle(q), "next round must still be possible");
    fb_run(20000u, 50u);
    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);
}

static void test_all_dead_timeout(void)
{
    section("servo_bus: bus silent -> timeout within 3 ms (acceptance #8)");

    fb_reset();
    fb.no_reply = true;
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    const uint32_t elapsed = fb_run(100000u, 50u);

    const servo_bus_stats_t *st = servo_bus_stats();
    CHECK_EQ_U(st->n_timeout, 1u);
    CHECK_EQ_U(st->n_ok_reads, 0u);
    CHECK(servo_bus_is_idle(), "must return to idle after timeout");

    /* ⚠️ 整帧超时必须【真的】在 3 ms 量级返回，不能等满 max_us。
     *    这里放宽到 5 ms 是为了给状态机几次 poll 的余量。 */
    printf("    timeout detected after %u us\n", (unsigned)elapsed);
    CHECK(elapsed <= 5000u, "timeout must fire near 3 ms, took %u us", (unsigned)elapsed);

    /* ⭐⭐ 语义已改（2026-10-17）：计数照加，但【错误位】要连续 N 轮失败才置。
     *
     *  原因：实测总线约 1%/轮 会偶发整轮收不到回复。旧实现"一次就置位"
     *        ⇒ err_flags 0/0x40 跳 ⇒ safety_reasons 0/0x20 跳
     *        ⇒ amp_scale 100%/75% 跳，面板上"错误"和"幅度"不停闪。
     *
     *  ⇒ 单次失败：【不置位】（诊断靠 n_timeout 计数器）
     *     连续 RLCA_BUS_FAIL_STREAK_ERR 轮：【置位】 */
    CHECK_EQ_U(servo_bus_fail_streak(), 1u);
    CHECK((servo_bus_err_flags() & RLCA_ERRF_BUS_TIMEOUT) == 0u,
          "a SINGLE missed round must NOT raise the error flag "
          "(otherwise the safety layer flickers at 1%% bus jitter)");

    /* 再来 RLCA_BUS_FAIL_STREAK_ERR-1 轮，凑满阈值 ⇒ 必须置位 */
    for (uint16_t i = 1u; i < (uint16_t)RLCA_BUS_FAIL_STREAK_ERR; ++i) {
        CHECK(servo_bus_start_cycle(q), "extra failing round %u must start",
              (unsigned)i);
        fb_run(100000u, 50u);
    }
    CHECK_EQ_U(servo_bus_fail_streak(), RLCA_BUS_FAIL_STREAK_ERR);
    CHECK_EQ_U(servo_bus_stats()->n_timeout, RLCA_BUS_FAIL_STREAK_ERR);
    CHECK((servo_bus_err_flags() & RLCA_ERRF_BUS_TIMEOUT) != 0u,
          "consecutive failures must be flagged for telemetry");

    /* 总线恢复后应能继续工作，且连续失败计数清零 */
    fb.no_reply = false;
    fb_reset_reply_only();
    CHECK(servo_bus_start_cycle(q), "recovery round must start");
    fb_run(20000u, 50u);
    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);
    CHECK_EQ_U(servo_bus_fail_streak(), 0u);
}

static void test_bad_crc_recovery(void)
{
    section("servo_bus: corrupted feedback is detected, bus keeps working");

    fb_reset();
    fb.badcrc_id = 7u;            /* 逻辑索引 6（rf_thigh）校验和错 */
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(20000u, 50u);

    const servo_bus_stats_t *st = servo_bus_stats();
    printf("    crc_err=%u short=%u rx_frames=%u ok=%u failed=%u idle=%u\n",
           (unsigned)st->n_crc_err, (unsigned)st->n_short,
           (unsigned)st->rx_frames, (unsigned)st->n_ok_reads,
           (unsigned)st->n_failed_reads, (unsigned)st->n_idle_timeout);
    CHECK(st->n_crc_err > 0u, "corrupted frame must be counted");
    CHECK_EQ_U(st->rx_frames, 11u);          /* 其余 11 个舵机仍被正确解析 */
    CHECK(!servo_bus_feedback()[6].valid, "corrupted servo must not be valid");
    CHECK(servo_bus_feedback()[0].valid, "other servos must still be valid");
    CHECK(servo_bus_is_idle(), "must return to idle");

    /* 恢复正常后立刻能读全 */
    fb.badcrc_id = 0u;
    CHECK(servo_bus_start_cycle(q), "next round");
    fb_run(20000u, 50u);
    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);
}

static void test_shuffled_and_noisy(void)
{
    section("servo_bus: shuffled order + leading noise bytes");

    fb_reset();
    fb.shuffle = true;
    fb.noise_prefix = true;
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(20000u, 50u);

    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);
    CHECK_EQ_U(servo_bus_stats()->rx_frames, 12u);

    /* 12 个舵机的反馈必须都归到正确的关节上（按 ID 匹配，不靠顺序） */
    const servo_fb_t *f = servo_bus_feedback();
    bool ok = true;
    for (uint8_t i = 0u; i < 12u; ++i) {
        if (!f[i].valid) { ok = false; }
        if (f[i].pos_step != fb.sim_pos[i]) { ok = false; }
    }
    CHECK(ok, "feedback must be matched by ID, not by arrival order");
}

static void test_partial_reply(void)
{
    section("servo_bus: partial reply -> parses what arrived, round is failed");

    fb_reset();
    fb.partial_frac = 5u;         /* 只回 5 帧 */
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(40000u, 50u);

    const servo_bus_stats_t *st = servo_bus_stats();
    printf("    partial: rx_frames=%u ok=%u failed=%u timeout=%u idle=%u\n",
           (unsigned)st->rx_frames, (unsigned)st->n_ok_reads,
           (unsigned)st->n_failed_reads, (unsigned)st->n_timeout,
           (unsigned)st->n_idle_timeout);

    /* ⚠️ 关键：5 帧也要被【解析出来】，不能白等 3 ms 超时全丢。
     *    这正是"用短暂静默判回复结束"的价值。 */
    CHECK_EQ_U(st->rx_frames, 5u);
    CHECK_EQ_U(st->n_ok_reads, 0u);          /* 没读全 12 个 → 整轮不算成功 */
    CHECK(st->n_failed_reads >= 1u, "partial reply must be a failed round");
    CHECK(servo_bus_is_idle(), "must return to idle");

    /* 且耗时应远小于 3 ms（静默判定提前结束） */
    printf("    partial round took %u us\n", (unsigned)st->last_txn_us);
    CHECK(st->last_txn_us < RLCA_SERVO_FRAME_TIMEOUT_US,
          "idle-gap detection must end the round before the 3 ms hard timeout");
}

static void test_servo_error_flags(void)
{
    section("servo_bus: servo ERR bits are aggregated (acceptance #9)");

    fb_reset();
    fb.sim_err[0] = STS_ERR_OVERHEAT;
    fb.sim_err[5] = STS_ERR_OVERCURRENT;
    fb.sim_temp_c  = 78u;
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(20000u, 50u);

    const uint8_t ef = servo_bus_err_flags();
    CHECK((ef & RLCA_ERRF_OVERHEAT) != 0u, "overheat must be flagged");
    CHECK((ef & RLCA_ERRF_OVERCURRENT) != 0u, "overcurrent must be flagged");
    CHECK_EQ_U(servo_bus_feedback()[0].err, STS_ERR_OVERHEAT);
    CHECK_EQ_U(servo_bus_feedback()[0].temp_c, 78u);

    /* ⚠️ ERR 非零【不能】导致整轮读失败 —— 错误码本身是有效数据 */
    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);
}

static void test_limit_clamping(void)
{
    section("servo_bus: out-of-range and NaN targets are clamped");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }
    q[RLCA_J_LF_HIP]   = 5.0f;                 /* 远超 +40° */
    q[RLCA_J_LF_THIGH] = -5.0f;                /* 远超 -35° */
    q[RLCA_J_LF_CALF]  = 0.0f / 0.0f;          /* NaN */

    CHECK(servo_bus_start_cycle(q), "start_cycle");
    fb_run(20000u, 50u);

    /* ⚠️ fb.goal_step 是从【报文字节】解出来的，所以它反映"线路上实际发出去的值"。
     *    2026-10-12 起 rlca_sts 会把目标位置按 4096 归一化到 [0, 4095]
     *    （Goal_Position 是 12 位单圈编码器目标，合法域 0..4095）。
     *    所以这里按【模 4096 等价】比较：
     *        hip   +40° = +455  -> 455
     *        thigh −35° = −398  -> 3698  (−398 + 4096)
     *        calf  −60° = −683  -> 3413  (−683 + 4096, NaN 落到下限) */
    /* ⚠️ calf 的符号由 dir 决定，而 dir 是会变的设计量（2026-10-16 由 +1 翻成 −1）。
     *    所以这里从 RLCA_JOINT_DIR_INIT 推导，不硬编码 —— 硬编码时 dir 一变就静默错。 */
    static const int8_t dtab[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
    const int16_t exp_hip   = 455;                                   /* dir=+1 */
    const int16_t exp_thigh = (int16_t)(-398 + 4096);                /* dir=+1 */
    int16_t exp_calf = 0;
    /* ⚠️ 必须自己环绕到 [0,4095]：dir=−1 时结果是 +683 已经在范围内，
     *    但 dir=+1 时是 −683，加一次 4096 不够表达"模 4096 等价"。 */
    {
        int32_t cs = (int32_t)dtab[RLCA_J_LF_CALF] * (-683);
        cs %= 4096;
        if (cs < 0) { cs += 4096; }
        exp_calf = (int16_t)cs;
    }

    CHECK_EQ_D(fb.goal_step[RLCA_J_LF_HIP], exp_hip);
    CHECK_EQ_D(fb.goal_step[RLCA_J_LF_THIGH], exp_thigh);
    CHECK_EQ_D(fb.goal_step[RLCA_J_LF_CALF], exp_calf);

    /* 并且三者都必须是合法的编码器位置 */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK((fb.goal_step[i] >= 0) && (fb.goal_step[i] <= 4095),
              "joint %u goal_step %d out of encoder range",
              (unsigned)i, (int)fb.goal_step[i]);
    }
}

static void test_single_servo_commands(void)
{
    section("servo_bus: ping / torque / single-servo write");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    /* 力矩卸载：应发出 INST_WRITE */
    CHECK(servo_bus_set_torque(0u, false), "set_torque must succeed");
    CHECK_EQ_U(fb.n_other, 1u);
    CHECK(fb.tx[4] == STS_INST_WRITE, "torque must use INST_WRITE");
    CHECK_EQ_U(fb.tx[5], STS_REG_TORQUE_ENABLE);
    CHECK_EQ_U(fb.tx[6], 0u);

    /* 全部卸力：12 条指令 */
    servo_bus_set_all_torque(false);
    CHECK_EQ_U(fb.n_other, 13u);

    /* 单关节写位：用 SyncWrite 发 1 个舵机 */
    CHECK(servo_bus_write_one(RLCA_J_LF_CALF, 100), "write_one must succeed");
    CHECK_EQ_U(fb.n_syncwrite, 1u);
    CHECK_EQ_D(fb.goal_step[RLCA_J_LF_CALF], 100);
    CHECK(fb.goal_valid[RLCA_J_LF_CALF], "goal must be recorded");
}

static void test_read_only_and_abort(void)
{
    section("servo_bus: read-only round and abort");

    fb_reset();
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    /* 只读：不应发 SyncWrite */
    CHECK(servo_bus_start_read_only(), "start_read_only");
    fb_run(20000u, 50u);
    CHECK_EQ_U(fb.n_syncwrite, 0u);
    CHECK_EQ_U(fb.n_syncread, 1u);
    CHECK_EQ_U(servo_bus_stats()->n_ok_reads, 1u);

    /* abort 后应立刻空闲，且能再发起 */
    servo_bus_abort();
    CHECK(servo_bus_is_idle(), "abort must force idle");
    CHECK(servo_bus_start_read_only(), "must be able to start after abort");
    fb_run(20000u, 50u);
}

static void test_start_rejected_while_busy(void)
{
    section("servo_bus: start rejected while a transaction is in flight");

    fb_reset();
    fb.no_reply = true;              /* 让第一轮停在 READ_RSP */
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    servo_bus_init(&fb_port, &cal);

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }

    CHECK(servo_bus_start_cycle(q), "first start");
    servo_bus_poll();
    fb.now_us += 100u;
    servo_bus_poll();                /* 进到 READ_RSP 附近 */

    const servo_bus_state_t st = servo_bus_state();
    if (!servo_bus_is_idle()) {
        CHECK(!servo_bus_start_cycle(q), "start must be rejected while busy (state=%d)", (int)st);
    } else {
        /* 已经超时结束也算合理，但至少不该崩 */
        CHECK(true, "bus already finished");
    }
}

/*==============================================================================
 * ⭐⭐ 总线互斥（pause / resume）回归测试
 *
 * ⚠️⚠️ 这一段钉住一个实测抓到的竞态（2026-10-11）：
 *     `bsp_shell_poll()` 与 `servo_bus_poll()` 跑在同一个主循环里，
 *     所以"后台同步读"与"shell 的单机指令"会交错：
 *        ① servo_bus_poll() 发出 SyncRead(1..12)
 *        ② 同一次主循环里 bsp_shell_poll() 执行 `A` → 又发出 PING
 *        ③ 总线上两家同时等回复，RX 里混着两种应答 → 双方都判失败
 *
 *     现象很反直觉：`Y 5` 能收到应答（它自己反复 poll、等 50 ms），
 *     而 `A` 全 no response（用 3 ms 状态机窗口，必然混入垃圾）。
 *========================================================================*/

static void test_pause_blocks_new_transactions(void)
{
    section("bus mutex: pause must stop the state machine from starting");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    fb_reset();
    servo_bus_init(&fb_port, &cal);

    /* 让总线进入"正在等回复"的中间状态 */
    float q[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) { q[i] = 0.0f; }
    CHECK(servo_bus_start_cycle(q), "start_cycle must be accepted when idle");

    /* 推进到 READ_REQ（发完 SyncRead、正在等回复） */
    for (int i = 0; i < 40; ++i) {
        fb.now_us += 100u;
        servo_bus_poll();
    }

    const uint32_t tx_before = servo_bus_stats()->tx_frames;
    const uint32_t rounds_before = 0u;
    (void)rounds_before;

    /* ---- 暂停后：推进状态机【不应】再发出任何新帧 ---- */
    servo_bus_pause();
    for (int i = 0; i < 200; ++i) {
        fb.now_us += 1000u;
        servo_bus_poll();
    }
    CHECK(servo_bus_stats()->tx_frames == tx_before,
          "paused: tx_frames must not grow (was %u, now %u)",
          (unsigned)tx_before, (unsigned)servo_bus_stats()->tx_frames);

    /* ---- 恢复后：状态机应能重新发起事务 ---- */
    servo_bus_resume();
    const uint32_t tx_after_pause = servo_bus_stats()->tx_frames;

    /* 恢复后需要上层重新 start（因为 pause 已把状态清回 IDLE） */
    CHECK(servo_bus_is_idle(), "after resume the bus must be idle");
    CHECK(servo_bus_start_cycle(q), "resume must allow a new cycle");
    for (int i = 0; i < 40; ++i) {
        fb.now_us += 100u;
        servo_bus_poll();
    }
    CHECK(servo_bus_stats()->tx_frames > tx_after_pause,
          "after resume tx_frames must grow (%u -> %u)",
          (unsigned)tx_after_pause, (unsigned)servo_bus_stats()->tx_frames);
}

static void test_pause_clears_inflight_state(void)
{
    section("bus mutex: pause must clear in-flight state (abort semantics)");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    fb_reset();
    servo_bus_init(&fb_port, &cal);

    float q[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) { q[i] = 0.0f; }
    (void)servo_bus_start_cycle(q);

    /* 制造"正在读、且缓冲里有半帧残留"的状态 */
    fb.now_us += 500u;
    servo_bus_poll();

    servo_bus_pause();
    CHECK(servo_bus_is_idle(),
          "pause must return the bus to IDLE (state=%d)",
          (int)servo_bus_state());

    /* 暂停期间 is_idle 必须为真，这样上层查询不会撞车 */
    CHECK(servo_bus_is_idle(), "still idle while paused");

    servo_bus_resume();
    CHECK(servo_bus_is_idle(), "idle after resume");
}

/*==============================================================================
 * ⭐⭐ PING 应答的累积解析（碎片化）
 *
 * ⚠️⚠️ 这一段钉住实测抓到的真 bug（2026-10-11）：
 *     `Y` 命令能收到完整的 6 字节 PING 应答（FF FF 05 02 00 F8），
 *     而 `servo_bus_ping()` 在同样的总线上、同样的 3 ms 窗口内总判失败。
 *
 *  原因：DMA 搬运与 IDLE 判帧是分开的，应答可能被切成两段先后进入
 *        环形缓冲。原实现"avail() 够 4 字节就 read() 一次、解析失败就
 *        return"，读到半帧就放弃 —— 而 `Y` 自己用 50 ms 窗口反复累积，
 *        所以 `Y` 的"成功"掩盖了本函数的缺陷。
 *
 *  修法：累积字节 + 重同步到帧头，直到拼出完整帧或超时。
 *========================================================================*/

static void test_ping_reply_intact(void)
{
    section("ping: intact reply must be accepted");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    fb_reset();
    fb.ping_reply_id = 5u;      /* ID 5 会应答 PING */
    fb.ping_frag1    = 0u;      /* 一次给完 */
    /* ⚠️ auto_tick 必须在 fb_reset() 【之后】设 —— 它会 memset 整个结构体。
     *    这是我实际踩到的坑：设在前面 → 时钟不前进 →
     *    txn_expect_ack 的忙等永久自旋 → 单测挂死、且没有任何报错。 */
    fb.auto_tick     = true;
    servo_bus_init(&fb_port, &cal);

    CHECK(servo_bus_ping(5u), "intact PING reply must succeed");
    CHECK(!servo_bus_ping(6u), "id without reply must fail");
}

static void test_ping_reply_fragmented(void)
{
    section("ping: reply split into two chunks must still be accepted");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);

    /* ⚠️ 先到 2 字节、30 µs 后再到 4 字节 —— 修复前必然失败 */
    fb_reset();
    fb.ping_reply_id = 5u;
    fb.ping_frag1    = 2u;
    fb.auto_tick     = true;
    servo_bus_init(&fb_port, &cal);
    CHECK(servo_bus_ping(5u),
          "fragmented PING reply (2+4 bytes) must still be accepted");

    /* 换一种切法：先到 4 字节、再到 2 字节 */
    fb_reset();
    fb.ping_reply_id = 5u;
    fb.ping_frag1    = 4u;
    fb.auto_tick     = true;
    servo_bus_init(&fb_port, &cal);
    CHECK(servo_bus_ping(5u),
          "fragmented PING reply (4+2 bytes) must still be accepted");
}

static void test_ping_reply_with_noise(void)
{
    section("ping: leading noise before the header must be resynced");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    fb_reset();
    fb.ping_reply_id = 5u;
    fb.ping_frag1    = 0u;
    fb.auto_tick     = true;
    servo_bus_init(&fb_port, &cal);

    /* 预置噪声字节（模拟上一次事务的残留） */
    for (uint8_t i = 0u; i < 3u; ++i) {
        fb.rxq[fb.rx_head] = 0x5Au;
        fb.rx_head = (uint16_t)((fb.rx_head + 1u) % FB_RXLEN);
    }
    CHECK(servo_bus_ping(5u),
          "reply after leading noise must be resynced and accepted");
}

/*==============================================================================
 * ⭐⭐ 部分舵机在位时，SyncRead 必须把【每个应答的】都标记为 valid
 *
 * ⚠️⚠️ 钉住的实测现象（2026-10-11，修好 DMA 搬运之后）：
 *     `A`（PING）报告 id 5 与 id 9 都 ACK，
 *     而 `B`（SyncRead）里只有 id 5 有 pos/V/T，id 9 全是 0。
 *     也就是说：两个舵机都在应答，但同步读只解析出其中一个。
 *
 *  这段测试区分两种可能：
 *     · 解析器对"只有部分舵机在位"的场景有 bug
 *     · 或者问题在真实总线的时序上（测试跑不出来）
 *========================================================================*/

static void test_partial_population_marks_all_valid(void)
{
    section("sync read: two servos present -> BOTH must become valid");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    fb_reset();

    /* 只让 ID 5 与 ID 9 在位；其余 10 个不回帧。
     * ⚠️ 模拟器按"dead_id 只能有一个"设计，这里用 partial_frac 不够用，
     *    改成让模拟器只回这两个 ID —— 见下面的 fb.only_ids。 */
    fb.only_ids[0] = 5u;
    fb.only_ids[1] = 9u;
    fb.n_only      = 2u;

    servo_bus_init(&fb_port, &cal);

    float q[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) { q[i] = 0.0f; }
    CHECK(servo_bus_start_cycle(q), "start_cycle accepted");

    for (int i = 0; i < 60; ++i) {
        fb.now_us += 100u;
        servo_bus_poll();
    }

    const servo_fb_t *f = servo_bus_feedback();
    /* 关节索引 -> ID 映射是 i+1，所以 5 -> idx 4，9 -> idx 8 */
    CHECK(f[4].valid == 1u, "id 5 (idx 4) must be valid");
    CHECK(f[8].valid == 1u, "id 9 (idx 8) must be valid");

    /* 其余必须无效（不能因为"上电初值"而误报有效） */
    CHECK(f[0].valid == 0u, "id 1 (idx 0) must NOT be valid");
    CHECK(f[11].valid == 0u, "id 12 (idx 11) must NOT be valid");
}

/*==============================================================================
 * main
 *============================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT servo_bus unit tests (fake bus)\n");
    printf("========================================\n");
    fflush(stdout);

    test_syncwrite_frame();
    test_happy_path_12_frames();
    test_limit_clamping();
    test_servo_error_flags();
    test_shuffled_and_noisy();
    test_one_dead_servo();
    test_bad_crc_recovery();
    test_partial_reply();
    test_all_dead_timeout();
    test_single_servo_commands();
    test_read_only_and_abort();
    test_start_rejected_while_busy();
    test_joint_state_follows_parsed_position();
    test_set_calib_takes_effect_immediately();
    test_1000_rounds_zero_error();
    test_pause_blocks_new_transactions();
    test_pause_clears_inflight_state();
    test_ping_reply_intact();
    test_ping_reply_fragmented();
    test_ping_reply_with_noise();
    test_partial_population_marks_all_valid();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
