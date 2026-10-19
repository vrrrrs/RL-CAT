/**
  ******************************************************************************
  * @file    test_link_proto.c
  * @brief   树莓派链路纯逻辑层单元测试（CMD_JOINT / TEL_FAST / TEL_STATUS）。
  *
  *  覆盖：
  *    · CMD_JOINT 字节级解码（含量程换算、模式白名单、kp_scale）
  *    · TEL_FAST 96 字节布局【逐偏移】核对（这是最容易错的一环）
  *    · TEL_STATUS 40 字节布局逐偏移核对
  *    · 指令合法化：NaN / Inf / 超限 / 限速（协议 §8.3）
  *    · 用 Python 参考实现（协议 §9）的同一组字节做交叉验证
  *
  *  ⚠️ 布局断言全部写成"偏移 + 期望值"，而不是"结构体字段" ——
  *     协议是紧凑字节流，必须先证明字节布局对，才谈得上功能对。
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
#define CHECK_EQ_D(a, b)                                                  \
    CHECK((a) == (b), "(got %d, want %d)", (int)(a), (int)(b))
#define CHECK_NEAR(a, b, tol)                                             \
    CHECK(fabs((double)(a) - (double)(b)) <= (tol),                       \
          "(got %.6f, want %.6f +- %g)", (double)(a), (double)(b), (double)(tol))

static void section(const char *name)
{
    printf("\n[%s]\n", name);
    fflush(stdout);
}

/*==============================================================================
 * 1. CMD_JOINT 解码
 *============================================================================*/

/** 按协议 §4.1 手工构造一个 CMD_JOINT 负载 */
static void build_cmd_joint(uint8_t *p, uint8_t mode, uint8_t flags,
                            const float *q_rad, int16_t kp_scale, uint8_t led)
{
    memset(p, 0, RLCA_LEN_CMD_JOINT);
    p[0] = mode;
    p[1] = flags;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* 编码：enc = q / 0.0001（四舍五入） */
        const int16_t enc = (int16_t)lroundf(q_rad[i] / RLCA_Q_UNIT_RAD);
        rlca_put_i16(&p[2u + (2u * i)], enc);
    }
    p[26] = 0u;                                    /* reserved */
    rlca_put_i16(&p[27], kp_scale);
    p[29] = led;
    p[30] = 0u;                                    /* reserved2 */
}

static void test_cmd_joint_decode(void)
{
    section("CMD_JOINT decode (protocol 4.1)");

    float q[12];
    for (uint8_t i = 0u; i < 12u; ++i) { q[i] = 0.0f; }
    q[0]  =  0.3000f;      /* 应编码成 3000 */
    q[4]  = -0.2500f;      /* -2500 */
    q[8]  =  0.5236f;      /* 5236 */
    q[11] = -0.0001f;      /* -1 */

    uint8_t pay[RLCA_LEN_CMD_JOINT];
    build_cmd_joint(pay, (uint8_t)RLCA_MODE_ACTIVE, 0x01u, q, 100, 1u);

    /* 先核对字节布局本身 */
    CHECK_EQ_U(pay[0], RLCA_MODE_ACTIVE);
    CHECK_EQ_U(pay[1], 0x01u);
    CHECK_EQ_D(rlca_get_i16(&pay[2]), 3000);
    CHECK_EQ_D(rlca_get_i16(&pay[2u + 8u]), -2500);   /* 索引 4 */
    CHECK_EQ_D(rlca_get_i16(&pay[2u + 16u]), 5236);   /* 索引 8 */
    CHECK_EQ_D(rlca_get_i16(&pay[2u + 22u]), -1);     /* 索引 11 */
    CHECK_EQ_D(rlca_get_i16(&pay[27]), 100);

    pi_cmd_t c;
    CHECK_EQ_U(link_decode_cmd_joint(pay, RLCA_LEN_CMD_JOINT, &c), LNK_DEC_OK);
    CHECK_EQ_U(c.mode, RLCA_MODE_ACTIVE);
    CHECK_EQ_U(c.flags, 0x01u);
    CHECK_NEAR(c.q_target[0],  0.3000f, 1e-6);
    CHECK_NEAR(c.q_target[4], -0.2500f, 1e-6);
    CHECK_NEAR(c.q_target[8],  0.5236f, 1e-6);
    CHECK_NEAR(c.q_target[11], -0.0001f, 1e-6);
    CHECK_EQ_D(c.kp_scale, 100);
    CHECK_EQ_U(c.led, 1u);

    /* ---- 长度不对必须被拒 ---- */
    CHECK_EQ_U(link_decode_cmd_joint(pay, 30u, &c), LNK_DEC_ERR_LEN);
    CHECK_EQ_U(link_decode_cmd_joint(pay, 32u, &c), LNK_DEC_ERR_LEN);
    CHECK_EQ_U(link_decode_cmd_joint(NULL, RLCA_LEN_CMD_JOINT, &c), LNK_DEC_ERR_PARAM);

    /* ---- 模式白名单：非法模式必须回落到 IDLE（协议 §8.3①） ---- */
    build_cmd_joint(pay, 99u, 0u, q, 0, 0u);
    CHECK_EQ_U(link_decode_cmd_joint(pay, RLCA_LEN_CMD_JOINT, &c), LNK_DEC_OK);
    CHECK_EQ_U(c.mode, RLCA_MODE_IDLE);
    build_cmd_joint(pay, 4u, 0u, q, 0, 0u);       /* 4 = ESTOP，合法 */
    (void)link_decode_cmd_joint(pay, RLCA_LEN_CMD_JOINT, &c);
    CHECK_EQ_U(c.mode, RLCA_MODE_ESTOP);

    /* ---- 量程：int16 满量程 = ±3.2767 rad ---- */
    build_cmd_joint(pay, 0u, 0u, q, 0, 0u);
    rlca_put_i16(&pay[2], 32767);
    rlca_put_i16(&pay[4], -32768);
    (void)link_decode_cmd_joint(pay, RLCA_LEN_CMD_JOINT, &c);
    CHECK_NEAR(c.q_target[0],  3.2767f, 1e-4);
    CHECK_NEAR(c.q_target[1], -3.2768f, 1e-4);
}

static void test_cmd_mode_and_calib(void)
{
    section("CMD_MODE / CMD_CALIB decode");

    const uint8_t m[2] = { (uint8_t)RLCA_MODE_DAMP, 7u };
    pi_mode_t mo;
    CHECK_EQ_U(link_decode_cmd_mode(m, 2u, &mo), LNK_DEC_OK);
    CHECK_EQ_U(mo.mode, RLCA_MODE_DAMP);
    CHECK_EQ_U(mo.param, 7u);
    CHECK_EQ_U(link_decode_cmd_mode(m, 3u, &mo), LNK_DEC_ERR_LEN);

    /* 非法模式回落 */
    const uint8_t bad[2] = { 200u, 0u };
    (void)link_decode_cmd_mode(bad, 2u, &mo);
    CHECK_EQ_U(mo.mode, RLCA_MODE_IDLE);

    pi_calib_t cb;
    for (uint8_t a = 0u; a <= 4u; ++a) {
        const uint8_t p[1] = { a };
        CHECK_EQ_U(link_decode_cmd_calib(p, 1u, &cb), LNK_DEC_OK);
        CHECK_EQ_U(cb.action, a);
    }
    const uint8_t bad2[1] = { 5u };
    CHECK_EQ_U(link_decode_cmd_calib(bad2, 1u, &cb), LNK_DEC_ERR_PARAM);
    CHECK_EQ_U(link_decode_cmd_calib(bad2, 2u, &cb), LNK_DEC_ERR_LEN);
}

/*==============================================================================
 * 2. TEL_FAST 布局（逐偏移核对）
 *============================================================================*/

static void test_tel_fast_layout(void)
{
    section("TEL_FAST 96-byte layout, offset by offset");

    tel_fast_src_t s;
    memset(&s, 0, sizeof(s));

    /* 填入【可辨认】的值，便于按偏移断言 */
    for (uint8_t i = 0u; i < 12u; ++i) {
        s.q_rad[i]     = (float)(i + 1) * 0.0100f;    /* → 100,200,...,1200 */
        s.qd_rad_s[i]  = (float)(i + 1) * 0.1000f;    /* → 1000,2000,...     */
        s.tau_nm[i]    = (float)(i + 1) * 0.0010f;    /* → 1,2,...,12        */
    }
    s.quat_wxyz[0] =  1.0000f;    /* → 10000 */
    s.quat_wxyz[1] = -0.5000f;    /* → -5000 */
    s.quat_wxyz[2] =  0.2500f;    /* →  2500 */
    s.quat_wxyz[3] =  0.1250f;    /* →  1250 */
    s.gyro_rad_s[0] =  1.000f;    /* → 1000  */
    s.gyro_rad_s[1] = -0.500f;    /* → -500  */
    s.gyro_rad_s[2] =  0.125f;    /* →  125  */
    s.gait_sin_i16 = 32767;
    s.gait_cos_i16 = -16384;
    s.cycle_cnt    = 0x1234u;
    s.voltage_mv   = 11500u;
    s.status       = 0x0Fu;
    s.err_flags    = 0xA5u;

    uint8_t p[RLCA_LEN_TEL_FAST];
    const uint8_t n = link_pack_tel_fast(&s, p);
    CHECK_EQ_U(n, RLCA_LEN_TEL_FAST);
    CHECK_EQ_U(n, 96u);

    /* ---- 偏移 0..23：q（0.0001 rad） ---- */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_Q + (2u * i)]), (int)((i + 1) * 100));
    }
    /* ---- 偏移 24..47：qd（0.0001 rad/s） ---- */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_QD + (2u * i)]), (int)((i + 1) * 1000));
    }
    /* ---- 偏移 48..71：tau（0.001 N·m） ---- */
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_TAU + (2u * i)]), (int)(i + 1));
    }
    /* ---- 偏移 72..79：四元数 ---- */
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_QUAT + 0u]),  10000);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_QUAT + 2u]),  -5000);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_QUAT + 4u]),   2500);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_QUAT + 6u]),   1250);
    /* ---- 偏移 80..85：陀螺 ---- */
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_GYRO + 0u]),  1000);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_GYRO + 2u]),  -500);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_GYRO + 4u]),   125);
    /* ---- 偏移 86..89：步态相位（1/32767） ---- */
    CHECK_EQ_D(rlca_get_i16(&p[86]), 32767);
    CHECK_EQ_D(rlca_get_i16(&p[88]), -16384);
    /* ---- 偏移 90..91：cycle_cnt ---- */
    CHECK_EQ_U(rlca_get_u16(&p[90]), 0x1234u);
    /* ---- 偏移 92..93：电压 mV ---- */
    CHECK_EQ_U(rlca_get_u16(&p[92]), 11500u);
    /* ---- 偏移 94 / 95 ---- */
    CHECK_EQ_U(p[94], 0x0Fu);
    CHECK_EQ_U(p[95], 0xA5u);

    /* ---- 关键：偏移常量必须自洽（防止将来手改）
     * Python 参考实现（协议 §9）用的就是这些字面量偏移，
     * 一旦常量漂移，这里立刻失败。 ---- */
    CHECK_EQ_U(RLCA_TF_OFF_Q, 0u);
    CHECK_EQ_U(RLCA_TF_OFF_QD, 24u);
    CHECK_EQ_U(RLCA_TF_OFF_TAU, 48u);
    CHECK_EQ_U(RLCA_TF_OFF_QUAT, 72u);
    CHECK_EQ_U(RLCA_TF_OFF_GYRO, 80u);
    CHECK_EQ_U(RLCA_TF_OFF_GAIT_SIN, 86u);
    CHECK_EQ_U(RLCA_TF_OFF_GAIT_COS, 88u);
    CHECK_EQ_U(RLCA_TF_OFF_CYCLE_CNT, 90u);
    CHECK_EQ_U(RLCA_TF_OFF_VOLTAGE, 92u);
    CHECK_EQ_U(RLCA_TF_OFF_STATUS, 94u);
    CHECK_EQ_U(RLCA_TF_OFF_ERR_FLAGS, 95u);

    /* ---- 饱和：超量程不能被回绕成反向值 ---- */
    memset(&s, 0, sizeof(s));
    s.tau_nm[0]     =  100.0f;     /* 100 / 0.001 = 100000 → 必须饱和到 32767 */
    s.tau_nm[1]     = -100.0f;     /* → -32768 */
    s.gyro_rad_s[0] =  100.0f;     /* 100 / 0.001 = 100000 → 饱和 */
    (void)link_pack_tel_fast(&s, p);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_TAU + 0u]), 32767);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_TAU + 2u]), -32768);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_GYRO]), 32767);

    /* ---- NaN 必须被量化成 0，不能是垃圾值 ---- */
    memset(&s, 0, sizeof(s));
    s.q_rad[0] = 0.0f / 0.0f;
    s.q_rad[1] = INFINITY;
    (void)link_pack_tel_fast(&s, p);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_Q + 0u]), 0);
    CHECK_EQ_D(rlca_get_i16(&p[RLCA_TF_OFF_Q + 2u]), 0);
}

/*==============================================================================
 * 3. TEL_STATUS 布局
 *============================================================================*/

static void test_tel_status_layout(void)
{
    section("TEL_STATUS 40-byte layout");

    static const uint8_t temps[12] = { 30u, 31u, 32u, 33u, 34u, 35u,
                                       36u, 37u, 38u, 39u, 40u, 41u };
    tel_status_src_t s;
    memset(&s, 0, sizeof(s));
    s.uptime_ms   = 0x0001E240u;    /* 123456 */
    s.voltage_mv  = 11234u;
    s.temp_deci_c = temps;
    s.rx_overflow = 3u;
    s.bus_err     = 7u;
    s.bus_timeout = 11u;
    s.mode        = (uint8_t)RLCA_MODE_ACTIVE;
    s.status      = 0x0Fu;
    s.err_flags   = 0x01u;
    s.seq_lost    = 42u;

    uint8_t p[RLCA_LEN_TEL_STATUS];
    const uint8_t n = link_pack_tel_status(&s, p);
    CHECK_EQ_U(n, RLCA_LEN_TEL_STATUS);
    CHECK_EQ_U(n, 40u);

    CHECK_EQ_U(rlca_get_u32(&p[0]), 123456u);
    CHECK_EQ_U(rlca_get_u16(&p[4]), 11234u);
    for (uint8_t i = 0u; i < 12u; ++i) {
        CHECK_EQ_D(rlca_get_i16(&p[6u + (2u * i)]), (int)temps[i]);
    }
    CHECK_EQ_U(rlca_get_u16(&p[30]), 3u);
    CHECK_EQ_U(rlca_get_u16(&p[32]), 7u);
    CHECK_EQ_U(rlca_get_u16(&p[34]), 11u);
    CHECK_EQ_U(p[36], RLCA_MODE_ACTIVE);
    CHECK_EQ_U(p[37], 0x0Fu);
    CHECK_EQ_U(p[38], 0x01u);
    CHECK_EQ_U(p[39], 42u);

    /* 每个字段的偏移都在 LEN-1 之内 */
    CHECK_EQ_U(39u + 1u, RLCA_LEN_TEL_STATUS);

    /* NULL 温度指针也不应崩（无数据时填 0） */
    s.temp_deci_c = NULL;
    CHECK_EQ_U(link_pack_tel_status(&s, p), 40u);
    CHECK_EQ_D(rlca_get_i16(&p[6]), 0);
}

/*==============================================================================
 * 4. 指令合法化（协议 §8.3）
 *============================================================================*/

static void test_limits_apply(void)
{
    section("command sanitising: NaN / clamp / slew rate (protocol 8.3)");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);

    float qin[12];
    float prev[12];
    float qout[12];

    /* 全部从 0 开始 */
    for (uint8_t i = 0u; i < 12u; ++i) { qin[i] = 0.0f; prev[i] = 0.0f; }

    /* ---- ① NaN / Inf → 落到该关节下限 ---- */
    qin[0] = 0.0f / 0.0f;            /* hip：下限 0 */
    qin[4] = INFINITY;               /* thigh：下限 -35° */
    qin[8] = -INFINITY;              /* calf：下限 -60° */
    link_limits_apply(&cal, qin, prev, qout);

    float hip_min, hip_max, th_min, th_max, cf_min, cf_max;
    rlca_q_range(&cal, 0u, &hip_min, &hip_max);
    rlca_q_range(&cal, 4u, &th_min, &th_max);
    rlca_q_range(&cal, 8u, &cf_min, &cf_max);

    CHECK(!rlca_float_is_valid(qin[0]), "sanity: input really was NaN");
    /* ⚠️ NaN → 下限，但限速会把这一步限制在 0.15 rad 内，
     *    所以第一周期只能走到 min(下限, prev-0.15)。
     *    thigh 下限 ≈ -0.61，prev = 0 → 一步最多 -0.15 → 结果是 -0.15。 */
    CHECK_NEAR(qout[4], -RLCA_MAX_DELTA_PER_CYCLE_RAD, 1e-6);
    CHECK_NEAR(qout[8], cf_min > -RLCA_MAX_DELTA_PER_CYCLE_RAD
                       ? cf_min : -RLCA_MAX_DELTA_PER_CYCLE_RAD, 1e-6);
    CHECK(rlca_float_is_valid(qout[0]), "output must never be NaN");

    /* ---- ② 超限必须被夹住（多跑几周期让限速走完） ---- */
    for (uint8_t i = 0u; i < 12u; ++i) { qin[i] = 0.0f; prev[i] = 0.0f; qout[i] = 0.0f; }
    qin[0] = 5.0f;                   /* hip 上限 +40° = 0.698 */
    qin[4] = -5.0f;
    for (uint8_t k = 0u; k < 100u; ++k) {
        link_limits_apply(&cal, qin, prev, qout);
    }
    CHECK_NEAR(qout[0], hip_max, 1e-4);
    CHECK_NEAR(qout[4], th_min, 1e-4);
    CHECK(qout[0] <= hip_max + 1e-6f, "must never exceed the clamp");

    /* ---- ③ 限速：单周期最大变化 0.15 rad ---- */
    for (uint8_t i = 0u; i < 12u; ++i) { qin[i] = 0.0f; prev[i] = 0.0f; qout[i] = 0.0f; }
    qin[4] = 1.0f;                   /* thigh 上限 +90°，但起点 0 */
    link_limits_apply(&cal, qin, prev, qout);
    CHECK_NEAR(qout[4], RLCA_MAX_DELTA_PER_CYCLE_RAD, 1e-6);

    /* 连续大跳变必须按固定斜率爬升（而不是突然到位） */
    float last = qout[4];
    for (uint8_t k = 0u; k < 5u; ++k) {
        link_limits_apply(&cal, qin, prev, qout);
        CHECK_NEAR(qout[4] - last, RLCA_MAX_DELTA_PER_CYCLE_RAD, 1e-6);
        last = qout[4];
    }

    /* ---- ④ 反向限速 ---- */
    for (uint8_t i = 0u; i < 12u; ++i) { qin[i] = 0.0f; prev[i] = 0.0f; qout[i] = 0.0f; }
    qin[4] = -1.0f;
    link_limits_apply(&cal, qin, prev, qout);
    CHECK_NEAR(qout[4], -RLCA_MAX_DELTA_PER_CYCLE_RAD, 1e-6);

    /* ---- ⑤ 小变化必须原样通过（限速不能把正常指令磨平） ---- */
    for (uint8_t i = 0u; i < 12u; ++i) { qin[i] = 0.0f; prev[i] = 0.0f; qout[i] = 0.0f; }
    qin[4] = 0.10f;                  /* < 0.15，应原样通过 */
    link_limits_apply(&cal, qin, prev, qout);
    CHECK_NEAR(qout[4], 0.10f, 1e-6);

    /* ---- ⑥ prev 必须被子更新（否则下一周期会从旧值重新起算） ---- */
    CHECK_NEAR(prev[4], qout[4], 1e-9);

    /* ---- ⑦ NULL 参数不能崩 ---- */
    link_limits_apply(&cal, NULL, prev, qout);
    link_limits_apply(NULL, qin, prev, qout);
    CHECK(true, "NULL arguments must not crash");
}

/*==============================================================================
 * 5. 与 Python 参考实现的交叉验证（协议 §9）
 *============================================================================*/

/** 协议 §9 的 Python 代码 pack 出来的 CMD_JOINT 负载字节。
 *  这里手工把它算出来并与 C 实现比对 —— 目的是证明"两端一致"，
 *  而不是各写一份、各自都对。 */
static void test_cross_check_python(void)
{
    section("cross-check against the Python reference (protocol 9)");

    /* Python: struct.pack('<BB', mode=2, flags=1)
     *       + 12 × struct.pack('<h', round(q/0.0001))
     *       + struct.pack('<B', 0) + struct.pack('<h', 100)
     *       + struct.pack('<B', 1) + struct.pack('<B', 0)   → 31 字节 */
    float q[12] = { 0.0f };
    q[0] = 0.25f;      /* → 2500 = 0x09C4 */
    q[1] = -0.25f;     /* → -2500 = 0xF63C */

    uint8_t pay[RLCA_LEN_CMD_JOINT];
    build_cmd_joint(pay, 2u, 1u, q, 100, 1u);

    /* 逐字节核对开头与关键位置（小端） */
    CHECK_EQ_U(pay[0], 0x02u);
    CHECK_EQ_U(pay[1], 0x01u);
    CHECK_EQ_U(pay[2], 0xC4u);                 /* 2500 低字节 */
    CHECK_EQ_U(pay[3], 0x09u);                 /* 2500 高字节 */
    CHECK_EQ_U(pay[4], 0x3Cu);                 /* -2500 = 0xF63C */
    CHECK_EQ_U(pay[5], 0xF6u);
    CHECK_EQ_U(pay[26], 0x00u);
    CHECK_EQ_U(pay[27], 0x64u);                /* 100 低字节 */
    CHECK_EQ_U(pay[28], 0x00u);
    CHECK_EQ_U(pay[29], 0x01u);
    CHECK_EQ_U(pay[30], 0x00u);

    /* 整帧长度必须是 38（LEN=31 + 7 头/CRC） */
    {
        static uint8_t frame[160];
        const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                           RLCA_TYPE_CMD_JOINT, 0u, pay, RLCA_LEN_CMD_JOINT);
        CHECK_EQ_U(n, 38u);
    }

    /* TEL_FAST 整帧必须是 103 字节（协议 §3 表） */
    {
        tel_fast_src_t ts;
        memset(&ts, 0, sizeof(ts));
        static uint8_t tel[RLCA_LEN_TEL_FAST];
        static uint8_t big[160];
        (void)link_pack_tel_fast(&ts, tel);

        const uint16_t tn = rlca_frame_pack(big, (uint16_t)sizeof(big),
                                            RLCA_TYPE_TEL_FAST, 0u, tel, RLCA_LEN_TEL_FAST);
        CHECK_EQ_U(tn, 103u);

        /* ⚠️ 缓冲不足时必须【拒绝】（返回 0）而不是截断成半帧。
         *    这一点很关键：截断的帧会让上位机 CRC 失败、看起来像"丢帧"，
         *    排查时完全指错方向。 */
        uint8_t small[64];
        CHECK_EQ_U(rlca_frame_pack(small, (uint16_t)sizeof(small),
                                   RLCA_TYPE_TEL_FAST, 0u, tel, RLCA_LEN_TEL_FAST), 0u);
    }
}

/*==============================================================================
 * 6. 量化辅助
 *============================================================================*/

static void test_enc_i16(void)
{
    section("float -> int16 saturation");

    CHECK_EQ_D(link_enc_i16(0.0f, 0.0001f), 0);
    CHECK_EQ_D(link_enc_i16(0.3f, 0.0001f), 3000);
    CHECK_EQ_D(link_enc_i16(-0.3f, 0.0001f), -3000);
    /* 四舍五入：0.00015 / 0.0001 = 1.5 → 2 */
    CHECK_EQ_D(link_enc_i16(0.00015f, 0.0001f), 2);
    /* 饱和 */
    CHECK_EQ_D(link_enc_i16(1000.0f, 0.0001f), 32767);
    CHECK_EQ_D(link_enc_i16(-1000.0f, 0.0001f), -32768);
    /* 单位 0 / NaN / Inf 的防御 */
    CHECK_EQ_D(link_enc_i16(1.0f, 0.0f), 0);
    CHECK_EQ_D(link_enc_i16(0.0f / 0.0f, 0.0001f), 0);
    CHECK_EQ_D(link_enc_i16(-INFINITY, 0.0001f), 0);
}

/*==============================================================================
 * main
 *============================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT link_proto unit tests\n");
    printf("========================================\n");
    fflush(stdout);

    test_enc_i16();
    test_cmd_joint_decode();
    test_cmd_mode_and_calib();
    test_limits_apply();
    test_tel_fast_layout();
    test_tel_status_layout();
    test_cross_check_python();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
