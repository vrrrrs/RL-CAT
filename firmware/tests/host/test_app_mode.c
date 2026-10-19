/**
  ******************************************************************************
  * @file    test_app_mode.c
  * @brief   运行模式状态机的单元测试（含 2026-10-12 的【撞机回归】）。
  *
  *  ⭐ 为什么需要这一套：
  *     app_mode_update() 是"最终要下发给舵机的目标角"的唯一产生处，
  *     但在 2026-10-12 之前【一行测试都没有】。
  *     而它恰好藏着一个已实测的撞机事故：
  *
  *       未标定时 offset[]=0、dir[]=+1（rlca_calib_load_defaults 的虚表），
  *       STAND 分支把 q_ref 设成 q_default = 0，
  *       经 rlca_q_to_step() 换算 ⇒ step = 0
  *       ⇒ 12 个舵机一起冲向机械行程一端。
  *
  *     所以本套测试的第一组用例就是把这个危害【钉死】：
  *       · 先把危险本身断言出来（q=0 + 默认表 ⇒ step 0）
  *       · 再断言修复后的行为（未标定时不再产出 0，而是跟随实测角 + 卸力矩）
  *
  *  ⚠️ 依赖：app_mode.c 调用了 app_safety_in_estop()（单一安全真值源），
  *     所以本套需要链接 app_safety.c。app_safety.c 的硬件依赖全在
  *     servo_bus 之外（它只读 battery/imu 的纯函数），可在主机侧编译。
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"
#include "app_mode.h"
#include "app_safety.h"

/*==============================================================================
 * 测试框架（与其它套件保持一致）
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

#define CHECK_M(cond)                                                     \
    do {                                                                  \
        if (cond) { g_pass++; }                                           \
        else {                                                            \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  (condition false)\n", __FILE__, __LINE__); \
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
 * 公共装置
 *============================================================================*/

/** 每个用例开头复位：模式回 IDLE，安全层回"一切正常"。 */
static void fixture_reset(void)
{
    app_safety_reset();
    app_mode_init();
    /* app_mode_init 内部也会 app_safety_reset()，这里再显式一次便于阅读 */
}

/** 填一个"实测 12 关节角"，默认用一组非零值，便于区分"跟随实测"与"给 0"。 */
static void fill_q_now(float *q_now, float v)
{
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_now[i] = v;
    }
}

#if defined(__GNUC__)
/* 便于逐维检查"全部 12 个都等于某值" */
#endif

/*==============================================================================
 * Part 0 —— ⭐⭐ 撞机回归：先把危害本身断言出来
 *============================================================================*/

static void test_hazard_default_calib_maps_zero_to_step_zero(void)
{
    section("HAZARD: uncalibrated (default) calib table maps q=0 -> servo step 0");

    rlca_calib_t def;
    rlca_calib_load_defaults(&def);

    /* 默认表是"全零虚表"：offset=0、dir=+1 */
    CHECK_EQ_D(def.offset[0], 0);
    CHECK_EQ_D(def.dir[0], 1);

    /* ⚠️ 这就是撞机的根源：q_target = 0（正是站姿）⇒ step = 0 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int16_t step = rlca_q_to_step(&def, i, RLCA_Q_DEFAULT_RAD);
        CHECK_EQ_D(step, 0);
    }

    /* 换句话说：未标定时"站姿"这个指令与"冲向机械零点"是同一个指令 */
    printf("    q_default=%.3f -> step %d  (12 joints all the same)\n",
           (double)RLCA_Q_DEFAULT_RAD,
           (int)rlca_q_to_step(&def, 0u, RLCA_Q_DEFAULT_RAD));
}

/*==============================================================================
 * Part 1 —— ⭐ 安全修复：未标定时绝不能产出 step 0
 *============================================================================*/

static void test_uncalibrated_stand_is_safe(void)
{
    section("FIX: uncalibrated STAND must NOT command q_default (torque off + follow q_now)");

    fixture_reset();

    rlca_calib_t def;
    rlca_calib_load_defaults(&def);

    float q_now[RLCA_N_JOINTS];
    fill_q_now(q_now, 0.5f);              /* 一个明显非零的"实测角" */

    float q_ref[RLCA_N_JOINTS];
    bool  torque = true;
    memset(q_ref, 0, sizeof(q_ref));

    app_mode_request((uint8_t)RLCA_MODE_STAND);
    /* ⚠️ 时间戳必须 > MODE_MIN_HOLD_MS：app_mode_init 把 mode_t0_ms 设为 0，
     *    传 now=0 会让 "now - mode_t0_ms < 100ms" 成立而把请求挡掉。 */
    app_mode_update(100000u, &def,
                    /*link_alive*/ true, /*imu_ok*/ true,
                    /*calib_ok*/   false, /*fb_ok*/  true,
                    /*pi_q_target*/ NULL,
                    q_now, q_ref, &torque);

    /* ① 模式仍应生效为 STAND（不是掉到别的模式） */
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);

    /* ② ⭐ 必须卸力矩 —— 不能用错位的目标去"保持姿态" */
    CHECK_M(!torque);

    /* ③ ⭐ 目标必须跟随实测角，而不是 q_default(0) */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_NEAR(q_ref[i], 0.5f, 1e-6f);
    }

    /* ④ ⭐⭐ 决定性断言：产出的目标角换算成步数，绝不能是 0
     *     （0 就是"冲向机械行程一端"） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int16_t step = rlca_q_to_step(&def, i, q_ref[i]);
        CHECK(step != 0, "joint %u still maps to step 0 (crash path!)", (unsigned)i);
    }

    /* ⑤ 而且应该正好等于"实测角对应的步数"，即待在原地 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_EQ_D(rlca_q_to_step(&def, i, q_ref[i]),
                   rlca_q_to_step(&def, i, q_now[i]));
    }
}

static void test_uncalibrated_stand_without_q_now(void)
{
    section("FIX: uncalibrated STAND with q_now==NULL must still disable torque");

    fixture_reset();

    rlca_calib_t def;
    rlca_calib_load_defaults(&def);

    float q_ref[RLCA_N_JOINTS];
    bool  torque = true;

    app_mode_request((uint8_t)RLCA_MODE_STAND);
    app_mode_update(100000u, &def, true, true, false, true, NULL,
                    NULL /* q_now 缺失 */, q_ref, &torque);

    /* 即便没有实测角可用，也必须卸力矩（宁可不动，也不要乱动） */
    CHECK_M(!torque);
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);
}

static void test_calibrated_stand_still_holds_pose(void)
{
    section("REGRESSION: calibrated STAND must still hold q_default with torque ON");

    fixture_reset();

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    /* 造一个"标定过"的表：给一个非零 offset，并让校验通过 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        cal.offset[i] = (int16_t)(1000 + 10 * i);
        cal.dir[i]    = 1;
    }
    cal.crc16 = rlca_calib_crc(&cal);
    CHECK_M(rlca_calib_is_valid(&cal));

    float q_now[RLCA_N_JOINTS];
    fill_q_now(q_now, 0.5f);              /* 实测角故意与目标不同 */

    float q_ref[RLCA_N_JOINTS];
    bool  torque = false;

    app_mode_request((uint8_t)RLCA_MODE_STAND);
    app_mode_update(2000u, &cal, true, true, /*calib_ok*/ true, true, NULL,
                    q_now, q_ref, &torque);

    /* ⭐ 已标定时行为不变：力矩使能 + 目标是 q_default */
    CHECK_M(torque);
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_NEAR(q_ref[i], RLCA_Q_DEFAULT_RAD, 1e-6f);
    }
    /* 并且此时 q_default ⇒ offset（不是 0）—— 标定表让"站姿"落在正确的步数上 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_EQ_D(rlca_q_to_step(&cal, i, q_ref[i]), cal.offset[i]);
    }
}

static void test_uncalibrated_idle_and_damp_are_safe(void)
{
    section("FIX: uncalibrated IDLE / DAMP must follow q_now (never q_default)");

    rlca_calib_t def;
    rlca_calib_load_defaults(&def);

    const uint8_t modes[2] = { (uint8_t)RLCA_MODE_IDLE, (uint8_t)RLCA_MODE_DAMP };

    for (int m = 0; m < 2; ++m) {
        fixture_reset();

        float q_now[RLCA_N_JOINTS];
        fill_q_now(q_now, -0.25f);

        float q_ref[RLCA_N_JOINTS];
        bool  torque = true;
        memset(q_ref, 0, sizeof(q_ref));

        app_mode_request(modes[m]);
        app_mode_update(100000u, &def, true, true, false, true, NULL, q_now, q_ref, &torque);

        CHECK_M(!torque);
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            CHECK_NEAR(q_ref[i], -0.25f, 1e-6f);
            CHECK(rlca_q_to_step(&def, i, q_ref[i]) != 0,
                  "mode %u joint %u maps to step 0", (unsigned)modes[m], (unsigned)i);
        }
    }
}

/*==============================================================================
 * Part 2 —— ACTIVE 准入（原有语义不能被我改坏）
 *============================================================================*/

static void test_active_gate_matrix(void)
{
    section("ACTIVE gate: keep the original semantics (order + reasons)");

    /* 未标定优先报出（最根本的缺失排在第一位） */
    CHECK_EQ_U(app_mode_active_gate(false, true,  true,  true,  false),
               (unsigned)RLCA_ACTIVE_BLOCK_NOT_CALIB);
    /* 安全层阻断优先于链路 */
    CHECK_EQ_U(app_mode_active_gate(true,  false, true,  true,  true),
               (unsigned)RLCA_ACTIVE_BLOCK_SAFETY);
    CHECK_EQ_U(app_mode_active_gate(true,  false, true,  true,  false),
               (unsigned)RLCA_ACTIVE_BLOCK_NO_LINK);
    CHECK_EQ_U(app_mode_active_gate(true,  true,  false, true,  false),
               (unsigned)RLCA_ACTIVE_BLOCK_NO_IMU);
    CHECK_EQ_U(app_mode_active_gate(true,  true,  true,  false, false),
               (unsigned)RLCA_ACTIVE_BLOCK_NO_FB);
    CHECK_EQ_U(app_mode_active_gate(true,  true,  true,  true,  false),
               (unsigned)RLCA_ACTIVE_OK);
}

static void test_uncalibrated_active_degrades_and_is_safe(void)
{
    section("FIX: uncalibrated ACTIVE degrades to STAND, and that STAND is safe");

    fixture_reset();

    rlca_calib_t def;
    rlca_calib_load_defaults(&def);

    float q_now[RLCA_N_JOINTS];
    fill_q_now(q_now, 0.3f);

    float pi_q[RLCA_N_JOINTS];
    fill_q_now(pi_q, 1.2f);               /* 策略要求一个很大的角 */

    float q_ref[RLCA_N_JOINTS];
    bool  torque = true;

    app_mode_request((uint8_t)RLCA_MODE_ACTIVE);
    app_mode_update(3000u, &def, true, true, /*calib_ok*/ false, true,
                    pi_q, q_now, q_ref, &torque);

    /* 降级到 STAND（原有设计：站着总比瘫着安全 —— 但未标定下"站"被改成"不动"） */
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);
    CHECK_EQ_U(app_mode_state()->active_block, (unsigned)RLCA_ACTIVE_BLOCK_NOT_CALIB);

    /* ⭐ 关键：绝不能把策略给的 1.2 rad 直接下发（那是未标定的表，全是错的） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_NEAR(q_ref[i], 0.3f, 1e-6f);        /* 跟随实测，不是 pi_q */
        CHECK(rlca_q_to_step(&def, i, q_ref[i]) != 0,
              "degraded joint %u maps to step 0", (unsigned)i);
    }
    CHECK_M(!torque);
}

static void test_calibrated_active_tracks_pi_target(void)
{
    section("REGRESSION: calibrated ACTIVE tracks pi_q_target with torque ON");

    fixture_reset();

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        cal.offset[i] = (int16_t)(2048 + 5 * i);
        cal.dir[i]    = 1;
    }
    cal.crc16 = rlca_calib_crc(&cal);

    float q_now[RLCA_N_JOINTS];
    fill_q_now(q_now, 0.0f);

    float pi_q[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        pi_q[i] = -0.2f + 0.01f * (float)i;
    }

    float q_ref[RLCA_N_JOINTS];
    bool  torque = false;

    app_mode_request((uint8_t)RLCA_MODE_ACTIVE);
    app_mode_update(5000u, &cal, true, true, true, true, pi_q, q_now, q_ref, &torque);

    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_ACTIVE);
    CHECK_M(torque);
    /* 幅度缩放为 1.0（安全层正常）⇒ 应原样跟随 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_NEAR(q_ref[i], pi_q[i], 1e-6f);
    }
}

/*==============================================================================
 * Part 3 —— 模式最短保持（原有语义）
 *============================================================================*/

static void test_mode_min_hold(void)
{
    section("mode min-hold: a flip within 100 ms must be suppressed");

    fixture_reset();

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        cal.offset[i] = 2048;
        cal.dir[i]    = 1;
    }
    cal.crc16 = rlca_calib_crc(&cal);

    float q_now[RLCA_N_JOINTS];
    float q_ref[RLCA_N_JOINTS];
    bool  torque = false;
    fill_q_now(q_now, 0.0f);

    /* 进入 STAND */
    app_mode_request((uint8_t)RLCA_MODE_STAND);
    app_mode_update(10000u, &cal, true, true, true, true, NULL, q_now, q_ref, &torque);
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);

    /* 10 ms 后请求 DAMP —— 被最短保持挡住，应仍是 STAND */
    app_mode_request((uint8_t)RLCA_MODE_DAMP);
    app_mode_update(10010u, &cal, true, true, true, true, NULL, q_now, q_ref, &torque);
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_STAND);

    /* 过 100 ms 后同一请求应生效 */
    app_mode_update(10101u, &cal, true, true, true, true, NULL, q_now, q_ref, &torque);
    CHECK_EQ_U(app_mode_current(), (unsigned)RLCA_MODE_DAMP);
}

/*==============================================================================
 * 入口
 *============================================================================*/

/*==============================================================================
 * ⭐⭐ 急停的处置方式：只卸力矩，不做"主动趴下斜坡"（2026-10-17 定案）
 *
 *  背景：旧行为是"用剩余力矩把 12 个关节主动折向趴下姿态"。
 *        那是一条**没人要求的运动** —— 链路一抖就折腿，
 *        调试时表现为"按一下 shell 的 R/K 机器人就抖一下"。
 *
 *  新行为（RLCA_SAFE_ESTOP_SITDOWN == 0）：
 *        · 立刻 torque_off = true（app_mode 会把力矩关掉）
 *        · done = true（没有斜坡可等）
 *        · q_ref **原地不动**（卸力后位置由重力与机械决定；
 *          若强行把 q_ref 拖到某值，力矩恢复时会跳变）
 *
 *  ⚠️ 这个用例同时是一道"防回退"闸：若有人把宏改回 1，
 *     本用例会失败并提示"你在用旧行为"。
 *============================================================================*/

static void test_estop_unloads_torque_without_ramping(void)
{
    section("safety: ESTOP must unload torque WITHOUT ramping to prone");

    app_safety_reset();

    /* 标定表要合法，否则 app_safety_update 会走"未标定短路" */
    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        cal.offset[i] = (int16_t)(2048);      /* 一个真实的装配中位 */
    }
    cal.crc16 = rlca_calib_crc(&cal);

    /* q_ref 给一组可辨认的非零值，用来验证"没有被改动" */
    float q_ref[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.25f;
    }

    /* ---- 急停：链路失联（link_level = 2） ---- */
    app_safety_update(1000u, 2u, &cal, q_ref, 30u, 0u);

    CHECK(app_safety_in_estop(), "link_loss must be ESTOP");
    CHECK_EQ_U(app_safety_severity(), 3u);

    const app_safety_state_t *st = app_safety_state();
    CHECK(st->torque_off,
          "ESTOP must unload torque IMMEDIATELY (no ramp to prone)");
    CHECK(st->done,
          "ESTOP must report done immediately when there is no ramp");
    CHECK_EQ_U(st->active ? 1u : 0u, 1u);   /* 仍在"急停降级中" */

    /* ⭐ 核心：q_ref 必须原封不动 */
    uint8_t unchanged = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        if (fabs((double)q_ref[i] - 0.25) < 1e-6) {
            unchanged++;
        }
    }
    CHECK_EQ_U(unchanged, RLCA_N_JOINTS);
    if (RLCA_SAFE_ESTOP_SITDOWN != 0) {
        printf("    NOTE: RLCA_SAFE_ESTOP_SITDOWN != 0 —— 你在用旧的"
               "\"主动趴下斜坡\"行为\n");
    }

    /* ---- 急停解除（链路恢复）---- */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.25f;
    }
    app_safety_update(1100u, 0u, &cal, q_ref, 30u, 0u);

    CHECK(!app_safety_in_estop(), "healthy link must leave ESTOP");
    CHECK(!app_safety_state()->torque_off,
          "leaving ESTOP must clear torque_off (mode machine regains control)");
    CHECK(!app_safety_state()->active, "leaving ESTOP must clear `active`");
}

/*==============================================================================
 * ⭐⭐ BUS_TIMEOUT 不得压缩动作幅度（2026-10-17 定案）
 *
 *  现场症状：pose.py 目标 20°，实测 ~13.7°，关节【往复摆动】，
 *            "保持 1 秒后下垂 7.34°"。
 *  算术：20° × 0.75 = 15°，再掉 1.3° 重力 = 13.7° —— 幅度缩放被激活了。
 *
 *  根因：BUS_TIMEOUT 是"某一轮整轮没收到回复"的通信统计（约 0.19% 轮次），
 *        置位后要 2 秒才自清 ⇒ amplitude_scale 在 1.0/0.75 之间反复跳
 *        ⇒ 目标角在 20°/15° 之间反复跳 ⇒ 看起来像关节在摆动。
 *
 *  ⭐ 语义拆分：
 *      · 舵机【自报】的错误（过温/过流/过载/电压/角度/限位）⇒ 降级 + 限幅 0.75
 *      · 只有 BUS_TIMEOUT（通信）                      ⇒ 仅 severity 1 告警
 *
 *  ⚠️ 安全性不降低：总线真死了 `servo_bus_feedback_fresh()` 会假，
 *     ACTIVE 准入第 ⑤ 道门（NO_FB）直接挡住，比"压 25% 幅度"强得多。
 *============================================================================*/

static void test_bus_timeout_must_not_shrink_amplitude(void)
{
    section("safety: BUS_TIMEOUT must NOT shrink amplitude (warn only)");

    rlca_calib_t cal;
    rlca_calib_load_defaults(&cal);
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        cal.offset[i] = (int16_t)2048;
    }
    cal.crc16 = rlca_calib_crc(&cal);

    float q_ref[RLCA_N_JOINTS];

    /* ---- ① 只有 BUS_TIMEOUT ⇒ 只告警，幅度必须 100% ---- */
    app_safety_reset();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.0f;
    }
    app_safety_update(1000u, 0u, &cal, q_ref, 30u, RLCA_ERRF_BUS_TIMEOUT);

    /* ⚠️ 不断言 severity 的具体值：宿主机上电池模块没有真实读数
     *    （battery_mv()=0 且 valid），会额外触发 VOLT_STOP ⇒ 恒为 2。
     *    本用例要钉的是【幅度】，那才是"目标角被缩放"的直接原因。 */
    CHECK(app_safety_severity() < 3u, "a bus timeout must not be an ESTOP");
    CHECK_NEAR(app_safety_amplitude_scale(), 1.0f, 1e-6);
    CHECK(!app_safety_in_estop(), "a bus timeout must not be an ESTOP");
    CHECK((app_safety_reasons() & RLCA_SAFE_R_SERVO_ERR) != 0u,
          "the timeout must still be REPORTED as SERVO_ERR");

    /* ---- ② 舵机自报的错误 ⇒ 必须降级 + 限幅 0.75 ---- */
    app_safety_reset();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.0f;
    }
    app_safety_update(1100u, 0u, &cal, q_ref, 30u, RLCA_ERRF_OVERLOAD);

    CHECK_NEAR(app_safety_amplitude_scale(), 0.75f, 1e-6);
    CHECK(!app_safety_in_estop(), "overload is a degrade, not an ESTOP");

    /* ---- ③ 两者同时存在 ⇒ 仍按自报错误限幅 ---- */
    app_safety_reset();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.0f;
    }
    app_safety_update(1200u, 0u, &cal, q_ref, 30u,
                      (uint8_t)(RLCA_ERRF_BUS_TIMEOUT | RLCA_ERRF_OVERHEAT));

    CHECK_NEAR(app_safety_amplitude_scale(), 0.75f, 1e-6);

    /* ---- ④ 无错误 ⇒ 100% ---- */
    app_safety_reset();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_ref[i] = 0.0f;
    }
    app_safety_update(1300u, 0u, &cal, q_ref, 30u, 0u);
    CHECK((app_safety_reasons() & RLCA_SAFE_R_SERVO_ERR) == 0u,
          "no error flags => no SERVO_ERR reason");
    CHECK_NEAR(app_safety_amplitude_scale(), 1.0f, 1e-6);
}

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT app_mode unit tests (crash regression)\n");
    printf("========================================\n");
    fflush(stdout);

    test_hazard_default_calib_maps_zero_to_step_zero();

    test_uncalibrated_stand_is_safe();
    test_uncalibrated_stand_without_q_now();
    test_calibrated_stand_still_holds_pose();
    test_uncalibrated_idle_and_damp_are_safe();

    test_active_gate_matrix();
    test_uncalibrated_active_degrades_and_is_safe();
    test_calibrated_active_tracks_pi_target();

    test_mode_min_hold();

    test_estop_unloads_torque_without_ramping();
    test_bus_timeout_must_not_shrink_amplitude();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
