/**
  ******************************************************************************
  * @file    test_safety_calib.c
  * @brief   P6 单元测试：安全判据（纯逻辑）+ 关节标定流程（假舵机总线）。
  *
  *  ⭐ 这一套测试是"安全保护能否独立工作"的证据链：
  *     · 每一个触发条件单独、以及组合起来，输出是否正确
  *     · 判据边界（30/100 ms、10.5/9.9/9.3 V、70 °C）是否精确
  *     · 标定流程能不能在"模拟真实舵机"的情况下真的跑完并算对 offset/dir
  *     · 各种失败路径（读失败、没动、方向不明、结果不合法）是否被拦下
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"
#include "rlca_safety.h"
#include "rlca_calib_proc.h"

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

/* ⚠️ CHECK() 要求带格式串（-Wformat 在 -Werror 下必须成立）。
 *   只判条件、不带说明时用 CHECK_M —— 失败时只打印文件与行号。 */
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
 * Part 1 —— 安全判据
 *============================================================================*/

/** 造一个"一切正常"的输入 */
static void safe_in(rlca_safety_in_t *in)
{
    memset(in, 0, sizeof(*in));
    in->link_level   = 0u;
    in->cmd_age_ms   = 0u;
    in->batt_mv      = 12000u;
    in->batt_valid   = true;
    in->max_temp_c   = 35u;
    in->servo_err_flags = 0u;
    in->imu_ok       = true;
    in->operator_estop = false;
}

static void test_safety_nominal(void)
{
    section("safety: nominal state must do nothing");

    rlca_safety_in_t in;
    rlca_safety_out_t out;
    safe_in(&in);
    rlca_safety_eval(&in, &out);

    CHECK_EQ_U(out.reasons, 0u);
    CHECK_EQ_U(out.severity, 0u);
    CHECK(!out.unload_torque, "nominal must not unload torque");
    CHECK(!out.sit_down, "nominal must not sit down");
    CHECK(!out.limit_amplitude, "nominal must not limit amplitude");
    CHECK(!out.block_active, "nominal must allow ACTIVE");
    CHECK_NEAR(out.amplitude_scale, 1.0, 1e-6);

    /* NULL 不能崩 */
    rlca_safety_eval(&in, NULL);
    rlca_safety_eval(NULL, &out);
    CHECK(true, "NULL arguments must not crash");
}

static void test_safety_link_loss(void)
{
    section("safety: link loss is an independent estop (the core requirement)");

    rlca_safety_in_t in;
    rlca_safety_out_t out;

    /* 降级：>30 ms 但不该趴下 */
    safe_in(&in);
    in.link_level = 1u;
    in.cmd_age_ms = 50u;
    rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, RLCA_SAFE_R_LINK_LOST);
    CHECK_EQ_U(out.severity, 2u);
    CHECK(!out.sit_down, "degrade must NOT sit down");
    CHECK(!out.unload_torque, "degrade must NOT unload torque");

    /* 急停：>100 ms 必须趴下 */
    safe_in(&in);
    in.link_level = 2u;
    in.cmd_age_ms = 150u;
    rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, RLCA_SAFE_R_LINK_LOST);
    CHECK_EQ_U(out.severity, 3u);
    CHECK(out.sit_down, "link loss must trigger sit-down");
    CHECK(out.unload_torque, "link loss must unload torque");
    CHECK(out.block_active, "must block ACTIVE after link loss");
    CHECK_NEAR(out.amplitude_scale, 0.0, 1e-6);
}

static void test_safety_battery(void)
{
    section("safety: battery thresholds (L1/L2/L3) at exact boundaries");

    rlca_safety_in_t in;
    rlca_safety_out_t out;

    /* L1 边界：10500 触发 WARN，但不限幅、不趴下 */
    safe_in(&in); in.batt_mv = 10501u; rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, 0u);
    safe_in(&in); in.batt_mv = 10500u; rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, RLCA_SAFE_R_VOLT_WARN);
    CHECK_EQ_U(out.severity, 1u);
    CHECK_M(!out.limit_amplitude);
    CHECK_M(!out.sit_down);

    /* L2 边界：9900 → 限幅一半 */
    safe_in(&in); in.batt_mv = 9901u; rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, RLCA_SAFE_R_VOLT_WARN);
    CHECK_M(!out.limit_amplitude);

    safe_in(&in); in.batt_mv = 9900u; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_VOLT_STOP) != 0u);
    CHECK_M((out.reasons & RLCA_SAFE_R_VOLT_WARN) != 0u);
#if (RLCA_BATT_LIMIT_EN != 0)
    CHECK(out.limit_amplitude, "L2 must limit amplitude");
    CHECK_NEAR(out.amplitude_scale, 0.5, 1e-6);
    CHECK_EQ_U(out.severity, 2u);
#else
    /* ⭐ 限幅已关闭（RLCA_BATT_LIMIT_EN = 0）：
     *    VOLT_STOP 仍要上报，但**不**改幅度、**不**降级。
     *    理由见 rlcat_config.h 里那个开关的注释
     *    （分压比写成 22k 而硬件是 20k ⇒ 读数偏低 7.6% ⇒ 误触发限幅）。 */
    CHECK(!out.limit_amplitude, "L2 must NOT limit while RLCA_BATT_LIMIT_EN == 0");
    CHECK_NEAR(out.amplitude_scale, 1.0, 1e-6);
    CHECK_EQ_U(out.severity, 1u);
#endif
    CHECK(!out.sit_down, "L2 must NOT sit down");

    /* L3 边界：9300 → 强制趴下 */
    safe_in(&in); in.batt_mv = 9301u; rlca_safety_eval(&in, &out);
    CHECK_M(!out.sit_down);
    safe_in(&in); in.batt_mv = 9300u; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_VOLT_FORCE) != 0u);
    CHECK_EQ_U(out.severity, 3u);
    CHECK(out.sit_down, "L3 must force sit-down");
    CHECK_M(out.unload_torque);
    CHECK(out.ramp_rate_rad_s < 0.5f, "L3 should ramp slower than normal");

    /* ⚠️ 电压读数失效时【不能】当成正常、也不能当成低压 */
    safe_in(&in);
    in.batt_valid = false;
    in.batt_mv = 0u;              /* 读不到时 mv 是 0，绝不能因此触发 L3 */
    rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, 0u);
    CHECK(!out.sit_down, "stale battery reading must not be treated as 0 V");
    printf("    stale battery reading -> reasons=0x%02X severity=%u\n",
           (unsigned)out.reasons, (unsigned)out.severity);
}

static void test_safety_servo_and_imu(void)
{
    section("safety: servo overtemp / err flags / IMU stale");

    rlca_safety_in_t in;
    rlca_safety_out_t out;

    /* 过温：>70 °C 强制趴下 */
    safe_in(&in); in.max_temp_c = 70u; rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, 0u);
    CHECK_M(!out.sit_down);

    safe_in(&in); in.max_temp_c = 71u; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_OVERTEMP) != 0u);
    CHECK(out.sit_down, "overtemp must force sit-down");
    CHECK_EQ_U(out.severity, 3u);

    /* 舵机错误码：限幅但不趴下 */
    safe_in(&in); in.servo_err_flags = RLCA_ERRF_OVERLOAD; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_SERVO_ERR) != 0u);
    CHECK(out.limit_amplitude, "servo error must limit amplitude");
    CHECK(!out.sit_down, "servo error alone must NOT sit down");
    CHECK_EQ_U(out.severity, 2u);
    CHECK_NEAR(out.amplitude_scale, 0.75, 1e-6);

    /* ⚠️ 总线超时（ERRF_BUS_TIMEOUT）也算舵机错误的一种 */
    safe_in(&in); in.servo_err_flags = RLCA_ERRF_BUS_TIMEOUT; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_SERVO_ERR) != 0u);

    /* IMU 失效：禁 ACTIVE，但不趴下 */
    safe_in(&in); in.imu_ok = false; rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_IMU_STALE) != 0u);
    CHECK(out.block_active, "IMU stale must block ACTIVE");
    CHECK(!out.sit_down, "IMU stale alone must NOT sit down");
    CHECK_EQ_U(out.severity, 2u);
    printf("    IMU stale -> block_active=%d sit_down=%d severity=%u\n",
           (int)out.block_active, (int)out.sit_down, (unsigned)out.severity);
}

static void test_safety_combinations_and_precedence(void)
{
    section("safety: combinations and precedence (worst wins)");

    rlca_safety_in_t in;
    rlca_safety_out_t out;

    /* L1 + IMU 失效：总 severity 取更严重的（2），但不趴下 */
    safe_in(&in);
    in.batt_mv = 10500u;
    in.imu_ok  = false;
    rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_VOLT_WARN) != 0u);
    CHECK_M((out.reasons & RLCA_SAFE_R_IMU_STALE) != 0u);
    CHECK_EQ_U(out.severity, 2u);
    CHECK_M(!out.sit_down);

    /* L2 + 舵机错误：幅度取更保守的那个 */
    safe_in(&in);
    in.batt_mv = 9900u;
    in.servo_err_flags = RLCA_ERRF_OVERLOAD;
    rlca_safety_eval(&in, &out);
#if (RLCA_BATT_LIMIT_EN != 0)
    CHECK_NEAR(out.amplitude_scale, 0.5, 1e-6);
#else
    /* ⭐ 电压限幅关闭 ⇒ 只剩"舵机自报错误"这一档（0.75） */
    CHECK_NEAR(out.amplitude_scale, 0.75, 1e-6);
#endif
    CHECK_EQ_U(out.severity, 2u);

    /* L3 + 过温 + 失联：三个原因都要出现在位域里 */
    safe_in(&in);
    in.batt_mv = 9000u;
    in.max_temp_c = 80u;
    in.link_level = 2u;
    rlca_safety_eval(&in, &out);
    CHECK_M((out.reasons & RLCA_SAFE_R_VOLT_FORCE) != 0u);
    CHECK_M((out.reasons & RLCA_SAFE_R_OVERTEMP) != 0u);
    CHECK_M((out.reasons & RLCA_SAFE_R_LINK_LOST) != 0u);
    CHECK_EQ_U(out.severity, 3u);
    CHECK_M(out.sit_down);
    /* ⚠️ 位域值 = 0x1F = 31，不是 0x19 = 25。
     *    原因：电池 9.0 V 同时 ≤ L1/L2/L3 三个阈值，而这三条判据
     *    【互相不排斥】，所以 VOLT_FORCE|VOLT_STOP|VOLT_WARN 三个位同时置起。
     *    这是刻意的：Pi 侧想知道"低到什么程度"，而不是只拿到一个级别。
     *    （我一开始手算成 25，漏了 STOP/WARN 两位。） */
    CHECK_EQ_U(out.reasons,
               (uint8_t)(RLCA_SAFE_R_VOLT_FORCE | RLCA_SAFE_R_VOLT_STOP |
                         RLCA_SAFE_R_VOLT_WARN | RLCA_SAFE_R_OVERTEMP |
                         RLCA_SAFE_R_LINK_LOST));
    CHECK_EQ_U(out.reasons, 31u);
    printf("    combo(volt_force|overtemp|link_lost) -> reasons=0x%02X severity=%u\n",
           (unsigned)out.reasons, (unsigned)out.severity);

    /* 最严重的原因存在时，限幅缩放必须是 0（不做任何主动动作） */
    CHECK_NEAR(out.amplitude_scale, 0.0, 1e-6);

    /* 操作者急停（预留位）单独也能触发 */
    safe_in(&in);
    in.operator_estop = true;
    rlca_safety_eval(&in, &out);
    CHECK_EQ_U(out.reasons, RLCA_SAFE_R_OPERATOR);
    CHECK_EQ_U(out.severity, 3u);
    CHECK_M(out.unload_torque);

    /* 原因名的查表接口 */
    CHECK(strcmp(rlca_safety_reason_name(0u /* VOLT_FORCE bit 0 */), "VOLT_FORCE") == 0,
          "reason name lookup");
}

/*==============================================================================
 * Part 2 —— 标定流程（假舵机总线）
 *============================================================================*/

/**
 * 假舵机总线：模拟 12 个真实舵机。
 *
 * ⚠️ 关键：它模拟的是【舵机行为】而不是"直接返回期望值"：
 *    · 目标位置要经过几个 poll 才"到位"（模拟舵机运动+稳定时间）
 *    · 位置反馈带 ±2 步的抖动（真实舵机就是这样）
 *    · dir = -1 的关节，编码器会朝反方向变化 —— 这正是标定要测出来的东西
 */
typedef struct {
    int16_t  phys[RLCA_N_JOINTS];     /* 物理步数（= 读位置接口的返回值） */
    int16_t  target[RLCA_N_JOINTS];   /* 物理目标                        */
    int16_t  cmd[RLCA_N_JOINTS];      /* 最后收到的命令值（步）           */

    int8_t   encoder_dir[RLCA_N_JOINTS];   /* 被标定要发现的真值          */

    bool     torque_on;
    bool     fail_read;
    bool     fail_write;
    bool     never_move;       /* 模拟"力矩没使能/机械卡死" */
    uint32_t ms;
    uint32_t n_reads;
    uint32_t n_writes;
} fake_bus_t;

static fake_bus_t bus;

static void bus_reset(void)
{
    memset(&bus, 0, sizeof(bus));
    bus.torque_on = false;      /* 上电默认没力矩，与真实情况一致 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* 站姿读数：模拟"弯腿站姿"下的位置，分散在 0..4095 里 */
        const int16_t p = (int16_t)(2048 + ((int32_t)i * 137) % 512 - 256);
        bus.phys[i]   = p;
        bus.target[i] = p;
        bus.cmd[i]    = p;
        /* 前 6 个关节 dir = +1，后 6 个 dir = -1 —— 覆盖两种情况 */
        bus.encoder_dir[i] = (i < 6u) ? 1 : -1;
    }
}

static bool bus_get_pos(uint8_t idx, int16_t *out)
{
    bus.n_reads++;
    if (bus.fail_read || (idx >= RLCA_N_JOINTS) || (out == NULL)) {
        return false;
    }
    /* 抖动 ±2 步（确定性伪随机，便于复现；真实舵机就是这样） */
    int16_t v = (int16_t)(bus.phys[idx] + (int16_t)(((bus.n_reads * 7u) % 5u) - 2));
    if (v < 0)    { v = (int16_t)(v + 4096); }
    if (v > 4095) { v = (int16_t)(v - 4096); }
    *out = v;
    return true;
}

static bool bus_set_pos(uint8_t idx, int16_t new_cmd)
{
    bus.n_writes++;
    if (bus.fail_write || (idx >= RLCA_N_JOINTS)) {
        return false;
    }
    /* ⚠️ 命令变化量 → 物理目标变化量，按编码器方向映射。
     *    enc_dir = +1：命令 +98 → 物理目标 +98
     *    enc_dir = −1：命令 +98 → 物理目标 −98  ← 标定要测出来的东西 */
    const int16_t dcmd = rlca_servo_pos_delta(bus.cmd[idx], new_cmd);
    int32_t t = (int32_t)bus.phys[idx] + ((int32_t)dcmd * (int32_t)bus.encoder_dir[idx]);
    while (t < 0)    { t += 4096; }
    while (t > 4095) { t -= 4096; }

    bus.target[idx] = (int16_t)t;
    bus.cmd[idx]    = new_cmd;
    return true;
}

static void bus_set_torque_all(bool enable)
{
    bus.torque_on = enable;
}

static uint32_t bus_millis(void)
{
    return bus.ms;
}

static const rlca_calib_hw_t bus_hw = {
    bus_get_pos, bus_set_pos, bus_set_torque_all, bus_millis
};

/** 推进假总线：让【物理位置】朝目标收敛，并推进时间。
 *
 *  ⚠️ 刻意不用 rlca_servo_pos_delta 做"到目标的误差"：
 *     它带 ±2048 的回绕上限，是给"相邻两次读数"判回绕用的；
 *     拿它当误差会在差半圈附近选错方向、绕远路。
 *     真实舵机内部是 PID 闭环，直接线性收敛到目标 —— 这里照此模拟。 */
static void bus_step(uint32_t dt_ms)
{
    bus.ms += dt_ms;
    if (!bus.torque_on || bus.never_move) {
        return;                 /* 没力矩 → 不动 */
    }
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        int32_t d = (int32_t)bus.target[i] - (int32_t)bus.phys[i];
        if (d == 0) {
            continue;
        }
        /* 每 10 ms 最多走 40 步（≈0.35 rad），像真实舵机的速度上限 */
        if (d > 40)  { d = 40; }
        if (d < -40) { d = -40; }
        int32_t v = (int32_t)bus.phys[i] + d;
        while (v < 0)    { v += 4096; }
        while (v > 4095) { v -= 4096; }
        bus.phys[i] = (int16_t)v;
    }
}

/** 跑标定直到结束或超时。返回消耗的模拟毫秒数。 */
static uint32_t run_calib(uint32_t max_ms)
{
    const uint32_t t0 = bus.ms;
    calib_proc_init(&bus_hw);
    calib_proc_start(NULL);

    uint32_t guard = 0u;
    while ((bus.ms - t0) < max_ms) {
        calib_proc_poll();
        bus_step(5u);                     /* 每次 poll 推进 5 ms 模拟时间 */
        const calib_state_t st = calib_proc_state();
        if ((st == CAL_DONE) || (st == CAL_FAILED)) {
            break;
        }
        if (++guard > 300000u) {
            printf("  !! run_calib guard tripped (state=%d)\n", (int)st);
            break;
        }
    }
    return bus.ms - t0;
}

static void test_calib_dir_logic(void)
{
    section("calib: direction decision from a position delta");

    CHECK_EQ_D(calib_dir_from_delta(98, 30), 1);
    CHECK_EQ_D(calib_dir_from_delta(-98, 30), -1);
    CHECK_EQ_D(calib_dir_from_delta(30, 30), 1);      /* 边界包含 */
    CHECK_EQ_D(calib_dir_from_delta(-30, 30), -1);
    CHECK_EQ_D(calib_dir_from_delta(29, 30), 0);      /* 不够 → 判不出 */
    CHECK_EQ_D(calib_dir_from_delta(-29, 30), 0);
    CHECK_EQ_D(calib_dir_from_delta(0, 30), 0);
    /* 非法阈值不应导致除零/误判 */
    CHECK_EQ_D(calib_dir_from_delta(1, 0), 1);
    CHECK_EQ_D(calib_dir_from_delta(1, -5), 1);
}

static void test_calib_validate(void)
{
    section("calib: result validation");

    rlca_calib_t c;
    rlca_calib_load_defaults(&c);
    CHECK_EQ_U(calib_validate_result(&c), CAL_ERR_NONE);


    c.dir[3] = 0;
    CHECK_EQ_U(calib_validate_result(&c), CAL_ERR_DIR_AMBIGUOUS);

    /* offset 离谱 → 限位错误 */
    rlca_calib_load_defaults(&c);
    c.offset[5] = 30000;
    CHECK_EQ_U(calib_validate_result(&c), CAL_ERR_LIMIT);

    /* magic 错 → 不合法 */
    rlca_calib_load_defaults(&c);
    c.magic = 0u;
    CHECK_EQ_U(calib_validate_result(&c), CAL_ERR_LIMIT);

    CHECK_EQ_U(calib_validate_result(NULL), CAL_ERR_LIMIT);
}

/** 把假总线摆到【标定姿态】。
 *  step_cal = dir × q_calib × SPR + stand_step（环绕到 0..4095）
 *  ⚠️ 真实世界的对应动作：人把四条腿掰到"伸直"姿态。 */
static void bus_apply_calib_pose(void)
{
    static const float q_calib[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int32_t dq = (int32_t)((int32_t)bus.encoder_dir[i] *
                                     q_calib[i] * RLCA_SERVO_STEPS_PER_RAD);
        int32_t p = (int32_t)bus.phys[i] + dq;
        p %= 4096;
        if (p < 0) { p += 4096; }
        bus.phys[i]   = (int16_t)p;
        bus.target[i] = (int16_t)p;
        /* ⚠️ cmd 要跟着走：bus_set_pos 是相对 cmd 算位移的 */
        bus.cmd[i]    = (int16_t)p;
    }
}

static void test_calib_happy_path(void)
{
    section("calib: full run against a fake servo bus");

    bus_reset();
    bus_apply_calib_pose();          /* ⭐ 标定是在"腿伸直"姿态下做的 */
    const uint32_t elapsed = run_calib(60000u);

    CHECK_EQ_U(calib_proc_state(), CAL_DONE);
    CHECK_EQ_U(calib_proc_err(), CAL_ERR_NONE);
    CHECK_EQ_U(calib_proc_progress(), 100u);
    printf("    calibration took %u ms (simulated), reads=%u writes=%u\n",
           (unsigned)elapsed, (unsigned)bus.n_reads, (unsigned)bus.n_writes);

    rlca_calib_t res;
    CHECK(calib_proc_take_done(&res), "must be able to take the result");
    CHECK(!calib_proc_take_done(&res), "result must be take-once");

    /* ⭐ 方向不再由标定"测量"，而是直接取自配置表 RLCA_JOINT_DIR_INIT
     *    （项目负责人实测给出）。这里验证流程确实把它落进了结果。 */
    static const int8_t expect_dir[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
    uint8_t dir_ok = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        if (res.dir[i] == expect_dir[i]) {
            dir_ok++;
        } else {
            printf("    dir mismatch at joint %u: got %d want %d\n",
                   (unsigned)i, (int)res.dir[i], (int)expect_dir[i]);
        }
    }
    CHECK_EQ_U(dir_ok, RLCA_N_JOINTS);

    /* ⭐⭐⭐ offset 的语义（2026-10-16 定案）：
     *
     *   offset[i] = 【URDF 零位（弯腿站姿）】下舵机 i 的编码器步数
     *
     *   而采样到的 base[] 是【标定姿态（腿伸直）】下的读数 —— 两者
     *   相差 dir × q_calib × SPR（这正是"转换"本身）。
     *
     *   ⇒ 判据必须写成：
     *         q_to_step(q_calib) ≡ base        （标定姿态，容差 ±8）
     *         q_to_step(0)       ≡ offset      （零位，恒等）
     *
     *   ⚠️ 若这里写成 "offset == base"，那只在 q_calib = 0（标定姿态就是
     *      零位）时才成立 —— 那是本项目【曾经】的错误模型。
     *      新模型下两者【本来就不等】，差值正好是转换量。 */
    const calib_proc_t *pi = calib_proc_info();
    static const float q_calib[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;

    uint8_t off_ok = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* 标定姿态：q_to_step(q_calib) 必须回到采样读数 */
        int32_t sc = (int32_t)rlca_q_to_step(&res, i, q_calib[i]) % 4096;
        if (sc < 0) { sc += 4096; }
        int32_t err = sc - (int32_t)pi->base[i];
        if (err >  2048) { err -= 4096; }
        if (err < -2048) { err += 4096; }

        if ((err >= -8) && (err <= 8)) {
            off_ok++;
        } else {
            printf("    offset mismatch at joint %u: q_to_step(q_cal)=%d, base %d (err %d)\n",
                   (unsigned)i, (int)sc, (int)pi->base[i], (int)err);
        }
    }
    CHECK_EQ_U(off_ok, RLCA_N_JOINTS);

    /* ⭐ 恒等式：q_to_step(0) 必须等于 offset（定义如此，容差 ±1） */
    uint8_t rt_ok = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        int32_t s = (int32_t)rlca_q_to_step(&res, i, 0.0f) % 4096;
        if (s < 0) { s += 4096; }
        int32_t off = (int32_t)res.offset[i] % 4096;
        if (off < 0) { off += 4096; }
        int32_t err = s - off;
        if (err >  2048) { err -= 4096; }
        if (err < -2048) { err += 4096; }
        if ((err >= -1) && (err <= 1)) { rt_ok++; }
        else {
            printf("    identity fail joint %u: q_to_step(0)=%d offset=%d\n",
                   (unsigned)i, (int)s, (int)off);
        }
    }
    CHECK_EQ_U(rt_ok, RLCA_N_JOINTS);

    /* 结构必须自洽 */
    CHECK(rlca_calib_is_valid(&res), "result must be structurally valid");
    CHECK_EQ_U(res.crc16, rlca_calib_crc(&res));
    CHECK_EQ_U(res.magic, RLCA_CALIB_MAGIC);
    CHECK_EQ_U(res.version, RLCA_CALIB_VERSION);

    /* ⚠️ limit_* 必须被保留（来自 URDF，不是标定出来的） */
    rlca_calib_t def;
    rlca_calib_load_defaults(&def);
    uint8_t lim_ok = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        if ((res.limit_min[i] == def.limit_min[i]) &&
            (res.limit_max[i] == def.limit_max[i])) {
            lim_ok++;
        }
    }
    CHECK_EQ_U(lim_ok, RLCA_N_JOINTS);

    /* 标定结束时力矩应被卸掉（避免保持别扭姿态） */
    CHECK(!bus.torque_on, "calibration must end with torque off");
}

/*==============================================================================
 * ⭐ 标定后 q_to_step(0) 必须回到采样位置（2026-10-16 简化后）
 *
 *  背景：标定姿态 = 装配姿态 = q 0 （项目负责人定义），
 *        所以 q_calib 全为 0，offset 就是采样读数本身。
 *
 *  ⚠️ 这个用例锁住两件事：
 *     · offset 确实来自"采样读数"，没有被任何反推/换算污染
 *     · 标定表能把 q = 0 无损换算回标定时的物理步数
 *       （若换算出错，标定完一进闭环就是大幅运动）
 *============================================================================*/

static void test_calib_offset_equals_sampled_position(void)
{
    section("calib: offset must equal the sampled position (calib pose == q0)");

    bus_reset();
    bus_apply_calib_pose();

    const uint32_t ms = run_calib(200000u);
    (void)ms;
    CHECK_EQ_U(calib_proc_state(), CAL_DONE);

    rlca_calib_t res;
    CHECK(calib_proc_take_done(&res), "must hand out the result");

    const calib_proc_t *pi = calib_proc_info();

    /* ⚠️ 容差取 ±8 步：假总线给位置加 ±2 步抖动，而 offset 与 base 都用
     *    【整数除法】截断（8 个值的平均），最大截断误差 < 1 步，
     *    叠加抖动余项后留 8 步余量足够，同时仍能抓住"偏一个读数"（上千步）。 */

    static const float q_calib[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;

    /* ⭐⭐⭐ 核心语义（2026-10-16 定案）：
     *
     *   标定姿态 = 腿伸直（上位机设成 2048），它的关节角是 q_calib ≠ 0；
     *   URDF 零位 = 弯腿站姿，它的关节角才是 0。
     *
     *   ⇒ 由  step(q) = dir × q × SPR + offset
     *      ① q_to_step(q_calib) 必须 == 采样到的 base（= 标定姿态读数）
     *      ② q_to_step(0)       必须 == offset（模 4096）
     *
     *   ⚠️ 这一条正好把"转换有没有做对"钉死：若漏掉 q_calib 那一段，
     *      ① 会偏 dir×q_calib×SPR（可达 1024 步），立刻被抓。 */
    uint8_t ok_at_cal = 0u;
    uint8_t ok_at_zero = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* ① 标定姿态 */
        int32_t sc = (int32_t)rlca_q_to_step(&res, i, q_calib[i]) % 4096;
        if (sc < 0) { sc += 4096; }
        int32_t e1 = sc - (int32_t)pi->base[i];
        if (e1 >  2048) { e1 -= 4096; }
        if (e1 < -2048) { e1 += 4096; }
        if ((e1 >= -8) && (e1 <= 8)) { ok_at_cal++; }
        else {
            printf("    joint %u: q_to_step(q_calib)=%d but sampled base=%d (err %d)\n",
                   (unsigned)i, (int)sc, (int)pi->base[i], (int)e1);
        }

        /* ② URDF 零位（弯腿站姿）→ 就是 offset 本身 */
        int32_t s0 = (int32_t)rlca_q_to_step(&res, i, 0.0f) % 4096;
        if (s0 < 0) { s0 += 4096; }
        int32_t off = (int32_t)res.offset[i] % 4096;
        if (off < 0) { off += 4096; }
        int32_t e2 = s0 - off;
        if (e2 >  2048) { e2 -= 4096; }
        if (e2 < -2048) { e2 += 4096; }
        if ((e2 >= -1) && (e2 <= 1)) { ok_at_zero++; }
        else {
            printf("    joint %u: q_to_step(0)=%d but offset=%d (err %d)\n",
                   (unsigned)i, (int)s0, (int)off, (int)e2);
        }
    }
    CHECK_EQ_U(ok_at_cal, RLCA_N_JOINTS);
    CHECK_EQ_U(ok_at_zero, RLCA_N_JOINTS);

    /* ⭐ dir 必须来自配置表（不再测量） */
    static const int8_t expect_dir[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK_EQ_D(res.dir[i], expect_dir[i]);
    }

    /* ⭐ q_calib 必须【非 0】（标定姿态 ≠ URDF 零位）；
     *    若全 0，说明配置被改回了"标定姿态 == 零位"的错误模型，
     *    上面 ① 的判别力就消失了。 */
    uint8_t nz = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        if (q_calib[i] != 0.0f) { nz++; }
    }
    CHECK_EQ_U(nz, 8u);   /* 4 个 thigh + 4 个 calf；hip 全 0 */

    /* ⑤ offset 必须落在合法范围内（±4096） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        CHECK(rlca_calib_offset_ok(res.offset[i]),
              "joint %u offset %d out of range", (unsigned)i, (int)res.offset[i]);
    }
}

static void test_calib_failure_paths(void)
{
    section("calib: failure paths must be caught, not silently produce garbage");

    /* ---- 读位置失败 ---- */
    bus_reset();
    bus.fail_read = true;
    (void)run_calib(30000u);
    CHECK_EQ_U(calib_proc_state(), CAL_FAILED);
    CHECK_EQ_U(calib_proc_err(), CAL_ERR_READ_POS);
    printf("    fail_read -> err=%d\n", (int)calib_proc_err());

    /* ⚠️ "舵机不动"与"写位置失败"两条失败路径已随方向测试一起删除
     *    （2026-10-16）：标定全程不再上力矩、不再写位置，
     *    所以这两个失败模式在标定流程里已经不存在了。
     *    ⚠️ 这【不是】降低了安全性 —— 恰恰相反：不动任何关节
     *       就消除了"标定期间撞机"这个最大风险源。 */

    /* ---- 失败之后必须能重新开始（不能锁死） ---- */
    bus_reset();
    const uint32_t e2 = run_calib(60000u);
    CHECK_EQ_U(calib_proc_state(), CAL_DONE);
    printf("    restart after failure -> DONE in %u ms\n", (unsigned)e2);
}

static void test_calib_cancel_and_idle(void)
{
    section("calib: cancel and idle safety");

    bus_reset();
    calib_proc_init(&bus_hw);

    /* IDLE 时 poll 不应做任何事 */
    for (uint8_t i = 0u; i < 100u; ++i) {
        calib_proc_poll();
        bus_step(5u);
    }
    CHECK_EQ_U(calib_proc_state(), CAL_IDLE);
    CHECK_EQ_U(bus.n_writes, 0u);
    CHECK_EQ_U(calib_proc_progress(), 0u);
    CHECK_M(!calib_proc_take_done(NULL));

    /* 中途取消：必须立刻回 IDLE，且不再动舵机 */
    calib_proc_start(NULL);
    calib_proc_poll();
    bus_step(5u);
    CHECK_M(calib_proc_state() != CAL_IDLE);
    calib_proc_cancel();
    CHECK_EQ_U(calib_proc_state(), CAL_IDLE);

    const uint32_t w_after_cancel = bus.n_writes;
    for (uint8_t i = 0u; i < 50u; ++i) {
        calib_proc_poll();
        bus_step(5u);
    }
    CHECK_EQ_U(bus.n_writes, w_after_cancel);
    CHECK_EQ_U(calib_proc_state(), CAL_IDLE);

    /* 端口为空不能崩 */
    calib_proc_init(NULL);
    calib_proc_start(NULL);
    calib_proc_poll();
    CHECK(true, "NULL hw must not crash");
}

/*==============================================================================
 * main
 *============================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT safety + calibration unit tests\n");
    printf("========================================\n");
    fflush(stdout);

    test_safety_nominal();
    test_safety_link_loss();
    test_safety_battery();
    test_safety_servo_and_imu();
    test_safety_combinations_and_precedence();

    test_calib_dir_logic();
    test_calib_validate();
    test_calib_happy_path();
    test_calib_offset_equals_sampled_position();
    test_calib_failure_paths();
    test_calib_cancel_and_idle();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
