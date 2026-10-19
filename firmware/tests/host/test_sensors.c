/**
  ******************************************************************************
  * @file    test_sensors.c
  * @brief   P5 单元测试：battery（ADC → mV → 分级）与 imu（JY901P 解析）。
  *
  *  两部分都通过注入端口驱动，所以不需要硬件就能确定性地验证：
  *    battery：分压换算、阈值分级、边界值、滑动平均、失效判定、采样超时
  *    imu    ：大端解析、欧拉角→四元数、换算式、假 JY901 端到端、
  *             读失败、离群值保护、失效判定
  *
  *  ⚠️ IMU 的寄存器映射未经核实（reference/ 里没有 JY901P 手册），
  *     所以这里测的是"映射给定之后的解析与换算是否正确"，
  *     而不是"映射本身对不对"。映射的核对在 P5-2 上机验收里做。
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "battery.h"
#include "imu.h"

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
 * Part 1 —— battery
 *============================================================================*/

static void test_batt_conversion(void)
{
    section("battery: ADC -> mV with the documented divider");

    /* ⚠️ 本函数测的是【换算公式本身】，所以故意用一组任意的比值 122/22
     *    （vref 3300 mV），与硬件实际的 120/20 无关 —— 见下面第 120 行附近的说明。
     * 期望：电池mV = raw/4095 × 3300 × 122/22 */
    const float vref = 3300.0f;
    const float num = 122.0f;
    const float den = 22.0f;

    /* 反推几个关键点的 ADC 读数，再验算回电压 */
    struct { uint16_t raw; uint16_t want_mv; } cases[] = {
        /* 12.6 V 满电：adc = 12.6 × 22/122 = 2.2721 V → raw = 2272/3300×4095 = 2819 */
        { 2819u, 12600u },
        /* 10.5 V（L1 阈值）：adc = 1.8934 V → raw = 2350 */
        { 2350u, 10500u },
        /* 9.9 V（L2）：adc = 1.7852 V → raw = 2215 */
        { 2215u, 9900u },
        /* 9.3 V（L3）：adc = 1.6770 V → raw = 2081 */
        { 2081u, 9300u },
        /* 0 V：raw 0 → 0 mV */
        { 0u, 0u },
    };

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(cases) / sizeof(cases[0])); ++i) {
        const uint16_t got = batt_adc_to_mv(cases[i].raw, vref, num, den);
        /* 允许 ±30 mV：raw 本身是量化过的（1 LSB ≈ 4.5 mV 电池电压） */
        CHECK(abs((int)got - (int)cases[i].want_mv) <= 30,
              "raw %u -> %u mV (want ~%u)", (unsigned)cases[i].raw,
              (unsigned)got, (unsigned)cases[i].want_mv);
    }

    /* 满量程 */
    const uint16_t full = batt_adc_to_mv(4095u, vref, num, den);
    printf("    raw 4095 -> %u mV\n", (unsigned)full);
    CHECK(full > 18000u && full < 18400u,
          "full scale should be ~18300 mV (got %u)", (unsigned)full);

    /* 超量程必须被钳住而不是回绕 */
    CHECK_EQ_U(batt_adc_to_mv(65535u, vref, num, den), full);

    /* 非法参数不能崩、不能返回垃圾 */
    CHECK_EQ_U(batt_adc_to_mv(1000u, 0.0f, num, den), 0u);
    CHECK_EQ_U(batt_adc_to_mv(1000u, vref, num, 0.0f), 0u);

    /* 默认参数版本应与显式调用一致 */
    /* ⚠️ 必须用【配置里】的分压比来对照 —— 上面那组 122/22 是"函数数学"测试，
     *    而默认参数走的是 RLCA_BATT_DIV_*（实测硬件 100k/20k = 6.000）。
     *    两者不是一回事，混用会让本断言在改硬件后失败。（2026-10-18 修） */
    CHECK_EQ_U(batt_adc_to_mv_default(2000u),
               batt_adc_to_mv(2000u, vref, RLCA_BATT_DIV_NUM, RLCA_BATT_DIV_DEN));
}

static void test_batt_thresholds(void)
{
    section("battery: level classification at exact thresholds");

    /* ⚠️ 阈值语义是"≤ 触发"，所以阈值本身必须归到更低一级。 */
    CHECK_EQ_U(batt_classify(10501u), BATT_LEVEL_OK);
    CHECK_EQ_U(batt_classify(10500u), BATT_LEVEL_WARN);      /* L1 边界 */
    CHECK_EQ_U(batt_classify(10499u), BATT_LEVEL_WARN);

    CHECK_EQ_U(batt_classify(9901u), BATT_LEVEL_WARN);
    CHECK_EQ_U(batt_classify(9900u), BATT_LEVEL_STOP);       /* L2 边界 */
    CHECK_EQ_U(batt_classify(9899u), BATT_LEVEL_STOP);

    CHECK_EQ_U(batt_classify(9301u), BATT_LEVEL_STOP);
    CHECK_EQ_U(batt_classify(9300u), BATT_LEVEL_FORCE);      /* L3 边界 */
    CHECK_EQ_U(batt_classify(9299u), BATT_LEVEL_FORCE);
    CHECK_EQ_U(batt_classify(0u), BATT_LEVEL_FORCE);

    /* 高电压保持 OK */
    CHECK_EQ_U(batt_classify(12600u), BATT_LEVEL_OK);
    CHECK_EQ_U(batt_classify(65535u), BATT_LEVEL_OK);

    /* ---- status 位域：必须与级别一致，且可以同时置多位 ---- */
    CHECK_EQ_U(batt_status_bits(12000u), 0u);
    CHECK_EQ_U(batt_status_bits(10500u), RLCA_ST_BATT_WARN);
    CHECK_EQ_U(batt_status_bits(9900u), RLCA_ST_BATT_WARN | RLCA_ST_BATT_STOP);
    CHECK_EQ_U(batt_status_bits(9300u),
               RLCA_ST_BATT_WARN | RLCA_ST_BATT_STOP | RLCA_ST_BATT_FORCE);
    CHECK_EQ_U(batt_status_bits(8000u),
               RLCA_ST_BATT_WARN | RLCA_ST_BATT_STOP | RLCA_ST_BATT_FORCE);
}

/* ---- 假 ADC 端口 ---- */

static uint16_t s_fake_raw = 2819u;
static bool     s_fake_ok = true;
static uint32_t s_fake_ms = 0u;

static bool batt_read_raw(uint16_t *raw)
{
    if (!s_fake_ok) {
        return false;
    }
    *raw = s_fake_raw;
    return true;
}

static uint32_t batt_millis(void)
{
    return s_fake_ms;
}

static const batt_port_t batt_port = { batt_read_raw, batt_millis };

static void test_batt_state(void)
{
    section("battery: sampling, filtering, staleness (fake ADC)");

    s_fake_raw = 2819u;      /* ≈12.6 V */
    s_fake_ok = true;
    s_fake_ms = 0u;

    battery_init(&batt_port);
    CHECK(!battery_is_valid(), "must start invalid before any sample");
    CHECK_EQ_U(battery_mv(), 0u);

    /* 第一次 poll：s_last_poll_ms 初始为 0，应立刻采样 */
    battery_poll();
    CHECK(battery_is_valid(), "must become valid after the first sample");
    CHECK_EQ_U(battery_state()->adc_raw, 2819u);
    CHECK_EQ_D(battery_state()->level, BATT_LEVEL_OK);
    /* ⚠️ raw 2819 在 100k/20k（比值 6.000）下是 ~13630 mV，
     *    在原以为的 100k/22k（5.545）下才是 ~12600 mV。
     *    分压比更正后这里必须改，否则测的就不是真硬件的行为。（2026-10-18） */
    CHECK(battery_mv() > 13400u && battery_mv() < 13800u,
          "voltage should be ~13.63 V with the 100k/20k divider (got %u mV)",
          (unsigned)battery_mv());

    /* 10 Hz 节流：100 ms 内的重复调用不应再采样 */
    const uint32_t n1 = battery_state()->n_samples;
    for (uint32_t t = 0u; t < 90u; t += 10u) {
        s_fake_ms = t;
        battery_poll();
    }
    CHECK_EQ_U(battery_state()->n_samples, n1);
    printf("    samples after 90 ms of polling = %u\n", (unsigned)battery_state()->n_samples);

    /* 超过 100 ms 应采样一次 */
    s_fake_ms = 100u;
    battery_poll();
    CHECK_EQ_U(battery_state()->n_samples, n1 + 1u);

    /* ---- 采样失败 → 计数增长，但短期内仍算有效 ---- */
    s_fake_ok = false;
    const uint32_t t1 = battery_state()->n_timeouts;
    s_fake_ms += RLCA_BATT_SAMPLE_PERIOD_MS;
    battery_poll();
    CHECK_EQ_U(battery_state()->n_timeouts, t1 + 1u);
    CHECK(battery_is_valid(), "a single failed sample must not invalidate immediately");

    /* ---- 持续失败超过 STALE → 失效 ---- */
    s_fake_ms += (RLCA_BATT_STALE_MS + RLCA_BATT_SAMPLE_PERIOD_MS);
    battery_poll();
    CHECK(!battery_is_valid(), "must go invalid after the stale window");

    /* ---- 恢复后重新有效 ---- */
    s_fake_ok = true;
    s_fake_ms += RLCA_BATT_SAMPLE_PERIOD_MS;
    battery_poll();
    CHECK(battery_is_valid(), "must recover once sampling works again");

    /* ---- 滑动平均：喂一串跳变值，输出应被平滑 ---- */
    battery_init(&batt_port);
    s_fake_ok = true;
    s_fake_ms = 0u;
    s_fake_raw = 2200u;
    battery_poll();
    const uint16_t v_first = battery_mv();

    /* 突然跳到 2819，单看 mv_raw 会立刻变大，但 mv 只动 1/4 */
    s_fake_raw = 2819u;
    s_fake_ms += RLCA_BATT_SAMPLE_PERIOD_MS;
    battery_poll();
    const uint16_t v_after1 = battery_mv();
    printf("    filter step: %u -> %u mV (raw jumped to %u)\n",
           (unsigned)v_first, (unsigned)v_after1,
           (unsigned)battery_state()->mv_raw);

    CHECK(v_after1 > v_first, "filtered value must move toward the new sample");
    CHECK(v_after1 < battery_state()->mv_raw,
          "filtered value must lag the raw value (got %u vs raw %u)",
          (unsigned)v_after1, (unsigned)battery_state()->mv_raw);

    /* 连续 4 次之后应完全跟上 */
    for (uint8_t i = 0u; i < RLCA_BATT_FILTER_LEN; ++i) {
        s_fake_ms += RLCA_BATT_SAMPLE_PERIOD_MS;
        battery_poll();
    }
    CHECK_EQ_U(battery_mv(), battery_state()->mv_raw);

    /* ---- 触发 L3 的电压应被正确分级 ---- */
    battery_init(&batt_port);
    s_fake_ms = 0u;
    /* ⚠️ 9.3 V 在新分压比下：adc = 9.3×20/120 = 1.55 V → raw = 1550/3300×4095 ≈ 1923
     *    （旧的 2081 是按 5.545 算的，改比值后会读成 10.06 V，落不到 FORCE） */
    s_fake_raw = 1923u;      /* ≈9.3 V */
    for (uint8_t i = 0u; i < RLCA_BATT_FILTER_LEN; ++i) {
        s_fake_ms += RLCA_BATT_SAMPLE_PERIOD_MS;
        battery_poll();
    }
    CHECK_EQ_D(battery_level(), BATT_LEVEL_FORCE);
    CHECK((batt_status_bits(battery_mv()) & RLCA_ST_BATT_FORCE) != 0u,
          "L3 must set STAT_BATT_FORCE (mv=%u)", (unsigned)battery_mv());
}

static void test_batt_calibration(void)
{
    section("battery: calibration override");

    battery_init(&batt_port);
    /* 标定前 */
    const uint16_t before = batt_adc_to_mv_default(2819u);

    /* 把 vref 从 3300 改成 3260（实测常见的偏低值），读数应按比例变化 */
    battery_set_calibration(3260.0f, 0.0f, 0.0f);   /* 只改 vref */
    battery_init(&batt_port);
    s_fake_raw = 2819u;
    s_fake_ok = true;
    s_fake_ms = 0u;
    battery_poll();
    const uint16_t after = battery_mv();

    printf("    vref 3300 -> %u mV ; vref 3260 -> %u mV\n",
           (unsigned)before, (unsigned)after);
    CHECK(after < before, "lower vref must read lower");

    /* 恢复默认，避免影响后续用例 */
    battery_set_calibration(RLCA_ADC_VREF_MV, RLCA_BATT_DIV_NUM, RLCA_BATT_DIV_DEN);
}

/*==============================================================================
 * Part 2 —— imu
 *============================================================================*/

static void test_imu_pure(void)
{
    section("imu: big-endian parsing and unit conversion");

    /* ⚠️ WIT 模块用大端，这是最容易搞混的地方 */
    /* ⚠️ 小端：低字节在前。0x0800 = 2048 -> 字节序 [0x00, 0x08]
     *    上机实测确认 JY901P 走小端（详见 imu.c 的 imu_i16 注释）。 */
    const uint8_t le_pos[2] = { 0x00, 0x08 };
    CHECK_EQ_D(imu_i16(le_pos), 2048);
    const uint8_t le_neg[2] = { 0x00, 0xF8 };
    CHECK_EQ_D(imu_i16(le_neg), -2048);
    const uint8_t le_ff[2] = { 0xFF, 0xFF };
    CHECK_EQ_D(imu_i16(le_ff), -1);
    const uint8_t le_one[2] = { 0x01, 0x00 };
    CHECK_EQ_D(imu_i16(le_one), 1);

    /* 满量程换算 */
    CHECK_NEAR(imu_raw_to_fs(32767, 16.0f), 16.0f, 1e-3);
    CHECK_NEAR(imu_raw_to_fs(-32768, 16.0f), -16.0f, 1e-3);
    CHECK_NEAR(imu_raw_to_fs(0, 16.0f), 0.0f, 1e-9);

    /* 角速度：±2000 °/s 满量程 → 32767 应约等于 34.9066 rad/s */
    const float gmax = imu_gyro_to_rad_s(32767);
    printf("    gyro full scale = %.4f rad/s\n", (double)gmax);
    CHECK_NEAR(gmax, 2000.0 * M_PI / 180.0, 0.01);
    /* 1 rad/s ≈ 57.2958 °/s → raw = 57.2958/2000×32768 = 938.7 */
    CHECK_NEAR(imu_gyro_to_rad_s(939), 1.0, 0.002);

    /* 加速度：±16 g */
    CHECK_NEAR(imu_accel_to_g(32767), 16.0, 1e-3);
    /* 1 g → raw = 32768/16 = 2048 */
    CHECK_NEAR(imu_accel_to_g(2048), 1.0, 1e-4);

    /* ⭐ 角度：手册是 raw/32768 × 180°（int16 满量程 = ±180°）
     *    → 180° 对应 raw = 32768；90° 对应 16384；1° 对应 182.04 */
    CHECK_NEAR(imu_angle_to_rad(16384), M_PI / 2.0, 1e-5);      /* 90°  */
    CHECK_NEAR(imu_angle_to_rad(32767), M_PI, 1e-4);            /* ≈180° */
    CHECK_NEAR(imu_angle_to_rad(-16384), -M_PI / 2.0, 1e-5);    /* −90° */
    CHECK_NEAR(imu_angle_to_rad(0), 0.0, 1e-9);

    /* ⚠️ 回归断言：早期版本误按 0.01°/LSB 换算，把 90° 算成了 164°（2.877 rad）。
     *    这里钉死"raw 16384 必须是 90°"（≈1.5708 rad），防止那个错误被改回来。
     *    容差取 0.05 rad 足够宽，但离错误值 2.877 差得极远，判别力充足。 */
    CHECK_NEAR(imu_angle_to_rad(16384), 1.5708, 0.05);
    CHECK(fabs((double)imu_angle_to_rad(16384) - 2.877) > 1.0,
          "raw 16384 must NOT be ~164 deg - did the 0.01deg/LSB bug come back? (got %.4f rad)",
          (double)imu_angle_to_rad(16384));
}

/** 造一个"角度寄存器"缓冲：6 字节（Roll/Pitch/Yaw 各 int16 大端）。
 *  ⚠️ 编码必须与固件的解码一致：raw = deg / 180 × 32768。
 *     早期版本这里也按 0.01° 编，等于"用同一个错误假设测自己"，
 *     所以那个 1.82 倍的换算错误一直没被单测发现。
 *     现在按手册编码，固件改错就会立刻红。 */
static void build_angle_buf(uint8_t buf[6], float roll_deg, float pitch_deg, float yaw_deg)
{
    const int16_t r = (int16_t)lroundf(roll_deg / 180.0f * 32768.0f);
    const int16_t p = (int16_t)lroundf(pitch_deg / 180.0f * 32768.0f);
    const int16_t y = (int16_t)lroundf(yaw_deg / 180.0f * 32768.0f);
    /* ⚠️ 小端写出（低字节在前），与固件的 imu_i16 一致，也与实测一致。 */
    buf[0] = (uint8_t)((uint16_t)r & 0xFFu); buf[1] = (uint8_t)((uint16_t)r >> 8);
    buf[2] = (uint8_t)((uint16_t)p & 0xFFu); buf[3] = (uint8_t)((uint16_t)p >> 8);
    buf[4] = (uint8_t)((uint16_t)y & 0xFFu); buf[5] = (uint8_t)((uint16_t)y >> 8);
}

static void test_imu_quat(void)
{
    section("imu: euler -> quaternion (the gravity direction depends on this)");

    float q[4];
    float e[3];
    uint8_t buf[6];

    /* ---- 全零姿态 → 单位四元数 ---- */
    build_angle_buf(buf, 0.0f, 0.0f, 0.0f);
    imu_parse_angle_to_quat(buf, q, e);
    CHECK_NEAR(q[0], 1.0, 1e-6);
    CHECK_NEAR(q[1], 0.0, 1e-6);
    CHECK_NEAR(q[2], 0.0, 1e-6);
    CHECK_NEAR(q[3], 0.0, 1e-6);

    /* ---- 绕 X 轴 90°（roll）：w=x=cos45 ---- */
    build_angle_buf(buf, 90.0f, 0.0f, 0.0f);
    imu_parse_angle_to_quat(buf, q, e);
    CHECK_NEAR(q[0], 0.70710678, 1e-4);
    CHECK_NEAR(q[1], 0.70710678, 1e-4);
    CHECK_NEAR(q[2], 0.0, 1e-4);
    CHECK_NEAR(q[3], 0.0, 1e-4);

    /* ---- 绕 Y 轴 90°（pitch） ---- */
    build_angle_buf(buf, 0.0f, 90.0f, 0.0f);
    imu_parse_angle_to_quat(buf, q, e);
    CHECK_NEAR(q[0], 0.70710678, 1e-4);
    CHECK_NEAR(q[1], 0.0, 1e-4);
    CHECK_NEAR(q[2], 0.70710678, 1e-4);
    CHECK_NEAR(q[3], 0.0, 1e-4);

    /* ---- 绕 Z 轴 90°（yaw） ---- */
    build_angle_buf(buf, 0.0f, 0.0f, 90.0f);
    imu_parse_angle_to_quat(buf, q, e);
    CHECK_NEAR(q[0], 0.70710678, 1e-4);
    CHECK_NEAR(q[1], 0.0, 1e-4);
    CHECK_NEAR(q[2], 0.0, 1e-4);
    CHECK_NEAR(q[3], 0.70710678, 1e-4);

    /* ---- 四元数必须始终是单位长度 ---- */
    for (int16_t r = -180; r <= 180; r += 37) {
        for (int16_t p = -90; p <= 90; p += 23) {
            build_angle_buf(buf, (float)r, (float)p, 45.0f);
            imu_parse_angle_to_quat(buf, q, e);
            const double n = sqrt((double)(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]));
            CHECK(fabs(n - 1.0) < 1e-5, "quaternion must be unit (got %.8f at r=%d p=%d)",
                  n, (int)r, (int)p);
        }
    }

    /* ---- ⭐ 与训练侧的约定核对：机身【水平】时，重力向量应为 (0,0,-1) ---- */
    build_angle_buf(buf, 0.0f, 0.0f, 0.0f);
    imu_parse_angle_to_quat(buf, q, e);
    {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        const double gx = -2.0 * (x * z - w * y);
        const double gy = -2.0 * (y * z + w * x);
        const double gz = -(1.0 - 2.0 * (x * x + y * y));
        printf("    level attitude -> gravity (%.4f, %.4f, %.4f)\n", gx, gy, gz);
        CHECK_NEAR(gx, 0.0, 1e-5);
        CHECK_NEAR(gy, 0.0, 1e-5);
        CHECK_NEAR(gz, -1.0, 1e-5);
    }

    /* ---- 俯仰 90° 时机身系重力应完全落到 X 轴 ---- */
    build_angle_buf(buf, 0.0f, 90.0f, 0.0f);
    imu_parse_angle_to_quat(buf, q, e);
    {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        const double gx = -2.0 * (x * z - w * y);
        const double gz = -(1.0 - 2.0 * (x * x + y * y));
        printf("    pitch 90deg -> gravity (%.4f, _, %.4f)\n", gx, gz);
        CHECK_NEAR(gz, 0.0, 1e-4);
        CHECK_NEAR(fabs(gx), 1.0, 1e-4);
    }

    /* euler 输出应回显输入 */
    build_angle_buf(buf, 30.0f, -20.0f, 10.0f);
    imu_parse_angle_to_quat(buf, q, e);
    CHECK_NEAR(e[0], 30.0 * M_PI / 180.0, 1e-4);
    CHECK_NEAR(e[1], -20.0 * M_PI / 180.0, 1e-4);
    CHECK_NEAR(e[2], 10.0 * M_PI / 180.0, 1e-4);

    /* NULL 不能崩 */
    imu_parse_angle_to_quat(NULL, q, e);
    imu_parse_angle_to_quat(buf, NULL, e);
    CHECK(true, "NULL arguments must not crash");
}

/* ---- 假 JY901P ---- */

typedef struct {
    uint16_t version;         /* VERSION@0x2E 的返回值（2 字节大端）  */
    bool     fail_acc;
    bool     fail_gyro;
    bool     fail_ang;
    bool     fail_version;
    int16_t  acc[3];
    int16_t  gyro[3];
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;
    uint32_t n_reads;
    uint8_t  last_reg;
} fake_jy_t;

static fake_jy_t jy;

static void jy_reset(void)
{
    memset(&jy, 0, sizeof(jy));
    /* ⚠️ VERSION 不校验固定值（手册：不同产品版本号不一样），
     *    所以这里随便给个非零值即可。 */
    jy.version = 0x0102u;
    jy.acc[2] = 2048;         /* 1 g 在 Z 轴（水平静止） */
}

static bool jy_read_reg(uint8_t dev, uint8_t reg, uint8_t *dst, uint8_t len)
{
    (void)dev;
    jy.n_reads++;
    jy.last_reg = reg;

    /* ⚠️ 版本寄存器是 0x2E，不是 0x00（0x00 是 SAVE，可写，读它无意义） */
    if (reg == RLCA_IMU_REG_VERSION) {
        if (jy.fail_version || (len != 2u)) { return false; }
        dst[0] = (uint8_t)(jy.version & 0xFFu);   /* 小端 */
        dst[1] = (uint8_t)(jy.version >> 8);
        return true;
    }

    if (reg == RLCA_IMU_REG_ACC) {
        if (jy.fail_acc || (len != 6u)) { return false; }
        for (uint8_t i = 0u; i < 3u; ++i) {
            dst[2u * i]     = (uint8_t)((uint16_t)jy.acc[i] & 0xFFu);  /* 小端 */
            dst[2u * i + 1u] = (uint8_t)((uint16_t)jy.acc[i] >> 8);
        }
        return true;
    }

    if (reg == RLCA_IMU_REG_GYRO) {
        if (jy.fail_gyro || (len != 6u)) { return false; }
        for (uint8_t i = 0u; i < 3u; ++i) {
            dst[2u * i]     = (uint8_t)((uint16_t)jy.gyro[i] & 0xFFu);  /* 小端 */
            dst[2u * i + 1u] = (uint8_t)((uint16_t)jy.gyro[i] >> 8);
        }
        return true;
    }

    if (reg == RLCA_IMU_REG_ANGLE) {
        if (jy.fail_ang || (len != 6u)) { return false; }
        uint8_t tmp[6];
        build_angle_buf(tmp, jy.roll_deg, jy.pitch_deg, jy.yaw_deg);
        memcpy(dst, tmp, 6u);
        return true;
    }

    return false;      /* 未知寄存器：模拟从机不响应 */
}

static uint32_t jy_millis_val = 0u;
static uint32_t jy_millis(void)
{
    return jy_millis_val;
}

static const imu_port_t jy_port = { jy_read_reg, jy_millis };

static void test_imu_selftest(void)
{
    section("imu: version-register self test");

    jy_reset();
    jy_millis_val = 0u;
    imu_init(&jy_port);

    CHECK_M(imu_selftest());
    /* ⚠️ 版本号【不校验固定值】—— 手册明说"不同产品，版本号不一样"。
     *    自检的判据是"能读到"，不是"等于某个魔法数"。 */
    CHECK_EQ_U(imu_state()->version, 0x0102u);

    /* ⚠️ 读失败必须报失败 —— 这是"接错器件 / 从机地址不对"的第一道提示 */
    jy_reset();
    jy.fail_version = true;
    imu_init(&jy_port);
    CHECK(!imu_selftest(), "selftest must fail when VERSION cannot be read");

    /* ⚠️ 回归断言：绝不能去读 0x00。
     *    手册里 0x00 是 SAVE（保存/重启/恢复出厂，可写）——
     *    早期版本把它当器件 ID 读，属于"往配置寄存器上乱碰"。 */
    jy_reset();
    imu_init(&jy_port);
    (void)imu_selftest();
    CHECK_EQ_U(jy.last_reg, RLCA_IMU_REG_VERSION);
    CHECK(jy.last_reg != 0x00u, "must NOT touch register 0x00 (that is SAVE!)");

    /* 端口为空不能崩 */
    imu_init(NULL);
    CHECK(!imu_selftest(), "selftest with no port must return false");

    jy_reset();
    imu_init(&jy_port);
}

static void test_imu_poll(void)
{
    section("imu: end-to-end poll against a fake JY901P");

    jy_reset();
    jy_millis_val = 1000u;
    jy.gyro[0] = 939;          /* ≈ +1 rad/s */
    jy.gyro[1] = -939;
    jy.gyro[2] = 0;
    jy.roll_deg = 0.0f; jy.pitch_deg = 0.0f; jy.yaw_deg = 0.0f;
    imu_init(&jy_port);

    CHECK(!imu_is_ok(), "must start invalid");
    CHECK_EQ_U(imu_age_ms(), 0xFFFFFFFFu);

    imu_poll();

    CHECK(imu_is_ok(), "must become valid after a successful read");
    CHECK_EQ_U(imu_state()->n_ok, 1u);
    CHECK_EQ_U(imu_state()->n_err, 0u);
    CHECK_EQ_U(imu_age_ms(), 0u);

    /* ⚠️ 第一次 poll 会多一次读：VERSION 的重试（见 imu_poll 的说明）。
     *    上电瞬间 JY901P 可能还没启动完，imu_selftest() 那一发会失败，
     *    所以 poll 里会在 version == 0 时低频补读一次。
     *    ⇒ 这次 poll = VERSION(补读) + ACC + GYRO + ANGLE = 4 次。 */
    CHECK_EQ_U(jy.n_reads, 4u);

    /* ⭐ 重试必须真的把 version 读进来（否则 imu_chip 上报恒 0） */
    CHECK_EQ_U(imu_state()->version, 0x0102u);

    /* ⭐ 第二次 poll 不该再读 VERSION（每 100 ms 最多一次，且成功后不再读） */
    {
        const uint32_t before = jy.n_reads;
        imu_poll();
        CHECK_EQ_U(jy.n_reads, before + 3u);
    }

    /* 角速度换算 */
    CHECK_NEAR(imu_state()->gyro_rad_s[0],  1.0, 0.002);
    CHECK_NEAR(imu_state()->gyro_rad_s[1], -1.0, 0.002);

    /* 加速度：Z 轴 2048 → 1 g */
    CHECK_NEAR(imu_state()->accel_g[2], 1.0, 1e-3);

    /* 水平姿态 → 单位四元数 */
    CHECK_NEAR(imu_state()->quat_wxyz[0], 1.0, 1e-4);
    CHECK_NEAR(imu_state()->quat_wxyz[3], 0.0, 1e-4);

    /* ---- 姿态变化能被反映 ---- */
    jy_millis_val += 10u;
    jy.pitch_deg = 30.0f;
    imu_poll();
    CHECK_NEAR(imu_state()->pitch_rad, 30.0 * M_PI / 180.0, 1e-4);
    CHECK(imu_state()->quat_wxyz[2] > 0.2, "pitch 30 deg must show up in qy");

    /* ---- 单路读失败 → 整帧作废，但保留上一帧有效数据 ---- */
    const uint32_t ok_before = imu_state()->n_ok;
    jy.fail_gyro = true;
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_ok, ok_before);
    CHECK_EQ_U(imu_state()->n_err, 1u);
    CHECK(imu_is_ok(), "one failed frame must not invalidate immediately");
    jy.fail_gyro = false;

    /* ---- 恢复 ---- */
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_ok, ok_before + 1u);

    /* ---- 持续失败超过 STALE → 失效 ---- */
    jy.fail_acc = true;
    jy_millis_val += (RLCA_IMU_STALE_MS + 20u);
    imu_poll();
    CHECK(!imu_is_ok(), "must go invalid after the stale window");
    CHECK(imu_age_ms() > RLCA_IMU_STALE_MS, "age must exceed the stale window");
    jy.fail_acc = false;

    /* ---- 恢复后重新有效 ---- */
    jy_millis_val += 10u;
    imu_poll();
    CHECK(imu_is_ok(), "must recover once reads work again");
}

static void test_imu_spike_protection(void)
{
    section("imu: gyro spike rejection (guard against mis-parsed frames)");

    jy_reset();
    jy_millis_val = 0u;
    jy.gyro[0] = 0;
    imu_init(&jy_port);

    imu_poll();                       /* 建立 prev = 0 */
    CHECK_EQ_U(imu_state()->n_spike, 0u);

    /* ---- 正常的小变化必须通过 ---- */
    jy.gyro[0] = 939;                 /* ≈1 rad/s */
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_spike, 0u);
    CHECK_NEAR(imu_state()->gyro_rad_s[0], 1.0, 0.002);

    /* ---- 巨大跳变（读到错位字节的典型形态）必须被拒 ---- */
    const uint32_t ok_before = imu_state()->n_ok;
    jy.gyro[0] = 30000;               /* ≈32 rad/s，物理上不可能在 10 ms 内出现 */
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_spike, 1u);
    CHECK_EQ_U(imu_state()->n_ok, ok_before);
    /* ⚠️ 被拒之后必须【保留上一帧有效值】，而不是写入垃圾 */
    CHECK_NEAR(imu_state()->gyro_rad_s[0], 1.0, 0.002);
    printf("    spike rejected; gyro still %.4f rad/s\n",
           (double)imu_state()->gyro_rad_s[0]);

    /* ---- 负方向的巨大跳变同样被拒 ---- */
    jy.gyro[0] = -30000;
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_spike, 2u);

    /* ---- 回到合理值后立刻恢复 ---- */
    jy.gyro[0] = 100;
    jy_millis_val += 10u;
    imu_poll();
    CHECK_EQ_U(imu_state()->n_spike, 2u);
    CHECK(imu_state()->n_ok > ok_before, "must accept a sane value again");
}

/*==============================================================================
 * main
 *============================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" RL-CAT sensor unit tests (battery + imu)\n");
    printf("========================================\n");
    fflush(stdout);

    test_batt_conversion();
    test_batt_thresholds();
    test_batt_state();
    test_batt_calibration();

    test_imu_pure();
    test_imu_quat();
    test_imu_selftest();
    test_imu_poll();
    test_imu_spike_protection();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");
    return (g_fail == 0) ? 0 : 1;
}
