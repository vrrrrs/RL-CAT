/**
  ******************************************************************************
  * @file    test_pure_logic.c
  * @brief   RL-CAT 纯逻辑层的 PC 端单元测试。
  *
  *  被测模块（全部不依赖 HAL）：
  *    rlca_crc16.c    CRC-16/CCITT-FALSE
  *    rlca_frame.c    帧编解码 + 接收状态机
  *    rlca_phase.c    步态相位（与训练侧严格一致）
  *    rlca_math.c     关节角 ↔ 舵机步数、反馈单位换算、标定表
  *    rlca_sts.c      Feetech STS 组帧/解帧
  *
  *  覆盖范围：协议规范 §10 的 10 项单元测试 + 相位/换算/STS 的边界用例。
  *
  *  构建与运行见 firmware/tools/test-host.ps1
  ******************************************************************************
  */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_crc16.h"
#include "rlca_frame.h"
#include "rlca_phase.h"
#include "rlca_math.h"
#include "rlca_sts.h"

/*==============================================================================
 * 极简测试框架
 *============================================================================*/

static int g_pass = 0;
static int g_fail = 0;
static int g_full = 0;      /* --full：跑 1 MB 随机流等耗时用例 */

#define CHECK(cond, ...)                                                  \
    do {                                                                  \
        if (cond) {                                                       \
            g_pass++;                                                     \
        } else {                                                          \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__);                 \
            printf(__VA_ARGS__);                                          \
            printf("\n");                                                 \
        }                                                                 \
    } while (0)

/* 用法：CHECK_EQ_U(actual, expected) —— 无符号比较；CHECK_EQ_D —— 有符号比较。
 * 分成两个宏而不是传 cast：`(unsigned)` 这类类型名作为宏参数会被当成表达式。 */
#define CHECK_EQ_U(a, b)                                                  \
    CHECK((a) == (b), "(got %u, want %u)",                                \
          (unsigned)(a), (unsigned)(b))

#define CHECK_EQ_D(a, b)                                                  \
    CHECK((a) == (b), "(got %d, want %d)",                                \
          (int)(a), (int)(b))

/* int16 比较（避免把 int16_t 直接当 unsigned 传进变参） */
#define CHECK_EQ_I16(a, b)                                                \
    CHECK((int)(a) == (int)(b), "(got %d, want %d)",                      \
          (int)(int16_t)(a), (int)(int16_t)(b))

static void section(const char *name)
{
    printf("\n[%s]\n", name);
    /* ⚠️ 必须 flush：测试挂在某个用例时，能看到它是在哪一节挂的 */
    fflush(stdout);
}

/*==============================================================================
 * 1. CRC-16/CCITT-FALSE（协议 §10 测试 1）
 *============================================================================*/

static void test_crc(void)
{
    section("CRC-16/CCITT-FALSE");

    static const uint8_t v[] = "123456789";
    CHECK(rlca_crc16(v, 9u) == 0x29B1u,
          "standard vector: got 0x%04X want 0x29B1", rlca_crc16(v, 9u));

    /* 增量式必须与一次性结果一致（固件边收边算时用得到） */
    uint16_t c = rlca_crc16_update(RLCA_CRC16_INIT, v, 4u);
    c = rlca_crc16_update(c, v + 4, 5u);
    CHECK(c == 0x29B1u, "incremental: got 0x%04X want 0x29B1", c);

    /* 空负载 */
    CHECK(rlca_crc16(NULL, 0u) == 0xFFFFu, "empty payload must return init value");

    /* 单字节已知值：crc16({0x00}) 用位运算手算 = 0x0000 ^ ... 这里只校验
     * 非平凡性：不同数据给不同结果 */
    const uint8_t a[2] = { 0x00, 0x00 };
    const uint8_t b[2] = { 0x00, 0x01 };
    CHECK(rlca_crc16(a, 2u) != rlca_crc16(b, 2u), "different data must give different CRC");
}

/*==============================================================================
 * 2. 帧编解码 + 接收状态机（协议 §10 测试 2~8）
 *============================================================================*/

static void test_frame_roundtrip(void)
{
    section("frame: roundtrip and layout");

    /* ---- 测试 2：空负载帧（HEARTBEAT） ---- */
    {
        uint8_t buf[16];
        const uint16_t n = rlca_frame_pack(buf, sizeof(buf), RLCA_TYPE_HEARTBEAT, 7u, NULL, 0u);
        CHECK(n == 7u, "HEARTBEAT frame length: got %u want 7", (unsigned)n);
        CHECK(buf[0] == 0xAAu && buf[1] == 0x55u, "frame header must be AA 55");
        CHECK(buf[2] == 0u, "LEN must be 0");
        CHECK(buf[3] == 7u, "SEQ must be 7");
        CHECK(buf[4] == RLCA_TYPE_HEARTBEAT, "TYPE must be 0x7F");

        rlca_rx_frame_t f;
        CHECK(rlca_frame_unpack(buf, n, &f), "unpack HEARTBEAT");
        CHECK(f.len == 0u && f.seq == 7u && f.type == RLCA_TYPE_HEARTBEAT, "unpacked fields");
    }

    /* ---- 测试 3：最大负载（LEN=128） ---- */
    {
        uint8_t payload[RLCA_FRAME_MAX_PAYLOAD];
        for (uint8_t i = 0; i < RLCA_FRAME_MAX_PAYLOAD; ++i) {
            payload[i] = (uint8_t)(i * 7u + 3u);
        }
        uint8_t buf[RLCA_FRAME_OVERHEAD + RLCA_FRAME_MAX_PAYLOAD];
        const uint16_t n = rlca_frame_pack(buf, sizeof(buf), 0x42u, 200u,
                                           payload, RLCA_FRAME_MAX_PAYLOAD);
        CHECK(n == 7u + 128u, "max frame length: got %u want 135", (unsigned)n);

        rlca_rx_frame_t f;
        CHECK(rlca_frame_unpack(buf, n, &f), "unpack max frame");
        CHECK(f.len == RLCA_FRAME_MAX_PAYLOAD, "max frame LEN");
        CHECK(memcmp(f.payload, payload, RLCA_FRAME_MAX_PAYLOAD) == 0, "max frame payload intact");
    }

    /* ---- TEL_FAST 实际尺寸：LEN=96 → 帧长 103 ---- */
    {
        uint8_t payload[RLCA_LEN_TEL_FAST];
        memset(payload, 0x5A, sizeof(payload));
        uint8_t buf[128];
        const uint16_t n = rlca_frame_pack(buf, sizeof(buf), RLCA_TYPE_TEL_FAST, 1u,
                                           payload, RLCA_LEN_TEL_FAST);
        CHECK(n == 103u, "TEL_FAST frame length: got %u want 103", (unsigned)n);
    }

    /* ---- CMD_JOINT 实际尺寸：LEN=31 → 帧长 38 ---- */
    {
        uint8_t payload[RLCA_LEN_CMD_JOINT];
        memset(payload, 0, sizeof(payload));
        uint8_t buf[64];
        const uint16_t n = rlca_frame_pack(buf, sizeof(buf), RLCA_TYPE_CMD_JOINT, 1u,
                                           payload, RLCA_LEN_CMD_JOINT);
        CHECK(n == 38u, "CMD_JOINT frame length: got %u want 38", (unsigned)n);
    }

    /* ---- 容量不足必须拒绝（而不是写越界） ---- */
    {
        uint8_t small[10];
        const uint8_t payload[8] = { 0 };
        CHECK(rlca_frame_pack(small, sizeof(small), 0x01u, 0u, payload, 8u) == 0u,
              "pack must reject insufficient capacity");
    }
}

static void test_frame_decoder(void)
{
    section("frame: receive state machine");

    uint8_t payload[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t buf[32];
    const uint16_t n = rlca_frame_pack(buf, sizeof(buf), 0x11u, 0x22u, payload, 8u);

    /* ---- 正常逐字节喂入 ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        rlca_feed_result_t r = RLCA_FEED_NONE;
        for (uint16_t i = 0; i < n; ++i) {
            r = rlca_rx_feed(&d, buf[i], 1000u, &f);
        }
        CHECK(r == RLCA_FEED_FRAME, "byte-wise decode must produce a frame");
        CHECK(f.len == 8u && f.seq == 0x22u && f.type == 0x11u, "decoded header fields");
        CHECK(memcmp(f.payload, payload, 8u) == 0, "decoded payload");
        CHECK(d.n_ok == 1u && d.n_crc_err == 0u, "decoder counters");
    }

    /* ---- 测试 4：CRC 错误 ---- */
    {
        uint8_t bad[32];
        memcpy(bad, buf, n);
        bad[n - 1] ^= 0xFFu;
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        rlca_feed_result_t r = RLCA_FEED_NONE;
        for (uint16_t i = 0; i < n; ++i) {
            r = rlca_rx_feed(&d, bad[i], 1000u, &f);
        }
        CHECK(r == RLCA_FEED_CRC_ERR, "corrupted CRC must be detected");
        CHECK(d.n_crc_err == 1u, "crc error counter");
        CHECK(d.state == RLCA_RX_WAIT_HEAD0, "decoder must reset after CRC error");
    }

    /* ---- 测试 5：截断帧（不崩溃、不误报） ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        rlca_feed_result_t r = RLCA_FEED_NONE;
        for (uint16_t i = 0; i + 3u < n; ++i) {
            r = rlca_rx_feed(&d, buf[i], 1000u, &f);
            CHECK(r == RLCA_FEED_NONE, "truncated frame must not yield a frame");
        }
    }

    /* ---- 测试 6：AA AA 55 序列（⚠️ 最容易写错的地方） ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        rlca_feed_result_t r = RLCA_FEED_NONE;

        r = rlca_rx_feed(&d, 0xAAu, 1000u, &f);
        CHECK(r == RLCA_FEED_NONE && d.state == RLCA_RX_WAIT_HEAD1, "first AA -> WAIT_HEAD1");
        r = rlca_rx_feed(&d, 0xAAu, 1000u, &f);
        CHECK(r == RLCA_FEED_NONE && d.state == RLCA_RX_WAIT_HEAD1, "second AA must stay in WAIT_HEAD1");
        r = rlca_rx_feed(&d, 0x55u, 1000u, &f);
        CHECK(d.state == RLCA_RX_HEADER, "then 55 must reach HEADER");

        /* 把剩余的帧体喂完（从 buf[2] 开始），应能正确解出 */
        for (uint16_t i = 2u; i < n; ++i) {
            r = rlca_rx_feed(&d, buf[i], 1000u, &f);
        }
        CHECK(r == RLCA_FEED_FRAME, "AA AA 55 prefix must still sync to the frame");
        CHECK(memcmp(f.payload, payload, 8u) == 0, "payload after AA AA 55");
    }

    /* ---- 测试 7：帧前有垃圾字节 ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        const uint8_t garbage[9] = { 0x00, 0x01, 0xFF, 0xAA, 0x00, 0x55, 0x13, 0x99, 0xAA };
        for (uint8_t i = 0; i < sizeof(garbage); ++i) {
            (void)rlca_rx_feed(&d, garbage[i], 1000u, &f);
        }
        rlca_feed_result_t r = RLCA_FEED_NONE;
        for (uint16_t i = 0; i < n; ++i) {
            r = rlca_rx_feed(&d, buf[i], 1000u, &f);
        }
        CHECK(r == RLCA_FEED_FRAME, "must resync after garbage bytes");
        CHECK(memcmp(f.payload, payload, 8u) == 0, "payload after garbage");
    }

    /* ---- 测试 8：LEN=255（超限）被拒绝并复位 ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        (void)rlca_rx_feed(&d, 0xAAu, 1000u, &f);
        (void)rlca_rx_feed(&d, 0x55u, 1000u, &f);
        const rlca_feed_result_t r = rlca_rx_feed(&d, 0xFFu, 1000u, &f);
        CHECK(r == RLCA_FEED_LEN_ERR, "LEN=255 must be rejected");
        CHECK(d.n_len_err == 1u, "len error counter");
        CHECK(d.state == RLCA_RX_WAIT_HEAD0, "must reset after LEN error");
    }

    /* ---- 测试 10：随机字节流不死机、不溢出 ----
     * ⚠️ 默认只灌 256 KB（逐字节调用，1 MB 在 CI 里太慢）；
     *    传 --full 时灌满 1 MB（协议规范 §10 的原始判据）。 */
    {
        const uint32_t n_bytes = g_full ? (1024u * 1024u) : (256u * 1024u);

        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        uint32_t seed = 0x12345678u;
        uint32_t frames = 0u;
        for (uint32_t i = 0; i < n_bytes; ++i) {
            /* xorshift32：确定性，便于复现 */
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            if (rlca_rx_feed(&d, (uint8_t)(seed & 0xFFu), 1000u, &f) == RLCA_FEED_FRAME) {
                frames++;
            }
        }
        printf("    random stream: %u bytes -> %u spurious frames accepted\n",
               (unsigned)n_bytes, (unsigned)frames);
        fflush(stdout);
        /* 关键判据：不崩溃、状态合法、计数一致 */
        CHECK(d.state <= RLCA_RX_CRC_HI, "decoder state must stay in range");
        CHECK(d.n_byte_total == n_bytes, "all bytes counted");
        CHECK(d.n_ok == frames, "n_ok must equal accepted frames");
    }

    /* ---- 测试：整帧超时 ---- */
    {
        rlca_rx_decoder_t d;
        rlca_rx_init(&d);
        rlca_rx_frame_t f;
        (void)rlca_rx_feed(&d, 0xAAu, 1000u, &f);
        (void)rlca_rx_feed(&d, 0x55u, 1000u, &f);
        (void)rlca_rx_feed(&d, 0x08u, 1000u, &f);   /* LEN，开始收 payload */
        CHECK(!rlca_rx_check_timeout(&d, 1000u + RLCA_FRAME_RX_TIMEOUT_MS),
              "must not time out exactly at the limit");
        CHECK(rlca_rx_check_timeout(&d, 1000u + RLCA_FRAME_RX_TIMEOUT_MS + 1u),
              "must time out past the limit");
        CHECK(d.n_timeout == 1u, "timeout counter");
        CHECK(d.state == RLCA_RX_WAIT_HEAD0, "must reset after timeout");
        /* 空闲状态不应报超时 */
        CHECK(!rlca_rx_check_timeout(&d, 999999u), "idle decoder must not report timeout");
    }
}

/*==============================================================================
 * 3. 步态相位（⭐ 必须与训练侧一致）
 *============================================================================*/

static void test_phase(void)
{
    section("gait phase (must match training side)");

    /* 相位步长必须是 3.0 Hz / 100 Hz = 0.03（用刻度表示 = 3/100） */
    const float expected_step = RLCA_GAIT_FREQ_HZ / (float)RLCA_CTRL_HZ;
    CHECK(fabsf(expected_step - 0.03f) < 1e-7f,
          "phase step: got %.8f want 0.03", (double)expected_step);

    /* 训练侧原式：phase = (phase + dt*gait_freq) % 1，sin(2π·phase)
     * 用 double 算参考值，与固件的多项式实现逐刻度比对。 */
    double worst_sin = 0.0;
    double worst_cos = 0.0;
    for (uint32_t k = 0; k < RLCA_PHASE_PERIOD; ++k) {
        const double ph = (double)k / (double)RLCA_PHASE_PERIOD;
        const double ref_sin = sin(2.0 * M_PI * ph);
        const double ref_cos = cos(2.0 * M_PI * ph);

        const double got_sin = (double)rlca_phase_sin(k);
        const double got_cos = (double)rlca_phase_cos(k);

        const double e_sin = fabs(got_sin - ref_sin);
        const double e_cos = fabs(got_cos - ref_cos);
        if (e_sin > worst_sin) { worst_sin = e_sin; }
        if (e_cos > worst_cos) { worst_cos = e_cos; }
    }
    printf("    max |err| vs math.h: sin %.2e  cos %.2e\n", worst_sin, worst_cos);

    /* 判据：量化到 int16（÷32767）后误差应 < 1 LSB 的若干倍。
     * 协议里相位以 1/32767 为单位编码，1 LSB = 3.05e-5。 */
    CHECK(worst_sin < 1e-4, "sin error %.3e exceeds tolerance 1e-4", worst_sin);
    CHECK(worst_cos < 1e-4, "cos error %.3e exceeds tolerance 1e-4", worst_cos);

    /* 关键点定点检查 */
    CHECK(fabsf(rlca_phase_sin(0u)) < 1e-6f, "sin(0) = 0");
    CHECK(fabsf(rlca_phase_cos(0u) - 1.0f) < 1e-5f, "cos(0) = 1");
    CHECK(fabsf(rlca_phase_sin(RLCA_PHASE_QUARTER) - 1.0f) < 1e-5f, "sin(pi/2) = 1");
    CHECK(fabsf(rlca_phase_cos(RLCA_PHASE_QUARTER)) < 1e-5f, "cos(pi/2) = 0");
    CHECK(fabsf(rlca_phase_sin(RLCA_PHASE_HALF)) < 1e-5f, "sin(pi) = 0");
    CHECK(fabsf(rlca_phase_cos(RLCA_PHASE_HALF) + 1.0f) < 1e-5f, "cos(pi) = -1");

    /* sin²+cos² = 1 */
    double worst_norm = 0.0;
    for (uint32_t k = 0; k < RLCA_PHASE_PERIOD; ++k) {
        const float s = rlca_phase_sin(k);
        const float c = rlca_phase_cos(k);
        const double norm = fabs((double)(s * s + c * c) - 1.0);
        if (norm > worst_norm) { worst_norm = norm; }
    }
    CHECK(worst_norm < 1e-5, "sin^2+cos^2 deviation %.3e", worst_norm);

    /* ---- 整数累加：100 步一圈，且不漂移 ---- */
    {
        rlca_phase_reset();
        CHECK(rlca_phase_get() == 0u, "reset -> 0");

        /* 推进 99 次：3×99 = 297 → 297 mod 100 = 97 */
        for (int i = 0; i < 99; ++i) {
            (void)rlca_phase_advance();
        }
        CHECK_EQ_U(rlca_phase_get(), 97u);

        /* 第 100 次推进：97+3 = 100 → 回绕到 0（一个完整步态周期） */
        (void)rlca_phase_advance();
        CHECK_EQ_U(rlca_phase_get(), 0u);

        /* 长期不漂移：跑 10 万个周期后仍应回到 0（100 与 3 互质，周期恰为 100） */
        for (uint32_t i = 0; i < 100000u; ++i) {
            (void)rlca_phase_advance();
        }
        CHECK_EQ_U(rlca_phase_get(), 0u);

        /* int16 编码：峰值处 sin 应达到 ~32767
         * ⚠️ 相位刻度只有 100 个，k=25 才是精确的 π/2；
         *    所以直接对纯函数验证，而不是靠累加走到峰值。 */
        CHECK((int16_t)(rlca_phase_sin(RLCA_PHASE_QUARTER) * 32767.0f) == 32767,
              "int16 sin at peak must be 32767, got %d",
              (int)(int16_t)(rlca_phase_sin(RLCA_PHASE_QUARTER) * 32767.0f));
        CHECK((int16_t)(rlca_phase_cos(RLCA_PHASE_QUARTER) * 32767.0f) == 0,
              "int16 cos at peak must be 0, got %d",
              (int)(int16_t)(rlca_phase_cos(RLCA_PHASE_QUARTER) * 32767.0f));

        /* 累加器路径：推进 8 次 = 24 刻度，sin/cos 编码值应与纯函数一致 */
        rlca_phase_reset();
        for (uint32_t i = 0; i < 8u; ++i) { (void)rlca_phase_advance(); }
        CHECK_EQ_U(rlca_phase_get(), 24u);
        int16_t s16;
        int16_t c16;
        rlca_phase_get_sincos_i16(&s16, &c16);
        CHECK_EQ_I16(s16, (int16_t)(rlca_phase_sin(24u) * 32767.0f));
        CHECK_EQ_I16(c16, (int16_t)(rlca_phase_cos(24u) * 32767.0f));
        /* 24 刻度比峰值早 1 个刻度（0.0628 rad），cos 约为 +0.0628×32767 ≈ +2057
         * （cos 在 π/2 左侧为正，过峰值后才变负） */
        CHECK(c16 > 1500 && c16 < 2600, "cos 1 tick before peak should be ~+2057, got %d", (int)c16);
    }
}

/*==============================================================================
 * 4. 关节换算与标定表
 *============================================================================*/

static void test_math(void)
{
    section("joint math and calibration");

    rlca_calib_t c;
    rlca_calib_load_defaults(&c);

    /* 默认表必须自洽 */
    CHECK(c.magic == RLCA_CALIB_MAGIC, "default magic");
    CHECK(c.version == RLCA_CALIB_VERSION, "default version");
    CHECK(rlca_calib_is_valid(&c), "default table must be valid");
    CHECK(rlca_calib_crc(&c) == c.crc16, "default crc must match");

    /* ---- 限位数值核对（docs/30-firmware.md §3.2） ----
     * ⚠️ 限位以【步】为权威存储，反算回 rad 会带上最多 1 步的量化误差
     *    （1 步 = 1.534e-3 rad = 0.088°）。所以这里比对的是【步数】，
     *    以及"反算值落在 1 步之内"。 */
    {
        float qmin;
        float qmax;

        rlca_q_range(&c, RLCA_J_LF_HIP, &qmin, &qmax);
        CHECK(fabsf(qmin - 0.0f) < 1e-6f, "hip lower: got %.6f want 0", (double)qmin);
        CHECK_EQ_D(rlca_q_to_step(&c, RLCA_J_LF_HIP, RLCA_HIP_MAX_RAD), 455);   /* +40° */
        CHECK(fabsf(qmax - RLCA_HIP_MAX_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "hip upper: got %.5f want %.5f (+-1 step)", (double)qmax, (double)RLCA_HIP_MAX_RAD);

        rlca_q_range(&c, RLCA_J_LF_THIGH, &qmin, &qmax);
        CHECK_EQ_D(rlca_q_to_step(&c, RLCA_J_LF_THIGH, RLCA_THIGH_MIN_RAD), -398); /* -35° */
        CHECK_EQ_D(rlca_q_to_step(&c, RLCA_J_LF_THIGH, RLCA_THIGH_MAX_RAD), 1024); /* +90° */
        CHECK(fabsf(qmin - RLCA_THIGH_MIN_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "thigh lower: got %.5f want %.5f", (double)qmin, (double)RLCA_THIGH_MIN_RAD);
        CHECK(fabsf(qmax - RLCA_THIGH_MAX_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "thigh upper: got %.5f want %.5f (90deg)", (double)qmax, (double)RLCA_THIGH_MAX_RAD);

        rlca_q_range(&c, RLCA_J_LF_CALF, &qmin, &qmax);
        /* ⚠️ 期望值用 dir 推导，不要硬编码符号：
         *    step = dir × q × SPR  ⇒  dir=−1 时 ±60° 的步数符号相反。
         *    （2026-10-16 calf 的 dir 由 +1 翻成 −1 —— 硬编码会静默错。） */
        {
            static const int8_t dtab[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
            const int32_t d = (int32_t)dtab[RLCA_J_LF_CALF];
            CHECK_EQ_D(rlca_q_to_step(&c, RLCA_J_LF_CALF, RLCA_CALF_MIN_RAD),
                       d * (-683)); /* −60° */
            CHECK_EQ_D(rlca_q_to_step(&c, RLCA_J_LF_CALF, RLCA_CALF_MAX_RAD),
                       d * 683);    /* +60° */
        }
        CHECK(fabsf(qmin - RLCA_CALF_MIN_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "calf lower: got %.5f", (double)qmin);
        CHECK(fabsf(qmax - RLCA_CALF_MAX_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "calf upper: got %.5f", (double)qmax);
    }

    /* ---- 换算往返：q → step → q 误差 < 1 步对应的弧度 ---- */
    {
        float worst = 0.0f;
        for (uint8_t i = 0; i < RLCA_N_JOINTS; ++i) {
            float qmin;
            float qmax;
            rlca_q_range(&c, i, &qmin, &qmax);
            for (int k = 0; k <= 40; ++k) {
                const float q = qmin + ((qmax - qmin) * (float)k / 40.0f);
                const int16_t s = rlca_q_to_step(&c, i, q);
                const float back = rlca_step_to_q(&c, i, s);
                const float e = fabsf(back - q);
                if (e > worst) { worst = e; }
            }
        }
        const float step_rad = RLCA_SERVO_RAD_PER_STEP;   /* 0.0879° */
        printf("    roundtrip max err = %.3e rad (1 step = %.3e rad)\n",
               (double)worst, (double)step_rad);
        CHECK(worst <= step_rad + 1e-9f,
              "roundtrip error %.3e exceeds 1 step %.3e", (double)worst, (double)step_rad);
    }

    /*==========================================================================
     * ⭐⭐ 几何标定表：不变量「标定姿态下 q 必须等于 q_calib」
     *      （2026-10-18）
     *
     *  ⚠️ 前提：标定姿态（腿伸直）下 12 个舵机编码器都读到 2048。
     *     则 offset = 2048 − dir × q_calib × SPR。
     *
     *  ⭐ 于是有一条**与 dir 无关**的强不变量：
     *
     *        step_to_q(2048) == q_calib        （q_calib 是幅值，恒为正）
     *
     *     推导：
     *        q = (2048 − offset) × dir × k
     *          = (2048 − 2048 + dir·q_calib·SPR) × dir × k
     *          = dir · q_calib · SPR · dir · k
     *          = q_calib                        （dir² = 1）
     *
     *  ⭐ 这条测试的价值：它同时钉住了
     *       · offset 的推导公式
     *       · `q_calib` 必须是【幅值】而不是带符号的角度
     *       · `dir` 只能在 offset 里出现一次（不能再双重取反）
     *     现场实测也印证：标定姿态下 `pose.py` 报 thigh +45°/+40°、
     *     calf +90°/+85° —— 正是 q_calib。
     *========================================================================*/
    {
        printf("    ---- geometric calib (step_cal = 2048) ----\n");
        rlca_calib_t g;
        rlca_calib_build_geometric(&g);

        /* ① 表本身必须合法，且不是"全零默认表" */
        CHECK(rlca_calib_is_valid(&g), "geometric table must be structurally valid");
        {
            uint8_t nz = 0u;
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                CHECK((g.offset[i] >= 0) && (g.offset[i] < 4096),
                      "offset[%u] = %d out of [0,4096)",
                      (unsigned)i, (int)g.offset[i]);
                if (g.offset[i] != 0) { nz++; }
            }
            /* ⚠️ hip 的 offset 确实是 2048（q_calib = 0），
             *    所以这里只要求"不是全零" */
            CHECK(nz >= 8u, "geometric table must not look like the all-zero default");
        }

        /* ② ⭐ 核心不变量：标定姿态(2048) 下 q == q_calib（与 dir 无关） */
        {
            static const float q_cal[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;
            const float tol = 1.5f * RLCA_SERVO_RAD_PER_STEP;
            uint8_t bad = 0u;
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                const float q = rlca_step_to_q(&g, i, (int16_t)RLCA_CALIB_POSE_STEP);
                if (fabsf(q - q_cal[i]) > tol) {
                    bad++;
                    printf("      j%u: q@2048 = %.4f rad, expect q_calib = %.4f\n",
                           (unsigned)i, (double)q, (double)q_cal[i]);
                }
            }
            CHECK_EQ_U(bad, 0u);
        }

        /* ③ hip 的 q_calib = 0 ⇒ offset 必须正好是 2048 */
        {
            CHECK_EQ_D(g.offset[RLCA_J_LF_HIP], 2048);
            CHECK_EQ_D(g.offset[RLCA_J_RB_HIP], 2048);
        }

        /* ④ 往返一致：q_step(offset) 必须等于 0（offset 就是 URDF 零位的读数） */
        {
            uint8_t bad = 0u;
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                const float q0 = rlca_step_to_q(&g, i, g.offset[i]);
                if (fabsf(q0) > 1.5f * RLCA_SERVO_RAD_PER_STEP) {
                    bad++;
                    printf("      j%u: q@offset = %.4f, expect 0\n",
                           (unsigned)i, (double)q0);
                }
            }
            CHECK_EQ_U(bad, 0u);
        }

        /* ⑤ 限位仍必须与 dir 无关（与上面那条回归呼应） */
        {
            float qmin = 0.0f;
            float qmax = 0.0f;
            rlca_q_range(&g, RLCA_J_LB_HIP, &qmin, &qmax);   /* dir = -1 */
            CHECK(fabsf(qmax - RLCA_HIP_MAX_RAD) <= 2.0f * RLCA_SERVO_RAD_PER_STEP,
                  "geometric: dir=-1 hip q_max = %.4f", (double)qmax);
        }
    }

    /*==========================================================================
     * ⭐⭐ 回归：关节限位【不得】随 dir 变化（2026-10-18 修的真 bug）
     *
     *  ⚠️ 现场现象：`lb_hip` / `rf_hip` 发正角完全不动。
     *     根因：`rlca_q_range()` 里多乘了一次 `dir`，
     *     而 `limit_*` 本来就是用 `dir=1, offset=0` 算出的 **q 空间**量
     *     ⇒ `dir = −1` 的关节限位被整体取反
     *     ⇒ hip 的范围从 [0°, 40°] 变成 [−39°, 0°]
     *     ⇒ 正角指令被夹成 0（范围上界）⇒ 完全不动。
     *
     *  ⭐ 这个 bug 能潜伏很久，是因为已有测试只覆盖了：
     *       · `dir = +1` 且范围不对称的 hip / thigh
     *       · `dir = −1` 但范围**对称**的 calf（取反后区间不变，看不出问题）
     *     ⇒ 本用例对 **两种 dir × 三种关节** 全组合断言"范围与 dir 无关"。
     *========================================================================*/
    {
        uint8_t bad = 0;
        for (uint8_t pass = 0u; pass < 2u; ++pass) {
            const int8_t d = (pass == 0u) ? (int8_t)1 : (int8_t)-1;
            rlca_calib_t t2;
            rlca_calib_load_defaults(&t2);
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                t2.dir[i] = d;
                t2.offset[i] = (int16_t)2048;      /* 限位与 offset 无关，随便给 */
            }
            rlca_calib_fill_limits(&t2);

            /* hip / thigh / calf 各取第一个作代表 */
            struct { uint8_t idx; float wmin; float wmax; const char *nm; } cases[] = {
                { (uint8_t)RLCA_J_LF_HIP,   RLCA_HIP_MIN_RAD,   RLCA_HIP_MAX_RAD,   "hip"   },
                { (uint8_t)RLCA_J_LF_THIGH, RLCA_THIGH_MIN_RAD, RLCA_THIGH_MAX_RAD, "thigh" },
                { (uint8_t)RLCA_J_LF_CALF,  RLCA_CALF_MIN_RAD,  RLCA_CALF_MAX_RAD,  "calf"  },
            };
            for (uint8_t k = 0u; k < (uint8_t)(sizeof(cases) / sizeof(cases[0])); ++k) {
                float qmin = 0.0f;
                float qmax = 0.0f;
                /* ⚠️ 用全部 4 条腿的同一类关节都验一遍（dir 表本来就不一致） */
                const uint8_t base = cases[k].idx;
                for (uint8_t leg = 0u; leg < 4u; ++leg) {
                    const uint8_t j = (uint8_t)(base + leg);
                    rlca_q_range(&t2, j, &qmin, &qmax);
                    if ((fabsf(qmin - cases[k].wmin) > (2.0f * RLCA_SERVO_RAD_PER_STEP)) ||
                        (fabsf(qmax - cases[k].wmax) > (2.0f * RLCA_SERVO_RAD_PER_STEP))) {
                        bad++;
                        printf("    dir=%+d j%u(%s): range [%.4f, %.4f] but expect [%.4f, %.4f]\n",
                               (int)d, (unsigned)j, cases[k].nm,
                               (double)qmin, (double)qmax,
                               (double)cases[k].wmin, (double)cases[k].wmax);
                    }
                }
            }
        }
        CHECK_EQ_U(bad, 0u);

        /* ⭐ 显式复现现场：dir = −1 的 hip 必须接受 +10° */
        rlca_calib_t t3;
        rlca_calib_load_defaults(&t3);
        t3.dir[RLCA_J_LB_HIP] = (int8_t)-1;
        t3.offset[RLCA_J_LB_HIP] = (int16_t)2024;
        t3.limit_min[RLCA_J_LB_HIP] = (int16_t)0;
        t3.limit_max[RLCA_J_LB_HIP] = (int16_t)455;
        const float q10 = 10.0f * 0.0174532925f;
        CHECK(rlca_q_in_limits(&t3, RLCA_J_LB_HIP, q10),
              "dir=-1 hip must accept +10deg (was clamped to 0 before the fix)");
        CHECK(fabsf(rlca_q_clamp(&t3, RLCA_J_LB_HIP, q10) - q10) < 1e-6f,
              "dir=-1 hip must NOT clamp +10deg");
        CHECK(!rlca_q_in_limits(&t3, RLCA_J_LB_HIP, -0.1f),
              "dir=-1 hip must still reject negative angles");
    }

    {
        const int16_t s = rlca_q_to_step(&c, RLCA_J_LF_HIP, 0.6981317f);
        CHECK(s == 455, "hip +40deg -> %d steps, expect 455", (int)s);
    }

    /* ---- 限幅 ---- */
    {
        CHECK(rlca_q_in_limits(&c, RLCA_J_LF_THIGH, 0.0f), "0 rad must be in limits");
        CHECK(!rlca_q_in_limits(&c, RLCA_J_LF_THIGH, 2.0f), "2 rad must be out of limits");
        CHECK(!rlca_q_in_limits(&c, RLCA_J_LF_HIP, -0.1f), "hip is asymmetric: -0.1 must be rejected");

        const float cl = rlca_q_clamp(&c, RLCA_J_LF_THIGH, 5.0f);
        CHECK(fabsf(cl - RLCA_THIGH_MAX_RAD) <= RLCA_SERVO_RAD_PER_STEP,
              "clamp must pin to upper limit, got %.5f", (double)cl);

        /* NaN / Inf 必须落到下限（安全侧） */
        const float nan_v = 0.0f / 0.0f;
        float qmin;
        float qmax;
        rlca_q_range(&c, RLCA_J_LF_CALF, &qmin, &qmax);
        const float cn = rlca_q_clamp(&c, RLCA_J_LF_CALF, nan_v);
        CHECK(fabsf(cn - qmin) < 1e-5f, "NaN must clamp to q_min");
        CHECK(!rlca_float_is_valid(nan_v), "NaN must be invalid");
        CHECK(!rlca_float_is_valid(INFINITY), "+Inf must be invalid");
        CHECK(!rlca_float_is_valid(-INFINITY), "-Inf must be invalid");
        CHECK(rlca_float_is_valid(0.0f), "0 must be valid");
        CHECK(rlca_float_is_valid(-3.0f), "-3 must be valid");
    }

    /* ---- dir = -1 的对称性 ---- */
    {
        rlca_calib_t m = c;
        m.dir[RLCA_J_RF_HIP] = -1;
        const int16_t sp = rlca_q_to_step(&m, RLCA_J_RF_HIP, 0.3f);
        const int16_t sn = rlca_q_to_step(&m, RLCA_J_LF_HIP, 0.3f);
        CHECK(sp == -sn, "dir=-1 must mirror the step: got %d vs %d", (int)sp, (int)sn);
    }

    /* ---- 标定表默认值里的限位：offset 必须不影响限位区间 ---- */
    {
        rlca_calib_t m = c;
        m.offset[RLCA_J_LF_CALF] = 2500;
        float a1;
        float b1;
        float a2;
        float b2;
        rlca_q_range(&c, RLCA_J_LF_CALF, &a1, &b1);
        rlca_q_range(&m, RLCA_J_LF_CALF, &a2, &b2);
        CHECK(fabsf(a1 - a2) < 1e-5f && fabsf(b1 - b2) < 1e-5f,
              "limits must be invariant to offset");
    }

    /* ---- 校验和：改一个字节必须失效 ---- */
    {
        rlca_calib_t m = c;
        m.offset[3] = 123;
        CHECK(rlca_calib_crc(&m) != c.crc16, "modified table must change crc");
        CHECK(rlca_calib_is_valid(&m), "still structurally valid");

        rlca_calib_t bad = c;
        bad.magic = 0xDEADBEEFu;
        CHECK(!rlca_calib_is_valid(&bad), "bad magic must be invalid");

        rlca_calib_t bad2 = c;
        bad2.dir[0] = 3;
        CHECK(!rlca_calib_is_valid(&bad2), "dir must be +1/-1 only");

        rlca_calib_t bad3 = c;
        bad3.limit_min[0] = 500;
        bad3.limit_max[0] = 100;
        CHECK(!rlca_calib_is_valid(&bad3), "limit_min > limit_max must be invalid");
    }

    /* ---- 反馈单位换算 ---- */
    {
        /* 速度：步/秒 → rad/s。4096 步/秒 = 1 圈/秒 = 2π rad/s */
        CHECK(fabsf(rlca_servo_speed_to_rad_s(4096) - 6.2831853f) < 1e-4f,
              "4096 step/s must be 2pi rad/s, got %.6f",
              (double)rlca_servo_speed_to_rad_s(4096));
        CHECK(rlca_servo_speed_to_rad_s(-2048) < 0.0f, "negative speed keeps sign");

        /* 力矩：0x03FF（幅值 1023 ≈ 100%）→ 约 3.01 N·m；bit10 置位则取负 */
        const float tau_pos = rlca_servo_load_to_torque_nm((int16_t)0x03FF);
        const float tau_neg = rlca_servo_load_to_torque_nm((int16_t)0x07FF);
        printf("    load 0x3FF -> %.4f N.m, load 0x7FF (dir bit) -> %.4f N.m\n",
               (double)tau_pos, (double)tau_neg);
        CHECK(tau_pos > 2.9f && tau_pos < 3.1f, "full load ~= 3.0 N.m, got %.4f", (double)tau_pos);
        CHECK(fabsf(tau_neg + tau_pos) < 1e-5f, "direction bit must negate torque");
        CHECK(fabsf(rlca_servo_load_to_torque_nm(0)) < 1e-9f, "zero load = zero torque");

        /* 位置回绕：4090 → 5 应为 +11 步（跨过 0），而不是 -4085 */
        CHECK_EQ_D(rlca_servo_pos_delta(4090, 5), 11);
        CHECK_EQ_D(rlca_servo_pos_delta(5, 4090), -11);
        CHECK_EQ_D(rlca_servo_pos_delta(100, 300), 200);
        CHECK_EQ_D(rlca_servo_pos_delta(300, 100), -200);
    }

    /* ---- 关节状态更新：回绕时关节角必须连续 ---- */
    {
        rlca_joint_state_t st;
        memset(&st, 0, sizeof(st));
        st.pos_step = 5;              /* 从 4090 绕到 5 */
        rlca_joint_state_update(&c, RLCA_J_LF_CALF, 4090, &st);
        /* 连续位置 = 4090 + 11 = 4101 → q = 4101 × 2π/4096 ≈ 6.29 rad
         * ⚠️ 本用例测的是【连续性】（不能跳 2π），不是符号 ——
         *    而 q 的符号由 dir 决定（calf 的 dir 2026-10-16 翻成 −1）。
         *    所以对幅值断言，避免 dir 变动时误报。 */
        const float qa = fabsf(st.q_rad);
        CHECK(qa > 6.28f && qa < 6.30f,
              "wrapped joint angle must be continuous, got %.4f", (double)st.q_rad);
    }
}

/*==============================================================================
 * 5. STS 协议组帧/解帧
 *============================================================================*/

static void test_sts(void)
{
    section("Feetech STS protocol");

    /* ---- SyncWrite 12 舵机：帧长 44 字节 ----
     * ⚠️ 伺服总线模块设计 §6.1 写的是 45 字节，那是按 LEN+5 算的；
     *    按它的 LEN 定义（LEN = 从 INSTR 到 CHK 之前的字节数）实际是：
     *    LEN = 2 + 12×3 + 2 = 40，帧长 = LEN + 4 = 44 */
    {
        uint8_t ids[12];
        int16_t pos[12];
        for (uint8_t i = 0; i < 12u; ++i) {
            ids[i] = (uint8_t)(i + 1u);
            pos[i] = (int16_t)(2048 + (int16_t)i);
        }

        uint8_t buf[64];
        uint16_t n = 0u;
        const sts_tx_result_t r = rlca_sts_build_sync_write_pos(ids, 12u, pos, buf, sizeof(buf), &n);
        CHECK(r == STS_TX_OK, "sync write build must succeed (r=%d)", (int)r);
        CHECK_EQ_U(n, 44u);

        /* 帧头/广播 ID/指令码 */
        CHECK(buf[0] == 0xFFu && buf[1] == 0xFFu, "STS header FF FF");
        CHECK(buf[2] == STS_BROADCAST_ID, "broadcast id must be 0xFE");
        CHECK(buf[4] == STS_INST_SYNC_WRITE, "instruction must be 0x83");
        CHECK_EQ_U(buf[3], 40u);   /* LEN = 4 + 3*12 */
        CHECK_EQ_U(buf[5], STS_REG_GOAL_POSITION_L);
        CHECK_EQ_U(buf[6], 2u);

        /* 逐个舵机的 (id, pos) 必须按小端排列 */
        uint16_t p = 7u;
        bool layout_ok = true;
        int32_t chk_sum = (int32_t)STS_BROADCAST_ID + (int32_t)buf[3] + (int32_t)buf[4];
        for (uint8_t i = 5u; i < (uint16_t)(n - 1u); ++i) {
            chk_sum += (int32_t)buf[i];
        }
        for (uint8_t i = 0; i < 12u; ++i) {
            if (buf[p] != ids[i]) { layout_ok = false; }
            const uint16_t v = rlca_get_u16(&buf[p + 1u]);
            if ((int16_t)v != pos[i]) { layout_ok = false; }
            p = (uint16_t)(p + 3u);
        }
        CHECK(layout_ok, "per-servo id/position layout must be little-endian and in order");
        CHECK_EQ_U(p, (uint16_t)(n - 1u));
        CHECK_EQ_U(buf[n - 1u], (uint8_t)(~chk_sum & 0xFF));
    }

    /* ---- SyncRead 12 舵机：帧长 20 字节 ----
     * LEN = 2 + 12 + 2 = 16，帧长 = LEN + 4 = 20（文档写的 21 同样是 LEN+5 的算法）*/
    {
        uint8_t ids[12];
        for (uint8_t i = 0; i < 12u; ++i) {
            ids[i] = (uint8_t)(i + 1u);
        }
        uint8_t buf[32];
        uint16_t n = 0u;
        const sts_tx_result_t r = rlca_sts_build_sync_read(STS_FB_START_ADDR, STS_FB_DATA_LEN,
                                                           ids, 12u, buf, sizeof(buf), &n);
        CHECK(r == STS_TX_OK, "sync read build must succeed (r=%d)", (int)r);
        CHECK_EQ_U(n, 20u);
        CHECK(buf[4] == STS_INST_SYNC_READ, "instruction must be 0x82");
        CHECK_EQ_U(buf[3], 16u);                     /* LEN */
        CHECK_EQ_U(buf[5], STS_FB_START_ADDR);
        CHECK_EQ_U(buf[6], STS_FB_DATA_LEN);
        CHECK_EQ_U(buf[7], 1u);
        CHECK_EQ_U(buf[18], 12u);
    }

    /* ---- 重复 ID 必须被拒绝（否则总线上会互相踩踏） ---- */
    {
        const uint8_t ids[3] = { 1u, 2u, 2u };
        int16_t pos[3] = { 0, 0, 0 };
        uint8_t buf[32];
        uint16_t n = 0u;
        CHECK(rlca_sts_build_sync_write_pos(ids, 3u, pos, buf, sizeof(buf), &n) == STS_TX_ERR_DUP_ID,
              "duplicate ids must be rejected");
    }

    /* ---- 容量不足必须拒绝 ---- */
    {
        const uint8_t ids[2] = { 1u, 2u };
        int16_t pos[2] = { 0, 0 };
        uint8_t small[10];
        uint16_t n = 0u;
        CHECK(rlca_sts_build_sync_write_pos(ids, 2u, pos, small, sizeof(small), &n) == STS_TX_ERR_CAPACITY,
              "insufficient capacity must be rejected");
    }

    /* ---- Ping / Read / Torque 单机指令 ---- */
    {
        uint8_t buf[16];
        uint16_t n = 0u;

        CHECK(rlca_sts_build_ping(5u, buf, sizeof(buf), &n) == STS_TX_OK, "ping build");
        CHECK_EQ_U(n, 6u);
        CHECK(buf[2] == 5u && buf[3] == 2u && buf[4] == STS_INST_PING, "ping layout");
        {
            /* CHK = ~(ID + LEN + INSTR) = ~(5 + 2 + 1) = ~8 = 0xF7 */
            CHECK_EQ_U(buf[5], 0xF7u);
        }

        /* Torque: 参数 2 字节(addr+val) → LEN = 4，帧长 = 4 + 4 = 8
         * 0xFF 0xFF ID 04 03 28 01 CHK */
        CHECK(rlca_sts_build_torque(3u, true, buf, sizeof(buf), &n) == STS_TX_OK, "torque build");
        CHECK_EQ_U(n, 8u);
        CHECK_EQ_U(buf[3], 4u);
        CHECK(buf[4] == STS_INST_WRITE, "torque uses INST_WRITE");
        CHECK_EQ_U(buf[5], STS_REG_TORQUE_ENABLE);
        CHECK_EQ_U(buf[6], 1u);
        {
            /* CHK = ~(ID + LEN + INSTR + addr + val) = ~(3+4+3+40+1) = ~51 = 0xCC */
            CHECK_EQ_U(buf[7], (uint8_t)(~((3u + 4u + 3u + STS_REG_TORQUE_ENABLE + 1u) & 0xFFu) & 0xFFu));
        }

        /* Read: 参数 2 字节(addr+len) → LEN = 4，帧长 = 8 */
        CHECK(rlca_sts_build_read(1u, STS_REG_PRESENT_POSITION_L, 2u, buf, sizeof(buf), &n) == STS_TX_OK,
              "read build");
        CHECK_EQ_U(n, 8u);
        CHECK_EQ_U(buf[3], 4u);
        CHECK(buf[5] == STS_REG_PRESENT_POSITION_L && buf[6] == 2u, "read layout");

        CHECK(rlca_sts_build_read(1u, STS_REG_PRESENT_POSITION_L, 0u, buf, sizeof(buf), &n) == STS_TX_ERR_PARAM,
              "read with 0 length must be rejected");
    }

    /* ---- 反馈帧解析：造一帧真实格局的反馈 ---- */
    {
        /* 0xFF 0xFF ID=5 LEN=11 ERR=0 | pos_L pos_H spd_L spd_H load_L load_H vol temp moving | CHK */
        uint8_t fb[32];
        fb[0] = 0xFFu;
        fb[1] = 0xFFu;
        fb[2] = 5u;
        fb[3] = (uint8_t)(STS_FB_DATA_LEN + 2u);   /* = 11 */
        fb[4] = 0u;                                 /* ERR */
        fb[5] = 0x00u; fb[6] = 0x08u;               /* pos  = 0x0800 = 2048 */
        fb[7] = 0x10u; fb[8] = 0x00u;               /* spd  = 16 step/s */
        fb[9] = 0xFFu; fb[10] = 0x03u;              /* load = 0x03FF (正方向, 满) */
        fb[11] = 120u;                              /* 电压 12.0 V */
        fb[12] = 42u;                               /* 温度 42 °C */
        fb[13] = 0u;                                /* moving */
        {
            uint32_t sum = (uint32_t)fb[2] + (uint32_t)fb[3] + (uint32_t)fb[4];
            for (uint8_t i = 0; i < STS_FB_DATA_LEN; ++i) {
                sum += fb[5u + i];
            }
            fb[14] = (uint8_t)(~sum & 0xFFu);
        }
        const uint16_t fb_len = (uint16_t)STS_FB_DATA_LEN + 6u;   /* = 15 */

        sts_feedback_t out;
        const sts_fb_result_t r = rlca_sts_parse_feedback(fb, fb_len, 5u, STS_FB_DATA_LEN, &out);
        CHECK(r == STS_FB_OK, "feedback parse must succeed (r=%d)", (int)r);
        CHECK(out.id == 5u && out.err == 0u && out.data_len == STS_FB_DATA_LEN, "feedback header");

        const int16_t pos = rlca_get_i16(&out.data[0]);
        const int16_t spd = rlca_get_i16(&out.data[2]);
        const int16_t load = rlca_get_i16(&out.data[4]);
        CHECK_EQ_D(pos, 2048);
        CHECK_EQ_D(spd, 16);
        CHECK_EQ_D(load, 0x03FF);
        CHECK_EQ_U(out.data[6], 120u);
        CHECK_EQ_U(out.data[7], 42u);

        /* 位置 → 关节角（offset=0, dir=+1） */
        rlca_calib_t c;
        rlca_calib_load_defaults(&c);
        const float q = rlca_step_to_q(&c, RLCA_J_LF_THIGH, pos);
        CHECK(fabsf(q - (2048.0f * RLCA_SERVO_RAD_PER_STEP)) < 1e-5f, "pos -> rad conversion");

        /* ---- 校验和错误必须被识别 ---- */
        fb[14] ^= 0xFFu;
        CHECK(rlca_sts_parse_feedback(fb, fb_len, 5u, STS_FB_DATA_LEN, &out) == STS_FB_ERR_CHK,
              "bad checksum must be detected");
        fb[14] ^= 0xFFu;

        /* ---- ID 不符必须被识别（多舵机总线上区分回复） ---- */
        CHECK(rlca_sts_parse_feedback(fb, fb_len, 6u, STS_FB_DATA_LEN, &out) == STS_FB_ERR_ID,
              "wrong id must be detected");
        CHECK(rlca_sts_parse_feedback(fb, fb_len, STS_BROADCAST_ID, STS_FB_DATA_LEN, &out) == STS_FB_OK,
              "broadcast expected id must skip the id check");

        /* ---- LEN 不符必须被识别 ---- */
        CHECK(rlca_sts_parse_feedback(fb, fb_len, 5u, 2u, &out) == STS_FB_ERR_LEN,
              "wrong data length must be detected");

        /*==========================================================================
         * ⭐⭐ 回归：真实 PING 应答帧必须能被解析
         *
         * ⚠️⚠️ 这一段记录了一次【我把正确常量改错】的全过程（2026-10-11）：
         *     ① 代码原为 exp_len = 0。
         *     ② 我拿一帧来源可疑的抓包（FF FF 01 04 03 28 00 CF，LEN=4）
         *        当标准，把常量改成 2 —— 但那帧是 USB-TTL 抢总线时的产物。
         *     ③ 后果：`A` 命令 12 个 ID 全部误报 no response。
         *     ④ 用 Y 命令（绕过解析、直接 dump 字节）实测真实应答是
         *        FF FF 05 02 00 F8（LEN=2 → data_len=0）
         *        → **原来的 exp_len = 0 才是对的**。
         *
         *  ⭐ 判据：用【实测】的帧，不用"看起来像标准"的帧。
         *========================================================================*/
        {
            /* 实测的真实 PING 应答（Y 命令 dump，来源可靠） */
            static const uint8_t ping_fb[6] = {
                0xFFu, 0xFFu, 0x05u, 0x02u, 0x00u, 0xF8u
            };
            sts_feedback_t pf;

            /* ① exp_len = STS_PING_FB_DATA_LEN（= 0）必须解析成功 */
            const sts_fb_result_t pr = rlca_sts_parse_feedback(ping_fb, sizeof(ping_fb),
                                                               STS_BROADCAST_ID,
                                                               STS_PING_FB_DATA_LEN, &pf);
            CHECK((pr == STS_FB_OK) || (pr == STS_FB_ERR_SERVO),
                  "real PING reply (LEN=2) must parse with exp_len=%u (got %d)",
                  (unsigned)STS_PING_FB_DATA_LEN, (int)pr);

            /* ② ⚠️ 回归断言：exp_len = 2 必须【失败】——
             *    那正是我改错后导致的坏状态，谁改回去这里就红。 */
            CHECK(rlca_sts_parse_feedback(ping_fb, sizeof(ping_fb),
                                          STS_BROADCAST_ID, 2u, &pf) == STS_FB_ERR_LEN,
                  "exp_len=2 must be rejected: real PING LEN is 2 -> data_len 0");

            /* ③ 常量自洽：LEN=2 → data_len = 0 */
            CHECK(STS_PING_FB_LEN == 2u, "STS_PING_FB_LEN must be 2");
            CHECK(STS_PING_FB_DATA_LEN == 0u,
                  "STS_PING_FB_DATA_LEN must be 0 (LEN - 2)");
        }

        /* ---- 帧头错误 ---- */
        fb[0] = 0x00u;
        CHECK(rlca_sts_parse_feedback(fb, fb_len, 5u, STS_FB_DATA_LEN, &out) == STS_FB_ERR_HEAD,
              "bad header must be detected");
        fb[0] = 0xFFu;

        /* ---- 舵机 ERR 非零：帧本身正常，但要能读出来 ---- */
        fb[4] = STS_ERR_OVERHEAT;
        {
            uint32_t sum = (uint32_t)fb[2] + (uint32_t)fb[3] + (uint32_t)fb[4];
            for (uint8_t i = 0; i < STS_FB_DATA_LEN; ++i) {
                sum += fb[5u + i];
            }
            fb[14] = (uint8_t)(~sum & 0xFFu);
        }
        const sts_fb_result_t r2 = rlca_sts_parse_feedback(fb, fb_len, 5u, STS_FB_DATA_LEN, &out);
        CHECK(r2 == STS_FB_ERR_SERVO, "non-zero ERR must be reported (r=%d)", (int)r2);
        CHECK(out.err == STS_ERR_OVERHEAT, "ERR byte must be readable");

        /* ---- 截断帧不能误判为成功 ---- */
        CHECK(rlca_sts_parse_feedback(fb, 5u, 5u, STS_FB_DATA_LEN, &out) != STS_FB_OK,
              "truncated feedback must not succeed");
        CHECK(rlca_sts_parse_feedback(NULL, fb_len, 5u, STS_FB_DATA_LEN, &out) == STS_FB_ERR_HEAD,
              "NULL buffer must be rejected");
    }

    /* ---- 反馈帧总长公式（⚠️ 这是最容易算错的一处） ----
     * 帧结构： H0 H1 ID LEN ERR DATA[9] CHK
     *        = 2 + 1 + 1 + 1 + 9 + 1 = 15 字节
     * ⚠️ 比指令帧多一个 ERR 字节；CHK 也常被漏算。
     *    P3 阶段曾把这里写成 14（漏 CHK），导致 12 帧反馈被全部丢弃。 */
    CHECK_EQ_U(STS_FB_FRAME_LEN, 15u);
    CHECK_EQ_U(STS_FB_DATA_LEN + 2u, 11u);       /* LEN = 数据 + ERR + CHK */
    CHECK_EQ_U(STS_FB_DATA_LEN + 6u, 15u);       /* = 头2+ID+LEN+ERR+数据+CHK */
    {
        /* 用真实的构造/解析路径端到端验一次长度 */
        uint8_t d9[STS_FB_DATA_LEN];
        rlca_sts_pack_fb_data(d9, 1234, -5, 0x03FF, 115, 41, 1u);
        uint8_t frame[32];
        const uint8_t n = rlca_sts_build_feedback(frame, 9u, 0u, d9);
        CHECK_EQ_U(n, STS_FB_FRAME_LEN);
        CHECK_EQ_U(n, 15u);
        /* 只给 14 字节必须被拒绝（这正是之前失败的形态） */
        sts_feedback_t f2;
        CHECK(rlca_sts_parse_feedback(frame, 14u, 9u, STS_FB_DATA_LEN, &f2) != STS_FB_OK,
              "14 bytes must NOT be accepted as a feedback frame");
    }
}

/*==============================================================================
 * 6. 配置表自洽性
 *============================================================================*/

static void test_config(void)
{
    section("config invariants");

    /* 控制周期 */
    CHECK_EQ_U(RLCA_CTRL_PERIOD_MS, 10u);
    CHECK_EQ_U(RLCA_N_JOINTS, 12u);

    /* 相位：周期 100 与步长 3 互质 → 周期恰好 100 个控制周期 */
    CHECK_EQ_U(RLCA_PHASE_PERIOD % 3u, 1u);   /* 100 mod 3 = 1（互质） */

    /* 协议：CRC 覆盖范围 = LEN + 3 字节；帧总长 = 7 + LEN */
    CHECK_EQ_U(RLCA_FRAME_OVERHEAD, 7u);
    CHECK_EQ_U(RLCA_LEN_TEL_FAST + RLCA_FRAME_OVERHEAD, 103u);
    CHECK_EQ_U(RLCA_LEN_CMD_JOINT + RLCA_FRAME_OVERHEAD, 38u);
    CHECK_EQ_U(RLCA_LEN_TEL_STATUS + RLCA_FRAME_OVERHEAD, 47u);

    /* TEL_FAST 偏移表的最后一个字段必须正好落到 LEN-1 */
    CHECK_EQ_U(RLCA_TF_OFF_ERR_FLAGS + 1u, RLCA_LEN_TEL_FAST);

    /* CMD_JOINT 的 kp_scale 偏移必须是 2 + 24 + 1 = 27 */
    CHECK_EQ_U(RLCA_CJ_OFF_KP_SCALE, 27u);

    /* 舵机 ID 不重复且在广播地址之外 */
    for (uint8_t i = 0; i < RLCA_N_JOINTS; ++i) {
        const uint8_t id = RLCA_SERVO_ID(i);
        CHECK(id >= RLCA_SERVO_ID_MIN && id <= RLCA_SERVO_ID_MAX, "servo id %u out of range", (unsigned)id);
        CHECK(id != STS_BROADCAST_ID, "servo id must not be broadcast");
        for (uint8_t j = (uint8_t)(i + 1u); j < RLCA_N_JOINTS; ++j) {
            CHECK(id != RLCA_SERVO_ID(j), "duplicate servo id %u", (unsigned)id);
        }
    }

    /* SyncWrite 12 舵机需要 7 + 3×12 = 43 字节（+2 头 = 45） */
    CHECK(45u <= 6u + STS_MAX_PAYLOAD + 16u, "STS_MAX_PAYLOAD must fit a 12-servo sync write");
    CHECK(21u <= 8u + 254u, "sync read capacity");

    /* 观测缩放：dof_vel 必须是 0.1（不是 upstream 的 0.05） */
    CHECK(fabsf(RLCA_OBS_SCALE_DOF_VEL - 0.1f) < 1e-7f, "dof_vel scale must be 0.1");
    CHECK(fabsf(RLCA_OBS_SCALE_ACTION - 4.0f) < 1e-7f, "action scale inversion must be 4.0");
    CHECK(fabsf(RLCA_ACTION_SCALE - 0.25f) < 1e-7f, "action_scale must be 0.25");
    CHECK(fabsf(RLCA_OBS_SCALE_ACTION * RLCA_ACTION_SCALE - 1.0f) < 1e-6f,
          "obs action scale must be the inverse of action_scale");

    /* 电压阈值必须单调递减 */
    CHECK(RLCA_BATT_WARN_MV > RLCA_BATT_STOP_MV, "warn > stop");
    CHECK(RLCA_BATT_STOP_MV > RLCA_BATT_FORCE_MV, "stop > force");

    /* 失联阈值 */
    CHECK(RLCA_LINK_LOST_MS < RLCA_LINK_ESTOP_MS, "degrade must trigger before estop");

    /* RAM 预算：UART 环缓 + 舵机 DMA 缓冲必须留有余量 */
    const uint32_t ram = RLCA_UART1_RX_BUF + RLCA_UART1_TX_BUF
                       + RLCA_UART2_RX_BUF + RLCA_UART2_TX_BUF
                       + RLCA_UART3_RX_BUF + RLCA_SERVO_DMA_RX_BUF;
    printf("    static buffers so far = %u bytes\n", (unsigned)ram);
    CHECK(ram < 4096u, "static buffers %u exceed 4 KB", (unsigned)ram);
}

/*==============================================================================
 * main
 *============================================================================*/

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--full") == 0) {
            g_full = 1;
        }
    }

    printf("========================================\n");
    printf(" RL-CAT pure-logic unit tests%s\n", g_full ? " (--full)" : "");
    printf("========================================\n");
    fflush(stdout);

    test_crc();
    test_frame_roundtrip();
    test_frame_decoder();
    test_phase();
    test_math();
    test_sts();
    test_config();

    printf("\n========================================\n");
    printf(" PASS %d   FAIL %d\n", g_pass, g_fail);
    printf("========================================\n");

    return (g_fail == 0) ? 0 : 1;
}
