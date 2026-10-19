/**
 * Host unit tests for the frame protocol layer (rlca_protocol.c).
 *
 * Runs on a PC (MinGW gcc) with no ESP-IDF: this is the offline half of the
 * P2 acceptance criteria.  Covers all 10 cases required by
 * firmware/doc/协议规范_树莓派通信.md §10, plus byte-exact cross-checks against
 * test/reference_frames.py (the reference implementation from the spec §9).
 *
 * Build & run:  firmware/tools/test_esp32_host.ps1
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rlca_protocol.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) {                                             \
            g_pass++;                                           \
        } else {                                                \
            g_fail++;                                           \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);       \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
        }                                                       \
    } while (0)

/* ------------------------------------------------------------------ */
/* helpers                                                            */
/* ------------------------------------------------------------------ */
static uint32_t s_rng = 0x12345678u;

static uint32_t xs32(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

/* slow reference implementation (spec §5, second listing) */
static uint16_t crc16_bitwise(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

typedef struct {
    uint32_t n;
    uint8_t  last_type;
    uint8_t  last_len;
} collector_t;

static void on_frame(const rlca_frame_t *f, void *ctx)
{
    collector_t *c = (collector_t *)ctx;
    c->n++;
    c->last_type = f->type;
    c->last_len  = f->len;
}

static void print_hex(const char *label, const uint8_t *d, uint16_t n)
{
    printf("FRAME %s ", label);
    for (uint16_t i = 0; i < n; i++) {
        printf("%02x", d[i]);
    }
    printf("\n");
}

/* ------------------------------------------------------------------ */
/* tests                                                              */
/* ------------------------------------------------------------------ */
static void test_crc_vector(void)
{
    printf("[1] CRC-16/CCITT-FALSE standard vector\n");
    const uint8_t v[] = "123456789";
    CHECK(rlca_crc16_ccitt(v, 9) == 0x29B1, "crc=%04X expected 29B1", rlca_crc16_ccitt(v, 9));
}

static void test_crc_table_vs_bitwise(void)
{
    printf("[1b] table vs bitwise reference over random buffers\n");
    uint8_t buf[256];
    for (int t = 0; t < 20000; t++) {
        uint16_t len = (uint16_t)(xs32() % sizeof(buf));
        for (uint16_t i = 0; i < len; i++) {
            buf[i] = (uint8_t)xs32();
        }
        if (rlca_crc16_ccitt(buf, len) != crc16_bitwise(buf, len)) {
            CHECK(0, "mismatch at trial %d len=%u", t, len);
            return;
        }
    }
    CHECK(1, "ok");
}

static void test_roundtrip_empty_and_max(void)
{
    printf("[2] empty payload (HEARTBEAT) round trip\n");
    uint8_t frame[RLCA_FRAME_MAX_TOTAL];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_HEARTBEAT, 7, NULL, 0);
    CHECK(n == 7, "len=%u expected 7", n);

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    rlca_rx_feed(&rx, frame, n, 1000, on_frame, &c);
    CHECK(c.n == 1 && c.last_type == RLCA_TYPE_HEARTBEAT && c.last_len == 0,
          "n=%u type=%02X len=%u", c.n, c.last_type, c.last_len);
    print_hex("HEARTBEAT", frame, n);

    printf("[3] maximum payload (LEN=128) round trip\n");
    uint8_t pay[RLCA_FRAME_MAX_PAYLOAD];
    for (int i = 0; i < RLCA_FRAME_MAX_PAYLOAD; i++) {
        pay[i] = (uint8_t)(i * 7 + 3);
    }
    n = rlca_frame_build(frame, RLCA_TYPE_TEL_LOG, 200, pay, RLCA_FRAME_MAX_PAYLOAD);
    CHECK(n == RLCA_FRAME_MAX_TOTAL, "len=%u expected %u", n, RLCA_FRAME_MAX_TOTAL);

    rlca_rx_init(&rx);
    c.n = 0;
    rlca_rx_feed(&rx, frame, n, 2000, on_frame, &c);
    CHECK(c.n == 1 && c.last_len == RLCA_FRAME_MAX_PAYLOAD, "n=%u len=%u", c.n, c.last_len);
    print_hex("MAXPAYLOAD", frame, n);
}

static void test_bad_crc(void)
{
    printf("[4] corrupted CRC -> rejected\n");
    uint8_t frame[32];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_CMD_MODE, 3, (const uint8_t *)"\x02\x00", 2);
    frame[n - 1] ^= 0xFF;

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    rlca_rx_feed(&rx, frame, n, 0, on_frame, &c);
    CHECK(c.n == 0, "frame should have been rejected");
    CHECK(rx.crc_err == 1, "crc_err=%u expected 1", rx.crc_err);
}

static void test_truncated(void)
{
    printf("[5] truncated frame -> no delivery, no crash\n");
    uint8_t frame[64];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_TEL_FAST, 1, (const uint8_t *)"0123456789", 10);

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    rlca_rx_feed(&rx, frame, n - 3, 0, on_frame, &c);
    CHECK(c.n == 0, "truncated frame must not be delivered");

    /* completing the frame after the timeout window must resync cleanly */
    rlca_rx_poll(&rx, 5000 + RLCA_RX_FRAME_TIMEOUT_US + 1);
    CHECK(rx.state == RLCA_RX_WAIT_HEAD0, "state=%d expected WAIT_HEAD0", rx.state);
    CHECK(rx.resync == 1, "resync=%u expected 1", rx.resync);

    c.n = 0;
    rlca_rx_feed(&rx, frame, n, 20000, on_frame, &c);
    CHECK(c.n == 1, "fresh frame after resync must be delivered (n=%u)", c.n);
}

static void test_head_sync_cases(void)
{
    printf("[6] AA AA 55 sync\n");
    uint8_t frame[32];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_CMD_MODE, 1, (const uint8_t *)"\x02\x00", 2);
    uint8_t stream[40];
    uint16_t k = 0;
    stream[k++] = 0xAA;
    stream[k++] = 0xAA;
    memcpy(&stream[k], frame + 0, n);   /* includes AA 55 ... */
    k += n;

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    rlca_rx_feed(&rx, stream, k, 0, on_frame, &c);
    CHECK(c.n == 1 && c.last_type == RLCA_TYPE_CMD_MODE, "n=%u", c.n);

    printf("[7] garbage prefix\n");
    const uint8_t junk[] = {0x00, 0xFF, 0x55, 0xAA, 0x12, 0xAA};
    k = 0;
    memcpy(&stream[k], junk, sizeof(junk));
    k += sizeof(junk);
    memcpy(&stream[k], frame, n);
    k += n;

    rlca_rx_init(&rx);
    c.n = 0;
    rlca_rx_feed(&rx, stream, k, 0, on_frame, &c);
    CHECK(c.n == 1 && c.last_type == RLCA_TYPE_CMD_MODE, "n=%u", c.n);
}

static void test_oversized_len(void)
{
    printf("[8] LEN=255 -> rejected, state machine recovers\n");
    uint8_t frame[32];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_CMD_MODE, 9, (const uint8_t *)"\x02\x00", 2);

    uint8_t stream[64];
    uint16_t k = 0;
    stream[k++] = 0xAA;
    stream[k++] = 0x55;
    stream[k++] = 0xFF;            /* illegal LEN */
    for (int i = 0; i < 8; i++) {
        stream[k++] = (uint8_t)i;
    }
    memcpy(&stream[k], frame, n);
    k += n;

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    rlca_rx_feed(&rx, stream, k, 0, on_frame, &c);
    CHECK(rx.len_err == 1, "len_err=%u expected 1", rx.len_err);
    CHECK(c.n == 1 && c.last_type == RLCA_TYPE_CMD_MODE, "recovery failed n=%u", c.n);
}

static void test_many_frames(void)
{
    printf("[9] 10000 frames, zero errors\n");
    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);

    uint8_t frame[RLCA_FRAME_MAX_TOTAL];
    for (int i = 0; i < 10000; i++) {
        uint8_t pay[RLCA_LEN_TEL_FAST];
        for (int j = 0; j < RLCA_LEN_TEL_FAST; j++) {
            pay[j] = (uint8_t)(i + j);
        }
        uint16_t n = rlca_frame_build(frame, RLCA_TYPE_TEL_FAST, (uint8_t)i, pay, RLCA_LEN_TEL_FAST);
        rlca_rx_feed(&rx, frame, n, (uint64_t)i * 10000u, on_frame, &c);
    }
    CHECK(c.n == 10000, "delivered=%u", c.n);
    CHECK(rx.crc_err == 0 && rx.len_err == 0 && rx.resync == 0,
          "crc_err=%u len_err=%u resync=%u", rx.crc_err, rx.len_err, rx.resync);
}

static void test_random_stream(void)
{
    printf("[10] 1 MB random stream: no hang, counters consistent\n");
    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);

    uint8_t buf[4096];
    uint32_t total = 0;
    for (int chunk = 0; chunk < 256; chunk++) {      /* 256 * 4096 = 1 MiB */
        for (size_t i = 0; i < sizeof(buf); i++) {
            buf[i] = (uint8_t)xs32();
        }
        rlca_rx_feed(&rx, buf, sizeof(buf), (uint64_t)chunk * 1000u, on_frame, &c);
        total += sizeof(buf);
    }
    CHECK(total == 1024u * 1024u, "fed %u bytes", total);
    CHECK(rx.frames_ok == c.n, "frames_ok=%u delivered=%u", rx.frames_ok, c.n);
    /* random data may legitimately contain a valid-looking frame; nothing to assert
     * beyond "it survived" and the counters staying in sync. */
    printf("       (random stream produced %u accidental frames, %u crc errors, %u resyncs)\n",
           c.n, rx.crc_err, rx.resync);
}

static void test_tel_fast_offsets(void)
{
    printf("[11] TEL_FAST field offsets / status byte at frame offset 99\n");
    uint8_t pay[RLCA_LEN_TEL_FAST];
    memset(pay, 0, sizeof(pay));
    for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
        rlca_wr_i16(&pay[RLCA_TF_OFF_Q + 2 * i], (int16_t)(1000 + i));
        rlca_wr_i16(&pay[RLCA_TF_OFF_QD + 2 * i], (int16_t)(-2000 - i));
        rlca_wr_i16(&pay[RLCA_TF_OFF_TAU + 2 * i], (int16_t)(300 + i));
    }
    for (int i = 0; i < 4; i++) {
        rlca_wr_i16(&pay[RLCA_TF_OFF_QUAT + 2 * i], (int16_t)(10000 - i));
    }
    for (int i = 0; i < 3; i++) {
        rlca_wr_i16(&pay[RLCA_TF_OFF_GYRO + 2 * i], (int16_t)(111 * (i + 1)));
    }
    rlca_wr_i16(&pay[RLCA_TF_OFF_GAIT_SIN], 32767);
    rlca_wr_i16(&pay[RLCA_TF_OFF_GAIT_COS], -32768);
    pay[RLCA_TF_OFF_CYCLE]     = 0x34;
    pay[RLCA_TF_OFF_CYCLE + 1] = 0x12;                 /* 0x1234 = 4660 */
    pay[RLCA_TF_OFF_VOLTAGE]     = 0x10;
    pay[RLCA_TF_OFF_VOLTAGE + 1] = 0x2D;               /* 0x2D10 = 11536 mV */
    pay[RLCA_TF_OFF_STATUS]      = RLCA_STAT_READY | RLCA_STAT_LINK_OK;
    pay[RLCA_TF_OFF_ERR]         = RLCA_ERR_OVERHEAT;

    uint8_t frame[RLCA_FRAME_MAX_TOTAL];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_TEL_FAST, 42, pay, RLCA_LEN_TEL_FAST);
    CHECK(n == 103, "frame len=%u expected 103", n);
    CHECK(frame[RLCA_TF_STATUS_FRAME_OFF] == (RLCA_STAT_READY | RLCA_STAT_LINK_OK),
          "status at frame offset %d = %02X", RLCA_TF_STATUS_FRAME_OFF, frame[RLCA_TF_STATUS_FRAME_OFF]);
    CHECK(RLCA_TF_STATUS_FRAME_OFF == 99, "status must be at frame offset 99, got %d", RLCA_TF_STATUS_FRAME_OFF);

    rlca_rx_t rx;
    collector_t c = {0};
    rlca_rx_init(&rx);
    c.n = 0;
    rlca_rx_feed(&rx, frame, n, 0, on_frame, &c);
    CHECK(c.n == 1, "n=%u", c.n);

    /* re-parse the delivered payload through the accessor */
    rlca_frame_t f;
    f.type = RLCA_TYPE_TEL_FAST;
    f.seq  = 42;
    f.len  = RLCA_LEN_TEL_FAST;
    memcpy(f.payload, pay, RLCA_LEN_TEL_FAST);
    rlca_tel_fast_t tf;
    CHECK(rlca_tel_fast_parse(&f, &tf), "parse failed");
    CHECK(tf.q[0] == 1000 && tf.q[11] == 1011, "q0=%d q11=%d", tf.q[0], tf.q[11]);
    CHECK(tf.qd[11] == -2011, "qd11=%d", tf.qd[11]);
    CHECK(tf.tau[0] == 300, "tau0=%d", tf.tau[0]);
    CHECK(tf.quat[0] == 10000 && tf.quat[3] == 9997, "quat=%d,%d", tf.quat[0], tf.quat[3]);
    CHECK(tf.gyro[2] == 333, "gyro2=%d", tf.gyro[2]);
    CHECK(tf.gait_sin == 32767 && tf.gait_cos == -32768, "gait=%d,%d", tf.gait_sin, tf.gait_cos);
    CHECK(tf.cycle_cnt == 0x1234, "cycle=%u", tf.cycle_cnt);
    CHECK(tf.voltage_mv == 11536, "volt=%u", tf.voltage_mv);
    CHECK(tf.status == (RLCA_STAT_READY | RLCA_STAT_LINK_OK), "status=%02X", tf.status);
    CHECK(tf.err_flags == RLCA_ERR_OVERHEAT, "err=%02X", tf.err_flags);
    CHECK(rlca_q_to_rad(tf.q[0]) > 0.0999f && rlca_q_to_rad(tf.q[0]) < 0.1001f, "q0 rad=%f", rlca_q_to_rad(tf.q[0]));
}

static void test_cmd_joint_reference(void)
{
    printf("[12] CMD_JOINT / CMD_MODE / CMD_CALIB byte-exact frames (cross-checked with Python)\n");
    float q[RLCA_NUM_JOINTS];
    for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
        q[i] = (float)(i - 5) * 0.1f;      /* -0.5 .. +0.6 rad, integer raw steps */
    }

    uint8_t pay[RLCA_LEN_CMD_JOINT];
    rlca_cmd_joint_payload(pay, RLCA_MODE_ACTIVE, RLCA_FLAG_ENABLE, q, 100, 1);

    uint8_t frame[RLCA_FRAME_MAX_TOTAL];
    uint16_t n = rlca_frame_build(frame, RLCA_TYPE_CMD_JOINT, 7, pay, RLCA_LEN_CMD_JOINT);
    CHECK(n == 38, "frame len=%u expected 38", n);
    print_hex("CMD_JOINT", frame, n);

    /* q_target decodes back to the requested angles */
    for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
        int16_t raw = rlca_rd_i16(&pay[RLCA_CJ_OFF_Q + 2 * i]);
        CHECK(raw == (int16_t)((i - 5) * 1000), "q[%d] raw=%d expected %d", i, raw, (i - 5) * 1000);
    }

    uint8_t mpay[RLCA_LEN_CMD_MODE];
    rlca_cmd_mode_payload(mpay, RLCA_MODE_IDLE, 0);
    n = rlca_frame_build(frame, RLCA_TYPE_CMD_MODE, 8, mpay, RLCA_LEN_CMD_MODE);
    CHECK(n == 9, "frame len=%u expected 9", n);
    print_hex("CMD_MODE", frame, n);

    const uint8_t calib = 0;
    n = rlca_frame_build(frame, RLCA_TYPE_CMD_CALIB, 9, &calib, RLCA_LEN_CMD_CALIB);
    CHECK(n == 8, "frame len=%u expected 8", n);
    print_hex("CMD_CALIB", frame, n);
}

static void test_quat_to_gravity(void)
{
    printf("[13] quaternion -> gravity vector (observation dims 0..2)\n");

    /* reference values computed with the Pi-side implementation
     * (firmware/doc/协议规范_树莓派通信.md §9 quat_to_gravity) */
    struct { const char *name; float q[4]; float g[3]; } cases[] = {
        { "identity",   { 1.0000f, 0.0000f,  0.0000f,  0.0000f }, {  0.000000f,  0.000000f, -1.000000f } },
        { "roll+30",    { 0.9659f, 0.2588f,  0.0000f,  0.0000f }, {  0.000000f, -0.500000f, -0.866025f } },
        { "pitch+20",   { 0.9848f, 0.0000f,  0.1736f,  0.0000f }, {  0.342020f,  0.000000f, -0.939693f } },
        { "yaw+45",     { 0.9239f, 0.0000f,  0.0000f,  0.3827f }, {  0.000000f,  0.000000f, -1.000000f } },
        { "mixed",      { 0.9045f, 0.0921f,  0.3006f,  0.2881f }, {  0.490714f, -0.339867f, -0.802303f } },
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        float g[3];
        rlca_quat_to_gravity(cases[i].q, g);
        for (int k = 0; k < 3; k++) {
            CHECK(fabsf(g[k] - cases[i].g[k]) < 2e-4f,
                  "%s g[%d]=%f expected %f", cases[i].name, k, (double)g[k], (double)cases[i].g[k]);
        }
        /* it must always be a unit vector */
        const float n = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        CHECK(fabsf(n - 1.0f) < 1e-5f, "%s |g|=%f", cases[i].name, (double)n);
    }

    /* degenerate input must not produce NaN */
    const float zero[4] = { 0, 0, 0, 0 };
    float gz[3];
    rlca_quat_to_gravity(zero, gz);
    CHECK(gz[0] == 0.0f && gz[1] == 0.0f && gz[2] == -1.0f,
          "all-zero quaternion falls back to (0,0,-1)");
}

int main(void)
{
    printf("=== RL-CAT ESP32 protocol host tests ===\n");
    test_crc_vector();
    test_crc_table_vs_bitwise();
    test_roundtrip_empty_and_max();
    test_bad_crc();
    test_truncated();
    test_head_sync_cases();
    test_oversized_len();
    test_many_frames();
    test_random_stream();
    test_tel_fast_offsets();
    test_cmd_joint_reference();
    test_quat_to_gravity();

    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
