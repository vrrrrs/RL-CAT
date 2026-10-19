/**
 * On-target self test (see rlca_selftest.h).
 */
#include <stdio.h>
#include <string.h>

#include "rlca_esp32_config.h"
#include "rlca_protocol.h"
#include "rlca_selftest.h"

/* deterministic pseudo random (no libc rand: keeps this reproducible) */
static uint32_t s_rng = 0x12345678u;
static uint32_t xs32(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

/* slow reference CRC, used to validate the table (protocol §5) */
static uint16_t crc_bitwise(const uint8_t *d, uint16_t len)
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
    uint32_t frames;
    uint8_t  last_type;
    uint8_t  last_seq;
} collect_t;

static void collect_cb(const rlca_frame_t *f, void *ctx)
{
    collect_t *c = (collect_t *)ctx;
    c->frames++;
    c->last_type = f->type;
    c->last_seq  = f->seq;
}

bool rlca_selftest_run(char *report, size_t report_len)
{
    char fail[64];
    fail[0] = '\0';
    int checks = 0;

#define SELFTEST_CHECK(cond, what)                        \
    do {                                                  \
        checks++;                                         \
        if (!(cond) && !fail[0]) {                        \
            snprintf(fail, sizeof(fail), "%s", what);     \
        }                                                 \
    } while (0)

    /* 1. CRC-16/CCITT-FALSE standard vector */
    SELFTEST_CHECK(rlca_crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1, "crc_vector");

    /* 2. CRC table vs bitwise reference over random buffers */
    {
        uint8_t buf[128];
        bool ok = true;
        for (int t = 0; t < 200 && ok; t++) {
            uint16_t len = (uint16_t)(xs32() % sizeof(buf));
            for (uint16_t i = 0; i < len; i++) {
                buf[i] = (uint8_t)xs32();
            }
            if (rlca_crc16_ccitt(buf, len) != crc_bitwise(buf, len)) {
                ok = false;
            }
        }
        SELFTEST_CHECK(ok, "crc_table_vs_bitwise");
    }

    /* 3. empty-payload frame round trip (HEARTBEAT) */
    {
        uint8_t frame[RLCA_FRAME_MAX_TOTAL];
        uint16_t n = rlca_frame_build(frame, RLCA_TYPE_HEARTBEAT, 7, NULL, 0);
        rlca_rx_t rx;
        collect_t c = {0};
        rlca_rx_init(&rx);
        rlca_rx_feed(&rx, frame, n, 0, collect_cb, &c);
        SELFTEST_CHECK(n == 7 && c.frames == 1 && c.last_type == RLCA_TYPE_HEARTBEAT &&
                       c.last_seq == 7, "frame_roundtrip");
    }

    /* 4. TEL_FAST offsets: status must be at frame offset 99, values must survive */
    {
        uint8_t pay[RLCA_LEN_TEL_FAST];
        memset(pay, 0, sizeof(pay));
        rlca_wr_i16(&pay[RLCA_TF_OFF_Q], 1234);
        pay[RLCA_TF_OFF_CYCLE]     = 0x34;
        pay[RLCA_TF_OFF_CYCLE + 1] = 0x12;
        pay[RLCA_TF_OFF_STATUS]    = RLCA_STAT_READY | RLCA_STAT_LINK_OK;

        uint8_t frame[RLCA_FRAME_MAX_TOTAL];
        uint16_t n = rlca_frame_build(frame, RLCA_TYPE_TEL_FAST, 1, pay, RLCA_LEN_TEL_FAST);

        rlca_rx_t rx;
        collect_t c = {0};
        rlca_rx_init(&rx);
        rlca_rx_feed(&rx, frame, n, 0, collect_cb, &c);

        rlca_frame_t f;
        f.type = RLCA_TYPE_TEL_FAST;
        f.seq  = 1;
        f.len  = RLCA_LEN_TEL_FAST;
        memcpy(f.payload, pay, RLCA_LEN_TEL_FAST);
        rlca_tel_fast_t tf;
        bool ok = rlca_tel_fast_parse(&f, &tf);

        SELFTEST_CHECK(n == 103 && c.frames == 1 && ok && tf.q[0] == 1234 &&
                       tf.cycle_cnt == 0x1234 &&
                       frame[RLCA_TF_STATUS_FRAME_OFF] == (RLCA_STAT_READY | RLCA_STAT_LINK_OK) &&
                       RLCA_TF_STATUS_FRAME_OFF == 99, "tel_fast_offsets");
    }

    /* 5. LEN > 128 is rejected and the parser recovers */
    {
        uint8_t good[RLCA_FRAME_MAX_TOTAL];
        uint16_t gn = rlca_frame_build(good, RLCA_TYPE_CMD_MODE, 9, (const uint8_t *)"\x02\x00", 2);

        uint8_t stream[32];
        uint16_t k = 0;
        stream[k++] = 0xAA;
        stream[k++] = 0x55;
        stream[k++] = 0xFF;                        /* illegal LEN */
        for (int i = 0; i < 6; i++) {
            stream[k++] = (uint8_t)i;
        }

        rlca_rx_t rx;
        collect_t c = {0};
        rlca_rx_init(&rx);
        rlca_rx_feed(&rx, stream, k, 0, collect_cb, &c);
        rlca_rx_feed(&rx, good, gn, 0, collect_cb, &c);
        SELFTEST_CHECK(rx.len_err == 1 && c.frames == 1 && c.last_type == RLCA_TYPE_CMD_MODE,
                       "len_reject_recover");
    }

    /* 6. CMD_JOINT quantisation round trip */
    {
        float q[RLCA_NUM_JOINTS];
        for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
            q[i] = (float)(i - 5) * 0.1f;
        }
        uint8_t pay[RLCA_LEN_CMD_JOINT];
        rlca_cmd_joint_payload(pay, RLCA_MODE_ACTIVE, RLCA_FLAG_ENABLE, q, 100, 1);

        bool ok = true;
        for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
            if (rlca_rd_i16(&pay[RLCA_CJ_OFF_Q + 2 * i]) != (int16_t)((i - 5) * 1000)) {
                ok = false;
            }
        }
        SELFTEST_CHECK(ok && pay[RLCA_CJ_OFF_MODE] == RLCA_MODE_ACTIVE &&
                       pay[RLCA_CJ_OFF_FLAGS] == RLCA_FLAG_ENABLE, "cmd_joint_quantise");
    }

#undef SELFTEST_CHECK

    if (report && report_len) {
        if (fail[0]) {
            snprintf(report, report_len, "selftest: FAIL at %s (%d checks)", fail, checks);
        } else {
            snprintf(report, report_len, "selftest: %d/%d checks passed", checks, checks);
        }
    }
    return fail[0] == '\0';
}
