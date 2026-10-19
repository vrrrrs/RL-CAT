/**
 * Console command handling (see rlca_injector.h).
 *
 * Everything that would *change* the robot goes through rlca_arb_can_inject():
 * while the Pi is online, or while the Pi's state is unknown, injection is
 * refused with an explanation.  Status queries (`i`) never inject anything --
 * deliberately, because the STM32 firmware ignores FLAG_ENABLE (it only reads
 * FLAG_REQ_STATUS), so a "status request" frame would nevertheless carry a mode
 * and a q_target and could change the robot's state.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "rlca_arbiter.h"
#include "rlca_esp32_config.h"
#include "rlca_bridge.h"
#include "rlca_injector.h"
#include "rlca_protocol.h"

static const char *TAG = "inject";

static uint8_t s_seq;

/** Send one protocol frame to the STM32 and describe the result. */
static bool send_frame(uint8_t type, const uint8_t *payload, uint8_t len,
                       char *reply, size_t reply_len)
{
    uint8_t frame[RLCA_FRAME_MAX_TOTAL];
    uint16_t total = rlca_frame_build(frame, type, s_seq++, payload, len);
    if (!total) {
        snprintf(reply, reply_len, "err: payload too long\r\n");
        return true;
    }

    size_t written = rlca_bridge_write(frame, total);
    snprintf(reply, reply_len, "%s %s seq=%u len=%u -> uart %u/%u bytes\r\n",
             written == total ? "sent" : "partial",
             type == RLCA_TYPE_CMD_JOINT ? "CMD_JOINT" :
             type == RLCA_TYPE_CMD_MODE  ? "CMD_MODE" :
             type == RLCA_TYPE_CMD_CALIB ? "CMD_CALIB" :
             type == RLCA_TYPE_HEARTBEAT ? "HEARTBEAT" : "frame",
             (unsigned)(s_seq - 1), (unsigned)len,
             (unsigned)written, (unsigned)total);
    return true;
}

static bool gated(char *reply, size_t reply_len, const char *action)
{
    char why[128];
    if (rlca_arb_can_inject(why, sizeof(why))) {
        return false;
    }
    snprintf(reply, reply_len, "blocked: %s cannot inject (%s)\r\n", action, why);
    return true;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/** `raw <hex bytes>` -- push arbitrary bytes to the STM32.
 *  Handy for bench tests and for talking to the PC-side STM32 simulator. */
static bool cmd_raw(const char *rest, char *reply, size_t reply_len)
{
    uint8_t out[RLCA_CONSOLE_REPLY_MAX / 2];
    size_t  n = 0;

    while (*rest && n < sizeof(out)) {
        while (*rest == ' ' || *rest == '\t') {
            rest++;
        }
        if (!*rest) {
            break;
        }
        int hi = hex_nibble(rest[0]);
        int lo = rest[1] ? hex_nibble(rest[1]) : -1;
        if (hi < 0 || lo < 0) {
            snprintf(reply, reply_len, "err: bad hex at '%s'\r\n", rest);
            return true;
        }
        out[n++] = (uint8_t)((hi << 4) | lo);
        rest += 2;
    }

    if (!n) {
        snprintf(reply, reply_len, "usage: raw <hex bytes>   e.g. raw AA55007F...\r\n");
        return true;
    }

    size_t written = rlca_bridge_write(out, n);
    snprintf(reply, reply_len, "raw: %u/%u bytes queued to the STM32\r\n",
             (unsigned)written, (unsigned)n);
    return true;
}

/** `i` -- report link/telemetry state. Never injects (see the file header). */
static bool cmd_info(char *reply, size_t reply_len)
{
    rlca_arb_stats_t st;
    rlca_arb_snapshot(&st);
    char why[128];
    bool can = rlca_arb_can_inject(why, sizeof(why));

    char age[24];
    if (st.age_ms == 0xFFFFFFFFu) {
        snprintf(age, sizeof(age), "never");
    } else {
        snprintf(age, sizeof(age), "%u ms", (unsigned)st.age_ms);
    }

    snprintf(reply, reply_len,
             "link: frames=%u (TEL_FAST=%u TEL_STATUS=%u other=%u) crc_err=%u len_err=%u resync=%u\r\n"
             "      telemetry age=%s status=0x%02X mode=%u voltage=%u mV cycle=%u gaps=%u q=%s\r\n"
             "      pi_online=%d  inject=%s\r\n",
             (unsigned)st.frames, (unsigned)st.tel_fast, (unsigned)st.tel_status,
             (unsigned)st.other, (unsigned)st.crc_err, (unsigned)st.len_err,
             (unsigned)st.resync, age, st.status, st.mode,
             (unsigned)st.voltage_mv, (unsigned)st.cycle_cnt, (unsigned)st.cycle_gaps,
             st.have_q ? "yes" : "no",
             rlca_arb_pi_online() ? 1 : 0, can ? "allowed" : "blocked");
    (void)why;
    return true;
}

/** `s <joint 0..11> <delta degrees>` -- move one joint relative to its last
 *  reported position, reusing CMD_JOINT (no protocol change, works uncalibrated). */
static bool cmd_step(const char *rest, char *reply, size_t reply_len)
{
    char *end = NULL;
    long joint = strtol(rest, &end, 10);
    if (end == rest || joint < 0 || joint >= RLCA_NUM_JOINTS) {
        snprintf(reply, reply_len, "usage: s <joint 0..11> <delta degrees>\r\n");
        return true;
    }
    long ddeg = strtol(end, &end, 10);
    if (end == rest) {
        snprintf(reply, reply_len, "usage: s <joint 0..11> <delta degrees>\r\n");
        return true;
    }
    if (ddeg == 0) {
        snprintf(reply, reply_len, "err: delta is 0, nothing to do\r\n");
        return true;
    }
    if (ddeg > RLCA_STEP_MAX_DEG || ddeg < -RLCA_STEP_MAX_DEG) {
        snprintf(reply, reply_len, "err: |delta| must be <= %.0f deg\r\n",
                 (double)RLCA_STEP_MAX_DEG);
        return true;
    }

    const int16_t *qraw = rlca_arb_last_q();
    if (!qraw) {
        snprintf(reply, reply_len,
                 "err: no joint telemetry yet -- the STM32 must send TEL_FAST on USART3 "
                 "before a single joint can be moved safely\r\n");
        return true;
    }
    if (gated(reply, reply_len, "s")) {
        return true;
    }

    float q[RLCA_NUM_JOINTS];
    for (int i = 0; i < RLCA_NUM_JOINTS; i++) {
        q[i] = rlca_q_to_rad(qraw[i]);
    }
    q[joint] += (float)ddeg * RLCA_DEG2RAD;

    uint8_t payload[RLCA_LEN_CMD_JOINT];
    rlca_cmd_joint_payload(payload, RLCA_MODE_ACTIVE, RLCA_FLAG_ENABLE, q, 100, 1);

    bool ok = send_frame(RLCA_TYPE_CMD_JOINT, payload, RLCA_LEN_CMD_JOINT, reply, reply_len);
    if (ok) {
        char extra[96];
        snprintf(extra, sizeof(extra), "  joint %ld %+.1f deg -> q=%.3f rad (others held)\r\n",
                 joint, (double)ddeg, (double)q[joint]);
        strncat(reply, extra, reply_len - strlen(reply) - 1);
    }
    return true;
}

static bool cmd_mode(uint8_t mode, const char *action, char *reply, size_t reply_len)
{
    if (gated(reply, reply_len, action)) {
        return true;
    }
    uint8_t payload[RLCA_LEN_CMD_MODE];
    rlca_cmd_mode_payload(payload, mode, 0);
    return send_frame(RLCA_TYPE_CMD_MODE, payload, RLCA_LEN_CMD_MODE, reply, reply_len);
}

static bool cmd_calib(char *reply, size_t reply_len)
{
    if (gated(reply, reply_len, "c")) {
        return true;
    }
    const uint8_t action = 0;   /* 0 = start calibration (protocol §4.9) */
    return send_frame(RLCA_TYPE_CMD_CALIB, &action, RLCA_LEN_CMD_CALIB, reply, reply_len);
}

/** `imu` -- decode the IMU part of the newest TEL_FAST (quaternion + gyro) and
 *  show the gravity vector the policy would see in observation dims 0..2. */
static bool cmd_imu(char *reply, size_t reply_len)
{
    rlca_arb_stats_t st;
    rlca_arb_snapshot(&st);

    if (!st.have_imu) {
        snprintf(reply, reply_len,
                 "err: no IMU data yet -- the STM32 must send TEL_FAST on this link "
                 "(check the wiring and the baud rate)\r\n");
        return true;
    }

    const float q[4] = { st.quat[0] * RLCA_QUAT_SCALE, st.quat[1] * RLCA_QUAT_SCALE,
                         st.quat[2] * RLCA_QUAT_SCALE, st.quat[3] * RLCA_QUAT_SCALE };
    const float gyro[3] = { st.gyro[0] * RLCA_GYRO_SCALE, st.gyro[1] * RLCA_GYRO_SCALE,
                            st.gyro[2] * RLCA_GYRO_SCALE };
    float g[3];
    rlca_quat_to_gravity(q, g);

    snprintf(reply, reply_len,
             "imu: age=%u ms  cycle=%u\r\n"
             "     quat  w=%+.4f x=%+.4f y=%+.4f z=%+.4f\r\n"
             "     gyro  x=%+.3f y=%+.3f z=%+.3f rad/s\r\n"
             "     gravity(body) x=%+.3f y=%+.3f z=%+.3f   <- observation dims 0..2\r\n",
             (unsigned)(st.age_ms == 0xFFFFFFFFu ? 0 : st.age_ms),
             (unsigned)st.cycle_cnt,
             (double)q[0], (double)q[1], (double)q[2], (double)q[3],
             (double)gyro[0], (double)gyro[1], (double)gyro[2],
             (double)g[0], (double)g[1], (double)g[2]);
    return true;
}

bool rlca_injector_handle_line(const char *line, char *reply, size_t reply_len)
{
    if (!line || !reply || reply_len < 2) {
        return false;
    }
    reply[0] = '\0';

    if (!strcmp(line, "i") || !strcmp(line, "info")) {
        return cmd_info(reply, reply_len);
    }
    if (!strcmp(line, "imu") || !strcmp(line, "m")) {
        return cmd_imu(reply, reply_len);
    }
    if (!strcmp(line, "hb")) {
        return send_frame(RLCA_TYPE_HEARTBEAT, NULL, 0, reply, reply_len);
    }
    if (!strcmp(line, "z")) {
        return cmd_mode(RLCA_MODE_IDLE, "z (unload torque)", reply, reply_len);
    }
    if (!strcmp(line, "c")) {
        return cmd_calib(reply, reply_len);
    }
    if (!strcmp(line, "r")) {
        rlca_arb_reset_counters();
        snprintf(reply, reply_len, "local state reset (counters cleared, parser kept)\r\n");
        return true;
    }
    if (!strncmp(line, "raw", 3) && (line[3] == '\0' || line[3] == ' ')) {
        return cmd_raw(line + 3, reply, reply_len);
    }
    if (!strncmp(line, "s", 1) && (line[1] == '\0' || line[1] == ' ')) {
        return cmd_step(line + 1, reply, reply_len);
    }

    ESP_LOGD(TAG, "unhandled console line: %s", line);
    return false;
}
