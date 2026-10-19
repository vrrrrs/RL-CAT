/**
 * RL-CAT frame protocol -- shared with the STM32 and (later) the Pi.
 * Spec: firmware/doc/协议规范_树莓派通信.md
 *
 * This module is deliberately free of ESP-IDF / HAL dependencies so that it can
 * be compiled and unit-tested on a PC (see test/test_protocol.c).  It is the
 * same code that runs on the ESP32-C3.
 *
 * Wire format:
 *   AA 55 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC16(LE, over LEN..PAYLOAD)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "rlca_esp32_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xor 0)  */
/* ------------------------------------------------------------------ */
uint16_t rlca_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);
uint16_t rlca_crc16_ccitt(const uint8_t *data, uint16_t len);

/* ------------------------------------------------------------------ */
/* frames                                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[RLCA_FRAME_MAX_PAYLOAD];
} rlca_frame_t;

/** Build a complete frame into `out` (must hold 7 + len bytes).
 *  @return total frame length, or 0 if len > RLCA_FRAME_MAX_PAYLOAD. */
uint16_t rlca_frame_build(uint8_t *out, uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len);

/** Build a CMD_JOINT payload (LEN=31) from logical-index joint angles.
 *  q_rad[i] is the target for logical index i; NULL means "all zeros". */
void rlca_cmd_joint_payload(uint8_t payload[RLCA_LEN_CMD_JOINT],
                            uint8_t mode, uint8_t flags,
                            const float q_rad[RLCA_NUM_JOINTS],
                            int16_t kp_scale_centi, uint8_t led);

/** Build a CMD_MODE payload (LEN=2). */
void rlca_cmd_mode_payload(uint8_t payload[RLCA_LEN_CMD_MODE], uint8_t mode, uint8_t param);

/* ------------------------------------------------------------------ */
/* streaming receiver state machine (spec §6)                          */
/* ------------------------------------------------------------------ */
typedef enum {
    RLCA_RX_WAIT_HEAD0 = 0,
    RLCA_RX_WAIT_HEAD1,
    RLCA_RX_HEADER,      /* LEN, SEQ, TYPE */
    RLCA_RX_PAYLOAD,
    RLCA_RX_CRC,
} rlca_rx_state_t;

typedef struct {
    rlca_rx_state_t state;
    uint8_t  hdr[3];         /* LEN, SEQ, TYPE */
    uint8_t  hdr_idx;
    uint16_t pay_idx;
    uint8_t  crc_idx;
    uint8_t  crc_bytes[2];
    uint64_t last_byte_us;   /* for the inter-byte timeout */
    /* statistics */
    uint32_t frames_ok;
    uint32_t crc_err;
    uint32_t len_err;
    uint32_t resync;         /* timeout / desync resets */
    uint8_t  payload[RLCA_FRAME_MAX_PAYLOAD];
} rlca_rx_t;

typedef void (*rlca_frame_cb_t)(const rlca_frame_t *frame, void *ctx);

void rlca_rx_init(rlca_rx_t *rx);

/** Feed `n` bytes; invokes `cb` for every complete, CRC-valid frame.
 *  @return number of valid frames delivered. */
uint32_t rlca_rx_feed(rlca_rx_t *rx, const uint8_t *data, uint16_t n,
                      uint64_t now_us, rlca_frame_cb_t cb, void *ctx);

/** Call periodically: drops a half-received frame after RLCA_RX_FRAME_TIMEOUT_US. */
void rlca_rx_poll(rlca_rx_t *rx, uint64_t now_us);

#define RLCA_RX_FRAME_TIMEOUT_US 5000u   /* spec §6: 5 ms整帧超时 */

/* ------------------------------------------------------------------ */
/* TEL_FAST accessors (spec §4.4)                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    int16_t  q[RLCA_NUM_JOINTS];      /* 0.0001 rad   */
    int16_t  qd[RLCA_NUM_JOINTS];     /* 0.0001 rad/s */
    int16_t  tau[RLCA_NUM_JOINTS];    /* 0.001 N.m    */
    int16_t  quat[4];                 /* w,x,y,z 1e-4 */
    int16_t  gyro[3];                 /* 0.001 rad/s  */
    int16_t  gait_sin;                /* /32767       */
    int16_t  gait_cos;                /* /32767       */
    uint16_t cycle_cnt;
    uint16_t voltage_mv;
    uint8_t  status;
    uint8_t  err_flags;
} rlca_tel_fast_t;

static inline int16_t rlca_rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t rlca_rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void rlca_wr_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

/** Parse a TEL_FAST frame. @return false if the frame is not TEL_FAST. */
bool rlca_tel_fast_parse(const rlca_frame_t *f, rlca_tel_fast_t *out);

/** Parse a TEL_STATUS frame (spec §4.8) -- fields the ESP32 cares about. */
typedef struct {
    uint32_t uptime_ms;
    uint16_t voltage_mv;
    uint8_t  mode;
    uint8_t  status;
    uint8_t  err_flags;
    uint8_t  seq_lost;
} rlca_tel_status_t;

bool rlca_tel_status_parse(const rlca_frame_t *f, rlca_tel_status_t *out);

/* quantisation helpers (float, so kept in the header as inline) */
static inline float rlca_q_to_rad(int16_t raw)   { return (float)raw * RLCA_Q_SCALE; }
static inline int16_t rlca_rad_to_q(float rad)   { return (int16_t)(rad / RLCA_Q_SCALE + (rad >= 0 ? 0.5f : -0.5f)); }

/** Gravity direction in the body frame, from a quaternion (w,x,y,z).
 *  Exactly the formula the Pi side uses (协议规范 §9 quat_to_gravity), so the
 *  first three observation dimensions can be inspected from the ESP32 console
 *  without the Pi. */
static inline void rlca_quat_to_gravity(const float q[4], float g[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float gx = -2.0f * (x * z - w * y);
    const float gy = -2.0f * (y * z + w * x);
    const float gz = -(1.0f - 2.0f * (x * x + y * y));
    const float n = sqrtf(gx * gx + gy * gy + gz * gz);
    if (n < 1e-6f) {
        g[0] = 0.0f;
        g[1] = 0.0f;
        g[2] = -1.0f;
        return;
    }
    g[0] = gx / n;
    g[1] = gy / n;
    g[2] = gz / n;
}

#ifdef __cplusplus
}
#endif
