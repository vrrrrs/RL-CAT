/**
 * Arbiter: watches the STM32 link, tracks TEL_FAST/TEL_STATUS, and decides
 * whether the ESP32 is allowed to inject commands (docs/32-esp32-bridge.md §4.3).
 *
 * Rule: while the Pi is online the ESP32 must not inject CMD_JOINT/CMD_MODE.
 * The gate is deliberately fail-safe: if telemetry is missing or stale the Pi
 * state is UNKNOWN, and unknown means "blocked" (RLCA_ARB_BLOCK_ON_UNKNOWN),
 * because "assume the Pi is gone and take over" is the classic accident.
 *
 * Byte source: the same stream the bridge forwards, via rlca_bridge_set_tap().
 * It is a pure observer -- it never consumes or delays bytes.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "rlca_protocol.h"

typedef struct {
    /* frame counters */
    uint32_t frames;
    uint32_t tel_fast;
    uint32_t tel_status;
    uint32_t other;
    uint32_t crc_err;
    uint32_t len_err;
    uint32_t resync;
    /* newest telemetry */
    bool     have_telemetry;
    uint32_t age_ms;          /* age of the newest TEL_FAST/TEL_STATUS */
    uint8_t  status;          /* protocol §4.5 bitfield */
    uint8_t  mode;            /* last mode seen in TEL_STATUS */
    uint16_t voltage_mv;
    uint16_t cycle_cnt;
    uint32_t cycle_gaps;      /* TEL_FAST cycle_cnt discontinuities */
    bool     have_q;
    /* newest IMU data (raw protocol units), for the console `imu` command */
    bool     have_imu;
    int16_t  quat[4];         /* w,x,y,z  1e-4 */
    int16_t  gyro[3];         /* x,y,z    0.001 rad/s */
} rlca_arb_stats_t;

/* ------------------------------------------------------------------ */
/* the gate itself -- kept as a pure function so it can be unit tested */
/* on the PC (firmware/esp32/test/test_arbiter_gate.c)                 */
/* ------------------------------------------------------------------ */
typedef enum {
    RLCA_GATE_ALLOW = 0,      /* safe to inject */
    RLCA_GATE_DISABLED,       /* RLCA_ARB_ENABLE == 0 */
    RLCA_GATE_NO_TELEMETRY,   /* the STM32 has never sent anything */
    RLCA_GATE_STALE,          /* telemetry older than RLCA_ARB_STATUS_STALE_MS */
    RLCA_GATE_PI_ONLINE,      /* STAT_LINK_OK: the Pi owns the robot */
} rlca_gate_t;

/* TEL_FAST cycle_cnt continuity.
 * The STM32 may *decimate* the mirror (e.g. send every 10th cycle at 10 Hz), so a
 * forward jump in cycle_cnt is expected and must not be reported as a dropped
 * frame.  Only a duplicated counter or an implausible jump is an anomaly. */
#define RLCA_CYCLE_MAX_STRIDE 500u

static inline bool rlca_cycle_ok(uint16_t prev, uint16_t cur)
{
    const uint16_t delta = (uint16_t)(cur - prev);   /* handles the 16-bit wrap */
    return delta != 0u && delta <= RLCA_CYCLE_MAX_STRIDE;
}

static inline rlca_gate_t rlca_gate_decide(bool have_telemetry, int64_t age_ms,
                                           uint8_t status)
{
    if (!RLCA_ARB_ENABLE) {
        return RLCA_GATE_DISABLED;
    }
    if (!have_telemetry) {
        return RLCA_GATE_NO_TELEMETRY;
    }
    if (age_ms > (int64_t)RLCA_ARB_STATUS_STALE_MS) {
        /* Unknown Pi state.  Fail safe unless explicitly configured otherwise. */
        return RLCA_ARB_BLOCK_ON_UNKNOWN ? RLCA_GATE_STALE : RLCA_GATE_ALLOW;
    }
    if (status & RLCA_STAT_LINK_OK) {
        return RLCA_GATE_PI_ONLINE;
    }
    return RLCA_GATE_ALLOW;
}

/** Register the byte tap and start the poll task (frame timeout + ages). */
void rlca_arb_start(void);

/** Snapshot of the arbiter state. */
void rlca_arb_snapshot(rlca_arb_stats_t *out);

/** True only when a *fresh* STAT_LINK_OK is present. */
bool rlca_arb_pi_online(void);

/** Injection gate. Returns true when injecting is allowed; when false, `why`
 *  receives a short human-readable reason (never NULL-terminated wrongly). */
bool rlca_arb_can_inject(char *why, size_t why_len);

/** Last reported joint positions in raw protocol units (0.0001 rad).
 *  Returns NULL when no TEL_FAST has been seen yet. */
const int16_t *rlca_arb_last_q(void);

/** Last mode reported by the STM32 (MODE_IDLE when unknown). */
uint8_t rlca_arb_last_mode(void);

/** Clear counters (console command `r`). */
void rlca_arb_reset_counters(void);
