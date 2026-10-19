/**
 * Arbiter implementation (see rlca_arbiter.h).
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "rlca_arbiter.h"
#include "rlca_bridge.h"
#include "rlca_esp32_config.h"

static const char *TAG = "arb";

static rlca_rx_t          s_rx;
static rlca_arb_stats_t   s_st;
static int16_t            s_q[RLCA_NUM_JOINTS];
static int64_t            s_last_telemetry_us;
static uint16_t           s_last_cycle;
static bool               s_have_cycle;
static uint32_t           s_cycle_gaps;

static inline uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void on_frame(const rlca_frame_t *f, void *ctx)
{
    (void)ctx;
    s_st.frames++;

    if (f->type == RLCA_TYPE_TEL_FAST) {
        rlca_tel_fast_t tf;
        if (!rlca_tel_fast_parse(f, &tf)) {
            return;
        }
        s_st.tel_fast++;
        s_st.status     = tf.status;
        s_st.voltage_mv = tf.voltage_mv;

        /* cycle_cnt must move forward; a decimated mirror (10 Hz) legitimately
         * jumps by ~10, so only duplicates / implausible jumps are anomalies. */
        if (s_have_cycle && !rlca_cycle_ok(s_last_cycle, tf.cycle_cnt)) {
            s_cycle_gaps++;
        }
        s_last_cycle = tf.cycle_cnt;
        s_have_cycle = true;
        s_st.cycle_cnt = tf.cycle_cnt;

        memcpy(s_q, tf.q, sizeof(s_q));
        s_st.have_q = true;
        memcpy(s_st.quat, tf.quat, sizeof(s_st.quat));
        memcpy(s_st.gyro, tf.gyro, sizeof(s_st.gyro));
        s_st.have_imu = true;
        s_last_telemetry_us = (int64_t)now_us();
        s_st.have_telemetry = true;
    } else if (f->type == RLCA_TYPE_TEL_STATUS) {
        rlca_tel_status_t ts;
        if (!rlca_tel_status_parse(f, &ts)) {
            return;
        }
        s_st.tel_status++;
        s_st.status     = ts.status;
        s_st.mode       = ts.mode;
        s_st.voltage_mv = ts.voltage_mv;
        s_last_telemetry_us = (int64_t)now_us();
        s_st.have_telemetry = true;
    } else {
        s_st.other++;
    }
}

/** Byte tap: called from the UART receive task for every byte batch. */
static void arb_tap(const uint8_t *data, size_t len)
{
    const uint64_t t = now_us();
    rlca_rx_feed(&s_rx, data, (uint16_t)len, t, on_frame, NULL);
}

static void arb_task(void *arg)
{
    (void)arg;
#if RLCA_TWDT_ENABLE
    esp_task_wdt_add(NULL);
#endif
    while (1) {
        rlca_rx_poll(&s_rx, now_us());
#if RLCA_TWDT_ENABLE
        esp_task_wdt_reset();
#endif
        vTaskDelay(pdMS_TO_TICKS(RLCA_ARB_POLL_MS));
    }
}

void rlca_arb_start(void)
{
    rlca_rx_init(&s_rx);
    memset(&s_st, 0, sizeof(s_st));
    s_st.mode = RLCA_MODE_IDLE;

    rlca_bridge_set_tap(arb_tap);
    xTaskCreate(arb_task, "arb", RLCA_TASK_STACK_ARB, NULL, RLCA_TASK_PRIO_ARB, NULL);
    ESP_LOGI(TAG, "arbiter watching the STM32 link (stale=%d ms, block_on_unknown=%d)",
             RLCA_ARB_STATUS_STALE_MS, RLCA_ARB_BLOCK_ON_UNKNOWN);
}

void rlca_arb_snapshot(rlca_arb_stats_t *out)
{
    if (!out) {
        return;
    }
    *out = s_st;
    out->crc_err = s_rx.crc_err;
    out->len_err = s_rx.len_err;
    out->resync  = s_rx.resync;
    out->cycle_gaps = s_cycle_gaps;
    out->age_ms = s_st.have_telemetry
                    ? (uint32_t)(((int64_t)now_us() - s_last_telemetry_us) / 1000)
                    : 0xFFFFFFFFu;
}

bool rlca_arb_pi_online(void)
{
    if (!s_st.have_telemetry) {
        return false;
    }
    int64_t age_ms = ((int64_t)now_us() - s_last_telemetry_us) / 1000;
    if (age_ms > (int64_t)RLCA_ARB_STATUS_STALE_MS) {
        return false;
    }
    return (s_st.status & RLCA_STAT_LINK_OK) != 0;
}

bool rlca_arb_can_inject(char *why, size_t why_len)
{
    const int64_t age_ms = s_st.have_telemetry
                             ? ((int64_t)now_us() - s_last_telemetry_us) / 1000
                             : -1;
    const rlca_gate_t gate = rlca_gate_decide(s_st.have_telemetry, age_ms, s_st.status);

    const char *reason = NULL;
    switch (gate) {
    case RLCA_GATE_ALLOW:
        break;
    case RLCA_GATE_DISABLED:
        reason = "arbitration disabled by config";
        break;
    case RLCA_GATE_NO_TELEMETRY:
        reason = "no telemetry: the STM32 has never sent a frame on USART3";
        break;
    case RLCA_GATE_STALE:
        reason = "telemetry stale (Pi online state unknown)";
        break;
    case RLCA_GATE_PI_ONLINE:
        reason = "Pi is online (STAT_LINK_OK) -- the Pi owns the robot";
        break;
    default:
        reason = "blocked";
        break;
    }

    if (why) {
        if (reason) {
            snprintf(why, why_len, "%s [age=%lld ms status=0x%02X]",
                     reason, (long long)(age_ms < 0 ? -1 : age_ms), s_st.status);
        } else {
            snprintf(why, why_len, "ok [age=%lld ms status=0x%02X]",
                     (long long)age_ms, s_st.status);
        }
    }
    return reason == NULL;
}

const int16_t *rlca_arb_last_q(void)
{
    return s_st.have_q ? s_q : NULL;
}

uint8_t rlca_arb_last_mode(void)
{
    return s_st.mode;
}

void rlca_arb_reset_counters(void)
{
    uint8_t mode = s_st.mode;
    memset(&s_st, 0, sizeof(s_st));
    s_st.mode = mode;
    s_cycle_gaps = 0;
    s_have_cycle = false;
}
