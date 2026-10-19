/**
 * UART1 <-> TCP byte pump (see rlca_bridge.h).
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "rlca_esp32_config.h"
#include "rlca_bridge.h"
#include "rlca_cfg.h"
#include "rlca_net.h"
#include "rlca_protocol.h"

static const char *TAG = "bridge";

static rlca_bridge_stats_t s_stats;
static rlca_bridge_tap_t   s_tap;
static SemaphoreHandle_t   s_tx_mutex;
static QueueHandle_t       s_uart_queue;
static bool                s_started;

static void bridge_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[RLCA_UART_READ_CHUNK];
    bool dumped_first = false;

#if RLCA_TWDT_ENABLE
    esp_task_wdt_add(NULL);
#endif

    while (1) {
        int n = uart_read_bytes(RLCA_UART_PORT, buf, sizeof(buf),
                                pdMS_TO_TICKS(RLCA_UART_RX_TIMEOUT_MS));
#if RLCA_TWDT_ENABLE
        esp_task_wdt_reset();
#endif
        if (n <= 0) {
            continue;
        }

        /* One-shot dump of the very first bytes seen: distinguishes "correct
         * AA 55 frame" from "baud mismatch garbage" straight from the log. */
        if (!dumped_first) {
            dumped_first = true;
            char hex[3 * 32 + 1];
            int m = (n < 32) ? n : 32;
            for (int i = 0; i < m; i++) {
                snprintf(&hex[3 * i], 4, "%02X ", buf[i]);
            }
            hex[3 * m] = '\0';
            ESP_LOGW(TAG, "first %d bytes on UART%d: %s", m, RLCA_UART_PORT, hex);
        }

        s_stats.rx_bytes += (uint32_t)n;

        /* Hand the bytes to the protocol/arbiter layer first (it only observes),
         * then forward them unchanged to every bridge client. */
        if (s_tap) {
            s_tap(buf, (size_t)n);
        }
        rlca_net_broadcast_bridge(buf, (size_t)n);
    }
}

static void bridge_event_task(void *arg)
{
    (void)arg;
    uart_event_t ev;

#if RLCA_TWDT_ENABLE
    esp_task_wdt_add(NULL);
#endif

    while (1) {
        /* Bounded wait so the watchdog can be fed even when the link is idle. */
        if (xQueueReceive((QueueHandle_t)arg, &ev,
                          pdMS_TO_TICKS(RLCA_UART_EVT_WAIT_MS)) != pdTRUE) {
#if RLCA_TWDT_ENABLE
            esp_task_wdt_reset();
#endif
            continue;
        }
#if RLCA_TWDT_ENABLE
        esp_task_wdt_reset();
#endif
        switch (ev.type) {
        case UART_DATA:
            s_stats.rx_events++;
            break;
        case UART_FIFO_OVF:
            s_stats.fifo_ovf++;
            ESP_LOGW(TAG, "UART FIFO overflow (hardware overrun)");
            uart_flush_input(RLCA_UART_PORT);
            break;
        case UART_BUFFER_FULL:
            s_stats.buffer_full++;
            ESP_LOGW(TAG, "UART driver ring buffer full");
            uart_flush_input(RLCA_UART_PORT);
            break;
        case UART_PARITY_ERR:
            s_stats.parity_err++;
            break;
        case UART_FRAME_ERR:
            s_stats.frame_err++;
            break;
        default:
            break;
        }
    }
}

esp_err_t rlca_bridge_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    const int baud = (int)rlca_cfg_get()->uart_baud;
    const int tx_gpio = (int)rlca_cfg_get()->uart_tx_gpio;
    const int rx_gpio = (int)rlca_cfg_get()->uart_rx_gpio;
    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RLCA_UART_PORT, RLCA_UART_RX_RING,
                                        RLCA_UART_TX_RING, RLCA_UART_EVT_QUEUE,
                                        &s_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(RLCA_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RLCA_UART_PORT, tx_gpio, rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_tx_mutex) {
        return ESP_ERR_NO_MEM;
    }

    /* ---- internal loopback self test (nothing connected, no jumper) --------
     * Routes UART1 TX into UART1 RX inside the chip.  If this passes, the ESP32
     * side of the link (driver, pin mux, RX path) is proven good, and any silence
     * on GPIO7 is therefore an external wiring / peer problem. */
    {
        static const uint8_t pattern[16] = {
            0xAA, 0x55, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
            0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7,
        };
        uint8_t rx[sizeof(pattern)];
        memset(rx, 0, sizeof(rx));

        esp_err_t e = uart_set_loop_back(RLCA_UART_PORT, true);
        if (e == ESP_OK) {
            uart_flush_input(RLCA_UART_PORT);
            uart_write_bytes(RLCA_UART_PORT, (const char *)pattern, sizeof(pattern));
            uart_wait_tx_done(RLCA_UART_PORT, pdMS_TO_TICKS(50));
            int got = uart_read_bytes(RLCA_UART_PORT, rx, sizeof(rx),
                                      pdMS_TO_TICKS(RLCA_UART_LOOPBACK_WAIT_MS));
            s_stats.loopback_rx = (got > 0) ? (uint32_t)got : 0u;
            s_stats.loopback_ok = (got == (int)sizeof(pattern)) &&
                                  (memcmp(pattern, rx, sizeof(pattern)) == 0);
            uart_flush_input(RLCA_UART_PORT);
        }
        (void)uart_set_loop_back(RLCA_UART_PORT, false);

        if (s_stats.loopback_ok) {
            ESP_LOGI(TAG, "UART%d internal loopback: OK (%u bytes returned) -- "
                          "the ESP32 RX path works, so a silent GPIO7 means the "
                          "cable/peer side",
                     RLCA_UART_PORT, (unsigned)s_stats.loopback_rx);
        } else {
            ESP_LOGE(TAG, "UART%d internal loopback FAILED (err=%s, %u bytes returned) "
                          "-- the ESP32 UART RX path itself is suspect",
                     RLCA_UART_PORT, esp_err_to_name(e), (unsigned)s_stats.loopback_rx);
        }

        /* ---- stage 2: can the RX path carry a 100 Hz TEL_FAST stream? --------
         * Sends whole frames through the internal loopback and counts how many
         * come back CRC-clean.  If this is perfect, any corruption seen on the
         * real link is external (ground, wire, or a second driver on the line). */
        if (uart_set_loop_back(RLCA_UART_PORT, true) == ESP_OK) {
            uint8_t pay[RLCA_LEN_TEL_FAST];
            uint8_t frame[RLCA_FRAME_MAX_TOTAL];
            memset(pay, 0xA5, sizeof(pay));
            uart_flush_input(RLCA_UART_PORT);

            const int frames = 20;
            int built_len = 0;
            for (int k = 0; k < frames; k++) {
                built_len = (int)rlca_frame_build(frame, RLCA_TYPE_TEL_FAST, (uint8_t)k,
                                                  pay, sizeof(pay));
                uart_write_bytes(RLCA_UART_PORT, (const char *)frame, built_len);
            }
            uart_wait_tx_done(RLCA_UART_PORT, pdMS_TO_TICKS(100));

            uint32_t got = 0, good = 0;
            rlca_rx_t rx;
            rlca_rx_init(&rx);
            uint8_t in[256];
            int want = frames * built_len;
            uint64_t t_start = (uint64_t)esp_timer_get_time();
            while (got < (uint32_t)want) {
                int n = uart_read_bytes(RLCA_UART_PORT, in, sizeof(in),
                                        pdMS_TO_TICKS(RLCA_UART_LOOPBACK_WAIT_MS));
                if (n <= 0) {
                    break;
                }
                got += (uint32_t)n;
                good += rlca_rx_feed(&rx, in, (uint16_t)n, (uint64_t)esp_timer_get_time(),
                                     NULL, NULL);
                if ((esp_timer_get_time() - (int64_t)t_start) > 500000) {
                    break;
                }
            }
            uart_flush_input(RLCA_UART_PORT);
            s_stats.lb_frames_ok = good;
            s_stats.lb_frames_sent = (uint32_t)frames;
            s_stats.lb_bytes = got;

            if (good == (uint32_t)frames) {
                ESP_LOGI(TAG, "UART%d loopback frame stream: %u/%d frames CRC-clean "
                              "(%u bytes) -- ESP32 RX is clean at this baud",
                         RLCA_UART_PORT, (unsigned)good, frames, (unsigned)got);
            } else {
                ESP_LOGW(TAG, "UART%d loopback frame stream: only %u/%d frames clean "
                              "(crc=%u len=%u, %u bytes) -- suspect the ESP32 side too",
                         RLCA_UART_PORT, (unsigned)good, frames,
                         (unsigned)rx.crc_err, (unsigned)rx.len_err, (unsigned)got);
            }
            (void)uart_set_loop_back(RLCA_UART_PORT, false);
        }
    }

    xTaskCreate(bridge_rx_task, "uart_rx", RLCA_TASK_STACK_BRIDGE_RX, NULL,
                RLCA_TASK_PRIO_BRIDGE_RX, NULL);
    if (s_uart_queue) {
        xTaskCreate(bridge_event_task, "uart_evt", RLCA_TASK_STACK_BRIDGE_EVT,
                    s_uart_queue, RLCA_TASK_PRIO_BRIDGE_EVT, NULL);
    }

    s_started = true;
    ESP_LOGI(TAG, "UART%d ready: tx=GPIO%d rx=GPIO%d %d 8N1 ring=%dB",
             RLCA_UART_PORT, tx_gpio, rx_gpio, baud, RLCA_UART_RX_RING);
    return ESP_OK;
}

size_t rlca_bridge_write(const uint8_t *data, size_t len)
{
    if (!s_started || !data || !len) {
        return 0;
    }

    /* One writer at a time; uart_write_bytes copies into the driver's TX ring. */
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(RLCA_UART_TX_LOCK_MS)) != pdTRUE) {
        s_stats.tx_ring_full += (uint32_t)len;
        return 0;
    }

    int written = uart_write_bytes(RLCA_UART_PORT, (const char *)data, len);
    xSemaphoreGive(s_tx_mutex);

    if (written < 0) {
        s_stats.tx_ring_full += (uint32_t)len;
        return 0;
    }
    if ((size_t)written < len) {
        s_stats.tx_ring_full += (uint32_t)(len - (size_t)written);
    }
    s_stats.tx_bytes += (uint32_t)written;
    return (size_t)written;
}

void rlca_bridge_set_tap(rlca_bridge_tap_t tap)
{
    s_tap = tap;
}

void rlca_bridge_stats(rlca_bridge_stats_t *out)
{
    if (out) {
        *out = s_stats;
    }
}
