/**
 * UART bridge: the STM32 link (UART1) and the byte pump to/from the TCP side.
 *
 * Wiring (docs/20-electronics.md §2.1, docs/32-esp32-bridge.md §2.1):
 *   ESP32 GPIO6 (UART1 TX) -> STM32 PB11 (USART3_RX)
 *   ESP32 GPIO7 (UART1 RX) <- STM32 PB10 (USART3_TX)   [cross-wired]
 *   921600 8N1, common ground.
 *
 * This module is deliberately protocol-agnostic: it moves bytes and counts
 * errors.  Frame parsing lives in rlca_protocol.c (P2/P3), which attaches a
 * byte tap so the arbiter can watch TEL_FAST without consuming the stream.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

/** Optional observer for every byte received from the STM32 (called from the
 *  UART receive task; must be fast and must not block). */
typedef void (*rlca_bridge_tap_t)(const uint8_t *data, size_t len);

typedef struct {
    uint32_t rx_bytes;        /* STM32 -> ESP32 */
    uint32_t tx_bytes;        /* ESP32 -> STM32 */
    uint32_t rx_events;       /* UART_DATA events */
    uint32_t fifo_ovf;        /* UART_FIFO_OVF events: hardware overrun */
    uint32_t buffer_full;     /* driver ring buffer full */
    uint32_t parity_err;
    uint32_t frame_err;
    uint32_t tx_ring_full;    /* bytes rejected because the TX ring was full */
    /* Power-on internal loopback test (uart_set_loop_back): proves whether the
     * chip's own UART TX->RX path works, with nothing connected. */
    bool     loopback_ok;
    uint32_t loopback_rx;     /* bytes that came back (expected: the test length) */
    uint32_t lb_frames_ok;    /* stage 2: whole frames that came back CRC-clean */
    uint32_t lb_frames_sent;  /* stage 2: frames pushed through the loopback */
    uint32_t lb_bytes;        /* stage 2: bytes read back */
} rlca_bridge_stats_t;

/** Configure UART1 and start the receive task. */
esp_err_t rlca_bridge_start(void);

/** Queue bytes for the STM32. Returns the number of bytes actually accepted. */
size_t rlca_bridge_write(const uint8_t *data, size_t len);

/** Register/replace the receive tap (used by the protocol/arbiter layers). */
void rlca_bridge_set_tap(rlca_bridge_tap_t tap);

/** Snapshot of the counters above. */
void rlca_bridge_stats(rlca_bridge_stats_t *out);
