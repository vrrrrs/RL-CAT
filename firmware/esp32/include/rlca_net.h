/**
 * ESP32-side network layer: WiFi AP + the two TCP servers.
 *
 *   3333  bridge  -- pure byte pipe between the laptop and the STM32 (both ways)
 *   3334  console -- ESP32 text commands, ESP32 log lines, live counters
 *
 * Rationale for the split (deviation from docs/32-esp32-bridge.md §4.1) is
 * recorded in docs/34-esp32-plan.md §3.4: it keeps 3333 a clean binary stream so
 * a client never has to parse frames to tell STM32 traffic from ESP32 text.
 *
 * Threading: one task owns the socket set and uses poll(); the UART receive task
 * only calls rlca_net_broadcast_bridge(), which never blocks.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

typedef struct {
    bool     up;               /* AP started */
    uint8_t  ssid[33];
    uint32_t bridge_clients;   /* currently connected */
    uint32_t console_clients;
    uint32_t accepts;          /* total accepted connections */
    uint32_t disconnects;
    uint32_t rx_bytes;         /* laptop -> ESP32 (bridge port) */
    uint32_t tx_bytes;         /* ESP32 -> laptop (bridge port) */
    uint32_t tx_drops;         /* frames dropped: client socket would block */
    uint32_t console_lines_dropped;
} rlca_net_stats_t;

/** Start AP mode and both TCP servers (spawns the network task). */
esp_err_t rlca_net_start(void);

/** Snapshot of the counters above. */
void rlca_net_stats(rlca_net_stats_t *out);

/** Broadcast raw bytes to every client of the bridge port (STM32 -> laptop).
 *  Never blocks: per-client drops are counted in tx_drops. */
void rlca_net_broadcast_bridge(const uint8_t *data, size_t len);

/** Push text to every client of the console port (ESP32 logs, stats, prompts). */
void rlca_net_console_write(const char *text, size_t len);
