/**
 * Runtime configuration (WiFi AP password, TCP ports) with NVS persistence.
 *
 * Values come from rlca_esp32_config.h unless they were overridden from NVS via
 * the console (`cfg ...`).  The validators are pure functions so they can be
 * unit tested on the PC (firmware/esp32/test/test_cfg.c).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "rlca_cfg_validate.h"
#include "rlca_esp32_config.h"

typedef struct {
    char     ap_password[64];
    uint16_t port_bridge;
    uint16_t port_console;
    uint32_t uart_baud;
    uint8_t  uart_tx_gpio;
    uint8_t  uart_rx_gpio;
    bool     password_from_nvs;
    bool     ports_from_nvs;
    bool     baud_from_nvs;
    bool     pins_from_nvs;
} rlca_cfg_t;

/* The validators (rlca_cfg_valid_password / rlca_cfg_valid_port) live in
 * rlca_cfg_validate.h so they can be unit tested without ESP-IDF. */

/* ---- NVS backed accessors (ESP-IDF) ---- */
const rlca_cfg_t *rlca_cfg_get(void);

/** Initialise NVS, load overrides.  Never fails: falls back to defaults. */
esp_err_t rlca_cfg_load(void);

/** Validate + persist.  Runtime values only take effect after a reboot. */
esp_err_t rlca_cfg_set_password(const char *pw);
esp_err_t rlca_cfg_set_ports(uint16_t bridge, uint16_t console);
esp_err_t rlca_cfg_set_baud(uint32_t baud);
esp_err_t rlca_cfg_set_pins(uint8_t tx_gpio, uint8_t rx_gpio);

/** Erase the stored overrides (reboot to apply the defaults). */
esp_err_t rlca_cfg_clear(void);
