/**
 * Runtime configuration with NVS persistence (see rlca_cfg.h).
 */
#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "rlca_cfg.h"

static const char *TAG = "cfg";

#define RLCA_NVS_NAMESPACE "rlca"
#define KEY_PASSWORD  "ap_pw"
#define KEY_PORT_BRIDGE  "port_bridge"
#define KEY_PORT_CONSOLE "port_console"
#define KEY_UART_BAUD    "uart_baud"
#define KEY_UART_TX      "uart_tx"
#define KEY_UART_RX      "uart_rx"

static rlca_cfg_t s_cfg;

static void cfg_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    strncpy(s_cfg.ap_password, RLCA_AP_PASSWORD, sizeof(s_cfg.ap_password) - 1);
    s_cfg.port_bridge  = RLCA_TCP_PORT_BRIDGE;
    s_cfg.port_console = RLCA_TCP_PORT_CONSOLE;
    s_cfg.uart_baud    = RLCA_UART_BAUD;
    s_cfg.uart_tx_gpio = RLCA_UART_TX_GPIO;
    s_cfg.uart_rx_gpio = RLCA_UART_RX_GPIO;
}

const rlca_cfg_t *rlca_cfg_get(void)
{
    return &s_cfg;
}

esp_err_t rlca_cfg_load(void)
{
    cfg_defaults();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs to be erased (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable (%s), using built-in defaults", esp_err_to_name(err));
        return err;
    }

    nvs_handle_t h;
    if (nvs_open(RLCA_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored overrides, using defaults");
        return ESP_OK;
    }

    char pw[sizeof(s_cfg.ap_password)];
    size_t pw_len = sizeof(pw);
    if (nvs_get_str(h, KEY_PASSWORD, pw, &pw_len) == ESP_OK) {
        if (rlca_cfg_valid_password(pw)) {
            strncpy(s_cfg.ap_password, pw, sizeof(s_cfg.ap_password) - 1);
            s_cfg.password_from_nvs = true;
        } else {
            ESP_LOGW(TAG, "stored AP password is invalid, ignoring it");
        }
    }

    uint16_t v = 0;
    if (nvs_get_u16(h, KEY_PORT_BRIDGE, &v) == ESP_OK && rlca_cfg_valid_port(v)) {
        s_cfg.port_bridge = v;
        s_cfg.ports_from_nvs = true;
    }
    if (nvs_get_u16(h, KEY_PORT_CONSOLE, &v) == ESP_OK && rlca_cfg_valid_port(v)) {
        s_cfg.port_console = v;
        s_cfg.ports_from_nvs = true;
    }

    uint32_t baud = 0;
    if (nvs_get_u32(h, KEY_UART_BAUD, &baud) == ESP_OK && rlca_cfg_valid_baud((int)baud)) {
        s_cfg.uart_baud = baud;
        s_cfg.baud_from_nvs = true;
    }

    uint8_t pin = 0;
    uint8_t tx = s_cfg.uart_tx_gpio, rx = s_cfg.uart_rx_gpio;
    bool have_tx = false, have_rx = false;
    if (nvs_get_u8(h, KEY_UART_TX, &pin) == ESP_OK && rlca_cfg_valid_gpio(pin)) {
        tx = pin;
        have_tx = true;
    }
    if (nvs_get_u8(h, KEY_UART_RX, &pin) == ESP_OK && rlca_cfg_valid_gpio(pin)) {
        rx = pin;
        have_rx = true;
    }
    if (have_tx && have_rx && tx != rx) {
        s_cfg.uart_tx_gpio = tx;
        s_cfg.uart_rx_gpio = rx;
        s_cfg.pins_from_nvs = true;
    } else if (have_tx || have_rx) {
        ESP_LOGW(TAG, "stored UART pins are incomplete or identical, ignoring them");
    }
    nvs_close(h);

    if (s_cfg.port_bridge == s_cfg.port_console) {
        ESP_LOGW(TAG, "stored ports collide, falling back to defaults");
        s_cfg.port_bridge  = RLCA_TCP_PORT_BRIDGE;
        s_cfg.port_console = RLCA_TCP_PORT_CONSOLE;
        s_cfg.ports_from_nvs = false;
    }

    ESP_LOGI(TAG, "config: ports %u/%u (%s), uart tx=GPIO%u rx=GPIO%u @ %u baud (%s/%s), AP password %s",
             (unsigned)s_cfg.port_bridge, (unsigned)s_cfg.port_console,
             s_cfg.ports_from_nvs ? "from NVS" : "default",
             (unsigned)s_cfg.uart_tx_gpio, (unsigned)s_cfg.uart_rx_gpio,
             (unsigned)s_cfg.uart_baud,
             s_cfg.baud_from_nvs ? "NVS" : "default",
             s_cfg.pins_from_nvs ? "NVS" : "default",
             s_cfg.password_from_nvs ? "from NVS" : "default");
    return ESP_OK;
}

esp_err_t rlca_cfg_set_password(const char *pw)
{
    if (!rlca_cfg_valid_password(pw)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(RLCA_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, KEY_PASSWORD, pw);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AP password stored (takes effect after reboot)");
    }
    return err;
}

esp_err_t rlca_cfg_set_ports(uint16_t bridge, uint16_t console)
{
    if (!rlca_cfg_valid_port(bridge) || !rlca_cfg_valid_port(console) ||
        bridge == console) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(RLCA_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u16(h, KEY_PORT_BRIDGE, bridge);
    if (err == ESP_OK) {
        err = nvs_set_u16(h, KEY_PORT_CONSOLE, console);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ports %u/%u stored (takes effect after reboot)",
                 (unsigned)bridge, (unsigned)console);
    }
    return err;
}

esp_err_t rlca_cfg_set_baud(uint32_t baud)
{
    if (!rlca_cfg_valid_baud((int)baud)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(RLCA_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(h, KEY_UART_BAUD, baud);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "uart baud %u stored (takes effect after reboot)", (unsigned)baud);
    }
    return err;
}

esp_err_t rlca_cfg_set_pins(uint8_t tx_gpio, uint8_t rx_gpio)
{
    if (!rlca_cfg_valid_gpio(tx_gpio) || !rlca_cfg_valid_gpio(rx_gpio) ||
        tx_gpio == rx_gpio) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(RLCA_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, KEY_UART_TX, tx_gpio);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, KEY_UART_RX, rx_gpio);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "uart pins tx=GPIO%u rx=GPIO%u stored (takes effect after reboot)",
                 (unsigned)tx_gpio, (unsigned)rx_gpio);
    }
    return err;
}

esp_err_t rlca_cfg_clear(void)
{
    esp_err_t err = nvs_flash_erase();
    if (err == ESP_OK) {
        cfg_defaults();
        ESP_LOGI(TAG, "stored overrides erased, defaults restored after reboot");
    }
    return err;
}
