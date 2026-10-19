/**
 * Pure validators for the runtime configuration.
 *
 * Kept free of any ESP-IDF dependency (no esp_err.h, no config header) so the
 * PC tests can compile them directly -- see firmware/esp32/test/test_cfg.c.
 * They guard values that come back out of NVS after a reboot: a stored value
 * must never be able to configure an unusable AP or a privileged TCP port.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/** WPA2-PSK: 8..63 printable ASCII characters. */
static inline bool rlca_cfg_valid_password(const char *pw)
{
    if (!pw) {
        return false;
    }
    const size_t n = strlen(pw);
    if (n < 8 || n > 63) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        if ((unsigned char)pw[i] < 0x20 || (unsigned char)pw[i] > 0x7E) {
            return false;
        }
    }
    return true;
}

/** TCP port for the bridge/console servers: unprivileged and not the top port. */
static inline bool rlca_cfg_valid_port(int port)
{
    return port >= 1024 && port <= 65534;
}

/** UART baud rate for the STM32 link.
 *  921600 is the USART3 contract; 1000000 matches STM32 USART1 (the debug shell),
 *  which is what makes "wireless STM32 shell" possible without STM32 changes. */
static inline bool rlca_cfg_valid_baud(int baud)
{
    return baud >= 1200 && baud <= 4000000;
}

/** GPIO usable for the STM32 UART link on this chip + board:
 *  GPIO0..10 and 20/21 only.  Excluded on purpose:
 *    11..17 embedded flash, 18/19 USB D+/D-, 2/8/9 strapping,
 *    everything above 21 does not exist on the ESP32-C3. */
static inline bool rlca_cfg_valid_gpio(int gpio)
{
    if (gpio == 2 || gpio == 8 || gpio == 9) {
        return false;                     /* strapping pins */
    }
    return (gpio >= 0 && gpio <= 10) || gpio == 20 || gpio == 21;
}
