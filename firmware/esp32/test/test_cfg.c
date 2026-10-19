/**
 * Host unit tests for the configuration validators (rlca_cfg.h).
 *
 * The validators guard data that comes back out of NVS after a reboot: a stored
 * value must never be able to configure an unusable AP or a privileged port.
 *
 * Build & run: firmware/tools/test_esp32_host.ps1
 */
#include <stdio.h>
#include <string.h>

#include "rlca_cfg_validate.h"

static int g_pass = 0;
static int g_fail = 0;

static void expect_bool(const char *what, bool got, bool want)
{
    if (got == want) {
        g_pass++;
        printf("  ok   %s\n", what);
    } else {
        g_fail++;
        printf("  FAIL %s (got %d, want %d)\n", what, (int)got, (int)want);
    }
}

int main(void)
{
    printf("=== RL-CAT ESP32 config validator tests ===\n");

    /* AP password: WPA2-PSK = 8..63 printable ASCII */
    expect_bool("password 'rlcat1234' accepted", rlca_cfg_valid_password("rlcat1234"), true);
    expect_bool("8 chars accepted", rlca_cfg_valid_password("12345678"), true);
    expect_bool("63 chars accepted",
                rlca_cfg_valid_password("012345678901234567890123456789012345678901234567890123456789012"),
                true);
    expect_bool("7 chars rejected", rlca_cfg_valid_password("1234567"), false);
    expect_bool("64 chars rejected",
                rlca_cfg_valid_password("0123456789012345678901234567890123456789012345678901234567890123"),
                false);
    expect_bool("empty rejected", rlca_cfg_valid_password(""), false);
    expect_bool("NULL rejected", rlca_cfg_valid_password(NULL), false);
    expect_bool("space is printable -> accepted",
                rlca_cfg_valid_password("pass word"), true);
    expect_bool("control char rejected", rlca_cfg_valid_password("pass\tword"), false);
    expect_bool("8 high-bit bytes rejected",
                rlca_cfg_valid_password("\xC3\xA9\xC3\xA9\xC3\xA9\xC3\xA9"), false);

    /* ports: 1024..65534 */
    expect_bool("3333 accepted", rlca_cfg_valid_port(3333), true);
    expect_bool("1024 accepted", rlca_cfg_valid_port(1024), true);
    expect_bool("65534 accepted", rlca_cfg_valid_port(65534), true);
    expect_bool("1023 (privileged) rejected", rlca_cfg_valid_port(1023), false);
    expect_bool("80 (privileged) rejected", rlca_cfg_valid_port(80), false);
    expect_bool("0 rejected", rlca_cfg_valid_port(0), false);
    expect_bool("65535 rejected", rlca_cfg_valid_port(65535), false);
    expect_bool("negative rejected", rlca_cfg_valid_port(-1), false);

    /* UART baud */
    expect_bool("921600 (USART3 contract) accepted", rlca_cfg_valid_baud(921600), true);
    expect_bool("1000000 (USART1 shell) accepted", rlca_cfg_valid_baud(1000000), true);
    expect_bool("1200 accepted (lower bound)", rlca_cfg_valid_baud(1200), true);
    expect_bool("4000000 accepted (upper bound)", rlca_cfg_valid_baud(4000000), true);
    expect_bool("1199 rejected", rlca_cfg_valid_baud(1199), false);
    expect_bool("4000001 rejected", rlca_cfg_valid_baud(4000001), false);
    expect_bool("0 rejected", rlca_cfg_valid_baud(0), false);
    expect_bool("negative rejected", rlca_cfg_valid_baud(-9600), false);

    /* UART pins: GPIO0..10 and 20/21, minus strapping 2/8/9 */
    expect_bool("GPIO20 accepted (module RXD)", rlca_cfg_valid_gpio(20), true);
    expect_bool("GPIO21 accepted (module TXD)", rlca_cfg_valid_gpio(21), true);
    expect_bool("GPIO6 accepted (devkit wiring)", rlca_cfg_valid_gpio(6), true);
    expect_bool("GPIO0 accepted", rlca_cfg_valid_gpio(0), true);
    expect_bool("GPIO10 accepted", rlca_cfg_valid_gpio(10), true);
    expect_bool("GPIO9 (BOOT strapping) rejected", rlca_cfg_valid_gpio(9), false);
    expect_bool("GPIO8 (strapping) rejected", rlca_cfg_valid_gpio(8), false);
    expect_bool("GPIO2 (strapping) rejected", rlca_cfg_valid_gpio(2), false);
    expect_bool("GPIO12 (embedded flash) rejected", rlca_cfg_valid_gpio(12), false);
    expect_bool("GPIO17 (embedded flash) rejected", rlca_cfg_valid_gpio(17), false);
    expect_bool("GPIO18 (USB D-) rejected", rlca_cfg_valid_gpio(18), false);
    expect_bool("GPIO19 (USB D+) rejected", rlca_cfg_valid_gpio(19), false);
    expect_bool("GPIO22 (does not exist) rejected", rlca_cfg_valid_gpio(22), false);
    expect_bool("negative GPIO rejected", rlca_cfg_valid_gpio(-1), false);

    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
