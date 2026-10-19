/**
 * Log mirroring to the console TCP port (see rlca_logmirror.h).
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "rlca_cfg.h"
#include "rlca_logmirror.h"
#include "rlca_net.h"

static vprintf_like_t s_prev_vprintf;
static volatile bool  s_active;       /* false until the net layer is up */
static volatile bool  s_in_hook;      /* recursion guard */

/** Send one log line to every console client, converting LF to CRLF. */
static void mirror_line(const char *buf, size_t len)
{
    size_t start = 0;
    for (size_t i = 0; i < len; i++) {
        if (buf[i] == '\n') {
            if (i > start) {
                rlca_net_console_write(&buf[start], i - start);
            }
            rlca_net_console_write("\r\n", 2);
            start = i + 1;
        }
    }
    if (start < len) {
        rlca_net_console_write(&buf[start], len - start);
    }
}

static int log_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    va_list copy;

    va_copy(copy, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);

    if (s_active && !s_in_hook && n > 0) {
        s_in_hook = true;
        mirror_line(buf, strlen(buf));
        s_in_hook = false;
    }

    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
}

void rlca_logmirror_start(void)
{
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);
    s_active = true;
    ESP_LOGI("log", "ESP32 logs are mirrored to console port %d",
             (int)rlca_cfg_get()->port_console);
}
