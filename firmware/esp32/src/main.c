/**
 * RL-CAT ESP32-C3 wireless bridge -- application entry point.
 *
 *   wifi_ap.c     AP + TCP servers (3333 bridge / 3334 console) + console cmds
 *   uart_bridge.c UART1 driver, receive task, byte pump, error counters
 *   protocol.c    frame state machine + CRC16/CCITT-FALSE (host-tested)
 *   arbiter.c     TEL_FAST/TEL_STATUS tracking + the injection gate
 *   injector.c    console text -> standard protocol frames
 *   cfg.c         NVS backed runtime configuration
 *   selftest.c    the PC test vectors, re-run on the real CPU
 *   logmirror.c   ESP_LOG -> console port
 *
 * See docs/32-esp32-bridge.md, docs/36-esp32-plan.md and
 * docs/37-esp32-acceptance.md.
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "rlca_esp32_config.h"
#include "rlca_arbiter.h"
#include "rlca_bridge.h"
#include "rlca_cfg.h"
#include "rlca_logmirror.h"
#include "rlca_net.h"
#include "rlca_selftest.h"

static const char *TAG = "rlca";

static void twdt_start(void)
{
#if RLCA_TWDT_ENABLE
    const esp_task_wdt_config_t cfg = {
        .timeout_ms      = RLCA_TWDT_TIMEOUT_S * 1000,
        .idle_core_mask  = 0,          /* do not police the idle task */
        .trigger_panic   = RLCA_TWDT_PANIC,
    };
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        err = esp_task_wdt_reconfigure(&cfg);   /* already started by the system */
    }
    ESP_LOGI(TAG, "task watchdog: %s (timeout %d s, panic=%d)",
             err == ESP_OK ? "on" : esp_err_to_name(err),
             RLCA_TWDT_TIMEOUT_S, RLCA_TWDT_PANIC);
#else
    ESP_LOGW(TAG, "task watchdog disabled by config");
#endif
}

static void log_board_info(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    const rlca_cfg_t *cfg = rlca_cfg_get();

    ESP_LOGI(TAG, "RL-CAT ESP32-C3 wireless bridge  cfg %s", RLCA_CFG_VERSION);
    ESP_LOGI(TAG, "chip  : model=%d cores=%d rev=v%d.%d  flash=%lu MB",
             (int)chip.model, chip.cores,
             chip.revision / 100, chip.revision % 100,
             (unsigned long)(flash_size / (1024 * 1024)));
    ESP_LOGI(TAG, "stm32 : UART%d tx=GPIO%u rx=GPIO%u @ %u 8N1%s%s",
             RLCA_UART_PORT, (unsigned)cfg->uart_tx_gpio, (unsigned)cfg->uart_rx_gpio,
             (unsigned)cfg->uart_baud,
             cfg->baud_from_nvs ? " (baud NVS)" : "",
             cfg->pins_from_nvs ? " (pins NVS)" : "");
    ESP_LOGI(TAG, "ports : %u = raw bridge%s, %u = ESP32 console%s",
             (unsigned)cfg->port_bridge, cfg->ports_from_nvs ? " (NVS)" : "",
             (unsigned)cfg->port_console, cfg->ports_from_nvs ? " (NVS)" : "");
    ESP_LOGI(TAG, "arb   : enable=%d block_on_unknown=%d stale=%d ms",
             RLCA_ARB_ENABLE, RLCA_ARB_BLOCK_ON_UNKNOWN, RLCA_ARB_STATUS_STALE_MS);
}

static void status_task(void *arg)
{
    (void)arg;
    char line[320];

    while (1) {
        rlca_net_stats_t    net;
        rlca_bridge_stats_t br;
        rlca_arb_stats_t    ar;
        rlca_net_stats(&net);
        rlca_bridge_stats(&br);
        rlca_arb_snapshot(&ar);

        snprintf(line, sizeof(line),
                 "alive: up=%lld s heap=%lu  cli=%u/%u  "
                 "uart rx/tx=%u/%u dr=%u ovf=%u bf=%u pe=%u fe=%u lb=%d/%u/%u/%u  "
                 "tel(age=%u ms frames=%u fast=%u st=%u crc=%u rs=%u le=%u) pi=%d",
                 esp_timer_get_time() / 1000000,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned)net.bridge_clients, (unsigned)net.console_clients,
                 (unsigned)br.rx_bytes, (unsigned)br.tx_bytes,
                 (unsigned)net.tx_drops, (unsigned)br.fifo_ovf, (unsigned)br.buffer_full,
                 (unsigned)br.parity_err, (unsigned)br.frame_err,
                 br.loopback_ok ? 1 : 0, (unsigned)br.loopback_rx,
                 (unsigned)br.lb_frames_ok, (unsigned)br.lb_frames_sent,
                 (unsigned)(ar.age_ms == 0xFFFFFFFFu ? 0 : ar.age_ms),
                 (unsigned)ar.frames, (unsigned)ar.tel_fast, (unsigned)ar.tel_status,
                 (unsigned)ar.crc_err, (unsigned)ar.resync, (unsigned)ar.len_err,
                 rlca_arb_pi_online() ? 1 : 0);
        /* One ESP_LOG is enough: logs are mirrored to the console port. */
        ESP_LOGI(TAG, "%s", line);

        vTaskDelay(pdMS_TO_TICKS(RLCA_BRIDGE_STATS_MS));
    }
}

void app_main(void)
{
    twdt_start();
    ESP_ERROR_CHECK(rlca_cfg_load());
    log_board_info();

#if RLCA_SELFTEST_ON_BOOT
    {
        char report[96];
        if (rlca_selftest_run(report, sizeof(report))) {
            ESP_LOGI(TAG, "%s", report);
        } else {
            ESP_LOGE(TAG, "%s", report);
        }
    }
#endif

    ESP_ERROR_CHECK(rlca_net_start());
    rlca_logmirror_start();
    ESP_ERROR_CHECK(rlca_bridge_start());
    rlca_arb_start();

    const rlca_cfg_t *cfg = rlca_cfg_get();
    ESP_LOGI(TAG, "P1..P4 ready -- join the AP and connect TCP %u (bridge) / %u (console)",
             (unsigned)cfg->port_bridge, (unsigned)cfg->port_console);

    xTaskCreate(status_task, "status", 3072, NULL, 3, NULL);
}
