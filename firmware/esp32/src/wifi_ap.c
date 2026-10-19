/**
 * WiFi AP + the two TCP servers (see rlca_net.h).
 *
 * Port 3333 (bridge) : bytes are forwarded verbatim to/from the STM32.
 * Port 3334 (console): line oriented ESP32 text interface (commands, stats).
 *
 * A single task owns every socket and uses poll(), so the bridge is never
 * blocked by a slow client: broadcasts are sent with MSG_DONTWAIT and dropped
 * (with a counter) when a client's socket would block.
 */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "rlca_arbiter.h"
#include "rlca_bridge.h"
#include "rlca_cfg.h"
#include "rlca_esp32_config.h"
#include "rlca_injector.h"
#include "rlca_net.h"
#include "rlca_selftest.h"

static const char *TAG = "net";

typedef enum {
    RLCA_CLIENT_FREE = 0,
    RLCA_CLIENT_BRIDGE,
    RLCA_CLIENT_CONSOLE,
} rlca_client_kind_t;

typedef struct {
    rlca_client_kind_t kind;
    int      fd;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    char     line[RLCA_CONSOLE_MAX_LINE];
    size_t   line_len;
} rlca_client_t;

static rlca_client_t      s_clients[RLCA_TCP_MAX_CLIENTS];
static int                s_listen_bridge = -1;
static int                s_listen_console = -1;
static rlca_net_stats_t   s_stats;
static bool               s_started;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
static void set_nonblocking(int fd, bool enable)
{
    int on = enable ? 1 : 0;
    ioctl(fd, FIONBIO, &on);
}

static int create_listener(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket() failed for port %u", port);
        return -1;
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind(%u) failed: errno %d", port, errno);
        close(fd);
        return -1;
    }
    if (listen(fd, RLCA_TCP_BACKLOG) != 0) {
        ESP_LOGE(TAG, "listen(%u) failed: errno %d", port, errno);
        close(fd);
        return -1;
    }

    set_nonblocking(fd, true);
    return fd;
}

static void client_close(rlca_client_t *c, const char *why)
{
    if (c->kind == RLCA_CLIENT_FREE) {
        return;
    }
    ESP_LOGI(TAG, "%s client #%d closed (%s): rx=%u tx=%u",
             c->kind == RLCA_CLIENT_BRIDGE ? "bridge" : "console",
             c->fd, why, (unsigned)c->rx_bytes, (unsigned)c->tx_bytes);
    close(c->fd);
    memset(c, 0, sizeof(*c));
    s_stats.disconnects++;
}

static void accept_client(int listen_fd, rlca_client_kind_t kind)
{
    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);

    int fd = accept(listen_fd, (struct sockaddr *)&peer, &len);
    if (fd < 0) {
        return;
    }

    int slot = -1;
    for (int i = 0; i < RLCA_TCP_MAX_CLIENTS; i++) {
        if (s_clients[i].kind == RLCA_CLIENT_FREE) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "no free client slot, rejecting connection");
        close(fd);
        return;
    }

    set_nonblocking(fd, true);
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    rlca_client_t *c = &s_clients[slot];
    memset(c, 0, sizeof(*c));
    c->kind = kind;
    c->fd   = fd;
    s_stats.accepts++;

    ESP_LOGI(TAG, "%s client connected: fd=%d from %s:%d (%u/%u slots used)",
             kind == RLCA_CLIENT_BRIDGE ? "bridge" : "console",
             fd, inet_ntoa(peer.sin_addr), ntohs(peer.sin_port),
             (unsigned)s_stats.accepts, (unsigned)RLCA_TCP_MAX_CLIENTS);
}

static int send_all(int fd, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int n = send(fd, data + sent, len - sent, MSG_DONTWAIT);
        if (n <= 0) {
            return (sent > 0) ? (int)sent : n;
        }
        sent += (size_t)n;
    }
    return (int)sent;
}

/* ------------------------------------------------------------------ */
/* console command handling (P1 skeleton; P2 adds the injector)        */
/* ------------------------------------------------------------------ */
static void console_reply(rlca_client_t *c, const char *text)
{
    size_t len = strlen(text);
    if (send_all(c->fd, (const uint8_t *)text, len) > 0) {
        c->tx_bytes += (uint32_t)len;
    }
}

static void console_print_help(rlca_client_t *c)
{
    console_reply(c,
        "RL-CAT ESP32-C3 wireless bridge\r\n"
        "  ?                 this help\r\n"
        "  i                 link/telemetry state (never injects)\r\n"
        "  imu               IMU detail: quaternion, gyro, gravity vector\r\n"
        "  s <j> <ddeg>      move joint j (0..11) by ddeg degrees, relative to its\r\n"
        "                    last reported position; other joints are held\r\n"
        "  z                 unload torque (CMD_MODE IDLE)\r\n"
        "  c                 start joint calibration (CMD_CALIB 0)\r\n"
        "  r                 reset local counters\r\n"
        "  stats             bridge / UART counters\r\n"
        "  selftest          re-run the on-target protocol/CRC self test\r\n"
        "  cfg [pw <new> | port <b> <c> | clear]   stored settings (NVS)\r\n"
        "  raw <hex>         push arbitrary bytes to the STM32 (bench test)\r\n"
        "  hb                send a HEARTBEAT frame\r\n"
        "  k                 keep-alive (reply: ok)\r\n"
        "  q                 close this connection\r\n"
        "  NOTE: s/z/c are refused while the Pi is online or while its state is\r\n"
        "        unknown (see docs/32-esp32-bridge.md §4.3). 'e' is out of scope.\r\n");
}

static void console_print_stats(rlca_client_t *c)
{
    rlca_net_stats_t    net;
    rlca_bridge_stats_t br;
    char buf[512];

    rlca_net_stats(&net);
    rlca_bridge_stats(&br);

    snprintf(buf, sizeof(buf),
             "stats: ap=%s ssid=%s\r\n"
             "  tcp : bridge_clients=%u console_clients=%u accepts=%u disconnects=%u\r\n"
             "  tcp : rx=%u tx=%u tx_drops=%u\r\n"
             "  uart: @%u baud  rx=%u tx=%u evt=%u fifo_ovf=%u ring_full=%u parity=%u frame=%u tx_ring_full=%u\r\n",
             net.up ? "up" : "down", net.ssid,
             (unsigned)net.bridge_clients, (unsigned)net.console_clients,
             (unsigned)net.accepts, (unsigned)net.disconnects,
             (unsigned)net.rx_bytes, (unsigned)net.tx_bytes, (unsigned)net.tx_drops,
             (unsigned)rlca_cfg_get()->uart_baud,
             (unsigned)br.rx_bytes, (unsigned)br.tx_bytes, (unsigned)br.rx_events,
             (unsigned)br.fifo_ovf, (unsigned)br.buffer_full,
             (unsigned)br.parity_err, (unsigned)br.frame_err,
             (unsigned)br.tx_ring_full);
    console_reply(c, buf);
}

static void console_selftest(rlca_client_t *c)
{
    char report[96];
    char buf[128];
    bool ok = rlca_selftest_run(report, sizeof(report));
    snprintf(buf, sizeof(buf), "%s: %s\r\n", ok ? "ok" : "FAIL", report);
    console_reply(c, buf);
}

static void console_cfg(rlca_client_t *c, const char *line)
{
    const rlca_cfg_t *cfg = rlca_cfg_get();
    const char *rest = line + 3;              /* skip "cfg" */
    while (*rest == ' ') {
        rest++;
    }

    if (!*rest) {
        char buf[512];
        snprintf(buf, sizeof(buf),
                 "cfg: bridge port %u (%s), console port %u (%s)\r\n"
                 "     uart tx=GPIO%u rx=GPIO%u @ %u baud (pins %s, baud %s)\r\n"
                 "     ap password: %s (default %s)\r\n"
                 "     built-in defaults: %u / %u / GPIO%u tx -> GPIO%u rx @ %u baud\r\n"
                 "     usage: cfg pw <new> | cfg port <b> <c> | cfg baud <n> | cfg pins <tx> <rx> | cfg clear\r\n"
                 "     NOTE: stored values apply after a reboot. "
                 "115200 = low rate debug link (default), 921600 = USART3 contract\r\n",
                 (unsigned)cfg->port_bridge, cfg->ports_from_nvs ? "NVS" : "default",
                 (unsigned)cfg->port_console, cfg->ports_from_nvs ? "NVS" : "default",
                 (unsigned)cfg->uart_tx_gpio, (unsigned)cfg->uart_rx_gpio,
                 (unsigned)cfg->uart_baud,
                 cfg->pins_from_nvs ? "NVS" : "default",
                 cfg->baud_from_nvs ? "NVS" : "default",
                 cfg->password_from_nvs ? "(from NVS)" : RLCA_AP_PASSWORD, RLCA_AP_PASSWORD,
                 (unsigned)RLCA_TCP_PORT_BRIDGE, (unsigned)RLCA_TCP_PORT_CONSOLE,
                 (unsigned)RLCA_UART_TX_GPIO, (unsigned)RLCA_UART_RX_GPIO,
                 (unsigned)RLCA_UART_BAUD);
        console_reply(c, buf);
        return;
    }

    if (!strncmp(rest, "pins ", 5)) {
        int tx = 0, rx = 0;
        if (sscanf(rest + 5, "%d %d", &tx, &rx) == 2) {
            esp_err_t err = rlca_cfg_set_pins((uint8_t)tx, (uint8_t)rx);
            console_reply(c, err == ESP_OK
                            ? "ok: uart pins stored (reboot to apply)\r\n"
                            : "err: pins must be 0..10 or 20/21, must differ, "
                              "and must not be 2/8/9/11..19\r\n");
        } else {
            console_reply(c, "usage: cfg pins <tx_gpio> <rx_gpio>   e.g. cfg pins 21 20\r\n");
        }
        return;
    }

    if (!strncmp(rest, "baud ", 5)) {
        int baud = atoi(rest + 5);
        esp_err_t err = rlca_cfg_set_baud((uint32_t)baud);
        console_reply(c, err == ESP_OK
                        ? "ok: uart baud stored (reboot to apply)\r\n"
                        : "err: baud must be 1200..4000000\r\n");
        return;
    }

    if (!strncmp(rest, "pw ", 3)) {
        esp_err_t err = rlca_cfg_set_password(rest + 3);
        console_reply(c, err == ESP_OK
                        ? "ok: password stored (reboot to apply)\r\n"
                        : "err: password must be 8..63 printable ASCII characters\r\n");
        return;
    }
    if (!strncmp(rest, "port ", 5)) {
        int bridge = 0, console = 0;
        if (sscanf(rest + 5, "%d %d", &bridge, &console) == 2) {
            esp_err_t err = rlca_cfg_set_ports((uint16_t)bridge, (uint16_t)console);
            console_reply(c, err == ESP_OK
                            ? "ok: ports stored (reboot to apply)\r\n"
                            : "err: ports must be 1024..65534 and must differ\r\n");
        } else {
            console_reply(c, "usage: cfg port <bridge> <console>\r\n");
        }
        return;
    }
    if (!strcmp(rest, "clear")) {
        esp_err_t err = rlca_cfg_clear();
        console_reply(c, err == ESP_OK
                        ? "ok: overrides erased (reboot to apply); "
                          "note this also clears the WiFi calibration data\r\n"
                        : "err: could not erase NVS\r\n");
        return;
    }

    console_reply(c, "usage: cfg | cfg pw <new> | cfg port <b> <c> | cfg clear\r\n");
}

static void console_handle_line(rlca_client_t *c, char *line)
{
    /* trim */
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    size_t n = strlen(line);
    while (n && (line[n - 1] == ' ' || line[n - 1] == '\t')) {
        line[--n] = '\0';
    }
    if (!n) {
        return;
    }

    if (!strcmp(line, "?") || !strcmp(line, "help")) {
        console_print_help(c);
        return;
    }
    if (!strcmp(line, "k")) {
        console_reply(c, "ok\r\n");
        return;
    }
    if (!strcmp(line, "stats")) {
        console_print_stats(c);
        return;
    }
    if (!strcmp(line, "selftest")) {
        console_selftest(c);
        return;
    }
    if (!strcmp(line, "cfg") || !strncmp(line, "cfg ", 4)) {
        console_cfg(c, line);
        return;
    }
    if (!strcmp(line, "q")) {
        client_close(c, "quit");
        return;
    }

    /* Everything else belongs to the injector (P2/P3): text -> protocol frame,
     * subject to the arbitration gate. */
    char reply[RLCA_CONSOLE_REPLY_MAX];
    if (rlca_injector_handle_line(line, reply, sizeof(reply))) {
        console_reply(c, reply);
    } else {
        console_reply(c, "err: unknown command (try '?')\r\n");
    }
}

static void console_rx(rlca_client_t *c, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char ch = (char)data[i];
        if (ch == '\n' || ch == '\r') {
            if (c->line_len) {
                c->line[c->line_len] = '\0';
                c->line_len = 0;
                console_handle_line(c, c->line);
                if (c->kind == RLCA_CLIENT_FREE) {
                    return;   /* closed by 'q' */
                }
            }
            continue;
        }
        if (c->line_len < sizeof(c->line) - 1) {
            c->line[c->line_len++] = ch;
        } else {
            c->line_len = 0;      /* overlong line: drop it */
            console_reply(c, "err: line too long\r\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/* network task                                                        */
/* ------------------------------------------------------------------ */
static void net_task(void *arg)
{
    (void)arg;
    uint8_t buf[RLCA_TCP_RX_CHUNK];
    uint32_t last_stats_ms = 0;

#if RLCA_TWDT_ENABLE
    esp_task_wdt_add(NULL);
#endif

    while (1) {
        struct pollfd pfds[2 + RLCA_TCP_MAX_CLIENTS];
        rlca_client_t *map[2 + RLCA_TCP_MAX_CLIENTS];
        int n = 0;

        pfds[n].fd = s_listen_bridge;  pfds[n].events = POLLIN; map[n] = NULL; n++;
        pfds[n].fd = s_listen_console; pfds[n].events = POLLIN; map[n] = NULL; n++;

        for (int i = 0; i < RLCA_TCP_MAX_CLIENTS; i++) {
            if (s_clients[i].kind != RLCA_CLIENT_FREE) {
                pfds[n].fd = s_clients[i].fd;
                pfds[n].events = POLLIN;
                map[n] = &s_clients[i];
                n++;
            }
        }

        int ready = poll(pfds, n, RLCA_NET_POLL_MS);
#if RLCA_TWDT_ENABLE
        esp_task_wdt_reset();
#endif
        if (ready < 0) {
            ESP_LOGW(TAG, "poll() error: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        for (int i = 0; i < n; i++) {
            if (!(pfds[i].revents & (POLLIN | POLLERR | POLLHUP))) {
                continue;
            }

            if (i == 0) {
                accept_client(s_listen_bridge, RLCA_CLIENT_BRIDGE);
                continue;
            }
            if (i == 1) {
                accept_client(s_listen_console, RLCA_CLIENT_CONSOLE);
                continue;
            }

            rlca_client_t *c = map[i];
            int r = recv(c->fd, buf, sizeof(buf), 0);
            if (r <= 0) {
                client_close(c, r == 0 ? "peer closed" : "recv error");
                continue;
            }
            c->rx_bytes += (uint32_t)r;

            if (c->kind == RLCA_CLIENT_BRIDGE) {
                s_stats.rx_bytes += (uint32_t)r;
                size_t written = rlca_bridge_write(buf, (size_t)r);
                if (written < (size_t)r) {
                    ESP_LOGW(TAG, "STM32 TX ring full: %u/%u bytes",
                             (unsigned)written, (unsigned)r);
                }
            } else {
                console_rx(c, buf, (size_t)r);
            }
        }

        /* Periodic one-liner for anyone watching the console port. */
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (s_stats.console_clients &&
            (now - last_stats_ms) >= RLCA_BRIDGE_STATS_MS) {
            last_stats_ms = now;
            char line[192];
            rlca_bridge_stats_t br;
            rlca_bridge_stats(&br);
            snprintf(line, sizeof(line),
                     "[stats] bridge=%u console=%u uart_rx=%u uart_tx=%u drops=%u ovf=%u\r\n",
                     (unsigned)s_stats.bridge_clients, (unsigned)s_stats.console_clients,
                     (unsigned)br.rx_bytes, (unsigned)br.tx_bytes,
                     (unsigned)s_stats.tx_drops, (unsigned)br.fifo_ovf);
            rlca_net_console_write(line, strlen(line));
        }
    }
}

/* ------------------------------------------------------------------ */
/* public API                                                          */
/* ------------------------------------------------------------------ */
static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    char ssid[33];
    snprintf(ssid, sizeof(ssid), "%s%02X%02X", RLCA_AP_SSID_PREFIX, mac[4], mac[5]);

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len       = (uint8_t)strlen(ssid);
    strncpy((char *)wc.ap.password, rlca_cfg_get()->ap_password,
            sizeof(wc.ap.password) - 1);
    wc.ap.channel        = RLCA_AP_CHANNEL;
    wc.ap.max_connection = RLCA_AP_MAX_STA;
    wc.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    wc.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    memcpy(s_stats.ssid, ssid, sizeof(s_stats.ssid));
    s_stats.ssid[sizeof(s_stats.ssid) - 1] = '\0';
    s_stats.up = true;

    ESP_LOGI(TAG, "AP up: ssid=%s password=%s ip=%s (max %d stations)",
             ssid, rlca_cfg_get()->ap_password, RLCA_AP_IP, RLCA_AP_MAX_STA);
}

esp_err_t rlca_net_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    wifi_start_ap();

    s_listen_bridge  = create_listener(rlca_cfg_get()->port_bridge);
    s_listen_console = create_listener(rlca_cfg_get()->port_console);
    if (s_listen_bridge < 0 || s_listen_console < 0) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP servers listening: %u (bridge) and %u (console)",
             (unsigned)rlca_cfg_get()->port_bridge,
             (unsigned)rlca_cfg_get()->port_console);

    xTaskCreate(net_task, "net", RLCA_TASK_STACK_NET, NULL, RLCA_TASK_PRIO_NET, NULL);
    s_started = true;
    return ESP_OK;
}

void rlca_net_stats(rlca_net_stats_t *out)
{
    if (!out) {
        return;
    }
    *out = s_stats;
    out->bridge_clients  = 0;
    out->console_clients = 0;
    for (int i = 0; i < RLCA_TCP_MAX_CLIENTS; i++) {
        if (s_clients[i].kind == RLCA_CLIENT_BRIDGE) {
            out->bridge_clients++;
        } else if (s_clients[i].kind == RLCA_CLIENT_CONSOLE) {
            out->console_clients++;
        }
    }
}

void rlca_net_broadcast_bridge(const uint8_t *data, size_t len)
{
    if (!data || !len) {
        return;
    }
    for (int i = 0; i < RLCA_TCP_MAX_CLIENTS; i++) {
        rlca_client_t *c = &s_clients[i];
        if (c->kind != RLCA_CLIENT_BRIDGE) {
            continue;
        }
        int n = send_all(c->fd, data, len);
        if (n < 0) {
            s_stats.tx_drops++;
            continue;
        }
        c->tx_bytes += (uint32_t)n;
        s_stats.tx_bytes += (uint32_t)n;
        if ((size_t)n < len) {
            s_stats.tx_drops++;
        }
    }
}

void rlca_net_console_write(const char *text, size_t len)
{
    if (!text || !len) {
        return;
    }
    for (int i = 0; i < RLCA_TCP_MAX_CLIENTS; i++) {
        rlca_client_t *c = &s_clients[i];
        if (c->kind != RLCA_CLIENT_CONSOLE) {
            continue;
        }
        int n = send_all(c->fd, (const uint8_t *)text, len);
        if (n <= 0) {
            s_stats.console_lines_dropped++;
        } else {
            c->tx_bytes += (uint32_t)n;
        }
    }
}
