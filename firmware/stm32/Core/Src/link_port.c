/**
  ******************************************************************************
  * @file    link_port.c
  * @brief   link_pi 端口的真实硬件实现（USART1 ↔ 树莓派）。
  *
  *  接线回顾（协议 §1）：
  *    PA9  (USART1_TX) → 树莓派 RX
  *    PA10 (USART1_RX) ← 树莓派 TX
  *    ⚠️ 是【标准交叉】，与 USART2↔舵机控制板那条【直连】不同。
  ******************************************************************************
  */

#include "link_port.h"
#include "bsp_uart.h"
#include "bsp_tick.h"
#include "esp_fwd.h"

static bool port_write(const uint8_t *data, uint16_t len)
{
    return bsp_uart_write(&g_uart_pi, data, len);
}

/** ⭐ 镜像：把同一份帧原样转发到 UART3（ESP32）。
 *  ⚠️ 这是【硬件实现】层 —— link_pi 只认端口，不认 esp_fwd，
 *     所以链路状态机在 PC 上仍然可以完整单测。 */
static bool port_write_mirror(const uint8_t *frame, uint16_t len)
{
    return esp_fwd_frame(frame, len);
}

static uint16_t port_avail(void)
{
    return bsp_uart_available(&g_uart_pi);
}

static uint16_t port_read(uint8_t *dst, uint16_t n)
{
    return bsp_uart_read(&g_uart_pi, dst, n);
}

static void port_flush(void)
{
    bsp_uart_flush(&g_uart_pi);
}

static uint32_t port_micros(void)
{
    return bsp_tick_us();
}

static uint32_t port_millis(void)
{
    return bsp_tick_ms();
}

static const rlca_link_port_t s_hw_port = {
    port_write,
    port_write_mirror,     /* ⭐ TEL_FAST/HEARTBEAT 原样镜像给 ESP32 */
    port_avail,
    port_read,
    port_flush,
    port_micros,
    port_millis
};

const rlca_link_port_t *link_port_hardware(void)
{
    return &s_hw_port;
}
