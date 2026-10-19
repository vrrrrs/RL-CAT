/**
  ******************************************************************************
  * @file    servo_port.c
  * @brief   servo_bus 端口抽象的真实硬件实现（USART2 ↔ 舵机控制板）。
  *
  *  接线回顾（docs/30-firmware.md §4）：
  *    PA2 (USART2_TX) ── 舵机控制板 TX 端子
  *    PA3 (USART2_RX) ── 舵机控制板 RX 端子
  *    ⚠️ 是【直连】（RX→RX、TX→TX），不是交叉 —— 控制板丝印按 MCU 侧标注。
  *    ⚠️ 控制板跳线必须在 A 位。
  *
  *  ⚠️ 接收走 DMA 循环 + IDLE 轮询（bsp_uart 的 g_uart_servo），
  *     所以这里的 avail/read/flush 都直接转发到那个环形缓冲。
  ******************************************************************************
  */

#include "servo_port.h"
#include "bsp_uart.h"
#include "bsp_tick.h"
#include "stm32f1xx_hal.h"

/*==============================================================================
 * 端口实现
 *============================================================================*/

static void port_write(const uint8_t *data, uint16_t len)
{
    /* ⚠️ 阻塞发送：SyncWrite 44 B ≈ 440 µs、SyncRead 请求 20 B ≈ 200 µs，
     *    调用点在 100 Hz 周期的固定相位，能接受。
     *    若实测抖动超标，再改成 DMA 发送。 */
    (void)bsp_uart_write(&g_uart_servo, data, len);
}

static uint16_t port_avail(void)
{
    /* ⚠️⚠️ 必须先搬 DMA，再报告可用字节数。
     *
     *  这是实测抓到的最后一个根因（2026-10-11），也是 `A` 命令
     *  长期失败的真正原因：
     *
     *     `bsp_uart_poll_dma()` 原先【只在 app_poll() 开头调用一次】：
     *
     *         app_poll() {
     *             ① bsp_uart_poll_dma();   ← 搬一次
     *             ② servo_bus_poll();
     *             ③ link_pi_poll();
     *             ④ bsp_shell_poll();      ← `A` 在这里 → servo_bus_ping()
     *                                        → txn_expect_ack() 自旋 3 ms
     *         }
     *
     *     而 s_port->avail() 读的是【环形缓冲】，不是 DMA 寄存器。
     *     所以在那 3 ms 自旋期间，舵机的应答一直躺在 DMA 缓冲里，
     *     没有人把它搬进环形缓冲 → avail() 恒为 0 → 判失败。
     *
     *  ⚠️ 这解释了那个最反直觉的现象：
     *       `Y` 命令能收到完整应答（它自己的等待循环里显式调用了
     *       bsp_uart_poll_dma），而 `A` 全 no response（没有人搬）。
     *       两者【根本不是同一条通路】—— 所以 `Y` 的成功一直是个假象。
     *
     *  ⚠️ 为什么把"搬"下沉到端口层而不是在 servo_bus 里调：
     *     servo_bus 是【纯逻辑层】，刻意不依赖 bsp_uart（HAL），
     *     否则就不能在 PC 上单测了（之前踩过：加 bsp_tick.h 把
     *     host 单测搞挂）。放在端口实现里，既修了问题又保住了分层。
     *
     *  ⚠️ 重复调用的代价：bsp_uart_poll_dma() 只是读一个 DMA 计数器
     *     并按需搬运，幂等且极短；自旋循环里每次调用是可接受的。 */
    bsp_uart_poll_dma();
    return bsp_uart_available(&g_uart_servo);
}

static uint16_t port_read(uint8_t *dst, uint16_t n)
{
    return bsp_uart_read(&g_uart_servo, dst, n);
}

static void port_flush(void)
{
    bsp_uart_flush(&g_uart_servo);
}

static uint32_t port_micros(void)
{
    return bsp_tick_us();
}

static const rlca_servo_port_t s_hw_port = {
    port_write,
    port_avail,
    port_read,
    port_flush,
    port_micros
};

const rlca_servo_port_t *servo_port_hardware(void)
{
    return &s_hw_port;
}
