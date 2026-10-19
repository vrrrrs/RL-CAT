/**
  ******************************************************************************
  * @file    esp_fwd.c
  * @brief   协议帧 → UART3（ESP32）镜像实现：中断发送 + 降频 + 卡死兜底。
  *
  *  ⚠️ 本模块【不组帧、不编号】—— 只负责"把给定字节搬到 UART3"。
  *     组帧与 SEQ 都在 link_pi 里完成，保证两条链路发的是同一份数据。
  *
  *  ⭐⭐ 为什么必须用中断发送（docs/36-esp32-plan.md §2.2）：
  *     阻塞发送 103 B 的耗时 = 103 × 10 / 波特率：
  *         921600 → 1.12 ms（实测把 100 Hz 控制环拖到 ~92 Hz）
  *         115200 → 8.93 ms（会砸到 ~53 Hz）
  *     而 RL 的 decimation=2 要求控制环【严格 100 Hz】。
  *     中断发送把这段时间还给 CPU，控制环不受影响。
  *
  *  ⚠️ 单缓冲 + busy 标志的取舍：
  *     上一帧没发完就【丢新帧】，绝不排队。
  *     理由：遥测是可丢的，而排队会让延迟无界增长
  *     （ESP32 慢一点 → 队列越积越长 → 看到的遥测越来越旧）。
  *     丢帧有计数（esp_fwd_stats），上机一眼能看出。
  ******************************************************************************
  */

#include <string.h>

#include "esp_fwd.h"
#include "rlcat_config.h"
#include "rlca_frame.h"
#include "bsp_uart.h"
#include "bsp_tick.h"

/*==============================================================================
 * 内部状态（全部静态分配）
 *============================================================================*/

/** 发送缓冲：一帧最长 103 B（96 载荷 + 7 开销）。
 *  ⚠️ 必须 static：HAL_UART_Transmit_IT 是异步的，
 *     回调返回前这块内存必须一直有效。 */
static uint8_t s_tx[RLCA_ESP_TX_MAX_FRAME];

/** 发送中标志。ISR 侧清零，主循环侧置位。 */
static volatile bool s_busy = false;

/** 本次发送的起始时刻（看门狗用）。 */
static volatile uint32_t s_tx_start_ms = 0u;

static uint32_t s_n_try  = 0u;
static uint32_t s_n_ok   = 0u;
static uint32_t s_n_lost = 0u;   /* 上一帧没发完就丢   */
static uint32_t s_n_err  = 0u;   /* HAL 拒绝启动发送   */
static uint32_t s_n_stall = 0u;  /* 看门狗强制复位次数 */

/** 降频计数（只对 TEL_FAST 生效）。 */
static uint32_t s_decim = 0u;

/** ⭐ 运行期开关。
 *  ⚠️ 为什么需要它：UART3 同时承载【遥测镜像】和【shell】，
 *     两者混在一起时，文本终端里会看到 ASCII 夹杂二进制帧 ——
 *     这正是 UART1 上一直存在的现象。
 *     无线调试时把它关掉，UART3 就变成一条干净的文本通道。
 *  依据：RLCA_ESP_FWD_EN 是编译期默认值，这里提供运行期覆盖。 */
static bool s_enabled = (RLCA_ESP_FWD_EN != 0);

/*==============================================================================
 * 发送完成回调（HAL 在 USART3 的 TC 中断里调用）
 *============================================================================*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return;
    }
    if (huart->Instance == USART3) {
        s_busy = false;
    }
}

/*==============================================================================
 * 接口
 *============================================================================*/

void esp_fwd_init(void)
{
    s_busy        = false;
    s_tx_start_ms = 0u;
    s_n_try       = 0u;
    s_n_ok        = 0u;
    s_n_lost      = 0u;
    s_n_err       = 0u;
    s_n_stall     = 0u;
    s_decim       = 0u;
    s_enabled     = (RLCA_ESP_FWD_EN != 0);
}

void esp_fwd_set_enabled(bool on)
{
    s_enabled = on;
    if (!on) {
        /* ⚠️ 不打断正在进行的发送：HAL_UART_Transmit_IT 已经接管了 s_tx，
         *    强行置 busy=0 会让下一帧覆盖正在发的缓冲。
         *    这里只关"新帧"，让当前帧自然发完。 */
        s_decim = 0u;      /* 重新开启时从整周期开始降频 */
    }
}

bool esp_fwd_is_enabled(void)
{
    return s_enabled;
}

bool esp_fwd_frame(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len == 0u) || (len > (uint16_t)sizeof(s_tx))) {
        return false;
    }

    /* ⭐ 运行期开关：关掉时直接返回 true（"已处理"），
     *    不计数、不算丢帧 —— 因为这是【操作者的意图】，不是故障。
     *    ⚠️ 返回 true 而不是 false：调用方（link_pi）只把 false 当异常。 */
    if (!s_enabled) {
        return true;
    }

    /* ---- 看门狗：上一个中断若永远完不成，链路会就此死掉 ----
     * ⚠️ 触发条件极罕见（正常 115200 下 103 B 只要 8.9 ms），
     *    所以阈值放到 100 ms，绝不会误伤正常发送。 */
    if (s_busy) {
        const uint32_t now = bsp_tick_ms();
        if ((now - s_tx_start_ms) > RLCA_ESP_TX_STALL_MS) {
            /* 强制复位：放弃那一帧，让后续帧还能发出 */
            s_busy = false;
            s_n_stall++;
        }
    }

    /* 帧布局：0=AA 1=55 2=LEN 3=SEQ 4=TYPE */
    const uint8_t type = (len >= 5u) ? frame[4] : 0u;

    /* ---- 降频：只针对 TEL_FAST ----
     * ⚠️ TEL_STATUS 本来就是 10 Hz、HEARTBEAT 只有 1 Hz，
     *    对它们再降频就没意义了（心跳还会失去"存活指示"的作用）。 */
    if (type == RLCA_TYPE_TEL_FAST) {
        s_decim++;
        if ((s_decim % RLCA_ESP_FWD_DECIM) != 0u) {
            return true;      /* 本周期刻意跳过；不算丢帧 */
        }
    }

    /* ---- 上一帧还没发完 → 丢新帧（不排队）---- */
    if (s_busy) {
        s_n_try++;
        s_n_lost++;
        return false;
    }

    s_n_try++;

    memcpy(s_tx, frame, len);
    s_tx_start_ms = bsp_tick_ms();
    s_busy = true;

    /* ⭐ 非阻塞发送：立刻返回，字节由 TC 中断逐个送出。
     *    控制环不会在这里停哪怕一个字节的时间。 */
    if (HAL_UART_Transmit_IT(g_uart_esp.huart, s_tx, len) != HAL_OK) {
        s_busy = false;       /* 启动失败，立刻释放，下次再试 */
        s_n_err++;
        return false;
    }

    s_n_ok++;
    return true;
}

void esp_fwd_stats(esp_fwd_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->n_try   = s_n_try;
    out->n_ok    = s_n_ok;
    out->n_lost  = s_n_lost;
    out->n_err   = s_n_err;
    out->n_stall = s_n_stall;
    out->busy    = s_busy ? 1u : 0u;
}
