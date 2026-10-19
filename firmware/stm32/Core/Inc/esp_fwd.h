/**
  ******************************************************************************
  * @file    esp_fwd.h
  * @brief   把已经组好的协议帧镜像到 UART3（ESP32），供无线桥转发。
  *
  *  ⭐ 设计依据：
  *     docs/32-esp32-bridge.md §0/§3.2/§4.3 —— ESP32 是【纯字节桥】，
  *        UART3 ↔ TCP 逐字节原样搬，不解析整帧；只读 TEL_FAST 的
  *        status 字节做指令仲裁。
  *     docs/36-esp32-plan.md §2.2 —— 本链路的硬约束：
  *        **必须中断发送（非阻塞）+ 必须降频到 10 Hz**。
  *        原因：阻塞发送 103 B 在 115200 下要 8.93 ms，
  *        会把 100 Hz 控制环拖垮，破坏 RL 的 decimation=2 时序契约。
  *
  *  ⚠️ 本模块接收的是【已组好的完整帧】（含 0xAA 0x55 头与 CRC），
  *     原样往 UART3 搬。刻意【不】重新组帧 —— 那样 SEQ 会与树莓派链路
  *     不同，两条链路看到的就不是同一份数据了。
  ******************************************************************************
  */

#ifndef ESP_FWD_H
#define ESP_FWD_H

#include <stdint.h>
#include <stdbool.h>

/*==============================================================================
 * 类型
 *============================================================================*/

/** 转发统计（供 shell 诊断）。 */
typedef struct {
    uint32_t n_try;    /* 尝试镜像的帧数（降频跳过的不算）      */
    uint32_t n_ok;     /* 成功交给 HAL 的帧数                   */
    uint32_t n_lost;   /* 上一帧没发完而丢弃的帧数              */
    uint32_t n_err;    /* HAL 拒绝启动发送的次数                */
    uint32_t n_stall;  /* 看门狗强制复位的次数（链路自愈）      */
    uint8_t  busy;     /* 当前是否正在发送                      */
} esp_fwd_stats_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/** 初始化（清零统计）。在 bsp_uart_init_all() 之后调用。 */
void esp_fwd_init(void);

/** 把一帧【已组好的完整帧】原样镜像到 UART3（非阻塞）。
 *
 *  @param frame  完整帧（0xAA 0x55 起，含 CRC），由 rlca_frame_pack() 产出
 *  @param len    帧总长（TEL_FAST = 103，TEL_STATUS = 47，HEARTBEAT = 7）
 *  @return true = 已交给 HAL（不代表已发完，也不代表 ESP32 收到）
 *
 *  ⚠️ 调用【立即返回】：字节由 USART3 的 TC 中断逐个送出，
 *     控制环不会在这里阻塞（这是时序契约的要求）。
 *
 *  ⚠️ 上一帧没发完时【丢弃新帧】而不是排队 ——
 *     排队会让遥测延迟无界增长，而遥测本就可丢。
 *     丢帧计数见 esp_fwd_stats()。
 *
 *  ⚠️ 降频只对 TEL_FAST 生效（RLCA_ESP_FWD_DECIM，100 Hz → 10 Hz）。
 *     TEL_STATUS（本就 10 Hz）与 HEARTBEAT（1 Hz）不降频。 */
bool esp_fwd_frame(const uint8_t *frame, uint16_t len);

/** 读统计。 */
/** ⭐ 运行期开关镜像（无线调试时关掉，UART3 就只剩 shell 的干净文本）。
 *  ⚠️ 关掉时 esp_fwd_frame() 直接返回 true 且不计数 ——
 *     这是操作者意图，不是故障，不该污染丢帧统计。
 *  @param on  true = 镜像照常，false = 停止镜像（当前帧发完即止） */
void esp_fwd_set_enabled(bool on);

/** 当前镜像是否开启。 */
bool esp_fwd_is_enabled(void);
void esp_fwd_stats(esp_fwd_stats_t *out);

#endif /* ESP_FWD_H */
