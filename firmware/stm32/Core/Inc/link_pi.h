/**
  ******************************************************************************
  * @file    link_pi.h
  * @brief   与树莓派链路的状态层：收帧、校验、失联判定、按节奏发送遥测。
  *
  *  纯编解码在 link_proto.{h,c}（可 PC 单测）；本文件只做状态与 I/O。
  *
  *  ⚠️ 时序模型（docs/33-firmware-plan.md §2①，负责人已确认）：
  *     模型 A（默认，RLCA_LINK_MODE_B = 0）
  *        发完 TEL_FAST → link_pi_wait_cmd(4000 µs) → 拿本帧指令
  *     模型 B（RLCA_LINK_MODE_B = 1）
  *        不等，直接用上一帧指令，周期抖动归零
  *     两者都【绝不阻塞超过等待窗口】，否则 100 Hz 节拍会崩。
  *
  *  ⚠️ 三个必须做的鲁棒性措施（协议 §6）：
  *     ① WAIT_HEAD1 收到 0xAA 时留在本状态（处理 AA AA 55）—— 在 rlca_frame.c 里
  *     ② LEN 上限检查 —— 在 rlca_frame.c 里
  *     ③ 整帧超时（5 ms）—— 本文件在 poll 里检查
  ******************************************************************************
  */

#ifndef LINK_PI_H
#define LINK_PI_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"
#include "link_proto.h"
#include "link_port.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

typedef struct {
    uint32_t rx_frames;        /* 收到的有效帧总数              */
    uint32_t crc_errors;       /* CRC 错                        */
    uint32_t len_errors;       /* LEN 超限                      */
    uint32_t rx_timeouts;      /* 半帧超时复位次数              */
    uint32_t unknown_type;     /* 未识别 TYPE（静默丢弃，但要计数） */
    uint32_t seq_lost;         /* 累计丢帧（由 SEQ 推出）        */
    uint32_t cmd_joint_count;  /* 收到的 CMD_JOINT 数            */
    uint32_t wait_timeouts;    /* 模型 A 等指令超时次数          */
} link_pi_stats_t;

/*==============================================================================
 * 生命周期与轮询
 *============================================================================*/

/** 绑定标定表并复位状态。 */
void link_pi_init(const rlca_calib_t *calib);

/** 覆盖链路端口（仅测试用）。传 NULL 恢复真实硬件端口。
 *  ⚠️ 与 servo_bus 的端口注入同一手法：让状态机能在 PC 上确定性单测。 */
void link_pi_set_port(const rlca_link_port_t *port);

/** 高频调用：把 USART1 收到的字节喂给状态机并分发已解析的帧。 */
void link_pi_poll(void);

/** 失联判定（主循环低频调用即可）。
 *  @param now_ms 当前毫秒时间
 *  @return 0 = 正常；1 = 已降级（> RLCA_LINK_LOST_MS）；
 *          2 = 应进入急停（> RLCA_LINK_ESTOP_MS） */
uint8_t link_pi_check_link(uint32_t now_ms);

/*==============================================================================
 * 发送
 *============================================================================*/

/** 发送 TEL_FAST（100 Hz）。 */
bool link_pi_send_tel_fast(const tel_fast_src_t *src);

/** 发送 TEL_STATUS（10 Hz 或事件触发）。 */
bool link_pi_send_tel_status(const tel_status_src_t *src);

/** 发送 CMD_CALIB 的应答（TEL_ACK）：2 字节 payload = [action, result]。 */
bool link_pi_send_ack(uint8_t action, uint8_t result);

/** 发送 HEARTBEAT（LEN=0）。 */
bool link_pi_send_heartbeat(void);

/*==============================================================================
 * 接收结果读取
 *============================================================================*/

/** 取得最近一次 CMD_JOINT（已解码，【未】限幅/限速）。
 *  @return true = 有【新的】指令；读后标志清除
 *  ⚠️ q_target 是原始解码值；调用方必须再过 link_limits_apply()。 */
bool link_pi_take_cmd_joint(pi_cmd_t *out);

/** 取得最近一次 CMD_JOINT 请求。
 *  @return true = 有新的模式切换请求 */
bool link_pi_take_cmd_mode(pi_mode_t *out);

/** 缓存 Pi 最新的目标角（由 app_main 在消费 CMD_JOINT 时写入）。
 *  ⚠️ 之所以在这里缓存一份：模式状态机每个周期都要读 q_target，
 *     但 CMD_JOINT 是"取走即清"的（take 语义）。
 *     缓存后语义变成"最新有效值一直可用"，正好符合策略控制的用法。 */
void link_pi_set_pi_q_target(const float *q_target);

/** 读缓存的 Pi 目标角（12 维，逻辑索引 A）。永不返回 NULL。 */
const float *link_pi_pi_q_target(void);

/** 取得最近一次 CMD_CALIB 请求。 */
bool link_pi_take_cmd_calib(pi_calib_t *out);

/** 当前【已接受】的运行模式（由上层通过 link_pi_set_mode 告知）。 */
uint8_t link_pi_mode(void);

/** 上层接受模式切换后回写，供 TEL_STATUS 上报。 */
void link_pi_set_mode(uint8_t mode);

/*==============================================================================
 * 模型 A 的等待窗口
 *============================================================================*/

/** 在最多 timeout_us 内等待一帧 CMD_JOINT 到达。
 *  @return true = 在窗口内拿到了新指令
 *  ⚠️ 这是唯一允许"占住主循环"的函数，且窗口固定 4 ms（RLCA_LINK_CMD_WAIT_US）。
 *     模型 B 下上层不应调用它。 */
bool link_pi_wait_cmd(uint32_t timeout_us);

/*==============================================================================
 * 查询
 *============================================================================*/

const link_pi_stats_t *link_pi_stats(void);

/** 距上次收到有效帧的毫秒数（从未收到时返回 UINT32_MAX）。 */
uint32_t link_pi_lost_ms(uint32_t now_ms);

/** ⭐⭐ 上电以来【最大】的链路空档（ms）—— 诊断用。
 *  ⚠️ `link_pi_lost_ms()` 是瞬时值，敲命令时看到的往往只有 20~40 ms，
 *     看不出到底有没有真的越过 ESTOP 阈值。这个值记住最大的那一次：
 *        · 远小于 RLCA_LINK_ESTOP_MS ⇒ ESTOP 不是链路触发的
 *        · 明显大于它               ⇒ 确实有长时间断流 */
uint32_t link_pi_max_gap_ms(void);

/** ⭐ 空档【越过 RLCA_LINK_ESTOP_MS】的次数（只数进入，不数持续周期）。 */
uint32_t link_pi_over_estop_count(void);

/** 近期是否收到过 Pi 指令（对应 STAT_LINK_OK 位）。 */
bool link_pi_is_alive(uint32_t now_ms);

#endif /* LINK_PI_H */
