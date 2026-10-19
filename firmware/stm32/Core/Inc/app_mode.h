/**
  ******************************************************************************
  * @file    app_mode.h
  * @brief   运行模式状态机：IDLE / STAND / ACTIVE / DAMP / ESTOP / CALIB。
  *
  *  模式语义（协议 §4.2）：
  *    IDLE    —— 卸力矩，什么都不做。上电默认。
  *    STAND   —— 使能力矩，保持默认站姿（q_default），不跟策略。
  *                这是"上机调试"和"策略未就绪"时的安全落脚点。
  *    ACTIVE  —— 使能力矩 + 跟踪 Pi 下发的 q_target。**唯一会跟着策略走的模式。**
  *    DAMP    —— 卸力矩（软），靠自重自然落地。用于"主动放弃控制"。
  *    ESTOP   —— 急停：缓慢趴下 → 卸力矩。由 app_safety 驱动。
  *    CALIB   —— 标定流程占用舵机总线，不下发正常控制。
  *
  *  ⚠️ ACTIVE 的准入条件（必须全部满足，否则退回 STAND）：
  *     ① 已标定（offset/dir 有效）
  *     ② 链路存活（近期收到过合法指令）
  *     ③ IMU 数据可用（否则策略的姿态输入是假的）
  *     ④ 安全层没有禁止（app_safety_blocks_active()）
  *     ⑤ 舵机总线最近读全过 12 帧（否则位置反馈不可信）
  *
  *  ⚠️⚠️ 未标定时 STAND 也必须被拦住（2026-10-12 安全修复）：
  *     未标定时 offset 全 0、dir 全 +1（rlca_calib_load_defaults），
  *     q_target = q_default = 0 经 rlca_q_to_step 换算正好落在【舵机 step 0】，
  *     12 个舵机会一起冲向机械行程一端 —— 这是已实测的撞机事故。
  *     ⇒ 未标定时的 STAND 降级为"卸力矩 + 目标跟随当前反馈位置"。
  *     ⭐「未标定」应视为一个独立的安全状态，不是 STAND 的变体。
  *
  *  ⚠️ 约束：本模块只做"模式决策 + 生成 q_ref"，
  *     实际的舵机下发在 app_main 的控制环里（保证只有一处调用 SyncWrite）。
  ******************************************************************************
  */

#ifndef APP_MODE_H
#define APP_MODE_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"

/** ACTIVE 的准入检查结果（用于上报/诊断，0 = 通过） */
#define RLCA_ACTIVE_OK              0u
#define RLCA_ACTIVE_BLOCK_NOT_CALIB 1u
#define RLCA_ACTIVE_BLOCK_NO_LINK   2u
#define RLCA_ACTIVE_BLOCK_NO_IMU    3u
#define RLCA_ACTIVE_BLOCK_SAFETY    4u
#define RLCA_ACTIVE_BLOCK_NO_FB     5u

typedef struct {
    uint8_t  mode;              /* 请求的模式（来自 Pi 或本地）        */
    uint8_t  active_mode;       /* 实际生效的模式（可能因准入被降级）  */
    uint8_t  active_block;      /* 上一次 ACTIVE 被拒的原因           */
    uint8_t  torque_on;         /* 当前是否使能力矩                   */
    uint32_t mode_t0_ms;        /* 进入当前模式的时刻                  */
    uint32_t n_mode_changes;    /* 模式切换次数                       */
} app_mode_state_t;

/** 初始化（复位到 IDLE）。 */
void app_mode_init(void);

/** 请求切换模式（来自 CMD_MODE 或 CMD_JOINT.mode）。
 *  ⚠️ 请求不等于生效：ESTOP 与 CALIB 有优先级，ACTIVE 有准入条件。 */
void app_mode_request(uint8_t mode);

/** 每个控制周期调用一次。
 *  @param now_ms        当前毫秒
 *  @param calib         标定表
 *  @param link_alive    链路是否存活
 *  @param imu_ok        IMU 是否可用
 *  @param calib_ok      是否已标定
 *  @param fb_ok         最近一轮舵机反馈是否读全
 *  @param pi_q_target   输入：Pi 下发的目标角（12 维；ACTIVE 时使用）
 *  @param q_now         ⭐输入：当前实测关节角（12 维）。
 *                      未标定的 STAND 用它当目标 —— "待在原地"，
 *                      免得把 12 个舵机推向 step 0（撞机）。
 *                      允许为 NULL，此时退回 q_default。
 *  @param q_ref         输出：本周期要下发的目标角（12 维）
 *  @param torque_out    输出：本周期要求的力矩使能状态
 */
void app_mode_update(uint32_t now_ms,
                     const rlca_calib_t *calib,
                     bool link_alive,
                     bool imu_ok,
                     bool calib_ok,
                     bool fb_ok,
                     const float *pi_q_target,
                     const float *q_now,
                     float *q_ref,
                     bool *torque_out);

/** 请求标定模式（标定流程启动时调用）。 */
void app_mode_enter_calib(void);

/** 标定流程结束时调用，回到 IDLE。 */
void app_mode_exit_calib(void);

/** 当前生效模式。 */
uint8_t app_mode_current(void);

/** 上面那个"请求的模式"。 */
uint8_t app_mode_requested(void);

/** 详细状态。 */
const app_mode_state_t *app_mode_state(void);

/** ACTIVE 准入检查（纯逻辑，可单测）。
 *  @return RLCA_ACTIVE_OK 或某个 BLOCK_* 原因 */
uint8_t app_mode_active_gate(bool calib_ok, bool link_alive, bool imu_ok,
                             bool fb_ok, bool safety_blocks);

#endif /* APP_MODE_H */
