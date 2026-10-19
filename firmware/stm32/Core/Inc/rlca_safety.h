/**
  ******************************************************************************
  * @file    rlca_safety.h
  * @brief   安全判据的【纯逻辑】层：由信号算出该做什么。
  *
  *  ⚠️ 与 app_safety.c 的分工：
  *     rlca_safety.c —— 纯决策：输入一组信号，输出"要不要卸力矩/要不要趴下"
  *     app_safety.c  —— 执行：把决策变成真实的舵机动作与状态位
  *     这样"什么时候该趴下"这件事能被确定性地单测覆盖，
  *     而不是埋在带 HAL 调用的大函数里没法验证。
  *
  *  ⭐ 最重要的一条（docs/00-overview.md §2.1）：
  *     **安全保护必须在 STM32 上独立完成，不能依赖上位机。**
  *     所以链路失联本身就是一条独立的触发条件，
  *     而且它自己有一套 30 ms 降级 / 100 ms 急停的时限。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_SAFETY_H
#define RLCA_SAFETY_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

/*==============================================================================
 * 输入：一组信号
 *============================================================================*/

typedef struct {
    /* ---- 链路 ---- */
    uint8_t  link_level;      /* 0 = 正常；1 = 降级(>30 ms)；2 = 失联(>100 ms) */
    uint32_t cmd_age_ms;      /* 距上次收到有效指令的毫秒数                    */

    /* ---- 电源 ---- */
    uint16_t batt_mv;
    bool     batt_valid;      /* 电压读数是否新鲜                              */

    /* ---- 舵机 ---- */
    uint8_t  max_temp_c;      /* 12 个舵机温度的最大值                          */
    uint8_t  servo_err_flags; /* RLCA_ERRF_* 位域（舵机错误汇总）              */

    /* ---- IMU ---- */
    bool     imu_ok;

    /* ---- 操作者 ---- */
    bool     operator_estop;  /* 预留：硬急停输入（本次不实现，恒 false）       */
} rlca_safety_in_t;

/*==============================================================================
 * 输出：该做什么
 *============================================================================*/

/** 触发原因（可多个同时成立，用位域表示） */
#define RLCA_SAFE_R_VOLT_FORCE   0x01u   /* 电压 ≤ 9.3 V  → 强制趴下          */
#define RLCA_SAFE_R_VOLT_STOP    0x02u   /* 电压 ≤ 9.9 V  → 限幅              */
#define RLCA_SAFE_R_VOLT_WARN    0x04u   /* 电压 ≤ 10.5 V → 仅告警            */
#define RLCA_SAFE_R_LINK_LOST    0x08u   /* 链路失联 > 100 ms                 */
#define RLCA_SAFE_R_OVERTEMP     0x10u   /* 舵机 > 70 °C                      */
#define RLCA_SAFE_R_SERVO_ERR    0x20u   /* 舵机错误码非零                    */
#define RLCA_SAFE_R_IMU_STALE    0x40u   /* IMU 数据失效                      */
#define RLCA_SAFE_R_OPERATOR     0x80u   /* 操作者急停                        */

typedef struct {
    /** ⭐ 是否必须立即卸力矩（真·紧急）。
     *  true 时上层应走"缓慢趴下"而不是直接断电 —— 直接断电会摔。 */
    bool     unload_torque;

    /** ⭐ 是否必须进入趴下流程（比 unload_torque 更强：要求主动降低目标角）。 */
    bool     sit_down;

    /** 是否应开始（或继续）缓慢趴下插值。
     *  = sit_down && 还有余量可走。由 app_safety 用来决定"本周期还推不推进"。 */
    bool     ramp_active;

    /** 是否应限制动作幅度（L2 及以下 / 舵机异常）。 */
    bool     limit_amplitude;

    /** 幅度缩放（1.0 = 不限；L2 时约 0.5）。 */
    float    amplitude_scale;

    /** 使用的趴下斜坡速度（rad/s）。电压越低越慢（越平稳）。 */
    float    ramp_rate_rad_s;

    /** 是否禁止进入 ACTIVE（跟踪策略目标角）。 */
    bool     block_active;

    /** 触发原因位域（RLCA_SAFE_R_*）。 */
    uint8_t  reasons;

    /** 严重程度（0=正常 1=告警 2=降级 3=急停），用于 LED 与上报。 */
    uint8_t  severity;
} rlca_safety_out_t;

/*==============================================================================
 * 纯逻辑接口
 *============================================================================*/

/** 由信号算出安全输出。
 *
 *  ⚠️ 判据的优先级（"最严重的赢"）：
 *        急停类：操作者急停 / 链路失联 / 电压 L3 / 舵机过温
 *        限幅类：电压 L2 / 舵机错误码非零
 *        告警类：电压 L1 / IMU 失效
 *
 *  ⚠️ 刻意【不】把"IMU 失效"和"电压读数失效"算作急停：
 *     它们会让机器人没法好好走，但不是"现在就要停"的理由；
 *     把它们当急停会让"IMU 线松了"变成"机器人趴下"，
 *     反而降低了可用性且掩盖了真问题。 */
void rlca_safety_eval(const rlca_safety_in_t *in, rlca_safety_out_t *out);

/** 把触发原因位域转成可读的短名（诊断用；无需 printf 浮点）。 */
const char *rlca_safety_reason_name(uint8_t bit_index);

#endif /* RLCA_SAFETY_H */
