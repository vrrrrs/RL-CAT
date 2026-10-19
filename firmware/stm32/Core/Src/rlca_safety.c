/**
  ******************************************************************************
  * @file    rlca_safety.c
  * @brief   安全判据的纯逻辑实现。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <stddef.h>

#include "rlca_safety.h"

/* 趴下斜坡速度（rad/s）
 * ⚠️ 取值理由：关节限位最宽的是 thigh（−35°~+90° ≈ 2.18 rad 跨度）。
 *     以 0.5 rad/s 走完 1.2 rad 约 2.4 s —— 与设计书 §5.8 的
 *     "3 秒内插值到趴下姿态"是一个量级，慢到不会摔、快到能及时停。 */
#define SAFE_RAMP_RATE_NORMAL   0.5f
#define SAFE_RAMP_RATE_LOWV     0.3f    /* 电压已经很低：再慢一点，省电流 */

/* L2 时的动作幅度缩放 */
#define SAFE_AMPLITUDE_L2       0.5f

/* 舵机过温阈值（与 rlcat_config.h 的 RLCA_SERVO_OVERTEMP_C 保持一致） */
#define SAFE_OVERTEMP_C         RLCA_SERVO_OVERTEMP_C

void rlca_safety_eval(const rlca_safety_in_t *in, rlca_safety_out_t *out)
{
    if ((in == NULL) || (out == NULL)) {
        return;
    }

    /* ---- 默认：正常 ---- */
    out->unload_torque    = false;
    out->sit_down         = false;
    out->ramp_active      = false;
    out->limit_amplitude  = false;
    out->amplitude_scale  = 1.0f;
    out->ramp_rate_rad_s  = SAFE_RAMP_RATE_NORMAL;
    out->block_active     = false;
    out->reasons          = 0u;
    out->severity         = 0u;

    bool estop = false;
    bool degrade = false;

    /*==========================================================================
     * 急停类
     *========================================================================*/

    /* 操作者急停（预留位；本次硬急停未实现，所以恒 false） */
    if (in->operator_estop) {
        out->reasons |= RLCA_SAFE_R_OPERATOR;
        estop = true;
    }

    /* ⭐ 链路失联 —— 安全保护不依赖上位机的核心体现 */
    if (in->link_level >= 2u) {
        out->reasons |= RLCA_SAFE_R_LINK_LOST;
        estop = true;
    } else if (in->link_level == 1u) {
        /* ⚠️ 降级级（>30 ms 但 ≤100 ms）也必须置位。
         *    最初只处理了 >=2，于是降级时 reasons 恒为 0，
         *    Pi 只能从 severity 反推，看不出"是链路在降级"。
         *    （单测抓到的。） */
        out->reasons |= RLCA_SAFE_R_LINK_LOST;
        degrade = true;
    }

    /* 电压 L3：强制趴下 */
    if (in->batt_valid && (in->batt_mv <= RLCA_BATT_FORCE_MV)) {
        out->reasons |= RLCA_SAFE_R_VOLT_FORCE;
        estop = true;
        out->ramp_rate_rad_s = SAFE_RAMP_RATE_LOWV;
    }

    /* 舵机过温：强制趴下（继续跑会烧） */
    if (in->max_temp_c > (uint8_t)SAFE_OVERTEMP_C) {
        out->reasons |= RLCA_SAFE_R_OVERTEMP;
        estop = true;
    }

    /*==========================================================================
     * 降级/限幅类
     *========================================================================*/

    if (in->batt_valid && (in->batt_mv <= RLCA_BATT_STOP_MV)) {
        out->reasons |= RLCA_SAFE_R_VOLT_STOP;
#if (RLCA_BATT_LIMIT_EN != 0)
        out->limit_amplitude = true;
        out->amplitude_scale = SAFE_AMPLITUDE_L2;
        degrade = true;
#else
        /* ⭐ 限幅已关闭（RLCA_BATT_LIMIT_EN = 0）—— 只上报，不改幅度。
         *
         *  ⚠️ 为什么可以关（2026-10-18）：
         *     ① 分压比原来写成 100k/22k（5.545），实际硬件是 100k/20k（6.000），
         *        读数被压低 7.6% ⇒ 电池其实 10.8 V 却被读成 9.95 V，
         *        于是长期落在 VOLT_STOP 区间、幅度被反复压缩。
         *        ⇒ 这是**测量错误**，不是真的电量不足，限幅没有意义。
         *     ② 限幅会让 `q_ref` 被缩放（app_mode 里乘 amplitude_scale），
         *        表现为"关节到不了目标角 / 步态幅度变小"，严重干扰调试。
         *
         *  ⚠️⚠️ 注意本开关【不影响】`VOLT_FORCE`（≤9300 mV → 卸力矩急停）：
         *     限幅是"性能调节"，急停是"安全底线"，刻意分开。
         *     真没电时仍然会停。 */
        (void)0;
#endif
    }

    if (in->servo_err_flags != 0u) {
        /* ⭐⭐⭐ 拆分"通信错误"与"舵机自报错误"（2026-10-17 修的关键 bug）
         *
         *  ⚠️⚠️ 现场症状：`pose.py` 目标 20°，最终实测 ~13.7°，
         *      而且关节【往复摆动】、保持 1 秒后"下垂 7.34°"。
         *
         *  ⭐ 算术对上了：`20° × 0.75 = 15°`，再掉 1.3° 重力 = 13.7°。
         *     也就是【幅度缩放】被激活了（app_mode.c 里
         *     `q_ref = q_def + (q - q_def) * amplitude_scale`）。
         *
         *  ⭐ 为什么会反复触发：`RLCA_ERRF_BUS_TIMEOUT` 一旦置位，
         *     要 2 秒（RLCA_ERR_AUTOCLEAR_MS）才自动清除；
         *     而它由"某一轮整轮没收到回复"置起（约 0.19% 的轮次）。
         *     ⇒ amplitude_scale 在 1.0 / 0.75 之间反复跳
         *     ⇒ 目标角在 20° / 15° 之间反复跳
         *     ⇒ **表现就是关节往复摆动**，还会被误判成"未收敛/下垂"。
         *
         *  ⭐ 关键认识：**BUS_TIMEOUT 是通信统计，不是力矩限制。**
         *     舵机没回话 ≠ 舵机过载。过载/过温/过流各有自己的位。
         *
         *  ⇒ 修法：只有舵机【自报】的错误位才降级限幅；
         *     BUS_TIMEOUT 只上报（severity 1 告警），不动幅度。
         *
         *  ⚠️ 安全性没有下降：
         *     · 总线真死了 ⇒ `servo_bus_feedback_fresh()` 为假
         *       ⇒ ACTIVE 准入第 ⑤ 道门（NO_FB）直接挡住，进不了 ACTIVE。
         *       这比"压 25% 幅度"强得多。
         *     · BUS_TIMEOUT 本身还要连续 RLCA_BUS_FAIL_STREAK_ERR 轮
         *       失败才置位，偶发抖动根本进不来。 */
        const uint8_t hard = (uint8_t)(in->servo_err_flags
                                       & (uint8_t)(~RLCA_ERRF_BUS_TIMEOUT));

        out->reasons |= RLCA_SAFE_R_SERVO_ERR;

        if (hard != 0u) {
            /* 舵机自报的错误（过温/过流/过载/电压/角度/限位）⇒ 降级 + 限幅 */
            out->limit_amplitude = true;
            if (out->amplitude_scale > 0.75f) {
                out->amplitude_scale = 0.75f;
            }
            degrade = true;
        }
        /* ⚠️ hard == 0（只有 BUS_TIMEOUT）⇒ 只上报，不降级、不限幅 */
    }

    /*==========================================================================
     * 告警类
     *========================================================================*/

    if (in->batt_valid && (in->batt_mv <= RLCA_BATT_WARN_MV)) {
        out->reasons |= RLCA_SAFE_R_VOLT_WARN;
    }

    if (!in->imu_ok) {
        out->reasons |= RLCA_SAFE_R_IMU_STALE;
        /* ⚠️ 不升级为急停：IMU 失效让姿态观测不可信，但机器人不该因此瘫倒。
         *    真正要拦的是"带着假姿态跑策略"，所以禁止进入 ACTIVE。 */
    }

    /*==========================================================================
     * 汇总
     *========================================================================*/

    if (estop) {
        out->sit_down        = true;
        out->ramp_active     = true;
        out->limit_amplitude = true;
        out->amplitude_scale = 0.0f;      /* 急停时不做任何主动动作，只往下走 */
        out->block_active    = true;
        out->severity        = 3u;
    } else if (degrade) {
        out->block_active = false;        /* 降级仍可跑，只是幅度受限 */
        out->severity     = 2u;
    } else if (out->reasons != 0u) {
        out->severity = 1u;
    }

    /* ⚠️ 卸力矩只在急停时发生。
     *    刻意【不】在降级时卸力矩：卸了力矩机器人立刻瘫软，
     *    比"限幅继续走"危险得多（除非本来就该趴下）。 */
    out->unload_torque = estop;

    /* ⚠️ IMU 失效单独禁止 ACTIVE，即使没有其它触发原因。
     *    理由：策略的全部姿态输入都是假的，跑起来必摔。 */
    if (!in->imu_ok) {
        out->block_active = true;
        if (out->severity < 2u) {
            out->severity = 2u;
        }
    }
}

const char *rlca_safety_reason_name(uint8_t bit_index)
{
    switch (1u << (bit_index & 7u)) {
    case RLCA_SAFE_R_VOLT_FORCE: return "VOLT_FORCE";
    case RLCA_SAFE_R_VOLT_STOP:  return "VOLT_STOP";
    case RLCA_SAFE_R_VOLT_WARN:  return "VOLT_WARN";
    case RLCA_SAFE_R_LINK_LOST:  return "LINK_LOST";
    case RLCA_SAFE_R_OVERTEMP:   return "OVERTEMP";
    case RLCA_SAFE_R_SERVO_ERR:  return "SERVO_ERR";
    case RLCA_SAFE_R_IMU_STALE:  return "IMU_STALE";
    case RLCA_SAFE_R_OPERATOR:   return "OPERATOR";
    default:                     return "?";
    }
}
