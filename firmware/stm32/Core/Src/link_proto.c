/**
  ******************************************************************************
  * @file    link_proto.c
  * @brief   与树莓派通信的纯逻辑编解码实现。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <string.h>

#include "link_proto.h"
#include "rlca_frame.h"      /* rlca_put_* / rlca_get_* 小端辅助 */

/*==============================================================================
 * 饱和量化
 *============================================================================*/

int16_t link_enc_i16(float value, float unit)
{
    if (!rlca_float_is_valid(value) || (unit == 0.0f)) {
        return 0;
    }
    float v = value / unit;
    if (v > 32767.0f) {
        v = 32767.0f;
    } else if (v < -32768.0f) {
        v = -32768.0f;
    }
    /* 对称取整：负数也要按绝对值四舍五入，否则会系统性偏小 */
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/*==============================================================================
 * 下行解码
 *============================================================================*/

lnk_dec_result_t link_decode_cmd_joint(const uint8_t *payload, uint8_t len, pi_cmd_t *out)
{
    if ((payload == NULL) || (out == NULL)) {
        return LNK_DEC_ERR_PARAM;
    }
    if (len != RLCA_LEN_CMD_JOINT) {
        return LNK_DEC_ERR_LEN;
    }

    memset(out, 0, sizeof(*out));

    out->mode  = payload[RLCA_CJ_OFF_MODE];
    out->flags = payload[RLCA_CJ_OFF_FLAGS];

    /* 12 个目标角，偏移 2 起，小端 int16，单位 0.0001 rad */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int16_t enc = rlca_get_i16(&payload[RLCA_CJ_OFF_Q_TARGET + (2u * i)]);
        out->q_target[i] = (float)enc * RLCA_Q_UNIT_RAD;
    }

    /* 偏移 26 是 1 字节 reserved；27 起是 kp_scale（int16，0.01 单位） */
    out->kp_scale = rlca_get_i16(&payload[RLCA_CJ_OFF_KP_SCALE]);
    out->led      = payload[29];

    /* 模式非法 → 回落到 IDLE（协议 §8.3 第 ① 条） */
    out->mode = link_mode_sanitize(out->mode);

    return LNK_DEC_OK;
}

lnk_dec_result_t link_decode_cmd_mode(const uint8_t *payload, uint8_t len, pi_mode_t *out)
{
    if ((payload == NULL) || (out == NULL)) {
        return LNK_DEC_ERR_PARAM;
    }
    if (len != RLCA_LEN_CMD_MODE) {
        return LNK_DEC_ERR_LEN;
    }
    out->mode  = link_mode_sanitize(payload[0]);
    out->param = payload[1];
    return LNK_DEC_OK;
}

lnk_dec_result_t link_decode_cmd_calib(const uint8_t *payload, uint8_t len, pi_calib_t *out)
{
    if ((payload == NULL) || (out == NULL)) {
        return LNK_DEC_ERR_PARAM;
    }
    if (len != RLCA_LEN_CMD_CALIB) {
        return LNK_DEC_ERR_LEN;
    }
    if (payload[0] > RLCA_CALIB_RESTORE_DEFAULT) {
        return LNK_DEC_ERR_PARAM;
    }
    out->action = payload[0];
    return LNK_DEC_OK;
}

uint8_t link_mode_sanitize(uint8_t mode)
{
    return (mode <= (uint8_t)RLCA_MODE_ESTOP) ? mode : (uint8_t)RLCA_MODE_IDLE;
}

/*==============================================================================
 * 指令合法化
 *============================================================================*/

void link_limits_apply(const rlca_calib_t *calib,
                       const float *q_in,
                       float *prev,
                       float *q_out)
{
    if ((calib == NULL) || (q_in == NULL) || (prev == NULL) || (q_out == NULL)) {
        return;
    }

    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        float q = q_in[i];

        /* ① NaN / Inf → 安全侧：回落到下限
         *    ⚠️ 必须在任何比较之前处理：NaN 与任何数比较都是 false，
         *       不先拦住会让 NaN 一路传到舵机。 */
        if (!rlca_float_is_valid(q)) {
            float qmin;
            float qmax;
            rlca_q_range(calib, i, &qmin, &qmax);
            q = qmin;
        }

        /* ② 夹到软限位 */
        q = rlca_q_clamp(calib, i, q);

        /* ③ 单周期变化限速（最后一道防线）
         *    ⚠️ 以"上一周期实际下发值"为基准，而不是上一帧的指令值 ——
         *       否则连续多帧大跳变会累积成无限速度。 */
        float delta = q - prev[i];
        if (delta > RLCA_MAX_DELTA_PER_CYCLE_RAD) {
            delta = RLCA_MAX_DELTA_PER_CYCLE_RAD;
        } else if (delta < -RLCA_MAX_DELTA_PER_CYCLE_RAD) {
            delta = -RLCA_MAX_DELTA_PER_CYCLE_RAD;
        }
        const float q_lim = prev[i] + delta;

        prev[i]  = q_lim;
        q_out[i] = q_lim;
    }
}

/*==============================================================================
 * 上行打包
 *============================================================================*/

uint8_t link_pack_tel_fast(const tel_fast_src_t *src, uint8_t *payload)
{
    if ((src == NULL) || (payload == NULL)) {
        return 0u;
    }
    memset(payload, 0, RLCA_LEN_TEL_FAST);

    /* 0..23：关节位置（0.0001 rad） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        rlca_put_i16(&payload[RLCA_TF_OFF_Q + (2u * i)],
                     link_enc_i16(src->q_rad[i], RLCA_Q_UNIT_RAD));
    }

    /* 24..47：关节速度（0.0001 rad/s） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        rlca_put_i16(&payload[RLCA_TF_OFF_QD + (2u * i)],
                     link_enc_i16(src->qd_rad_s[i], RLCA_QD_UNIT_RAD_S));
    }

    /* 48..71：关节力矩（0.001 N·m） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        rlca_put_i16(&payload[RLCA_TF_OFF_TAU + (2u * i)],
                     link_enc_i16(src->tau_nm[i], RLCA_TAU_UNIT_NM));
    }

    /* 72..79：四元数 w,x,y,z（0.0001） */
    for (uint8_t i = 0u; i < 4u; ++i) {
        rlca_put_i16(&payload[RLCA_TF_OFF_QUAT + (2u * i)],
                     link_enc_i16(src->quat_wxyz[i], RLCA_QUAT_UNIT));
    }

    /* 80..85：角速度（0.001 rad/s） */
    for (uint8_t i = 0u; i < 3u; ++i) {
        rlca_put_i16(&payload[RLCA_TF_OFF_GYRO + (2u * i)],
                     link_enc_i16(src->gyro_rad_s[i], RLCA_GYRO_UNIT_RAD_S));
    }

    /* 86..89：步态相位（已是 int16，直接原样写） */
    rlca_put_i16(&payload[RLCA_TF_OFF_GAIT_SIN], src->gait_sin_i16);
    rlca_put_i16(&payload[RLCA_TF_OFF_GAIT_COS], src->gait_cos_i16);

    /* 90..91：100 Hz 周期计数 */
    rlca_put_u16(&payload[RLCA_TF_OFF_CYCLE_CNT], src->cycle_cnt);

    /* 92..93：电池电压（mV） */
    rlca_put_u16(&payload[RLCA_TF_OFF_VOLTAGE], src->voltage_mv);

    /* 94：状态位域；95：舵机错误汇总 */
    payload[RLCA_TF_OFF_STATUS]    = src->status;
    payload[RLCA_TF_OFF_ERR_FLAGS] = src->err_flags;

    return RLCA_LEN_TEL_FAST;
}

uint8_t link_pack_tel_status(const tel_status_src_t *src, uint8_t *payload)
{
    if ((src == NULL) || (payload == NULL)) {
        return 0u;
    }
    memset(payload, 0, RLCA_LEN_TEL_STATUS);

    rlca_put_u32(&payload[0], src->uptime_ms);
    rlca_put_u16(&payload[4], src->voltage_mv);

    /* 6..29：12 个舵机温度（int16，0.1 °C）。无数据时填 0。
     * ⚠️ 不能用 `(int16_t *)&payload[6]` 这种写法：既违反严格别名规则
     *    （gcc -O2 下可能被优化掉），也依赖编译器对齐假设。
     *    协议是小端，所以逐字节显式写。 */
    if (src->temp_deci_c != NULL) {
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            rlca_put_i16(&payload[6u + (2u * i)], (int16_t)src->temp_deci_c[i]);
        }
    }

    rlca_put_u16(&payload[30], src->rx_overflow);
    rlca_put_u16(&payload[32], src->bus_err);
    rlca_put_u16(&payload[34], src->bus_timeout);
    payload[36] = src->mode;
    payload[37] = src->status;
    payload[38] = src->err_flags;
    payload[39] = src->seq_lost;

    return RLCA_LEN_TEL_STATUS;
}
