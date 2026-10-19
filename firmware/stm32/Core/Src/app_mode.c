/**
  ******************************************************************************
  * @file    app_mode.c
  * @brief   运行模式状态机实现。
  ******************************************************************************
  */

#include <string.h>

#include "app_mode.h"
#include "link_proto.h"   /* link_mode_sanitize */
#include "app_safety.h"

/*==============================================================================
 * 内部状态
 *============================================================================*/

static app_mode_state_t s_st;

/** 模式进入后的最短保持时间（毫秒）。
 *  ⚠️ 防止 Pi 每周期都发不同的 mode 导致模式抖动、力矩反复通断。
 *     100 ms = 10 个控制周期，足够把"抖动的指令"滤掉。 */
#define MODE_MIN_HOLD_MS   100u

/*==============================================================================
 * 纯逻辑：ACTIVE 准入
 *============================================================================*/

uint8_t app_mode_active_gate(bool calib_ok, bool link_alive, bool imu_ok,
                             bool fb_ok, bool safety_blocks)
{
    /* ⚠️ 顺序有意义：先报"最根本的缺失"。
     *    未标定 → 角度换算本身就是错的，比链路问题更严重。 */
    if (!calib_ok)      { return RLCA_ACTIVE_BLOCK_NOT_CALIB; }
    if (safety_blocks)  { return RLCA_ACTIVE_BLOCK_SAFETY; }
    if (!link_alive)    { return RLCA_ACTIVE_BLOCK_NO_LINK; }
    if (!imu_ok)        { return RLCA_ACTIVE_BLOCK_NO_IMU; }
    if (!fb_ok)         { return RLCA_ACTIVE_BLOCK_NO_FB; }
    return RLCA_ACTIVE_OK;
}

/*==============================================================================
 * 初始化与请求
 *============================================================================*/

void app_mode_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.mode        = (uint8_t)RLCA_MODE_IDLE;
    s_st.active_mode = (uint8_t)RLCA_MODE_IDLE;
    s_st.active_block = RLCA_ACTIVE_BLOCK_NOT_CALIB;
    s_st.torque_on   = 0u;
    app_safety_reset();
}

void app_mode_request(uint8_t mode)
{
    s_st.mode = link_mode_sanitize(mode);
}

void app_mode_enter_calib(void)
{
    s_st.mode = (uint8_t)RLCA_MODE_IDLE;   /* CALIB 不是协议里的模式值 */
    s_st.active_mode = (uint8_t)RLCA_MODE_IDLE;
}

void app_mode_exit_calib(void)
{
    s_st.mode = (uint8_t)RLCA_MODE_IDLE;
}

/*==============================================================================
 * 主更新
 *============================================================================*/

void app_mode_update(uint32_t now_ms,
                     const rlca_calib_t *calib,
                     bool link_alive,
                     bool imu_ok,
                     bool calib_ok,
                     bool fb_ok,
                     const float *pi_q_target,
                     const float *q_now,
                     float *q_ref,
                     bool *torque_out)
{
    if ((q_ref == NULL) || (torque_out == NULL)) {
        return;
    }

    /*==========================================================================
     * ① 计算"生效模式"
     *
     *  ⚠️ 优先级：ESTOP > DAMP > ACTIVE(需过准入) > STAND > IDLE
     *     - ESTOP 来自安全层，不可被覆盖
     *     - ACTIVE 过不了准入就【降级到 STAND】，而不是掉到 IDLE：
     *       站着总比瘫着安全，而且操作者能立刻看出"力矩还在"
     *========================================================================*/

    uint8_t want = s_st.mode;

    /* 安全层是否要求急停 */
    const bool safety_estop = app_safety_in_estop();
    const bool safety_blocks = app_safety_blocks_active();

    if (safety_estop) {
        want = (uint8_t)RLCA_MODE_ESTOP;
    }

    /* ACTIVE 准入 */
    uint8_t gate = RLCA_ACTIVE_OK;
    if (want == (uint8_t)RLCA_MODE_ACTIVE) {
        gate = app_mode_active_gate(calib_ok, link_alive, imu_ok, fb_ok, safety_blocks);
        if (gate != RLCA_ACTIVE_OK) {
            want = (uint8_t)RLCA_MODE_STAND;     /* 降级而不是掉 IDLE */
        }
    }
    s_st.active_block = gate;

    /* 模式最短保持（但 ESTOP 可以立刻生效，不受保持时间限制） */
    if ((want != s_st.active_mode) && (want != (uint8_t)RLCA_MODE_ESTOP)) {
        if ((now_ms - s_st.mode_t0_ms) < MODE_MIN_HOLD_MS) {
            want = s_st.active_mode;
        }
    }

    if (want != s_st.active_mode) {
        s_st.active_mode = want;
        s_st.mode_t0_ms  = now_ms;
        s_st.n_mode_changes++;
    }

    /*==========================================================================
     * ② 生成 q_ref 与力矩状态
     *========================================================================*/

    const float q_def = RLCA_Q_DEFAULT_RAD;

    switch (s_st.active_mode) {

    case RLCA_MODE_ACTIVE:
    {
        /* 跟踪策略目标角，但先过幅度缩放（安全层给的限制） */
        const float scale = app_safety_amplitude_scale();
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            const float q = (pi_q_target != NULL) ? pi_q_target[i] : q_def;
            /* ⚠️ 幅度限制作用在"相对 q_default 的偏移"上，而不是绝对值上 ——
             *    否则低电压时会连"站姿"本身都被缩放掉，腿会软。 */
            q_ref[i] = q_def + ((q - q_def) * scale);
        }
        *torque_out = true;
        break;
    }

    case RLCA_MODE_STAND:
        /* ⚠️⚠️ 未标定时【绝不能】保持默认站姿（2026-10-12 安全修复）
         *
         *  未标定时标定表是 rlca_calib_load_defaults() 的全零虚表：
         *      offset[] = 0，dir[] = +1
         *  于是 q_ref = q_default = 0 经 rlca_q_to_step() 换算：
         *      step = dir × (0 / 0.0001) × (4096/2π) + offset = 0
         *  ⇒ 12 个舵机全部冲向【绝对位置 step 0】（机械行程一端）。
         *  这正是 app_main.c 里记录过的撞机事故。
         *
         *  ⇒ 未标定时退化为：卸力矩 + 目标跟随【当前实测角度】（待在原地）。
         *     跟随实测角而不是硬编一个值，是为了：
         *       ① 不产生任何目标位移动，力矩卸掉时舵机是静止的
         *       ② 万一日后有人把力矩重新使能，首条指令也是"待在原地"
         */
        if (!calib_ok) {
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                q_ref[i] = (q_now != NULL) ? q_now[i] : q_def;
            }
            *torque_out = false;
            break;
        }
        /* 保持默认站姿（此时 offset/dir 可信，q_ref=0 就是站姿） */
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            q_ref[i] = q_def;
        }
        *torque_out = true;
        break;

    case RLCA_MODE_DAMP:
        /* 卸力矩，靠自重自然落地。
         * ⚠️ q_ref 仍然给站姿：力矩一卸，目标角就无所谓了；
         *    但保持一个合理值能让"重新使能力矩"时不跳变。
         * ⚠️ 未标定时同样不能给 0（会落在 step 0），改为跟随实测角。 */
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            q_ref[i] = (!calib_ok && (q_now != NULL)) ? q_now[i] : q_def;
        }
        *torque_out = false;
        break;

    case RLCA_MODE_ESTOP:
        /* ⚠️ 趴下斜坡由 app_safety 在【本函数之前】已经推进过 q_ref，
         *    这里绝不能覆盖它 —— 否则趴下永远走不动。
         *    只负责在斜坡结束或超时后卸力矩。 */
        *torque_out = !app_safety_state()->torque_off;
        break;

    case RLCA_MODE_IDLE:
    default:
        /* ⚠️ 未标定时不能给 q_def(=0)，理由同 STAND —— 会换算成舵机 step 0。
         *    这里虽然卸力矩，但目标值仍会被上层合法化后参与限速基准，
         *    保持"跟随实测角"最安全。 */
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            q_ref[i] = (!calib_ok && (q_now != NULL)) ? q_now[i] : q_def;
        }
        *torque_out = false;
        break;
    }

    s_st.torque_on = *torque_out ? 1u : 0u;
}

/*==============================================================================
 * 查询
 *============================================================================*/

uint8_t app_mode_current(void)
{
    return s_st.active_mode;
}

uint8_t app_mode_requested(void)
{
    return s_st.mode;
}

const app_mode_state_t *app_mode_state(void)
{
    return &s_st;
}
