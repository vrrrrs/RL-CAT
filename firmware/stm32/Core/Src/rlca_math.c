/**
  ******************************************************************************
  * @file    rlca_math.c
  * @brief   关节角 ↔ 舵机步数 换算与标定表校验。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <math.h>
#include <string.h>

#include "rlca_math.h"
#include "rlca_crc16.h"

/*==============================================================================
 * 常量
 *============================================================================*/

#define RLCA_TWO_PI   6.28318530718f
#define RLCA_PI       3.14159265359f

/* 舵机负载寄存器：bit10 是方向位，低 10 位是幅度（0~1000） */
#define RLCA_LOAD_DIR_BIT   0x0400
#define RLCA_LOAD_MAG_MASK  0x03FF

/*==============================================================================
 * 标定表
 *============================================================================*/

void rlca_calib_default_dirs(rlca_calib_t *c)
{
    /* ⚠️ 方向表集中在 rlcat_config.h 的 RLCA_JOINT_DIR_INIT。
     *    默认全 +1（理由见该宏的注释）；P6 标定时实测确认后只改那一处。 */
    static const int8_t k_dirs[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
    for (uint8_t i = 0; i < RLCA_N_JOINTS; ++i) {
        c->dir[i] = k_dirs[i];
    }
}

void rlca_calib_fill_limits(rlca_calib_t *c)
{
    for (uint8_t i = 0; i < RLCA_N_JOINTS; ++i) {
        float qmin;
        float qmax;

        if (i <= (uint8_t)RLCA_J_RB_HIP) {
            qmin = RLCA_HIP_MIN_RAD;
            qmax = RLCA_HIP_MAX_RAD;
        } else if (i <= (uint8_t)RLCA_J_RB_THIGH) {
            qmin = RLCA_THIGH_MIN_RAD;
            qmax = RLCA_THIGH_MAX_RAD;
        } else {
            qmin = RLCA_CALF_MIN_RAD;
            qmax = RLCA_CALF_MAX_RAD;
        }

        /* limit_* 与 offset 无关：它描述的是关节角的允许区间，
         * 因此用 offset = 0 换算，得到"相对零点的步数范围"。 */
        rlca_calib_t tmp = *c;
        tmp.offset[i] = 0;
        tmp.dir[i] = 1;
        c->limit_min[i] = rlca_q_to_step(&tmp, i, qmin);
        c->limit_max[i] = rlca_q_to_step(&tmp, i, qmax);
    }
}

void rlca_calib_load_defaults(rlca_calib_t *c)
{
    memset(c, 0, sizeof(*c));
    c->magic   = RLCA_CALIB_MAGIC;
    c->version = RLCA_CALIB_VERSION;
    rlca_calib_default_dirs(c);
    rlca_calib_fill_limits(c);
    c->crc16 = rlca_calib_crc(c);
}

/*==============================================================================
 * ⭐⭐ 几何标定表（2026-10-18）
 *============================================================================*/

/** 由【纯常量】推导整张标定表，不读 flash、不测任何东西。
 *
 *  ⚠️⚠️ 前提（已由项目负责人在机械上验证）：
 *     **把机器人摆成标定姿态（腿伸直、髋水平）时，12 个舵机的编码器都读到 2048。**
 *     （舵机内部零位已手动校准到这一姿态。）
 *
 *  ⭐ 于是 offset 可以完全由常量算出：
 *
 *       offset = step_cal − dir × q_calib × (4096/2π)
 *              = 2048      − dir × q_calib × (4096/2π)
 *
 *  ⚠️⚠️ **offset 不等于 2048，也不该等于 2048**：
 *     · `offset` 的物理含义是"**URDF 零位（弯腿站姿）**时的编码器读数"
 *     · 2048 是"**标定姿态（伸直腿）**时的编码器读数"
 *     · 两者相差 q_calib（thigh 45°/40°、calf 90°/85°）
 *     ⇒ 直接用 2048 会让 thigh 偏 512 步（45°）、calf 偏 1024 步（90°），
 *       机器人一上电就会疯狂伸展。**必须走这个公式。**
 *
 *  ⭐ 为什么这样做（解决"重启有概率变成未标定"）：
 *     · 完全不依赖 flash ⇒ 重启结果**永远一致**
 *     · 不需要标定流程 ⇒ 少一条出错路径
 *     · 只由 `RLCA_JOINT_DIR_INIT` + `RLCA_CALIB_POSE_Q` 两个编译期常量决定
 *       ⇒ **可复现、可 review、可 diff**
 *
 *  ⚠️ 代价：放弃了"吸收装配公差"的能力。
 *     本表与逐台实测值的差约 0.2~11°（最大在 calf），因为 `q_calib` 常量
 *     是按设计几何给的，而实测姿态很难做到完全精确。
 *     ⇒ 若日后某个关节系统性偏，可微调 `RLCA_CALIB_POSE_*_DEG`，
 *       或把 `RLCA_CALIB_GEOMETRIC_EN` 置 0 回到 flash 标定。
 */
void rlca_calib_build_geometric(rlca_calib_t *c)
{
    if (c == NULL) {
        return;
    }

    memset(c, 0, sizeof(*c));
    c->magic   = RLCA_CALIB_MAGIC;
    c->version = RLCA_CALIB_VERSION;
    rlca_calib_default_dirs(c);

    /* ⚠️ q_calib 是【幅值】（RLCA_CALIB_POSE_Q 里全是正数），
     *    符号只通过 dir 体现 —— 与 offset 的推导公式一致。 */
    static const float k_q_calib[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;
    const float spr = RLCA_SERVO_STEPS_PER_REV / RLCA_TWO_PI;

    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const float d = (float)c->dir[i] * k_q_calib[i] * spr;

        /* 对称取整，与 rlca_q_to_step 保持一致 */
        const float r = (d >= 0.0f) ? (d + 0.5f) : (d - 0.5f);

        int32_t off = (int32_t)RLCA_CALIB_POSE_STEP - (int32_t)r;

        /* 归一到 [0, 4096) —— 编码器是单圈的 */
        while (off < 0) {
            off += (int32_t)RLCA_SERVO_STEPS_PER_REV;
        }
        while (off >= (int32_t)RLCA_SERVO_STEPS_PER_REV) {
            off -= (int32_t)RLCA_SERVO_STEPS_PER_REV;
        }
        c->offset[i] = (int16_t)off;
    }

    rlca_calib_fill_limits(c);
    c->crc16 = rlca_calib_crc(c);
}

uint16_t rlca_calib_crc(const rlca_calib_t *c)
{
    /* 把 crc16 字段自身当作 0 参与计算：这样"计算"与"校验"用同一份代码，
     * 不需要两块内存做比较。 */
    rlca_calib_t tmp = *c;
    tmp.crc16 = 0u;
    return rlca_crc16((const uint8_t *)&tmp, (uint16_t)sizeof(tmp));
}

bool rlca_calib_offset_ok(int16_t offset)
{
    /* ⚠️ 用绝对值比较，而不是 `offset > 4096` ——
     *    后者对负值恒为真，负的超范围 offset 会漏过检查。 */
    const int32_t v = (int32_t)offset;
    return (v >= -4096) && (v <= 4096);
}

bool rlca_calib_is_valid(const rlca_calib_t *c)
{
    if (c->magic != RLCA_CALIB_MAGIC) {
        return false;
    }
    if (c->version != RLCA_CALIB_VERSION) {
        return false;
    }
    for (uint8_t i = 0; i < RLCA_N_JOINTS; ++i) {
        if ((c->dir[i] != 1) && (c->dir[i] != -1)) {
            return false;
        }
        if (c->limit_min[i] > c->limit_max[i]) {
            return false;
        }
        if (!rlca_calib_offset_ok(c->offset[i])) {
            return false;
        }
    }
    return true;
}

/*==============================================================================
 * 换算
 *============================================================================*/

int16_t rlca_q_to_step(const rlca_calib_t *c, uint8_t idx, float q_rad)
{
    if (idx >= RLCA_N_JOINTS) {
        return 0;
    }

    /* 步 = dir × q × (4096 / 2π) + offset
     * ⚠️ 用 float 常量后缀，避免在 -Wdouble-promotion 下提升为 double。 */
    const float steps = q_rad * (RLCA_SERVO_STEPS_PER_REV / RLCA_TWO_PI);
    const float s = ((float)c->dir[idx] * steps) + (float)c->offset[idx];

    /* 对称取整：+0.5 / -0.5，避免负数向零截断带来的偏差 */
    const float rounded = (s >= 0.0f) ? (s + 0.5f) : (s - 0.5f);
    return (int16_t)rounded;
}

float rlca_step_to_q(const rlca_calib_t *c, uint8_t idx, int16_t step)
{
    if (idx >= RLCA_N_JOINTS) {
        return 0.0f;
    }
    const float d = (float)((int32_t)step - (int32_t)c->offset[idx]);
    const float s = d * (float)c->dir[idx];
    return s * (RLCA_TWO_PI / RLCA_SERVO_STEPS_PER_REV);
}

void rlca_q_range(const rlca_calib_t *c, uint8_t idx, float *q_min, float *q_max)
{
    if (idx >= RLCA_N_JOINTS) {
        if (q_min != NULL) { *q_min = 0.0f; }
        if (q_max != NULL) { *q_max = 0.0f; }
        return;
    }

    /* ⚠️⚠️ 这里曾经多乘了一次 dir，导致 dir = −1 的关节限位被整体取反
     *      （2026-10-18 由现场现象定位：`lb_hip` / `rf_hip` 发正角完全不动）。
     *
     *  正确推导：
     *    · `rlca_calib_fill_limits()` 用 `dir = 1, offset = 0` 算出
     *          limit = q × (4096/2π)     ⇒ limit **本身就是 q 空间**的量
     *    · `q_rad` 也已经是"按 dir 归一化过的关节角"：
     *          step = dir × q × SPR + offset      (q_to_step)
     *          q    = (step − offset) × dir × k    (step_to_q)
     *          两者互为逆 ⇒ q 不含 dir 的影响
     *    ⇒ 反算回 q 只需 `limit × k`，**不能再乘 dir**。
     *
     *  ⚠️ 乘了 dir 的后果（实测）：
     *      dir = −1 的 6 个关节（lb_hip, rf_hip, rf_thigh, rb_thigh,
     *      lf_calf, lb_calf）限位被取反 ⇒ 正方向的合法指令被夹成 0
     *      ⇒ 表现为"发正角完全不动"。对 hip 最明显（[0°,40°] 全被吃空）。
     *
     *  ⚠️ 这与之前修过的 `q_calib` 必须取【幅值】是同一类"双重取反"错误 ——
     *     凡同时带 dir 的两处换算，都要检查是否重复应用了符号。 */
    const float k = RLCA_TWO_PI / RLCA_SERVO_STEPS_PER_REV;
    float a = (float)c->limit_min[idx] * k;
    float b = (float)c->limit_max[idx] * k;

    if (a > b) {
        const float t = a;
        a = b;
        b = t;
    }
    if (q_min != NULL) { *q_min = a; }
    if (q_max != NULL) { *q_max = b; }
}

bool rlca_q_in_limits(const rlca_calib_t *c, uint8_t idx, float q_rad)
{
    float qmin;
    float qmax;
    rlca_q_range(c, idx, &qmin, &qmax);
    return (q_rad >= qmin) && (q_rad <= qmax);
}

float rlca_q_clamp(const rlca_calib_t *c, uint8_t idx, float q_rad)
{
    float qmin;
    float qmax;
    rlca_q_range(c, idx, &qmin, &qmax);

    if (!rlca_float_is_valid(q_rad)) {
        return qmin;                 /* NaN / Inf → 回落到下限（安全侧） */
    }
    if (q_rad < qmin) { return qmin; }
    if (q_rad > qmax) { return qmax; }
    return q_rad;
}

/*==============================================================================
 * 舵机反馈 → 物理单位
 *============================================================================*/

float rlca_servo_speed_to_rad_s(int16_t speed_step)
{
    return (float)speed_step * RLCA_SERVO_SPEED_RAD_PER_LSB;
}

float rlca_servo_load_to_torque_nm(int16_t load_raw)
{
    const uint16_t raw = (uint16_t)load_raw;
    const float mag = (float)(raw & RLCA_LOAD_MAG_MASK);
    float tau = mag * RLCA_SERVO_TORQUE_NM_PER_LSB;

    if ((raw & RLCA_LOAD_DIR_BIT) != 0u) {
        tau = -tau;
    }
    return tau;
}

int16_t rlca_servo_pos_delta(int16_t prev, int16_t now)
{
    int32_t d = (int32_t)now - (int32_t)prev;
    if (d > (int32_t)RLCA_SERVO_POS_WRAP_HALF) {
        d -= 4096;
    } else if (d < -(int32_t)RLCA_SERVO_POS_WRAP_HALF) {
        d += 4096;
    }
    return (int16_t)d;
}

void rlca_joint_state_update(const rlca_calib_t *c,
                             uint8_t idx,
                             int16_t prev_step,
                             rlca_joint_state_t *st)
{
    if ((idx >= RLCA_N_JOINTS) || (st == NULL)) {
        return;
    }

    const int8_t dir = c->dir[idx];

    /* 位置：用回绕修正后的连续位置反算关节角。
     * prev_step + delta 得到"物理连续"的位置，避免 0↔4095 跳变。 */
    const int32_t cont = (int32_t)prev_step + (int32_t)rlca_servo_pos_delta(prev_step, st->pos_step);

    const float d = (float)(cont - (int32_t)c->offset[idx]);
    st->q_rad = d * (float)dir * (RLCA_TWO_PI / RLCA_SERVO_STEPS_PER_REV);

    /* 速度：符号与位置一致，同样乘 dir */
    st->qd_rad_s = rlca_servo_speed_to_rad_s(st->speed_step) * (float)dir;

    /* 力矩：符号已在 load 的方向位里；再乘 dir 是为了统一到"关节正方向" */
    st->tau_nm = rlca_servo_load_to_torque_nm(st->load_raw) * (float)dir;
}

/*==============================================================================
 * 数值健壮性
 *============================================================================*/

bool rlca_float_is_valid(float v)
{
    /* 不用 isnan/isinf：在 -ffast-math 下它们会被优化掉。
     * 自比较法 + 区间法对 IEEE-754 单精度是可靠的。 */
    if (v != v) {
        return false;                      /* NaN */
    }
    if (v > 3.4e38f) {
        return false;                      /* +Inf */
    }
    if (v < -3.4e38f) {
        return false;                      /* -Inf */
    }
    return true;
}
