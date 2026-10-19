/**
  ******************************************************************************
  * @file    rlca_phase.c
  * @brief   步态相位的整数定点累加 + 多项式 sin/cos。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <stddef.h>

#include "rlca_phase.h"

/*==============================================================================
 * 常量
 *============================================================================*/

#define RLCA_TWO_PI_F       6.28318531f
#define RLCA_HALF_PI        1.5707964f     /* π/2，单精度可精确表示        */
#define RLCA_QUARTER_PI     0.78539816f    /* π/4，十六分区步长             */
#define RLCA_FOUR_OVER_PI   1.27323954f    /* 4/π，用于十六分区归约          */

/** 每控制周期推进的刻度数 = GAIT_FREQ_HZ / CTRL_HZ × PHASE_PERIOD
 *                                              = 3.0 / 100 × 100 = 3 */
#define RLCA_PHASE_STEP_TICKS \
    ((uint32_t)((RLCA_PHASE_PERIOD * 3u) / 100u))   /* = 3 */

/*==============================================================================
 * 内部状态
 *============================================================================*/

static uint32_t s_phase_ticks = 0u;

/*==============================================================================
 * 纯函数
 *============================================================================*/

/** sin(t)，|t| ≤ π/4 —— 9 阶泰勒展开。
 *
 *  取 |t| ≤ π/4（而不是 π/2）：z = t² ≤ 0.6169，
 *  截断项 t^11/11! ≤ (π/4)^11/39916800 ≈ 7.0e-11，远小于单精度分辨率。
 *  同时保证"补角"π/2 − |t| 也落在本函数可用的区间内（见 octant 分支）。 */
static float sin_small(float t)
{
    const float z = t * t;
    const float p = 1.0f
                  + z * (-1.0f / 6.0f
                  + z * ( 1.0f / 120.0f
                  + z * (-1.0f / 5040.0f
                  + z * ( 1.0f / 362880.0f))));
    return t * p;
}

float rlca_phase_sin(uint32_t k)
{
    /* ① 取模到一圈 */
    k %= RLCA_PHASE_PERIOD;

    /* ② 转成 rad —— 与训练侧的 2π×phase 完全同构 */
    const float x = (float)k * (RLCA_TWO_PI_F / (float)RLCA_PHASE_PERIOD);

    /* ③ 十六分区归约：q = round(x / (π/4))，r = x − q·π/4 ∈ [−π/8, π/8]
     *    ⚠️ 必须用 π/4 而不是 π/2 作步长：
     *       若按 π/2 归约，余量可到 ±π/4，此时"补角" π/2 − r 会达到 3π/4，
     *       超出泰勒展开的精度区间（实测引入 ~2.6e-4 误差）。
     *       按 π/4 归约后，两个多项式的自变量都 ≤ 3π/8，安全。 */
    const float qf = (x * RLCA_FOUR_OVER_PI) + 0.5f;
    const int32_t q = (int32_t)qf;
    const float r  = x - ((float)q * RLCA_QUARTER_PI);

    /* ④ sin(r) 与 cos(r)（用补角，|r| ≤ π/8 ⇒ π/2 − |r| ∈ [3π/8, π/2]） */
    const float sn = sin_small(r);
    const float ar = (r < 0.0f) ? -r : r;
    const float cs = sin_small(RLCA_HALF_PI - ar);

    /* ⑤ 用 sin 在 0..π/2 上的值 + 对称性还原：
     *      x = q·(π/4) + r
     *      q mod 8 == 0 → sin(r)
     *                 1 → sin(π/4 + r) = (sin r + cos r)·(√2/2)
     *                 2 → sin(π/2 + r) =  cos r
     *                 3 → sin(3π/4 + r) = (cos r − sin r)·(√2/2)
     *                 4 → sin(π + r)    = −sin r
     *                 5 → −(sin r + cos r)·(√2/2)
     *                 6 → −cos r
     *                 7 → −(cos r − sin r)·(√2/2)
     *   ⚠️ 注意 q 为负时 % 在 C 里是向零取整，必须先归一化到 [0,7]。 */
    const int32_t oct = ((q % 8) + 8) % 8;
    const float s2 = 0.70710678f;                 /* √2 / 2 */

    switch (oct) {
        case 0:  return  sn;
        case 1:  return (sn + cs) * s2;
        case 2:  return  cs;
        case 3:  return (cs - sn) * s2;
        case 4:  return -sn;
        case 5:  return -(sn + cs) * s2;
        case 6:  return -cs;
        default: return -(cs - sn) * s2;
    }
}

float rlca_phase_cos(uint32_t k)
{
    /* cos(θ) = sin(θ + π/2)，π/2 对应 25 个刻度 */
    return rlca_phase_sin(k + RLCA_PHASE_QUARTER);
}

void rlca_phase_sincos_float(uint32_t k, float *sin_out, float *cos_out)
{
    if (sin_out != NULL) { *sin_out = rlca_phase_sin(k); }
    if (cos_out != NULL) { *cos_out = rlca_phase_cos(k); }
}

/*==============================================================================
 * 有状态接口
 *============================================================================*/

void rlca_phase_reset(void)
{
    s_phase_ticks = 0u;
}

uint32_t rlca_phase_advance(void)
{
    s_phase_ticks += RLCA_PHASE_STEP_TICKS;
    if (s_phase_ticks >= RLCA_PHASE_PERIOD) {
        s_phase_ticks -= RLCA_PHASE_PERIOD;
    }
    return s_phase_ticks;
}

uint32_t rlca_phase_get(void)
{
    return s_phase_ticks;
}

void rlca_phase_get_sincos_i16(int16_t *sin_out, int16_t *cos_out)
{
    /* 协议 §4.4：编码单位 1/32767，即 int16 = round(sin × 32767) */
    if (sin_out != NULL) {
        *sin_out = (int16_t)(rlca_phase_sin(s_phase_ticks) * 32767.0f);
    }
    if (cos_out != NULL) {
        *cos_out = (int16_t)(rlca_phase_cos(s_phase_ticks) * 32767.0f);
    }
}
