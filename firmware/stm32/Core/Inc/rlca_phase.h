/**
  ******************************************************************************
  * @file    rlca_phase.h
  * @brief   步态相位（观测的第 45–46 维）。
  *
  *  ⭐ 相位由 STM32 产生，经 TEL_FAST 下发给树莓派 —— 理由见
  *     docs/30-firmware.md §5.1.1（唯一源头 / 时钟精度 / STM32 本就是节拍源）。
  *
  *  算法必须与训练侧严格一致（Sim/.../rlcat_env.py:104-106）：
  *
  *      训练侧：phase = (phase + dt × gait_freq) mod 1
  *              dt = 1/200（物理步），decimation = 2 → 每控制周期推进 0.03
  *      固件  ：完全等价，但用【整数定点】累加避免浮点误差累积：
  *              相位周期 RLCA_PHASE_PERIOD = 100 个刻度 = 1 圈
  *              每控制周期 +3 个刻度（= 3.0 Hz / 100 Hz × 100）
  *
  *  ⚠️ 为什么不用 float 累加：float 加 0.03 跑 10 万周期会累积漂移，
  *     而整数在 100 刻度模下是精确的（100 与 3 互质，周期恰为 100 步）。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_PHASE_H
#define RLCA_PHASE_H

#include <stdint.h>

/* 相位刻度：1 圈 = 100 刻度（与 GAIT_FREQ_HZ / CTRL_HZ = 0.03 对应） */
#define RLCA_PHASE_PERIOD   100u
#define RLCA_PHASE_HALF     (RLCA_PHASE_PERIOD / 2u)     /* = 50 → π         */
#define RLCA_PHASE_QUARTER  (RLCA_PHASE_PERIOD / 4u)     /* = 25 → π/2       */

/** 纯函数：按 10 段多项式计算 sin(2π·k/100)，k ∈ [0, 100)。
 *
 *  用 9 阶泰勒展开（截断误差 < 1e-9），而不是查表：
 *    · 不需要 1 KB 常量表，代码约 20 行；
 *    · 不调用 sinf()，不引 CMSIS-DSP（本项目明确不用）；
 *    · 确定性：同样的 k 在任何编译选项下得到同样的浮点结果。
 *
 *  @param k 相位刻度，自动对 RLCA_PHASE_PERIOD 取模
 *  @return sin 值，范围 [-1, 1] */
float rlca_phase_sin(uint32_t k);

/** 纯函数：cos(2π·k/100) —— 复用 sin(k + 25)。 */
float rlca_phase_cos(uint32_t k);

/*==============================================================================
 * 有状态接口（固件用）
 *============================================================================*/

/** 复位相位到 0（上电、以及每个 episode 开始时用）。 */
void rlca_phase_reset(void);

/** 推进一个控制周期（100 Hz），返回新的相位刻度。
 *  ⚠️ 必须在每个控制周期【调用且仅调用一次】。 */
uint32_t rlca_phase_advance(void);

/** 读取当前相位刻度（不推进）。 */
uint32_t rlca_phase_get(void);

/** 当前相位对应的 sin/cos，已编码为 TEL_FAST 用的 int16（÷32767）。 */
void rlca_phase_get_sincos_i16(int16_t *sin_out, int16_t *cos_out);

/** 按训练侧公式（浮点）计算 sin/cos，仅用于自检与单元测试比对。
 *  固件运行时不走这条路径，但结果必须与 rlca_phase_sin/cos 一致。 */
void rlca_phase_sincos_float(uint32_t k, float *sin_out, float *cos_out);

#endif /* RLCA_PHASE_H */
