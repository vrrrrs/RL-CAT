/**
  ******************************************************************************
  * @file    bsp_tick.h
  * @brief   时基：毫秒来自 HAL_GetTick()，微秒来自 DWT 周期计数器。
  *
  *  为什么两套：
  *    · 毫秒 —— SysTick 已被 HAL 占用（1 ms 中断），直接用 HAL_GetTick()，
  *              不需要额外的 TIM2（CubeMX 配置清单 §4.1 方案 A）
  *    · 微秒 —— 舵机总线整帧超时是 3 ms 量级、turnaround 是几十 µs，
  *              需要 µs 分辨率。DWT 的 CYCCNT 在 Cortex-M3 上零外设占用、
  *              零中断开销（CubeMX 配置清单 §4.1）。
  *
  *  ⚠️ DWT CYCCNT 是 32 位、72 MHz → 约 59.6 s 回绕一次。
  *     所有"算差值"的接口都用无符号减法，回绕天然正确；
  *     但【绝对时间】比较不能用（必须用差值）。
  ******************************************************************************
  */

#ifndef BSP_TICK_H
#define BSP_TICK_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"   /* HAL_GetTick / HAL_Delay / __get_PRIMASK */

/** 使能 DWT 周期计数器。必须在 SystemClock_Config() 之后调用。 */
void bsp_tick_init(void);

/** 自检：DWT 是否真的在走。返回 true = 可用。 */
bool bsp_tick_selftest(void);

/** 毫秒时间戳（HAL_GetTick 的别名，便于统一改名）。 */
uint32_t bsp_tick_ms(void);

/** 微秒时间戳（由 DWT 折算，32 位回绕）。 */
uint32_t bsp_tick_us(void);

/** 距今已过多少毫秒（回绕安全）。 */
uint32_t bsp_ms_since(uint32_t since_ms);

/** 距今已过多少微秒（回绕安全）。 */
uint32_t bsp_us_since(uint32_t since_us);

/** 忙等延时（微秒）。⚠️ 只在初始化与 bit-bang 时序里用，不要在控制环里用。 */
void bsp_delay_us(uint32_t us);

/** 当前 CPU 周期计数（供调试/性能打点）。 */
uint32_t bsp_cycles(void);

#endif /* BSP_TICK_H */
