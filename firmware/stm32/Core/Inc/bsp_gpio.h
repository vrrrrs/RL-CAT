/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   LED / 心跳 / 错误指示。
  *
  *  引脚分工（统一口径，docs/30-firmware.md §4）：
  *    PA8 = HEARTBEAT  —— 心跳：固件主循环在跑就 1 Hz 翻转
  *    PB0 = LED        —— 状态灯：表示业务状态（正常/警告/急停/未标定）
  *
  *  ⚠️ PA8 与 MCO 冲突：CubeMX 配置清单 §2.1 提到"用 MCO 输出时钟到 PA8 量频"
  *     只是临时调试手段，测完必须恢复成 GPIO 输出，否则心跳失效。
  ******************************************************************************
  */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

/** LED 状态灯的显示模式 */
typedef enum {
    BSP_LED_OFF = 0,
    BSP_LED_ON,
    BSP_LED_SLOW_BLINK,     /* 1 Hz —— 电压 L1 警告 / 未标定 */
    BSP_LED_FAST_BLINK,     /* 5 Hz —— 降级 / 总线异常        */
    BSP_LED_PATTERN_ESTOP,  /* 双闪停 —— 急停                */
    BSP_LED_PATTERN_ERROR   /* 10 Hz —— Error_Handler         */
} bsp_led_mode_t;

void bsp_gpio_init(void);

/** 设置状态灯模式。 */
void bsp_led_set(bsp_led_mode_t mode);

/** 当前 LED 模式。
 *  ⚠️ 供上层判断"现在这个闪烁是不是我设的"，从而安全地做自愈 ——
 *     例如周期超时指示在恢复后要能自己清掉，
 *     但绝不能误清业务状态（急停图案等）。 */
bsp_led_mode_t bsp_led_mode(void);

/** 低频调用（每 1 ms 一次即可）：推进闪烁模式。 */
void bsp_led_poll(void);

/** 心跳：每 500 ms 翻转一次 → 周期 1 Hz。主循环低频调用。 */
void bsp_heartbeat_poll(void);

/** 心跳/状态灯立即置位（自检用）。 */
void bsp_led_force(bool on);

/** 致命错误：LED 10 Hz 快闪 + 关中断死循环。
 *  ⚠️ 用它替换 CubeMX 生成的 Error_Handler()，
 *     以便区分"启动失败（快闪）"与"根本没跑（不亮）"。 */
void bsp_fatal_error(void);

#endif /* BSP_GPIO_H */
