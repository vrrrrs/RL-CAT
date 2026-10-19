/**
  ******************************************************************************
  * @file    bsp_shell.h
  * @brief   上机自检用的极简命令行（走 USART1，1 Mbps 文本）。
  *
  *  ⚠️ 定位于【P2 阶段的上机验收工具】，不是产品功能：
  *     link_pi 实现后，USART1 会被 0xAA55 二进制协议占用，
  *     届时本 shell 由 RLCA_SHELL_EN 关掉即可（编译期开关）。
  *
  *  为什么需要它：
  *     P2/P3 阶段树莓派程序还不存在，若没有 shell，
  *     "UART 能不能收发""I2C 上拉装了没""舵机在不在总线上"都没法验证。
  *
  *  命令（均为 ASCII，'\r' 或 '\n' 结束）：
  *     h / ?        帮助
  *     s            状态：时钟、DWT、缓冲计数、溢出、Flash 标定状态
  *     l <0|1>      LED 灭/亮
  *     m <off|on|slow|fast|estop>   状态灯模式
  *     e            擦除 Flash 标定页
  *     i            I2C 自检：总线释放电平 + 探测 0x50
  *     p <n>        在 USART1 回显 n 个字节（默认 16）—— 验链路
  *     w <type> <len> <hex...>   往 USART1 发一个 RLCA 协议帧（验帧层）
  *     r            复位统计计数
  ******************************************************************************
  */

#ifndef BSP_SHELL_H
#define BSP_SHELL_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

#if RLCA_SHELL_EN

void bsp_shell_init(void);

/** 主循环高频调用：解析 USART1 收到的字符。 */
void bsp_shell_poll(void);

#else  /* RLCA_SHELL_EN == 0 */

static inline void bsp_shell_init(void) { }
static inline void bsp_shell_poll(void) { }

#endif /* RLCA_SHELL_EN */

#endif /* BSP_SHELL_H */
