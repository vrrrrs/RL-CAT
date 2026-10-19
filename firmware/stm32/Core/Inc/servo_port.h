/**
  ******************************************************************************
  * @file    servo_port.h
  * @brief   把 BSP 的 UART/时基适配成 servo_bus 需要的端口。
  *
  *  ⚠️ 为什么要这一层：
  *     servo_bus 通过 rlca_servo_port_t 做依赖注入，好处是状态机可以在
  *     PC 上用假总线单测（见 firmware/tests/host/test_servo_bus.c）。
  *     固件侧的"真实实现"就落在这个文件里 —— 只有这里碰 HAL。
  ******************************************************************************
  */

#ifndef SERVO_PORT_H
#define SERVO_PORT_H

#include "servo_bus.h"

/** 取得"真实硬件"端口（USART2 + DWT 微秒时基）。 */
const rlca_servo_port_t *servo_port_hardware(void);

#endif /* SERVO_PORT_H */
