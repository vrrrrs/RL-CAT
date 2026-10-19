/**
  ******************************************************************************
  * @file    sensor_ports.h
  * @brief   battery / imu 端口的真实硬件实现声明。
  *
  *  ⚠️ 与 servo_port / link_port 同样的分层约定：
  *     这两个 .c 是【唯一碰 HAL 的地方】，业务层只认注入的端口，
  *     所以业务逻辑能在 PC 上单测。
  ******************************************************************************
  */

#ifndef SENSOR_PORTS_H
#define SENSOR_PORTS_H

#include "battery.h"
#include "imu.h"

/* ---- battery（ADC1 + DMA 循环） ---- */

/** 取得 ADC 端口（从 DMA 循环缓冲读最新样本）。 */
const batt_port_t *battery_port_hardware(void);

/** 启动 ADC + DMA。必须在 MX_ADC1_Init() / MX_DMA_Init() 之后调用一次。
 *  ⚠️ CubeMX 只生成配置，不会生成启动调用 —— 这一步必须由固件补。 */
void battery_port_start(void);

/* ---- imu（PB8/PB9 软件 I2C） ---- */

/** 取得 IMU 端口。 */
const imu_port_t *imu_port_hardware(void);

#endif /* SENSOR_PORTS_H */
