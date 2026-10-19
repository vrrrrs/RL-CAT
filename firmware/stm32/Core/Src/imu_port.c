/**
  ******************************************************************************
  * @file    imu_port.c
  * @brief   imu 端口的真实硬件实现（PB8/PB9 软件 I2C）。
  *
  *  ⚠️ 为什么是软件 I2C：STM32F103 的硬件 I2C1 复用脚是 PB6/PB7，
  *     而本项目 PCB 把 JY901P 接在 PB8/PB9 上（已配成开漏）。
  *     依据：docs/33-firmware-plan.md §4 取舍 3。
  *
  *  ⚠️ 一帧的耗时预算（固件设计书 §5.4）：
  *     读 acc(6) + gyro(6) + angle(8) = 20 数据字节，
  *     加地址/寄存器/ACK 开销约 3 × 4 = 12 字节等效，
  *     @ 400 kHz ≈ 0.72 ms；@ 100 kHz ≈ 2.9 ms（100 Hz 周期里偏多）。
  *     所以 SCL 目标 100~400 kHz，且上机要用示波器实测。
  ******************************************************************************
  */

#include "imu.h"
#include "sensor_ports.h"
#include "bsp_softi2c.h"
#include "bsp_tick.h"

static bool port_read_reg(uint8_t dev, uint8_t reg, uint8_t *dst, uint8_t len)
{
    return bsp_softi2c_read_reg(dev, reg, dst, len);
}

static const imu_port_t s_hw_port = {
    port_read_reg,
    bsp_tick_ms
};

const imu_port_t *imu_port_hardware(void)
{
    return &s_hw_port;
}
