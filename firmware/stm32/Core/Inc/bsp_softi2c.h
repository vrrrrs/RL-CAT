/**
  ******************************************************************************
  * @file    bsp_softi2c.h
  * @brief   软件 I2C 主机（PB8 = SCL，PB9 = SDA，开漏 + 外部 4.7 kΩ 上拉）。
  *
  *  ⚠️ 为什么必须是软件 I2C：
  *     STM32F103 的硬件 I2C1 复用脚是 PB6/PB7，而本项目 PCB 把 JY901P
  *     接在 PB8/PB9 上（且已配成开漏）。所以只能用 GPIO 模拟。
  *     依据：docs/33-firmware-plan.md §4 取舍 3。
  *
  *  ⚠️ 开漏模式的正确用法：
  *     写 0 = 主动拉低；写 1 = 释放总线（靠外部上拉变高）。
  *     所以"读引脚电平"与"写电平"是两回事：读必须读 IDR。
  *
  *  ⚠️ 时序：用 DWT 微秒计时做半周期延时，SCL 频率 ≈ 1/(2×半周期)。
  *     目标区间 100~400 kHz（固件设计书 §5.4）。
  *     ⚠️ 必须用示波器实测 SCL 频率，不要靠数 __NOP() 猜。
  ******************************************************************************
  */

#ifndef BSP_SOFTI2C_H
#define BSP_SOFTI2C_H

#include <stdint.h>
#include <stdbool.h>

/** 半周期延时（微秒）。0.5 µs ⇒ 1 MHz 理论值，实际受 GPIO/上拉限制约 300 kHz。
 *  ⚠️ 这是上机验收单里要实测并微调的唯一参数。 */
#define BSP_I2C_HALF_US   1u

/** 初始化（配置 GPIO 为开漏输出并释放总线）。 */
void bsp_softi2c_init(void);

/** 总线自检：检查 SCL/SDA 在释放状态下是否被上拉到高。
 *  返回 false 说明外部上拉缺失、或总线被拉死 —— 这是硬件问题，
 *  比"读不到 IMU 数据"更早暴露。 */
bool bsp_softi2c_bus_ok(void);

/** 写寄存器（标准 I2C：START + 器件地址(写) + 寄存器 + 数据 + STOP）。 */
bool bsp_softi2c_write_reg(uint8_t dev, uint8_t reg, const uint8_t *data, uint8_t len);

/** 读寄存器（START + 地址(写) + 寄存器 + RESTART + 地址(读) + 数据 + STOP）。 */
bool bsp_softi2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t len);

/** 探测器件是否应答（发地址 + 等 ACK）。 */
bool bsp_softi2c_probe(uint8_t dev);

/** 恢复总线：若从机把 SDA 拉死，发 9 个时钟让它释放，再补一个 STOP。 */
void bsp_softi2c_bus_recover(void);

#endif /* BSP_SOFTI2C_H */
