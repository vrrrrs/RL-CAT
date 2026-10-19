/**
  ******************************************************************************
  * @file    battery.h
  * @brief   电池电压监测：ADC(DMA) → mV → L1/L2/L3 分级。
  *
  *  硬件（docs/20-electronics.md §1.4）：
  *    电池 + ── R1 = 100 kΩ ──┬── PA0 (ADC1_IN0)
  *                            └── R2 = 20 kΩ ── GND
  *    分压比 = (100 + 20) / 20 = 6.000
  *
  *  ⚠️⚠️ 2026-10-18 更正：原来是 22 kΩ（5.545），实测硬件是 **20 kΩ（6.000）**。
  *     按旧值算电压会偏低 7.6%，导致长期落在 VOLT_STOP 区间、幅度被误压缩。
  *     详见 docs/49-progress-review.md §4-B1。
  *
  *  ⚠️ 两步标定（必须做，docs/30-firmware.md §6.4）：
  *    ① 万用表量 STM32 模块 3.3 V 引脚的真实值 → 填 RLCA_ADC_VREF_MV
  *       （标称 3300，实际常在 3260~3340）
  *    ② 对比固件读数与万用表量到的电池电压 → 微调分压比
  *       目标：误差 < 50 mV
  *
  *  纯逻辑部分（换算、分级、滤波）不依赖 HAL，可 PC 单测。
  *  ADC 的启动与 DMA 读取在 battery.c 里，通过注入的采样端口隔离。
  ******************************************************************************
  */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

/*==============================================================================
 * 纯逻辑：换算与分级
 *============================================================================*/

/** 电池告警级别（对应协议 §4.5 的 status 位） */
typedef enum {
    BATT_LEVEL_OK = 0,      /* > 10.5 V                        */
    BATT_LEVEL_WARN,        /* ≤ 10.5 V，L1：LED 慢闪 + 上报    */
    BATT_LEVEL_STOP,        /* ≤  9.9 V，L2：上报 + 限制动作幅度 */
    BATT_LEVEL_FORCE        /* ≤  9.3 V，L3：卸载力矩 → 趴下     */
} batt_level_t;

/** ADC 原始值 → 电池电压（mV，已钳到 uint16 范围）。
 *  @param raw      ADC 采样值（0..4095）
 *  @param vref_mv  ADC 参考电压（mV）。⚠️ 必须实测校准
 *  @param div_num  分压比分子（R1 + R2）
 *  @param div_den  分压比分母（R2）
 *
 *  公式：电池mV = raw / 4095 × vref_mv × div_num / div_den */
uint16_t batt_adc_to_mv(uint16_t raw, float vref_mv, float div_num, float div_den);

/** 用默认标定参数（rlcat_config.h）做换算。 */
uint16_t batt_adc_to_mv_default(uint16_t raw);

/** 电压 → 告警级别。 */
batt_level_t batt_classify(uint16_t mv);

/** 电压 → status 位域（协议 §4.5 的 BIT4/5/6）。 */
uint8_t batt_status_bits(uint16_t mv);

/*==============================================================================
 * 有状态层
 *============================================================================*/

typedef struct {
    uint16_t mv;            /* 滤波后的电池电压（mV）      */
    uint16_t mv_raw;        /* 未滤波的瞬时值（诊断用）    */
    uint16_t adc_raw;       /* 最近一次 ADC 原始值         */
    uint8_t  level;         /* batt_level_t                */
    uint8_t  valid;         /* 是否已取得有效采样          */
    uint32_t n_samples;     /* 累计采样次数                */
    uint32_t n_timeouts;    /* 采样超时次数                */
} batt_state_t;

/** ADC 采样端口（依赖注入）。
 *  ⚠️ 与 servo_port / link_port 同样的手法：把"读 DMA 缓冲"这件事抽象出来，
 *     这样滤波、分级、状态机都能在 PC 上用假采样器确定性地单测。 */
typedef struct {
    /** 读取最近一次 ADC 转换结果（由 DMA 循环转换后台完成）。
     *  @return true = 取到有效值 */
    bool (*read_raw)(uint16_t *raw);

    /** 毫秒时间戳（用于失效判定） */
    uint32_t (*millis)(void);
} batt_port_t;

/** 绑定端口并复位状态。 */
void battery_init(const batt_port_t *port);

/** 低频调用（1~10 Hz 足够）。
 *  每次调用取一个采样并更新滤波值；丢失采样超过 RLCA_BATT_STALE_MS 则 mark 失效。 */
void battery_poll(void);

/** 当前状态。 */
const batt_state_t *battery_state(void);

/** 当前级别。 */
batt_level_t battery_level(void);

/** 当前电压（mV）。 */
uint16_t battery_mv(void);

/** 数据是否新鲜（近期取到过有效采样）。
 *  ⚠️ 电池读数失效不应导致急停（那不是安全项），但必须让 Pi 知道。 */
bool battery_is_valid(void);

/** 覆盖标定参数（上机标定用；传 0 表示用默认值）。 */
void battery_set_calibration(float vref_mv, float div_num, float div_den);

#endif /* BATTERY_H */
