/**
  ******************************************************************************
  * @file    battery_port.c
  * @brief   battery 端口的真实硬件实现（ADC1 + DMA 循环）。
  *
  *  CubeMX 已配好（见 firmware/doc/CubeMX配置清单.md §2.4）：
  *    ADC1.Channel-0           = ADC_CHANNEL_0  → PA0
  *    ADC1.SamplingTime        = 239.5 cycles
  *    ADC1.ContinuousConvMode  = ENABLE          ← 连续转换
  *    DMA1_Channel1            = ADC1，循环模式，半字，内存递增
  *    ADC 时钟                  = APB2/6 = 12 MHz
  *
  *  ⚠️ 唯一需要固件补的一步：调一次 HAL_ADC_Start_DMA()。
  *     CubeMX 只生成配置，不会生成启动调用。
  *
  *  ⚠️ 转换时间核算：(239.5 + 12.5) / 12 MHz ≈ 21 µs → 约 47.6 kHz，
  *     远超电池监测需求（10 Hz 就够）。所以 DMA 缓冲足够长，
  *     读的时候取最后一个已完成的结果即可。
  ******************************************************************************
  */

#include "battery.h"
#include "sensor_ports.h"

#include "main.h"       /* ADC_HandleTypeDef */
#include "stm32f1xx_hal.h"
#include "bsp_tick.h"

/* ADC 句柄由 CubeMX 在 main.c 中定义（这里只引用，不重新定义） */
extern ADC_HandleTypeDef hadc1;

/* DMA 循环缓冲：哪怕 ADC 以 47.6 kHz 在跑，这里也只需要"最近的值"。
 * 取 8 个样本，写入位置由 DMA 计数器推出来，避免读到正在被写的单元。 */
#define ADC_DMA_LEN  8u
static uint16_t s_dma[ADC_DMA_LEN];
static bool     s_started = false;

/** 从 DMA 循环缓冲里取一个"刚写好的、稳定的"样本。
 *
 *  ⚠️ 关键点：DMA 在后台持续搬运，直接读 s_dma[i] 可能读到"写了一半"的值。
 *     做法是取"当前位置往回一格"的那个元素 —— 它一定已经写完
 *     （下一个要写的位置才是当前计数器指向的那个）。 */
static bool port_read_raw(uint16_t *raw)
{
    if ((raw == NULL) || !s_started) {
        return false;
    }

    /* 已写入的元素个数 = 缓冲长度 − DMA 剩余传输数 */
    const uint16_t done = (uint16_t)(ADC_DMA_LEN -
                          (uint16_t)__HAL_DMA_GET_COUNTER(hadc1.DMA_Handle));

    /* 当前位置之前那个元素 = 最近一个完整写入的样本 */
    const uint16_t idx = (uint16_t)((done + ADC_DMA_LEN - 1u) % ADC_DMA_LEN);
    const uint16_t v = s_dma[idx];

    /* 12 位 ADC 的上限是 4095；超出说明读到的是未初始化/异常值 */
    if (v > 4095u) {
        return false;
    }
    *raw = v;
    return true;
}

static const batt_port_t s_hw_port = {
    port_read_raw,
    bsp_tick_ms
};

const batt_port_t *battery_port_hardware(void)
{
    return &s_hw_port;
}

void battery_port_start(void)
{
    if (s_started) {
        return;
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma, ADC_DMA_LEN) == HAL_OK) {
        s_started = true;
    }
}
