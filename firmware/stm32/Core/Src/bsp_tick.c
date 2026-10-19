/**
  ******************************************************************************
  * @file    bsp_tick.c
  * @brief   毫秒（HAL_GetTick）+ 微秒（DWT CYCCNT）时基实现。
  ******************************************************************************
  */

#include "bsp_tick.h"
#include "rlcat_config.h"

/* Cortex-M3 调试寄存器（不占外设，也不需要 CubeMX 配置） */
#define RLCA_DWT_CTRL    (*(volatile uint32_t *)0xE0001000u)
#define RLCA_DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004u)
#define RLCA_DEMCR       (*(volatile uint32_t *)0xE000EDFCu)

#define RLCA_DEMCR_TRCENA   (1u << 24)
#define RLCA_DWT_CYCCNTENA  (1u << 0)

/** 一个微秒等于多少 CPU 周期（向上取整至少 1，避免除零/零延时） */
#define RLCA_CYCLES_PER_US  ((RLCA_SYS_CLK_HZ + 999999u) / 1000000u)

static bool s_dwt_ok = false;

void bsp_tick_init(void)
{
    /* 使能跟踪与调试模块（TRCENA），否则 DWT 寄存器不可写 */
    RLCA_DEMCR |= RLCA_DEMCR_TRCENA;

    RLCA_DWT_CYCCNT = 0u;
    RLCA_DWT_CTRL  |= RLCA_DWT_CYCCNTENA;

    /* 验证它在真的走：读两次，中间插一段已知延时 */
    const uint32_t a = RLCA_DWT_CYCCNT;
    for (volatile uint32_t i = 0u; i < 200u; ++i) {
        /* 空转，只为一个可观的时间跨度 */
    }
    const uint32_t b = RLCA_DWT_CYCCNT;

    s_dwt_ok = (b != a);
}

bool bsp_tick_selftest(void)
{
    return s_dwt_ok;
}

uint32_t bsp_tick_ms(void)
{
    return HAL_GetTick();
}

uint32_t bsp_cycles(void)
{
    return RLCA_DWT_CYCCNT;
}

uint32_t bsp_tick_us(void)
{
    if (!s_dwt_ok) {
        /* DWT 不可用时的退化方案：1 ms 分辨率。
         * 这会让舵机总线的微秒级超时变粗，所以启动自检必须报出来。 */
        return HAL_GetTick() * 1000u;
    }
    return RLCA_DWT_CYCCNT / RLCA_CYCLES_PER_US;
}

uint32_t bsp_ms_since(uint32_t since_ms)
{
    return HAL_GetTick() - since_ms;          /* 无符号减法，回绕安全 */
}

uint32_t bsp_us_since(uint32_t since_us)
{
    if (!s_dwt_ok) {
        return (HAL_GetTick() * 1000u) - since_us;
    }
    return (RLCA_DWT_CYCCNT / RLCA_CYCLES_PER_US) - since_us;
}

void bsp_delay_us(uint32_t us)
{
    if (!s_dwt_ok) {
        /* DWT 不可用：用 HAL 的毫秒延时（粗，但不会卡死） */
        HAL_Delay((us + 999u) / 1000u);
        return;
    }

    const uint32_t start = RLCA_DWT_CYCCNT;
    const uint32_t need  = us * RLCA_CYCLES_PER_US;
    while ((RLCA_DWT_CYCCNT - start) < need) {
        /* 忙等 */
    }
}
