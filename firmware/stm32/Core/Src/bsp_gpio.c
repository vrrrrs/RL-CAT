/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   LED / 心跳 / 错误指示实现。
  ******************************************************************************
  */

#include "bsp_gpio.h"
#include "rlcat_config.h"
#include "bsp_tick.h"

/*==============================================================================
 * 内部状态
 *============================================================================*/

static bsp_led_mode_t s_led_mode = BSP_LED_OFF;
static uint32_t       s_led_t0   = 0u;      /* 模式起始时刻（用于图案相位） */
static bool           s_led_level = false;

static uint32_t s_hb_last = 0u;
static bool     s_hb_level = false;

/* 闪烁参数（毫秒） */
#define LED_SLOW_HALF_MS     500u      /* 1 Hz：500 ms 亮 / 500 ms 灭 */
#define LED_FAST_HALF_MS     100u      /* 5 Hz */
#define LED_ERROR_HALF_MS     50u      /* 10 Hz */
#define LED_ESTOP_PERIOD_MS  600u      /* 双闪停的图案周期 */
#define HEARTBEAT_HALF_MS    500u      /* 心跳 1 Hz */

/*==============================================================================
 * ⚠️⚠️ LED 极性（踩过坑，别想当然）
 *
 *   STM32 Blue Pill 的板载 LED 在【PC13】，且是【低电平点亮】：
 *       引脚拉低 → LED 亮
 *       引脚拉高 → LED 灭
 *   （PC13 接 LED 阳极到 3.3V、阴极到引脚，所以是灌电流驱动。）
 *
 *   ⚠️ 早期版本把 LED 配在 PB0 上且按【高电平点亮】写，
 *      结果板载灯永远不亮 —— 所有状态指示（含"启动失败"提示）全部静默，
 *      而 PB0 在真实的 Blue Pill 上根本没接 LED。
 *      详见 docs/39-firmware-handover.md。
 *
 *   ⚠️ 另一处容易错的地方：PC13 属于【后备域】，
 *      驱动能力只有约 3 mA，且不适合快速翻转。
 *      这里只做 1~10 Hz 的状态闪烁，完全没问题；
 *      但【不要】拿它当数据线或高频 PWM 用。
 *============================================================================*/

#if RLCA_LED_ACTIVE_LOW
#define LED_ON_LEVEL      GPIO_PIN_RESET
#define LED_OFF_LEVEL     GPIO_PIN_SET
#else
#define LED_ON_LEVEL      GPIO_PIN_SET
#define LED_OFF_LEVEL     GPIO_PIN_RESET
#endif

/** 按【逻辑亮灭】驱动 LED。
 *  ⚠️ 全模块只允许通过这个函数碰 LED 引脚 ——
 *     这样极性只在一处定义，换板子只改一个宏。 */
static void led_write(bool on)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/*==============================================================================
 * 初始化与强制控制
 *============================================================================*/

void bsp_gpio_init(void)
{
    /* GPIO 时钟与引脚模式已由 CubeMX 的 MX_GPIO_Init() 配好：
     *   PC13 推挽输出（板载 LED，低电平点亮）、PA8 推挽输出（心跳）
     * 这里只保证初态：心跳灭、状态灯灭。 */
    HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);
    led_write(false);          /* ⚠️ 走 led_write，自动处理低电平点亮 */

    s_led_mode  = BSP_LED_OFF;
    s_led_level = false;
    s_hb_level  = false;
    s_hb_last   = bsp_tick_ms();
    s_led_t0    = s_hb_last;
}

void bsp_led_force(bool on)
{
    s_led_level = on;
    led_write(on);             /* ⚠️ 极性在 led_write 里统一处理 */
}

void bsp_led_set(bsp_led_mode_t mode)
{
    if (mode != s_led_mode) {
        s_led_mode = mode;
        s_led_t0   = bsp_tick_ms();
        if ((mode == BSP_LED_OFF) || (mode == BSP_LED_ON)) {
            bsp_led_force(mode == BSP_LED_ON);
        }
    }
}

bsp_led_mode_t bsp_led_mode(void)
{
    return s_led_mode;
}

/*==============================================================================
 * 轮询
 *============================================================================*/

void bsp_led_poll(void)
{
    const uint32_t now = bsp_tick_ms();
    const uint32_t dt  = now - s_led_t0;

    switch (s_led_mode) {
    case BSP_LED_OFF:
    case BSP_LED_ON:
        return;                                  /* 已由 bsp_led_set 设定 */

    case BSP_LED_SLOW_BLINK:
        bsp_led_force(((dt / LED_SLOW_HALF_MS) & 1u) == 0u);
        return;

    case BSP_LED_FAST_BLINK:
        bsp_led_force(((dt / LED_FAST_HALF_MS) & 1u) == 0u);
        return;

    case BSP_LED_PATTERN_ERROR:
        bsp_led_force(((dt / LED_ERROR_HALF_MS) & 1u) == 0u);
        return;

    case BSP_LED_PATTERN_ESTOP:
    {
        /* 双闪停：一个周期 = 200 亮 / 200 灭 / 200 亮 / 600 灭 */
        const uint32_t t = dt % (2u * LED_ESTOP_PERIOD_MS);
        const bool on = (t < 200u) || ((t >= 400u) && (t < 600u));
        bsp_led_force(on);
        return;
    }

    default:
        return;
    }
}

void bsp_heartbeat_poll(void)
{
    const uint32_t now = bsp_tick_ms();
    if ((now - s_hb_last) >= HEARTBEAT_HALF_MS) {
        s_hb_last = now;
        s_hb_level = !s_hb_level;
        HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin,
                          s_hb_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/*==============================================================================
 * 致命错误
 *============================================================================*/

void bsp_fatal_error(void)
{
    /* ⚠️ 先关中断，再进死循环手动翻转 —— 不依赖 SysTick 中断
     *    （中断可能已被关闭，或时钟配置失败导致 SysTick 未起）。 */
    __disable_irq();

    /* ⚠️⚠️ 图案刻意做成【双闪停】，而不是普通快闪。理由：
     *      正常运行时的慢闪(1 Hz) / 快闪(5 Hz) 都是【均匀】的，
     *      如果致命错误也用均匀快闪，肉眼分不清"进 Error_Handler"和
     *      "自检发现时钟异常"。双闪停是独一无二的签名，一眼可辨。
     *
     * ⚠️ 每次翻转之间【必须有延时】：早期版本两行 WritePin 背靠背、
     *    中间没有延时，等于以 CPU 主频翻转，肉眼看到常亮 ——
     *    这个诊断信息的全部价值就丢了。
     *
     * ⚠️ 延时不能用 bsp_tick_ms()：时钟可能已经配置失败，而本函数的
     *    职责恰恰是"连时钟都坏了也要能报错"。所以用纯软件循环。
     *    72 MHz 下 50000 次空转约 5 ms（含循环开销），
     *    下面 4 段延时约 20 ms + 停顿 160 ms → 约 5 Hz 的双闪停。 */
    const uint32_t tick = 50000u;

    for (;;) {
        /* 闪 1 */
        led_write(true);
        for (volatile uint32_t i = 0u; i < tick; ++i) { }
        led_write(false);
        for (volatile uint32_t i = 0u; i < tick; ++i) { }
        /* 闪 2 */
        led_write(true);
        for (volatile uint32_t i = 0u; i < tick; ++i) { }
        led_write(false);
        /* 长停顿（4 倍），形成"双闪—停"的节奏 */
        for (volatile uint32_t i = 0u; i < (tick * 8u); ++i) { }
    }
}
