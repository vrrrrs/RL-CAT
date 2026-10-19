/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   应用层入口：上电自检 + 100 Hz 主循环调度。
  *
  *  分层约定（固件设计书 §4.1）：
  *    BSP     —— 只碰 HAL/寄存器，无业务逻辑
  *    Service —— 一个模块一个外设/概念，提供 _init()/_poll()/_selftest()
  *    App     —— 只做调度、状态机、决策
  *
  *  ⚠️ 100 Hz 周期由 HAL_GetTick() 驱动（不使用 TIM2），
  *     因为 SysTick 已经是 1 ms 中断，再开一个定时器没有收益。
  ******************************************************************************
  */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>
#include <stdbool.h>

#include "rlca_math.h"   /* rlca_calib_t */

/** 上电自检结果 */
typedef struct {
    bool dwt_ok;        /* DWT 周期计数器可用（微秒计时）  */
    bool i2c_bus_ok;    /* SCL/SDA 释放后能被上拉到高      */
    bool clock_ok;      /* 系统时钟 = 72 MHz               */
    bool imu_ok;        /* IMU 在读（上电瞬间通常还是 false）*/
    uint8_t imu_chip;   /* 读到的 IMU 器件 ID（诊断用）     */
} app_boot_result_t;

/** 初始化全部服务层。必须在所有 MX_*_Init() 之后调用。 */
void app_init(void);

/** 上电自检：返回各项结果。不做阻塞等待，只做即时判定。 */
app_boot_result_t app_boot_selftest(void);

/** 高频任务：主循环每圈都调用（推进接收状态机、shell）。 */
void app_poll(void);

/** 100 Hz 控制环周期任务。 */
void app_cycle_100hz(void);

/** 低频任务：心跳灯、状态灯、统计汇总。主循环每圈调用即可。 */
void app_housekeeping(void);

/** 100 Hz 节拍判定：返回 true = 本圈应执行一次 app_cycle_100hz()。
 *  ⚠️ 用"累加 10 ms"而不是"对齐到当前时刻"，避免单次超时把节拍永久推后；
 *     但落后超过一个周期时会对齐，避免追赶式连发。 */
bool app_should_run_cycle(void);

/** 当前 100 Hz 周期计数（遥测里的 cycle_cnt）。 */
uint32_t app_cycle_count(void);

/** 上一个周期的 CPU 耗时（微秒），用于抖动统计。 */
uint32_t app_last_cycle_us(void);

/** 周期耗时统计（P4 判振动的依据） */

/** ⭐ 主循环各步骤的下标（用于 app_cycle_stats_t.step_max_us） */
enum {
    APP_STEP_UART_DMA = 0,   /* bsp_uart_poll_dma  */
    APP_STEP_SERVO_BUS,      /* servo_bus_poll     */
    APP_STEP_CALIB,          /* calib_proc_poll    */
    APP_STEP_LINK_RX,        /* link_pi_poll       */
    APP_STEP_SHELL,          /* bsp_shell_poll     */
    APP_STEP_LED,            /* bsp_led_poll       */
    APP_STEP_HOUSEKEEP,      /* app_housekeeping   */
    APP_STEP_CYCLE,          /* app_cycle_100hz    */
    APP_STEP_N
};

/** 步骤名（给 shell 的 t 命令用）。越界返回 "?" */
const char *app_step_name(uint8_t idx);

/** 记录某一步的耗时（us）。t0_us 是调用前的 bsp_tick_us()。 */
void app_step_record(uint8_t idx, uint32_t t0_us);

/** ⭐ 清零全部计时统计（shell 的 X 命令）。
 *  ⚠️ 用法：先 X 清零 → 跑运动 → 之后（不必立刻）敲 t。 */
void app_stats_clear(void);

typedef struct {
    uint32_t last_us;
    uint32_t max_us;
    uint32_t sum_us;
    uint32_t n;
    uint32_t overruns;      /* 周期耗时 > 10 ms 的次数（累计，不清零） */
    uint32_t healthy_run;   /* 连续达标周期数（用于超时指示自愈）      */

    /** ⭐⭐ 主循环里相邻两次 app_poll() 的最大间隔（us）。
     *
     *  ⚠️ 为什么需要它（2026-10-17 加）：
     *     现场现象：带力矩运动时 `LINK_OK` 反复熄灭（一次 40~150 ms），
     *     导致固件在 ACTIVE↔STAND 之间切，关节来回摆。
     *     而 `max_us`（周期内部耗时）只有 18 ms，解释不了 40~150 ms。
     *
     *  ⭐ 关键：`link_pi_poll()` 只在 `app_poll()` 里被调用。
     *     主循环是 `app_poll(); [周期]; housekeeping();`
     *     ⇒ 若【周期】阻塞 50 ms，`app_poll()` 就 50 ms 不跑
     *       ⇒ 接收解析被饿死 ⇒ `s_last_rx_ms` 冻结 ⇒ LINK_OK 熄灭。
     *
     *  ⇒ 这个值 > RLCA_LINK_LOST_MS 就说明"接收被饿死"是确凿的，
     *     而且上限值 ≈ 周期耗时，能直接对上。 */
    uint32_t poll_gap_max_us;

    /** ⭐⭐ 主循环【每一步】的最大耗时（us），下标见 APP_STEP_*。
     *
     *  ⚠️ 为什么需要（2026-10-17 加）：实测 `poll_gap_max_us = 173 ms`
     *     而 `max_us`（周期内部）只有 15 ms ⇒ 阻塞不在周期里，
     *     而在 `app_poll()` 的其余步骤或 `app_housekeeping()` 里。
     *     没有分步计时就只能继续猜。
     *
     *  ⇒ `t` 命令会打印每一步的最大值，并标出最慢的那一步。 */
    uint32_t step_max_us[APP_STEP_N];
} app_cycle_stats_t;

const app_cycle_stats_t *app_cycle_stats(void);

/*==============================================================================
 * 标定流程的外部入口（供 shell 与 link_pi 调用）
 *============================================================================*/

/** 启动全自动标定：卸力矩 → 等沉稳 → 采样 → 逐关节测方向 → 落盘。
 *  @return false = 舵机总线正忙，稍后重试 */
bool app_calib_start(void);

/** 操作者把机器人摆到装配站姿后调用：立即采样当前位置作为零点。
 *  ⚠️ 只定 offset，不定 dir（默认全 +1；方向需靠全自动流程测）。 */
void app_calib_manual_zero(void);

/** 取消标定。 */
void app_calib_cancel(void);

/** 取走标定结果（成功后）。
 *  @return true = 拿到了结果（并已置"待落盘"标志） */
bool app_calib_take_result(rlca_calib_t *out);

/** 标定是否正在跑。 */
bool app_calib_busy(void);

/** 标定进度 0~100。 */
uint8_t app_calib_progress(void);

/** 标定是否失败。 */
bool app_calib_failed(void);

/** 最近一次标定的失败原因码（见 rlca_calib_proc.h 的 calib_err_t）。 */
uint8_t app_calib_err(void);

/** 删除 Flash 里的标定并回到默认值。 */
bool app_calib_erase(void);

/** 复述当前生效的标定表（只读）。 */
const rlca_calib_t *app_calib_table(void);

/** ⭐⭐ 上电安全兜底的诊断（见 app_main.c 的 ①b）。
 *  在闸门期内把 GOAL_POSITION 镜像成实测位置，绕开标定与角度换算。
 *
 *  @param seeded  输出：是否至少成功镜像过一次
 *                 （false = 闸门期内一个有效反馈都没读到）
 *  @param writes  输出：累计写出的单舵机帧数（闸门期内应持续增长）
 *
 *  ⚠️ 上机判据：若 `seeded == false`，说明"镜像"根本没发生
 *     （读不到反馈）—— 此时舵机若仍动，问题在总线读取，不在播种。
 */
void app_boot_goal_seed_stat(bool *seeded, uint32_t *writes);

/** ⭐⭐ 持续重申泄力的诊断（见 app_main.c 的 ⑥b）。
 *  只要期望力矩为关，就周期性重申 torque-off（不再依赖上电窗口假设）。
 *
 *  @param count       输出：累计重申的帧组数（每周期一组 12 帧）
 *  @param burst_left  输出：密集重申还剩多少周期
 */
void app_torque_reassert_stat(uint32_t *count, uint32_t *burst_left);

/** ⭐⭐⭐ 位置写许可的诊断（见 app_main.c 的 ⑦）。
 *
 *  ⭐ 唯一规则：未标定 ⇒ 关节角度没有意义 ⇒ 一条位置指令都不发 + 持续卸力矩。
 *     已标定 ⇒ 才允许写位置。
 *
 *  @param boot_calib_present 输出：上电时 Flash 里有没有合法标定表
 *  @param pos_write_allowed  输出：当前是否允许写位置
 *  @param blocked_count      输出：累计被"未标定"挡住的周期数
 *  @param q_seeded           输出：是否已用反馈播种过关节状态
 */
void app_pos_write_gate_stat(bool *boot_calib_present, bool *pos_write_allowed,
                             uint32_t *blocked_count, bool *q_seeded);

/** ⭐⭐⭐ 是否已"解除上锁"（收到过带 ENABLE 的明确指令）。
 *
 *  ⚠️ 项目负责人要求：没有明确指令之前，不准上力矩、不准写位置。
 *     本函数返回 true 表示已经收到过那种明确指令，允许接管。
 */
bool app_is_armed(void);

/** ⭐ 实际下发的力矩状态（经过 s_armed 锁与安全层之后的最终值）。
 *  ⚠️ 与 `app_mode_state()->torque_on`（模式状态机的期望值）可能不同：
 *     相同 = 一切正常；不同 = 被 s_armed 锁挡了。
 *     诊断用（shell 的 R 命令）。 */
bool app_torque_actual(void);

/** ⭐⭐ 操作者手动力矩覆盖（供 shell 的 T 命令与 W 命令调试用）。
 *
 *  ⚠️ 打开覆盖会暂停主循环里"周期性重申卸力"（⑥b）。
 *     因为那一步内部会 bus_abort_now()，会把刚发出的位置帧掐断 ——
 *     表现为"W 要反复连发才能走到目标"。
 *
 *  @param on  true = 打开（并立刻使能力矩）；false = 关闭（并立刻卸力）
 */
void app_set_manual_torque(bool on);

/** 当前是否处于手动力矩覆盖。 */
bool app_get_manual_torque(void);

/** 由链路指令路径调用：收到明确指令时【自动解除】手动覆盖，回到正常管控。 */
void app_clear_manual_torque(void);

#endif /* APP_MAIN_H */
