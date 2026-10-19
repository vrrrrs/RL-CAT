/**
  ******************************************************************************
  * @file    app_safety.c
  * @brief   安全保护的执行层实现。
  ******************************************************************************
  */

#include <math.h>
#include <string.h>

#include "app_safety.h"
#include "rlca_safety.h"
#include "imu.h"
#include "battery.h"

/*==============================================================================
 * 内部状态
 *============================================================================*/

static app_safety_state_t s_st;
static rlca_safety_out_t   s_out;

/** ⭐⭐ IMU 失效的"调试旁路"开关（见 app_safety_update 的详细说明）。
 *  ⚠️ 只由 shell 的 `J` 命令显式打开，默认关闭。 */
static bool s_imu_bypass = false;

void app_safety_set_imu_bypass(bool on)
{
    s_imu_bypass = on;
}

bool app_safety_get_imu_bypass(void)
{
    return s_imu_bypass;
}

/* 趴下目标角（第一次进入趴下时算好，之后不变）
 * ⚠️ 只在 RLCA_SAFE_ESTOP_SITDOWN == 1 时使用；
 *    默认（== 0，急停只卸力矩）下用不到，否则编译器报 unused。 */
#if (RLCA_SAFE_ESTOP_SITDOWN != 0)
static float s_sitdown[RLCA_N_JOINTS];
static bool  s_sitdown_ready = false;
#endif

/*==============================================================================
 * 趴下姿态
 *============================================================================*/

void app_safety_compute_sitdown(const rlca_calib_t *calib, float *out)
{
    if (out == NULL) {
        return;
    }

    if (calib == NULL) {
        rlca_calib_t def;
        rlca_calib_load_defaults(&def);
        calib = &def;
    }

    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        float qmin;
        float qmax;
        rlca_q_range(calib, i, &qmin, &qmax);

        /* 趴下姿态的选择（按关节类型不同）：
         *   hip  （外摆）：回 0 —— 腿收直，不往外撑
         *   thigh（髋俯仰）：往【下限】靠 —— 大腿向前折
         *   calf （膝）  ：往【上限】靠 —— 小腿向后折
         * ⚠️ 三者组合起来是"把腿折起来、机身降低"的姿态。
         *    刻意都往限位内侧留 10%，避免顶死在机械限位上
         *    （顶死会让舵机持续堵转 → 过流保护跳闸）。 */
        float target;
        if (i <= (uint8_t)RLCA_J_RB_HIP) {
            target = qmin + (qmax - qmin) * 0.10f;      /* 靠下限但不到底 */
        } else if (i <= (uint8_t)RLCA_J_RB_THIGH) {
            target = qmin + (qmax - qmin) * 0.10f;      /* 靠下限 */
        } else {
            target = qmin + (qmax - qmin) * 0.90f;      /* 靠上限 */
        }
        out[i] = target;
    }

    /* calf 靠上限意味着"膝盖弯到底"；与 thigh 靠下限配合就是蹲姿。
     * ⚠️ 这只是一个【保守的降级姿态】，不追求好看 ——
     *    目标是"用最小的力矩把机身放到地面上"。 */
}

/*==============================================================================
 * 复位
 *============================================================================*/

void app_safety_reset(void)
{
    memset(&s_st, 0, sizeof(s_st));
    memset(&s_out, 0, sizeof(s_out));
#if (RLCA_SAFE_ESTOP_SITDOWN != 0)
    s_sitdown_ready = false;
#endif
    s_out.amplitude_scale = 1.0f;
}

/*==============================================================================
 * 主更新
 *============================================================================*/

/* ⭐ 快照（类型定义在 app_safety.h，便于 shell 直接读字段） */
static app_estop_snapshot_t s_last_estop;

/*==============================================================================
 * ⭐⭐⭐ 舵机温度读数过滤（2026-10-18 修的真 bug）
 *============================================================================*/

/** 读数合理性上限。Feetech STS 系列手册报 0~100℃，留余量取 125。
 *  ⚠️ 超过它一律视为【坏读数】，不进判据。 */
#define TEMP_PLAUSIBLE_MAX   125u

/** 需要连续多少次越阈才认。100 Hz 下 5 次 = 50 ms。
 *  ⚠️ 为什么可以这么慢：热的惯性以【秒】计，50 ms 的延迟对真实过温毫无影响；
 *     但一次坏读数的持续时间往往只有几个周期，50 ms 的门槛正好把它挡掉。 */
#define TEMP_CONFIRM_N       5u

static uint8_t s_temp_over_cnt = 0u;     /* 连续越阈计数 */

/** 把原始温度读数变成"可以进判据的值"。
 *
 *  ⚠️⚠️ 这个函数存在的原因（现场实测）：
 *      M1 站立测试里 6% 的周期被 ESTOP 卸力矩，分原因计数显示
 *      **7/9 次是"过温"**，而当时 12 个舵机实测只有 28~32℃、阈值是 70℃。
 *      ⇒ 说明 `servo_bus_max_temp()` 在某些周期返回了 >70 的**坏值**。
 *        那一刻 link_level = 0、电压 11.8 V、servo_err = 0，
 *        唯一可能的触发源就是它。
 *
 *  ⚠️ 为什么不干脆把阈值提高：
 *      提阈值会让【真正的过温】也失去保护。正确做法是
 *      ① 把明显不可能的读数当坏数据丢掉（>125℃）
 *      ② 要求连续 N 次确认（坏读数很少能连续 5 次都坏）
 *      ⇒ 真过温照旧会停（延迟 50 ms），假尖峰被挡在门外。
 */
static uint8_t filter_temp(uint8_t raw)
{
    if (raw > (uint8_t)TEMP_PLAUSIBLE_MAX) {
        s_temp_over_cnt = 0u;
        return 0u;                       /* 坏读数：当作"无数据" */
    }

    if (raw > (uint8_t)RLCA_SERVO_OVERTEMP_C) {
        if (s_temp_over_cnt < 255u) {
            s_temp_over_cnt++;
        }
    } else {
        s_temp_over_cnt = 0u;
    }

    return (s_temp_over_cnt >= (uint8_t)TEMP_CONFIRM_N) ? raw : 0u;
}

/** ⭐ 急停分原因计数（见快照块里的说明）。 */
static struct {
    uint32_t link;
    uint32_t volt;
    uint32_t temp;
    uint32_t oper;
    uint32_t unknown;               /* ⭐ 非 0 ⇒ 判据不止 4 条 */
    uint32_t by_link_level[3];      /* level 0/1/2 各出现几次 */
} s_estop_cnt;

uint32_t app_safety_estop_count_cause(uint8_t idx)
{
    switch (idx) {
    case 0u: return s_estop_cnt.link;
    case 1u: return s_estop_cnt.volt;
    case 2u: return s_estop_cnt.temp;
    case 3u: return s_estop_cnt.oper;
    case 4u: return s_estop_cnt.unknown;
    default: return 0u;
    }
}

uint32_t app_safety_estop_count_level(uint8_t lvl)
{
    return (lvl < 3u) ? s_estop_cnt.by_link_level[lvl] : 0u;
}

const app_estop_snapshot_t *app_safety_last_estop(void)
{
    return &s_last_estop;
}

void app_safety_update(uint32_t now_ms,
                       uint8_t link_level,
                       const rlca_calib_t *calib,
                       float *q_ref,
                       uint8_t servo_temp_max,
                       uint8_t servo_err)
{
    if (q_ref == NULL) {
        return;
    }

    /* ---- ① 收集信号 ---- */
    rlca_safety_in_t in;
    memset(&in, 0, sizeof(in));

    in.link_level    = link_level;
    in.cmd_age_ms    = 0u;                       /* link_level 已经蕴含了时限 */
    /* 电池：直接读 battery 模块，避免在接口上多传两个参数 */
    in.batt_mv       = battery_mv();
    in.batt_valid    = battery_is_valid();
    in.max_temp_c    = filter_temp(servo_temp_max);   /* ⭐ 过滤坏读数 + 迟滞 */
    in.servo_err_flags = servo_err;
    in.imu_ok        = imu_is_ok();
    in.operator_estop = false;                   /* 硬急停本次不实现 */

    /* ⭐⭐ IMU 失效的"调试旁路"（2026-10-16 加）
     *
     *  ⚠️ 为什么需要它：
     *     ACTIVE 准入的第 ④ 道门是 imu_ok。IMU 失效时 ACTIVE 会被【静默降级
     *     成 STAND】，而 STAND 分支【忽略 pi_q_target】、只保持 q_def = 0
     *     ⇒ 于是"用 Pi 发角度调步态"这条通路被彻底堵死。
     *
     *  ⚠️ 但它不该堵死【脚本化步态调试】：
     *     IMU 存在的意义是"策略的 47 维观测里有姿态"。而按脚本直接下发
     *     关节角（pi/pose.py、手写步态）时，观测根本不被使用 ——
     *     这时 IMU 失效不影响动作的正确性，只影响"策略推理"这一条路。
     *
     *  ⚠️ 实现方式：不碰 in.imu_ok（那会同时抹掉 RLCA_SAFE_R_IMU_STALE 位，
     *     让"IMU 是坏的"这件事在 K/R 里看不见 —— 不可接受）。
     *     而是【只放开 ACTIVE 准入】这一道门，见下方 block_active 的处理。
     *
     *  ⚠️ 安全边界：
     *     · 只允许"人显式打开"（shell 的 `J` 命令），不做任何自动判定
     *     · RLCA_SAFE_R_IMU_STALE 照旧置位 ⇒ K/R 一眼看出 ACTIVE 是靠旁路进来的
     *     · 不影响急停/降级/趴下等任何其他安全动作
     *     · 断链失联时的行为不变
     *
     *  ⇒ 跑【策略】之前必须关掉它；跑【脚本步态】时可以开着。
     *
     *  ⚠️ 实现落在 app_safety_blocks_active()（ACTIVE 门第 ② 条）与
     *     app_main.c 的 active_gate 第 ④ 条 —— 两处都要放行才真的生效。 */

    /* ---- ② 纯逻辑评估 ---- */
    const uint8_t prev_sev = s_out.severity;
    rlca_safety_eval(&in, &s_out);

    /* ⭐⭐⭐ 急停现场快照（2026-10-18 加）
     *
     *  ⚠️ 起因：现场出现"约 6~16% 的周期进 ESTOP"，但所有可观测数据
     *     都和它对不上 ——
     *       · rx_bytes 全收到、crc/len/ovf/restarts 全 0  ⇒ 不是接收问题
     *       · poll_gap_max_us = 20 ms                      ⇒ 不是主循环阻塞
     *       · Pi 发送间隔 p50 10.8 ms、零失败              ⇒ 不是上位机
     *       · 电压 11.4 V、舵机 37℃                        ⇒ 电压/温度都正常
     *     而 ESTOP 只有 4 个来源（操作者/链路/电压/过温），
     *     靠"事后读 R"根本抓不到那个瞬态。
     *
     *  ⇒ 干脆在【进入急停的那一个周期】把全部输入抄下来。
     *     跑完敲 R 就能看到 last_estop_* 那一组值 —— 直接定罪，不用再推。 */
    if ((s_out.severity >= 3u) && (prev_sev < 3u)) {
        s_last_estop.reason      = s_out.reasons;
        s_last_estop.link_level  = in.link_level;
        s_last_estop.batt_mv     = in.batt_mv;
        s_last_estop.batt_valid  = in.batt_valid ? 1u : 0u;
        s_last_estop.temp_c      = in.max_temp_c;
        s_last_estop.temp_raw    = servo_temp_max;   /* ⭐ 原始值：>125 即坏读数 */
        s_last_estop.servo_err   = in.servo_err_flags;
        s_last_estop.imu_ok      = in.imu_ok ? 1u : 0u;
        s_last_estop.count++;

        /* ⭐⭐ 分原因计数（2026-10-18 加）
         *
         *  ⚠️ 为什么必须分原因：单个"最近一次"快照会被【最后一次】覆盖，
         *     而最后一次往往是"工具退出后的长时间空档"（实测 6313 ms），
         *     于是前面真正的问题原因就看不见了 —— 这次就是这样：
         *     last_estop_reason=8(LINK_LOST) 但 link_over_estop 只有 1 次，
         *     说明前面 11 次并不是链路，却被同一条快照掩盖。
         *
         *  ⭐ `unknown` 是最关键的一项：如果它不为 0，就证明
         *     `rlca_safety_eval` 里还有【我读代码时漏掉的第五个来源】。 */
        if (in.link_level >= 2u) {
            s_estop_cnt.link++;
        } else if (in.batt_valid && (in.batt_mv <= RLCA_BATT_FORCE_MV)) {
            s_estop_cnt.volt++;
        } else if (in.max_temp_c > (uint8_t)RLCA_SERVO_OVERTEMP_C) {
            s_estop_cnt.temp++;
        } else if (in.operator_estop) {
            s_estop_cnt.oper++;
        } else {
            s_estop_cnt.unknown++;      /* ⭐ 出现即说明判据不止 4 条 */
        }

        /* 同时记 link_level 的分布（区分"降级"与"急停"） */
        if (in.link_level < 3u) {
            s_estop_cnt.by_link_level[in.link_level]++;
        }
    }

    /* ---- ③ 更新状态 ---- */
    s_st.reason      = s_out.reasons;
    s_st.severity    = s_out.severity;
    s_st.max_temp_c  = servo_temp_max;

    /* ---- ④ 执行：缓慢趴下 ---- */
    /* ⚠️⚠️ 未标定的短路（2026-10-12 安全修复）
     *
     *  趴下目标角依赖标定表（把关节角限位换算成目标角）。未标定时标定表是
     *  rlca_calib_load_defaults() 的全零虚表，算出来的目标角【本身就是错的】——
     *  推进这个斜坡等于把 12 个关节主动推向一个错误位置。
     *  所以未标定时绝不推进斜坡，直接把力矩卸掉：
     *      · 卸力矩后舵机是自由的，机器人靠自重自然落地（不会砸，因为已在低处）
     *      · 也避免了"未标定 → 上电即错误运动"
     *
     *  ⚠️ 这里只短路【执行】，不动 rlca_safety_eval 的判据 ——
     *     判据层的 sit_down/severity/block_active 语义已被单测覆盖
     *     （firmware/tests/host/test_safety_calib.c），不应因未标定而改变。
     */
    if ((calib == NULL) || !rlca_calib_is_valid(calib)) {
        s_st.active     = false;
        s_st.done       = false;
        s_st.torque_off = true;
        return;
    }

    if (s_out.sit_down) {
#if (RLCA_SAFE_ESTOP_SITDOWN == 0)
        /* ⭐⭐⭐ 急停 = 【只卸力矩】，不做主动趴下斜坡（2026-10-17 项目负责人决定）
         *
         *  ⚠️ 理由：
         *     · 舵机断电后自身有阻尼 + 减速比，机器人会**缓慢塌下**，不会砸。
         *     · 旧行为（用剩余力矩把 12 个关节主动折向趴下姿态）是
         *       **一条没人要求的运动**：链路一抖就折腿，调试时非常危险。
         *       现场表现就是"按一下 shell 的 R/K 机器人就抖一下"。
         *     · 急停的语义应该是"停止输出"，而不是"换个姿态继续输出"。
         *
         *  ⚠️ 所以这里：
         *     · 立刻 torque_off = true（让 app_mode 把力矩关掉）
         *     · **不动 q_ref**（卸力后位置由重力与机械决定；
         *       若强行把 q_ref 拖到某个值，力矩恢复时会跳变）
         *     · done = true（没有斜坡，无"进行中"状态）
         *
         *  ⚠️ 想恢复旧行为：把 RLCA_SAFE_ESTOP_SITDOWN 改成 1。 */
        s_st.active     = true;      /* 仍表示"处于急停降级中" */
        s_st.done       = true;      /* 没有斜坡，直接视为完成 */
        s_st.torque_off = true;      /* ⭐ 立即卸力 */
        (void)calib;                 /* 本分支不需要标定表（不算趴下目标） */
#else
        if (!s_st.active) {
            /* 第一次进入：算好趴下目标，记录起点时间 */
            s_st.active = true;
            s_st.done   = false;
            s_st.t0_ms  = now_ms;
            if (!s_sitdown_ready) {
                app_safety_compute_sitdown(calib, s_sitdown);
                s_sitdown_ready = true;
            }
        }

        /* 斜坡推进：按 ramp_rate 每毫秒走一点，逐关节朝趴下目标靠 */
        const float dt_s = (float)RLCA_CTRL_PERIOD_MS * 0.001f;
        const float max_step = s_out.ramp_rate_rad_s * dt_s;

        bool all_done = true;
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            float d = s_sitdown[i] - q_ref[i];
            if (d > max_step)  { d =  max_step; }
            if (d < -max_step) { d = -max_step; }
            q_ref[i] += d;

            if (fabsf(s_sitdown[i] - q_ref[i]) > RLCA_SAFE_SITDOWN_TOL_RAD) {
                all_done = false;
            }
        }
        s_st.done = all_done;

        /* ⚠️ 独立兜底：斜坡最长允许 RLCA_SAFE_SITDOWN_TIMEOUT_MS。
         *    到点还没到位（舵机没力矩/卡死），也必须卸力矩 ——
         *    绝不允许"因为趴不下去就一直硬撑着使能"。 */
        const uint32_t elapsed = now_ms - s_st.t0_ms;
        if (all_done || (elapsed >= RLCA_SAFE_SITDOWN_TIMEOUT_MS)) {
            s_st.torque_off = true;
        }
#endif
    } else {
        /* 非急停：清掉趴下状态，允许恢复 */
        if (s_st.active && !s_st.torque_off) {
            /* 曾经趴下但又解除（例如链路恢复）：保留已经走到的位置，
             * 由上层模式状态机决定下一步（一般会回 STAND）。 */
            s_st.active = false;
            s_st.done   = false;
        }
        if (s_st.torque_off) {
            s_st.torque_off = false;
        }
        s_st.active = false;
        s_st.done   = false;
    }

    /* ---- ⑤ 限制动作幅度 ---- */
    if (s_out.limit_amplitude) {
        /* ⚠️ 这里【不】直接改 q_ref —— 那会和"斜坡推进"打架。
         *    幅度限制的作用点在上层：把 q_target 相对 q_default 缩放。
         *    本模块只提供 scale，具体应用见 app_mode。 */
    }
}

/*==============================================================================
 * 查询
 *============================================================================*/

bool app_safety_in_estop(void)
{
    return (s_out.severity >= 3u);
}

bool app_safety_blocks_active(void)
{
    /* ⭐⭐ IMU 调试旁路（见 app_safety_update 里的详细说明）
     *
     *  ⚠️ 只在"IMU 是唯一的阻塞原因"时才放行 —— 即 reasons 里除了
     *     RLCA_SAFE_R_IMU_STALE 之外没有别的位。
     *     ⇒ 舵机报错、过温、电压不足等任何其他原因都【照旧挡住】。
     *
     *  ⚠️ 旁路只改这里的返回值（= ACTIVE 准入的第 ② 道门），
     *     s_out.reasons 完全不变 ⇒ K/R 里 RLCA_SAFE_R_IMU_STALE 照旧可见。
     *
     *  ⚠️ 注意：这正是 ACTIVE 门第 ④ 条（!imu_ok）的上游 ——
     *     放行这里之后，第 ④ 条仍然会挡住 ACTIVE。
     *     所以旁路必须在 app_mode_active_gate 里一并放行，见 app_main.c。 */
    if (s_imu_bypass) {
        const uint8_t others = (uint8_t)(s_out.reasons & (uint8_t)~RLCA_SAFE_R_IMU_STALE);
        if (others == 0u) {
            return false;                     /* 只剩 IMU 一个原因 → 放行 */
        }
    }
    return s_out.block_active;
}

float app_safety_amplitude_scale(void)
{
    return (s_out.amplitude_scale > 0.0f) ? s_out.amplitude_scale : 1.0f;
}

const app_safety_state_t *app_safety_state(void)
{
    return &s_st;
}

uint8_t app_safety_reasons(void)
{
    return s_out.reasons;
}

uint8_t app_safety_severity(void)
{
    return s_out.severity;
}
