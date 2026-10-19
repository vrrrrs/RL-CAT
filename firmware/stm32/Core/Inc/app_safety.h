/**
  ******************************************************************************
  * @file    app_safety.h
  * @brief   安全保护的【执行层】：把 rlca_safety 的判据变成真实动作。
  *
  *  ⭐⭐ 这是本项目最重要的一条设计要求的落地处
  *      （docs/00-overview.md §2.1）：
  *      **安全保护必须在 STM32 上独立完成，不能依赖上位机。**
  *
  *  职责：
  *    · 从各模块收集信号（链路、电池、舵机温度、IMU）
  *    · 交给 rlca_safety_eval() 得到"该做什么"（纯逻辑，已单测）
  *    · 执行：
  *        - 卸力矩（真·紧急）
  *        - ⭐ 缓慢趴下：不是"直接断电"，而是按斜坡把目标角推到趴下姿态
  *        - 限制动作幅度
  *
  *  ⚠️ 为什么必须"缓慢趴下"而不是直接卸力矩/断电：
  *     直接卸力矩机器人会以自重砸下（机身 1.386 kg，站立高 133 mm），
  *     可能摔坏舵机或结构。所以宁可多花 2~3 秒慢慢蹲下去。
  *
  *  ⚠️ 趴下斜坡的独立兜底
  *     ⭐ 关键条件：斜坡 → 到位 → 卸力矩
  *     若斜坡一直不到位（例如舵机没力矩/卡死），
  *     RLCA_SAFE_SITDOWN_TIMEOUT_MS 之后【强制卸力矩】，
  *     绝不允许"因为趴不下去就一直硬撑着使能"。
  *
  *  ⚠️⚠️ 未标定时【不做趴下斜坡】（2026-10-12 安全修复）：
  *     趴下目标角由 app_safety_compute_sitdown(calib, ...) 算出，
  *     它依赖标定表把"关节角限位"换算成目标角。
  *     未标定时标定表是全零虚表 ⇒ 算出的目标角本身就是错的，
  *     斜坡会把 12 个关节推向一个【错误的目标】。
  *     ⇒ 未标定时直接卸力矩，不推进任何斜坡。
  ******************************************************************************
  */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"
#include "rlca_safety.h"

/*==============================================================================
 * 趴下斜坡配置
 *============================================================================*/

/** 斜坡最长允许时间；超过就强制卸力矩（不再等它到位） */
#define RLCA_SAFE_SITDOWN_TIMEOUT_MS   4000u

/** 判定"已经蹲到位"的关节角容差（rad）。约 0.6°，肉眼看不出来 */
#define RLCA_SAFE_SITDOWN_TOL_RAD      0.01f

/** 状态 */
typedef struct {
    bool     active;          /* 是否正在趴下                       */
    bool     done;            /* 是否已到位                         */
    uint32_t t0_ms;           /* 进入趴下的时刻                     */
    uint8_t  reason;          /* 触发原因位域（rlca_safety 的）      */
    uint8_t  severity;        /* 严重程度                           */
    uint16_t max_temp_c;      /* 观测到的最高舵机温度                */
    bool     torque_off;      /* 是否已卸力矩                       */
} app_safety_state_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/** 收集信号并评估。
 *  每周期调用一次（在控制环里），内部会：评估 → 更新状态 → 推进斜坡。
 *  @param now_ms      当前毫秒
 *  @param link_level  link_pi_check_link() 的返回值（0/1/2）
 *  @param calib       标定表（算趴下目标角用）
 *  @param q_ref       输入输出：当前【将要下发】的目标角；趴下时会在此斜坡推进
 *  @param servo_temp_max 12 个舵机温度最大值（°C）
 *  @param servo_err   servo_bus_err_flags()
 */
void app_safety_update(uint32_t now_ms,
                       uint8_t link_level,
                       const rlca_calib_t *calib,
                       float *q_ref,
                       uint8_t servo_temp_max,
                       uint8_t servo_err);

/** 是否处于急停状态（对应 STAT_ESTOP 位）。 */
bool app_safety_in_estop(void);

/** ⭐ 最近一次进入急停时的输入快照（诊断用，2026-10-18）。
 *
 *  ⚠️ 用途：抓住"某个瞬态条件触发急停"的现场 —— 事后读 R 就能看，
 *     不用再靠"Pi 侧数据反推"（那条路已经走到极限了）。
 *
 *  ⚠️ 为什么需要：现场出现过"约 6~16% 的周期进 ESTOP"，但
 *     rx_bytes 全收到、crc/len/ovf/restarts 全 0、poll_gap 只有 20 ms、
 *     Pi 发送零失败、电压 11.4 V、舵机 37℃ —— 所有可观测数据都对不上。
 *     ESTOP 只有 4 个来源，靠事后读 R 抓不到瞬态，所以在这里抄现场。 */
typedef struct {
    uint8_t  reason;        /* rlca_safety 的 reasons 位图（1=LINK_LOST 等） */
    uint8_t  link_level;    /* 0 正常 / 1 降级(>80ms) / 2 急停(>300ms) */
    uint16_t batt_mv;
    uint8_t  batt_valid;
    uint8_t  temp_c;        /* 经过滤后交给判据的值（可能是 0） */
    uint8_t  temp_raw;      /* ⭐ servo_bus_max_temp() 的原始值（定罪用）*/
    uint8_t  servo_err;     /* servo_bus_err_flags() */
    uint8_t  imu_ok;
    uint32_t count;         /* 累计"进入急停"次数 */
} app_estop_snapshot_t;

/** 取最近一次急停快照（指针指向静态存储）。 */
const app_estop_snapshot_t *app_safety_last_estop(void);

/** ⭐ 急停分原因计数（诊断用）。
 *  idx: 0=链路 1=电压 2=过温 3=操作者 4=未知
 *  ⚠️ `4(未知)` 非 0 ⇒ 说明 rlca_safety_eval 的急停判据不止 4 条（代码没读全）。 */
uint32_t app_safety_estop_count_cause(uint8_t idx);

/** ⭐ 进入急停时 link_level 的分布（0/1/2 各几次），用于区分"降级"与"真急停"。 */
uint32_t app_safety_estop_count_level(uint8_t lvl);

/** 是否禁止进入 ACTIVE。 */
bool app_safety_blocks_active(void);

/** ⭐⭐ IMU 失效的"调试旁路"（2026-10-16 加，供 shell 的 `J` 命令使用）。
 *
 *  ⚠️ 为什么需要：ACTIVE 准入的第 ④ 道门是 imu_ok。IMU 失效时 ACTIVE 被
 *     静默降级成 STAND，而 STAND 分支【忽略 pi_q_target】⇒ 脚本化步态
 *     （pi/pose.py 等）完全没法调。
 *     但 IMU 只对"策略推理"必需 —— 直接下发关节角时观测不被使用。
 *
 *  ⚠️ 边界：
 *     · 只在【IMU 是唯一阻塞原因】时放行，其他原因照旧挡住
 *     · RLCA_SAFE_R_IMU_STALE 位照旧置位 ⇒ K/R 里完全可审计
 *     · 只影响 ACTIVE 准入，不影响急停/降级/趴下
 *
 *  ⇒ 跑【策略】之前必须关掉；跑【脚本步态】时可以开着。 */
void app_safety_set_imu_bypass(bool on);

/** 当前是否打开了 IMU 调试旁路。 */
bool app_safety_get_imu_bypass(void);

/** 动作幅度缩放（1.0 = 不限）。 */
float app_safety_amplitude_scale(void);

/** 详细状态（上报/诊断）。 */
const app_safety_state_t *app_safety_state(void);

/** 最近一次评估的原因位域。 */
uint8_t app_safety_reasons(void);

/** 严重程度（0=正常 1=告警 2=降级 3=急停）。 */
uint8_t app_safety_severity(void);

/** ⭐ 计算"趴下姿态"的目标角（写进 out[12]）。
 *  ⚠️ 独立成一个纯函数，便于单测验证"趴下姿态一定在限位内"。 */
void app_safety_compute_sitdown(const rlca_calib_t *calib, float *out);

/** 复位（上电或急停解除后）。 */
void app_safety_reset(void);

#endif /* APP_SAFETY_H */
