/**
  ******************************************************************************
  * @file    link_proto.h
  * @brief   与树莓派通信的【纯逻辑】编解码：CMD_JOINT / TEL_FAST / TEL_STATUS。
  *
  *  分两层（与 servo_bus 同样的思路）：
  *    link_proto —— 纯字节编解码 + 指令合法化，不依赖 HAL，可 PC 单测
  *    link_pi    —— 状态层：收字节、跑状态机、按 100 Hz 节奏发送、失联判定
  *
  *  ⚠️ 协议是【紧凑字节流】，不是 C 结构体内存布局（协议 §4.4 明确警告）。
  *     所有多字节量一律小端，逐字段写入 —— 绝不能把结构体指针强转成字节数组。
  *
  *  依据：firmware/doc/协议规范_树莓派通信.md
  *    §4.1 CMD_JOINT（LEN=31）
  *    §4.4 TEL_FAST （LEN=96）
  *    §4.8 TEL_STATUS（LEN=40）
  *    §8.3 指令合法化（NaN / 限幅 / 限速）
  ******************************************************************************
  */

#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"

/*==============================================================================
 * 下行：Pi → STM32
 *============================================================================*/

/** 解码后的 CMD_JOINT（已通过合法性检查，但【尚未】限幅/限速） */
typedef struct {
    uint8_t  mode;                              /* RLCA_MODE_*             */
    uint8_t  flags;                             /* RLCA_FLAG_*             */
    float    q_target[RLCA_N_JOINTS];           /* rad，逻辑索引 A          */
    int16_t  kp_scale;                          /* 0.01 单位（预留）        */
    uint8_t  led;
} pi_cmd_t;

/** CMD_MODE 的负载 */
typedef struct {
    uint8_t mode;
    uint8_t param;
} pi_mode_t;

/** CMD_CALIB 的负载 */
typedef struct {
    uint8_t action;                             /* RLCA_CALIB_*            */
} pi_calib_t;

/** 解码结果 */
typedef enum {
    LNK_DEC_OK = 0,
    LNK_DEC_ERR_LEN,        /* 负载长度不对            */
    LNK_DEC_ERR_PARAM       /* 字段取值非法            */
} lnk_dec_result_t;

/** 解码 CMD_JOINT。
 *  @param payload 负载首地址
 *  @param len     负载长度（必须 = RLCA_LEN_CMD_JOINT）
 *  @param out     解码结果
 *
 *  ⚠️ 本函数只做"长度 + 量程"检查，不做限幅/限速 ——
 *     限幅/限速必须由 rlca_cmd_apply_limits() 在这之后做，
 *     因为那一步需要标定表与上一帧状态。 */
lnk_dec_result_t link_decode_cmd_joint(const uint8_t *payload, uint8_t len, pi_cmd_t *out);

/** 解码 CMD_MODE。 */
lnk_dec_result_t link_decode_cmd_mode(const uint8_t *payload, uint8_t len, pi_mode_t *out);

/** 解码 CMD_CALIB。 */
lnk_dec_result_t link_decode_cmd_calib(const uint8_t *payload, uint8_t len, pi_calib_t *out);

/*==============================================================================
 * 指令合法化（协议 §8.3）
 *============================================================================*/

/** 把 q_target 逐关节处理成"可安全下发"的值：
 *    ① NaN / Inf      → 回落到该关节下限（安全侧）
 *    ② 超软限位        → 夹到限位
 *    ③ 单周期变化超限  → 按 RLCA_MAX_DELTA_PER_CYCLE_RAD 限速
 *
 *  @param calib   标定表（提供限位与方向）
 *  @param q_in    解码得到的原始目标角（逻辑索引 A）
 *  @param prev    输入 = 上一周期【实际下发】的目标角；输出 = 本周期结果
 *                 ⚠️ 是输入输出参数，调用方必须用同一份数组连着传
 *  @param q_out   本周期结果（可与 q_in 相同）
 *
 *  ⚠️ 这是"策略给跳变目标角"时的最后一道防线：
 *     即使上位机发来 +5 rad，一个周期最多也只走 0.15 rad。 */
void link_limits_apply(const rlca_calib_t *calib,
                       const float *q_in,
                       float *prev,
                       float *q_out);

/** 判断模式取值是否合法（非法则回落到 IDLE）。 */
uint8_t link_mode_sanitize(uint8_t mode);

/*==============================================================================
 * 上行：STM32 → Pi
 *============================================================================*/

/** TEL_FAST 的数据源（由调用方从各模块收集后传入）
 *  ⚠️ 这里不做单位换算，全部要求已经是协议规定的编码单位之前的物理量。 */
typedef struct {
    float    q_rad[RLCA_N_JOINTS];        /* 关节角（rad）           */
    float    qd_rad_s[RLCA_N_JOINTS];     /* 关节角速度（rad/s）      */
    float    tau_nm[RLCA_N_JOINTS];       /* 关节力矩（N·m）          */
    float    quat_wxyz[4];                /* 四元数 w,x,y,z           */
    float    gyro_rad_s[3];               /* 角速度 x,y,z（rad/s）    */
    int16_t  gait_sin_i16;                /* 已按 1/32767 编码        */
    int16_t  gait_cos_i16;
    uint16_t cycle_cnt;
    uint16_t voltage_mv;
    uint8_t  status;
    uint8_t  err_flags;
} tel_fast_src_t;

/** TEL_STATUS 的数据源 */
typedef struct {
    uint32_t uptime_ms;
    uint16_t voltage_mv;
    const uint8_t *temp_deci_c;           /* 12 个舵机温度（0.1 °C）   */
    uint16_t rx_overflow;
    uint16_t bus_err;
    uint16_t bus_timeout;
    uint8_t  mode;
    uint8_t  status;
    uint8_t  err_flags;
    uint8_t  seq_lost;
} tel_status_src_t;

/** 量化辅助：float → int16（带饱和，避免溢出成反向值）。
 *  ⚠️ 必须饱和而不是回绕：例如 tau 编码 0.001 N·m 时，
 *     3 N·m → 3000，不会溢出；但万一算错单位就可能炸成负数。 */
int16_t link_enc_i16(float value, float unit);

/** 打包 TEL_FAST 负载（写入 payload[0..95]）。
 *  @return 负载长度（恒为 RLCA_LEN_TEL_FAST） */
uint8_t link_pack_tel_fast(const tel_fast_src_t *src, uint8_t *payload);

/** 打包 TEL_STATUS 负载（写入 payload[0..39]）。
 *  @return 负载长度（恒为 RLCA_LEN_TEL_STATUS） */
uint8_t link_pack_tel_status(const tel_status_src_t *src, uint8_t *payload);

#endif /* LINK_PROTO_H */
