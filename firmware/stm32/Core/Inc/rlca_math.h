/**
  ******************************************************************************
  * @file    rlca_math.h
  * @brief   关节角 ↔ 舵机步数 的换算，以及舵机反馈量的物理单位换算。
  *
  *  这一层回答一个问题：【关节角 q(rad) 与舵机编码器读数(step) 怎么互转】。
  *
  *      step = dir × q × (4096 / 2π) + offset
  *
  *  其中：
  *    offset —— 标定得到的零点偏置（q = 0 时舵机的读数）
  *    dir    —— +1 / -1，吸收"舵机安装朝向 + 关节转轴朝向"的差异
  *
  *  ⚠️ 标定流程见 固件设计书 §6：机器人自然站立（q 全 0）→ 卸力矩 →
  *     等稳定 → 读 12 个舵机位置 → 存为 offset。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_MATH_H
#define RLCA_MATH_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

/*==============================================================================
 * 关节标定表（从 Flash 加载；未标定时为默认值）
 *============================================================================*/

/** ⚠️ 布局须与 Flash 存储一致（固件设计书 §6.3）。
 *  注意：这是【持久化结构】，字段顺序改动会导致旧数据失效 —— 同时递增
 *  RLCA_CALIB_VERSION 即可让固件自动回退到默认值。 */
typedef struct {
    uint32_t magic;                      /* RLCA_CALIB_MAGIC              */
    uint16_t version;                    /* RLCA_CALIB_VERSION            */
    uint16_t crc16;                      /* 覆盖本结构的其余字节           */
    int16_t  offset[RLCA_N_JOINTS];      /* 零点偏置（步）                 */
    int8_t   dir[RLCA_N_JOINTS];         /* +1 / -1                       */
    int16_t  limit_min[RLCA_N_JOINTS];   /* 软限位下限（步）               */
    int16_t  limit_max[RLCA_N_JOINTS];   /* 软限位上限（步）               */
} rlca_calib_t;

/** 舵机反馈换算后的关节状态（供 TEL_FAST 组装） */
typedef struct {
    int16_t pos_step;      /* 舵机原始位置反馈（步）                     */
    int16_t speed_step;    /* 舵机原始速度反馈（步/秒，带符号）           */
    int16_t load_raw;      /* 舵机原始负载反馈（bit10 = 方向位）          */
    uint8_t voltage_dv;    /* 舵机电压（0.1 V）                          */
    uint8_t temp_c;        /* 舵机温度（°C）                             */
    uint8_t moving;        /* 移动标志                                   */
    uint8_t err;           /* STS 错误码位域                             */
    uint8_t valid;         /* 本帧是否解析成功                           */
    float   q_rad;         /* 关节角（rad）                              */
    float   qd_rad_s;      /* 关节角速度（rad/s）                        */
    float   tau_nm;        /* 关节力矩（N·m）                            */
} rlca_joint_state_t;

/*==============================================================================
 * 标定表接口
 *============================================================================*/

/** 用默认值填充标定表（未标定状态：offset 全 0、dir 取初始表、限位取 URDF 值） */
void rlca_calib_load_defaults(rlca_calib_t *c);

/** ⭐⭐ 用【纯常量】构造标定表（2026-10-18）。
 *
 *  ⚠️ 前提：标定姿态（腿伸直）下 12 个舵机编码器都读到 2048
 *     （舵机内部零位已手动校准到该姿态）。
 *     则 offset = 2048 − dir × q_calib × (4096/2π)，全部可算。
 *
 *  ⚠️ offset **不等于** 2048：offset 是"URDF 零位（弯腿站姿）"的读数，
 *     与标定姿态相差 q_calib。直接用 2048 会让 thigh 偏 45°、calf 偏 90°。
 *
 *  ⭐ 好处：不读 flash、不需要标定流程 ⇒ 重启结果永远一致。 */
void rlca_calib_build_geometric(rlca_calib_t *c);

/** 按 RLCA_JOINT_DIR_INIT 填充默认方向；上层不应重复硬编码方向。 */
void rlca_calib_default_dirs(rlca_calib_t *c);

/** 由 q 限位（rad）换算并填充 limit_min/max（步） */
void rlca_calib_fill_limits(rlca_calib_t *c);

/** 校验标定表：magic / version / dir 合法性 / 限位区间。（不校验 crc16） */
bool rlca_calib_is_valid(const rlca_calib_t *c);

/** 单个 offset 是否在合法范围内。
 *
 *  ⚠️ 抽成函数是为了让"结构校验"和"标定结果校验"用同一个判据 ——
 *     最初两处各写了一遍，其中一处写成 `offset > 4096`，
 *     对负值恒为真，于是负的 offset 逃过了检查（单测抓到的）。
 *
 *  范围取 ±4096：编码器一圈正好 4096 步，合法零点偏置必然落在这一圈内。 */
bool rlca_calib_offset_ok(int16_t offset);

/** 计算标定表校验用的 crc16（覆盖 magic 之后、crc16 字段之前的字节）。
 *  ⚠️ 实现方式：把 crc16 字段自身当作 0 参与计算，避免"校验值参与自身"的问题。 */
uint16_t rlca_calib_crc(const rlca_calib_t *c);

/*==============================================================================
 * 换算
 *============================================================================*/

/** 关节角(rad) → 舵机步数。
 *  返回未做限幅的结果；是否越限由 rlca_q_in_limits() 判断。 */
int16_t rlca_q_to_step(const rlca_calib_t *c, uint8_t idx, float q_rad);

/** 舵机步数 → 关节角(rad)。 */
float rlca_step_to_q(const rlca_calib_t *c, uint8_t idx, int16_t step);

/** q 是否在软限位内（含端点） */
bool rlca_q_in_limits(const rlca_calib_t *c, uint8_t idx, float q_rad);

/** 把 q 夹到软限位内；返回夹取后的值。NaN 时返回 q_min。 */
float rlca_q_clamp(const rlca_calib_t *c, uint8_t idx, float q_rad);

/** 获取第 idx 个关节的限位范围（rad）。 */
void rlca_q_range(const rlca_calib_t *c, uint8_t idx, float *q_min, float *q_max);

/*==============================================================================
 * 舵机反馈 → 物理单位
 *============================================================================*/

/** 舵机速度反馈（步/秒） → 关节角速度（rad/s）。
 *  ⚠️ 不含 dir：dir 只影响"关节角正方向"的定义，速度的符号约定
 *     与位置一致，因此同样需要乘 dir。这里保留原始值，
 *     由 rlca_joint_state_update() 统一处理符号。 */
float rlca_servo_speed_to_rad_s(int16_t speed_step);

/** 舵机负载反馈 → 力矩（N·m）。
 *  bit10 = 方向位，低 10 位 = 幅度（0~1000 ≈ 0~100% 堵转扭矩）。 */
float rlca_servo_load_to_torque_nm(int16_t load_raw);

/** 位置回绕处理：4096 步一圈，相邻两次读数差超过半圈即认为回绕。
 *  @param prev 上一次读数
 *  @param now  本次读数
 *  @return 修正后的角度增量（步，带符号，范围 ±2048） */
int16_t rlca_servo_pos_delta(int16_t prev, int16_t now);

/** 用一帧反馈更新关节状态（含 dir 符号、位置回绕、力矩方向）。 */
void rlca_joint_state_update(const rlca_calib_t *c,
                             uint8_t idx,
                             int16_t prev_step,
                             rlca_joint_state_t *st);

/*==============================================================================
 * 数值健壮性
 *============================================================================*/

/** NaN / Inf 检查（防御上位机发来的异常值，协议规范 §8.3）。 */
bool rlca_float_is_valid(float v);

#endif /* RLCA_MATH_H */
